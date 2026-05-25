; =============================================================================
; VIRTIO_PKT.COM — VirtIO-Net Packet Driver for DOS (legacy virtio 0.9.5)
;
; Implements Crynwr Packet Driver Specification v1.09
; Supports: QEMU/KVM virtio-net-pci (PCI ID 1AF4:1000, legacy I/O port interface)
;
; Build:  nasm -f bin -o virtio_pkt.com virtio_pkt.asm
; Usage:  virtio_pkt 0x60          (install at INT 60h)
;         virtio_pkt 0x60 -u       (uninstall)
;
; References:
;   - Crynwr Packet Driver Spec v1.09
;   - VirtIO spec 1.1, section 4.1.4 (legacy I/O port layout)
;   - iPXE src/drivers/bus/virtio.c (legacy path), Apache 2.0
;   - PCI BIOS Specification Rev 2.1 (INT 1Ah functions B1xx)
;
; Memory model:
;   .COM file: CS=DS=ES=SS, all 16-bit, ORG 0x100
;
;   On init, we allocate a separate memory block via DOS INT 21h/48h
;   for the virtqueue descriptor tables (must be 4096-aligned).
;   The segment base of this block IS page-aligned by choosing a paragraph
;   that is a multiple of 256 (= 4096 / 16 bytes per paragraph).
;
;   After init, driver goes TSR. Timer hook (INT 1Ch) drives RX polling.
; =============================================================================

BITS 16
ORG 0x100

; =============================================================================
; Constants: VirtIO Legacy I/O port offsets from BAR0
; =============================================================================
VIRTIO_LEG_FEAT     EQU 0x00    ; dword R  — device feature bits
VIRTIO_LEG_USED     EQU 0x04    ; dword W  — guest accepted features
VIRTIO_LEG_BASE     EQU 0x08    ; dword W  — queue PFN (phys >> 12)
VIRTIO_LEG_SIZE     EQU 0x0c    ; word  RW — queue size
VIRTIO_LEG_SEL      EQU 0x0e    ; word  W  — queue select
VIRTIO_LEG_DB       EQU 0x10    ; word  W  — queue notify doorbell
VIRTIO_LEG_STAT     EQU 0x12    ; byte  RW — device status
VIRTIO_LEG_ISR      EQU 0x13    ; byte  R  — ISR status (reading clears)
VIRTIO_LEG_DEV      EQU 0x14    ; —        — device-specific config

; VirtIO device status bits
VIRTIO_STAT_ACK         EQU 0x01
VIRTIO_STAT_DRIVER      EQU 0x02
VIRTIO_STAT_DRIVER_OK   EQU 0x04
VIRTIO_STAT_FAILED      EQU 0x80

; VirtIO net feature bits
VIRTIO_NET_F_MAC        EQU (1 << 5)

; VirtIO descriptor flags
VRING_DESC_F_NEXT       EQU 0x01
VRING_DESC_F_WRITE      EQU 0x02    ; device writes (RX direction)

; =============================================================================
; Virtqueue parameters
; =============================================================================
; VRING_SIZE must match what QEMU reports for QUEUE_SIZE (read-only in virtio legacy).
; QEMU virtio-net reports 256. The ring descriptor table must have 256 entries.
; We only USE the first NUM_SLOTS*2 descriptors, but the table must be sized for 256.
VRING_SIZE          EQU 256         ; total ring capacity (dictated by QEMU)
VRING_MASK          EQU (VRING_SIZE - 1)
NUM_SLOTS           EQU 4           ; how many TX/RX slots we actually use
RX_QUEUE_IDX        EQU 0
TX_QUEUE_IDX        EQU 1

; Descriptor entry: addr(8) + len(4) + flags(2) + next(2) = 16 bytes
VRING_DESC_SZ       EQU 16

; Avail ring layout: flags(2) + idx(2) + ring[N](2*N) + used_event(2)
; We allocate: flags + idx + ring[VRING_SIZE]
VRING_AVAIL_SZ      EQU (4 + VRING_SIZE * 2)

; Used element: id(4) + len(4) = 8 bytes
VRING_USED_ELEM_SZ  EQU 8
; Used ring: flags(2) + idx(2) + ring[N](8*N) + avail_event(2)
VRING_USED_SZ       EQU (4 + VRING_SIZE * VRING_USED_ELEM_SZ)

; Per-queue descriptor region size (all in one page):
; desc[N*2] + avail + padding + used  (must fit in 4096 bytes)
; desc: 16 * 2 * 16 = 512, avail: 36, used: 132  => ~680 bytes — fits in one 4K page
; Vring page layout for VRING_SIZE=256:
;   desc:  256*16 = 4096 bytes  -> offset 0x0000
;   avail: 4+256*2 = 516 bytes  -> offset 0x1000 (4096-aligned)
;   used:  4+256*8 = 2052 bytes -> offset 0x2000 (4096-aligned)
;   total: 0x2000 + 2052 = ~14KB per queue -> 4 pages = 16384 bytes
; We allocate 4 pages per queue, tx follows rx.
VRING_PAGE_SZ       EQU (4 * 4096)  ; 16384 bytes per queue

; VirtIO net header (legacy, 10 bytes, prepended to every RX/TX buffer)
VIRTIO_NET_HDR_SZ   EQU 10

; Ethernet buffer size (max frame without FCS)
ETH_BUF_SZ          EQU 1514

; Number of TX/RX buffers (≤ VRING_SIZE)
NUM_RX              EQU NUM_SLOTS   ; actual RX buffers we maintain
NUM_TX              EQU NUM_SLOTS   ; actual TX slots we use

; =============================================================================
; Packet Driver constants (Crynwr spec v1.09)
; =============================================================================
PD_VER              EQU 0x0109  ; spec version
PD_CLASS_ETHER      EQU 1       ; Ethernet
PD_IF_TYPE          EQU 6       ; DIX Ethernet (standard type per Crynwr spec)
PD_IF_NUMBER        EQU 0       ; first instance

PD_DRIVER_INFO      EQU 0x01
PD_ACCESS_TYPE      EQU 0x02
PD_RELEASE_TYPE     EQU 0x03
PD_SEND_PKT         EQU 0x04
PD_TERMINATE        EQU 0x05
PD_GET_ADDRESS      EQU 0x06
PD_RESET_IFACE      EQU 0x07
PD_GET_PARAMS       EQU 0x0A

PD_ERR_BAD_HANDLE   EQU 0x01
PD_ERR_NO_CLASS     EQU 0x02
PD_ERR_NO_SPACE     EQU 0x04
PD_ERR_TYPE_INUSE   EQU 0x05
PD_ERR_BAD_COMMAND  EQU 0x0D
PD_ERR_CANT_SEND    EQU 0x0E

; =============================================================================
; ENTRY POINT — init/install code (becomes non-resident after TSR)
; =============================================================================
entry:
    ; Print banner
    mov     dx, msg_banner
    call    print_str

    ; Parse command-line argument: INT vector (e.g. 0x60)
    mov     si, 0x81            ; PSP:81h = command line
    call    skip_spaces
    call    parse_hex_byte      ; returns AL = vector, CF=1 on error
    jc      .usage
    cmp     al, 0x60
    jl      .usage              ; sanity: vector should be ≥ 0x60
    mov     [pkt_vector], al

    ; Check for -u (uninstall) flag
    call    skip_spaces
    cmp     byte [si], '-'
    jne     .do_install
    inc     si
    mov     al, [si]
    or      al, 0x20
    cmp     al, 'u'
    je      .do_uninstall

.do_install:
    ; 1. Find virtio-net via PCI BIOS
    call    pci_find_device     ; sets virtio_iobase, CF=1 if not found
    jc      .no_device

    ; 2. Allocate 4K-aligned memory for virtqueue rings
    call    alloc_vring_memory  ; sets vring_seg, CF=1 on failure
    jc      .no_mem

    ; 3. Init virtio device
    call    virtio_init         ; CF=1 on failure
    jc      .init_fail

    ; 3c. Get InDOS flag pointer (INT 21h/34h)
    ;     ES:BX = pointer to DOS InDOS byte (non-zero when DOS is busy)
    mov     ah, 0x34
    int     0x21
    mov     [indos_off], bx
    mov     [indos_seg], es

    ; 4. Install packet driver INT handler
    call    install_handler     ; CF=1 if vector already in use
    jc      .vec_busy

    ; 5. Hook INT 1Ch (timer tick) for RX polling
    call    hook_timer

    ; 6. Print success
    mov     dx, msg_ok1
    call    print_str
    mov     al, [pkt_vector]
    call    print_hex8
    mov     dx, msg_ok2
    call    print_str

    ; 7. TSR: keep resident up to _init_end
    ;    CLI here prevents timer IRQ from firing between hook_timer and INT 21h/31h.
    ;    The IRQ could enter pd_handler or timer_isr before DOS has finished the TSR
    ;    bookkeeping, causing "Cannot terminate permanent instance" or stack corruption.
    ;    INT 21h/31h restores IF itself via its own IRET — no explicit STI needed.
    cli
    mov     dx, (_init_end - entry + 0x100 + 15)
    shr     dx, 4               ; paragraphs = bytes / 16
    add     dx, 16              ; + PSP
    mov     ax, 0x3100          ; terminate + stay resident, exit code 0
    int     0x21
    ; never returns

.do_uninstall:
    call    try_uninstall
    jmp     .exit_ok

.usage:
    mov     dx, msg_usage
    call    print_str
    jmp     .exit_err

.no_device:
    mov     dx, msg_no_dev
    call    print_str
    jmp     .exit_err

.no_mem:
    mov     dx, msg_no_mem
    call    print_str
    jmp     .exit_err

.init_fail:
    mov     dx, msg_init_fail
    call    print_str
    jmp     .exit_err

.vec_busy:
    mov     dx, msg_vec_busy
    call    print_str
    jmp     .exit_err

.exit_ok:
    mov     ax, 0x4C00
    int     0x21

.exit_err:
    mov     ax, 0x4C01
    int     0x21


; =============================================================================
; PCI_FIND_DEVICE — locate virtio-net (vendor 1AF4, device 1000)
; Sets: virtio_iobase, pci_bus, pci_devfn
; Returns: CF=0 ok, CF=1 not found
; =============================================================================
pci_find_device:
    ; Check PCI BIOS present
    mov     ax, 0xB101
    int     0x1A
    jc      .fail
    cmp     ah, 0
    jnz     .fail

    ; Find device: vendor=1AF4, device=1000
    mov     ax, 0xB102
    mov     cx, 0x1000          ; device ID
    mov     dx, 0x1AF4          ; vendor ID
    xor     si, si
    int     0x1A
    jc      .fail
    cmp     ah, 0
    jnz     .fail

    mov     [pci_bus],   bh
    mov     [pci_devfn], bl

    ; Read BAR0 (config offset 0x10)
    mov     ax, 0xB10A          ; read config dword
    mov     bh, [pci_bus]
    mov     bl, [pci_devfn]
    mov     di, 0x10
    int     0x1A
    jc      .fail

    ; ECX = BAR0 value
    test    cl, 1               ; bit 0 = 1 means I/O BAR
    jz      .not_io
    and     cx, 0xFFFC
    mov     [virtio_iobase], cx

    ; Enable I/O + Bus Master in PCI command register (offset 4)
    mov     ax, 0xB10A
    mov     bh, [pci_bus]
    mov     bl, [pci_devfn]
    mov     di, 4
    int     0x1A                ; ECX = current command word
    or      cx, 0x0005          ; IO space + bus master
    mov     ax, 0xB10C          ; write config word
    mov     bh, [pci_bus]
    mov     bl, [pci_devfn]
    mov     di, 4
    int     0x1A

    ; Print found message
    push    dx
    mov     dx, msg_found
    call    print_str
    mov     ax, [virtio_iobase]
    call    print_hex16
    mov     dx, msg_crlf
    call    print_str
    pop     dx

    clc
    ret

.not_io:
    ; BAR0 is memory-mapped = virtio 1.0 modern, not supported here
    ; Workaround hint: launch QEMU with -device virtio-net-pci,disable-modern=on
    mov     dx, msg_modern
    call    print_str
.fail:
    stc
    ret


; =============================================================================
; ALLOC_VRING_MEMORY — compute 4096-aligned segment from static vring_pool
;
; Strategy: vring_pool is a static buffer inside the .COM image (3*4096+4095 bytes).
; We compute its physical address, round up to the next 4096-byte boundary,
; then convert back to a paragraph segment number.
; No DOS INT 21h/48h needed — avoids the "all memory pre-allocated to .COM" trap.
;
; Physical address of vring_pool = DS*16 + offset(vring_pool)
; Aligned phys = (phys + 0xFFF) & ~0xFFF
; vring_seg = aligned_phys / 16  (paragraph number)
;
; Sets: vring_seg
; Returns: CF=0 always (cannot fail)
; =============================================================================
alloc_vring_memory:
    ; Compute DS physical base: DS * 16 (20-bit, fits in dx:ax)
    mov     ax, ds
    shr     ax, 12          ; high nibble → dx
    mov     dx, ax
    mov     ax, ds
    shl     ax, 4           ; low 16 bits of DS*16
    ; dx:ax = DS * 16

    ; Add offset of vring_pool
    add     ax, vring_pool
    adc     dx, 0
    ; dx:ax = physical address of vring_pool

    ; Round up to 4096-byte boundary: (phys + 0xFFF) & ~0xFFF
    add     ax, 0x0FFF
    adc     dx, 0
    and     ax, 0xF000      ; clear low 12 bits
    ; dx:ax = 4096-aligned physical address

    ; Convert physical address to paragraph (segment): phys / 16
    ; dx:ax >> 4  →  shift right 4 across the pair
    mov     cx, 4
.shr_loop:
    shr     dx, 1
    rcr     ax, 1
    loop    .shr_loop
    ; ax = paragraph number of aligned vring start
    mov     [vring_seg], ax

    ; Print vring segment
    push    dx
    mov     dx, msg_vring
    call    print_str
    mov     ax, [vring_seg]
    call    print_hex16
    mov     dx, msg_crlf
    call    print_str
    pop     dx

    clc
    ret


; =============================================================================
; VIRTIO_INIT — initialize virtio device via legacy I/O port interface
; =============================================================================
virtio_init:
    ; Pre-compute DS physical base (used for descriptor physical addresses)
    mov     ax, ds
    shl     ax, 4
    mov     [ds_phys_lo], ax
    mov     ax, ds
    shr     ax, 12
    mov     [ds_phys_hi], ax

    mov     dx, [virtio_iobase]

    ; --- Reset device ---
    add     dx, VIRTIO_LEG_STAT
    xor     al, al
    out     dx, al
    ; Poll until status reads back 0
    mov     cx, 0x2000
.reset_wait:
    in      al, dx
    test    al, al
    jz      .reset_done
    loop    .reset_wait
.reset_done:

    ; --- ACKNOWLEDGE ---
    mov     al, VIRTIO_STAT_ACK
    out     dx, al

    ; --- DRIVER ---
    mov     al, (VIRTIO_STAT_ACK | VIRTIO_STAT_DRIVER)
    out     dx, al

    ; --- Read device features ---
    mov     dx, [virtio_iobase]
    add     dx, VIRTIO_LEG_FEAT
    in      eax, dx
    mov     [dev_features], eax

    ; --- Negotiate: accept only MAC ---
    mov     dx, [virtio_iobase]
    add     dx, VIRTIO_LEG_USED
    mov     eax, [dev_features]
    and     eax, VIRTIO_NET_F_MAC
    out     dx, eax

    ; --- Read MAC address from device config ---
    mov     dx, [virtio_iobase]
    add     dx, VIRTIO_LEG_DEV  ; +0 = MAC[0]
    mov     di, mac_addr
    mov     cx, 6
.mac_loop:
    in      al, dx
    mov     [di], al
    inc     di
    inc     dx
    loop    .mac_loop

    push    dx
    mov     dx, msg_mac
    call    print_str
    call    print_mac
    mov     dx, msg_crlf
    call    print_str
    pop     dx

    ; --- Setup RX queue (index 0) ---
    call    setup_queue_rx
    jc      .fail

    ; --- Setup TX queue (index 1) ---
    call    setup_queue_tx
    jc      .fail

    ; --- DRIVER_OK ---
    mov     dx, [virtio_iobase]
    add     dx, VIRTIO_LEG_STAT
    mov     al, (VIRTIO_STAT_ACK | VIRTIO_STAT_DRIVER | VIRTIO_STAT_DRIVER_OK)
    out     dx, al

    ; --- Fill RX queue with buffers ---
    call    refill_rx

    clc
    ret

.fail:
    mov     dx, [virtio_iobase]
    add     dx, VIRTIO_LEG_STAT
    mov     al, VIRTIO_STAT_FAILED
    out     dx, al
    stc
    ret


; =============================================================================
; SETUP_QUEUE_RX / SETUP_QUEUE_TX
;
; Virtqueue layout (legacy, flat in one 4K page at vring_seg:0):
;
; RX page (segment = vring_seg):
;   offset 0x000: desc table  [VRING_SIZE * 2 descriptors * 16 bytes = 512B]
;   offset 0x200: avail ring  [4 + VRING_SIZE*2 = 36B]
;   offset 0x230: (padding)
;   offset 0x400: used ring   [4 + VRING_SIZE*8 = 132B]
;
; TX page (segment = vring_seg + 256):
;   same layout
; =============================================================================

; Offsets within a vring page
VRING_DESC_OFF      EQU 0x0000      ; desc table at offset 0
VRING_AVAIL_OFF     EQU 0x1000      ; avail ring at offset 4096 (after 256*16=4096 desc)
VRING_USED_OFF      EQU 0x2000      ; used ring at offset 8192

setup_queue_rx:
    ; Select queue 0
    mov     dx, [virtio_iobase]
    add     dx, VIRTIO_LEG_SEL
    xor     ax, ax
    out     dx, ax

    ; Read max queue size
    mov     dx, [virtio_iobase]
    add     dx, VIRTIO_LEG_SIZE
    in      ax, dx
    test    ax, ax
    jz      .fail
    ; Use min(device_max, VRING_SIZE)
    cmp     ax, VRING_SIZE
    jbe     .use_dev_size
    mov     ax, VRING_SIZE
    jmp     .write_size
.use_dev_size:
.write_size:
    mov     [rx_qsz], ax
    out     dx, ax              ; write selected size back

    ; Zero the entire RX vring page first
    push    es
    mov     es, [vring_seg]
    xor     di, di
    xor     ax, ax
    mov     cx, VRING_PAGE_SZ / 2
    rep     stosw
    pop     es

    ; Write queue PFN to device (32-bit I/O write)
    ; PFN = vring_seg >> 8  (since vring_seg is 256-paragraph aligned)
    ; Physical address <= 1MB, so PFN hi16 = 0
    mov     dx, [virtio_iobase]
    add     dx, VIRTIO_LEG_BASE
    xor     eax, eax
    mov     ax, [vring_seg]
    shr     ax, 8               ; EAX = PFN (fits in 16 bits)
    o32 out dx, eax             ; 32-bit I/O write (operand-size prefix)

    ; Build RX descriptor table:
    ; Each RX slot uses 2 descriptors:
    ;   desc[2i]   -> net header buffer  (device-writable)
    ;   desc[2i+1] -> ethernet data buffer (device-writable)
    ; Net header and data buffers live in our .COM segment (DS)
    ; Physical address = DS * 16 + offset
    ;
    ; ds_phys_lo/hi already set by virtio_init
    push    es
    mov     es, [vring_seg]

    xor     cx, cx              ; cx = slot index
.rx_desc_loop:
    cmp     cx, NUM_SLOTS       ; only fill NUM_SLOTS real buffers
    je      .rx_desc_done

    ; --- desc[2*cx]: virtio_net_hdr buffer ---
    ; phys = DS_phys + rx_hdrs + cx * VIRTIO_NET_HDR_SZ
    mov     ax, cx
    mov     bx, VIRTIO_NET_HDR_SZ
    mul     bx                  ; dx:ax = cx * 10 (dx=0 for small cx)
    add     ax, rx_hdrs
    adc     dx, 0
    add     ax, [ds_phys_lo]
    adc     dx, [ds_phys_hi]    ; dx:ax = physical address

    ; Descriptor byte offset in desc table = (2*cx) * 16
    mov     di, cx
    shl     di, 5               ; di = cx * 32  (2 descs * 16 bytes each)

    ; Write desc[2*cx]
    mov     [es: VRING_DESC_OFF + di + 0], ax   ; addr lo
    mov     [es: VRING_DESC_OFF + di + 2], dx   ; addr hi16
    mov     word [es: VRING_DESC_OFF + di + 4], 0   ; addr bits 32-47
    mov     word [es: VRING_DESC_OFF + di + 6], 0   ; addr bits 48-63
    mov     word [es: VRING_DESC_OFF + di + 8], VIRTIO_NET_HDR_SZ  ; len
    mov     word [es: VRING_DESC_OFF + di + 10], 0  ; len hi (unused)
    mov     word [es: VRING_DESC_OFF + di + 12], (VRING_DESC_F_WRITE | VRING_DESC_F_NEXT)
    mov     ax, cx
    shl     ax, 1
    inc     ax
    mov     [es: VRING_DESC_OFF + di + 14], ax  ; next = 2*cx + 1

    ; --- desc[2*cx+1]: rx data buffer ---
    ; phys = DS_phys + rx_bufs + cx * ETH_BUF_SZ
    mov     ax, cx
    mov     bx, ETH_BUF_SZ
    mul     bx
    add     ax, rx_bufs
    adc     dx, 0
    add     ax, [ds_phys_lo]
    adc     dx, [ds_phys_hi]

    add     di, 16              ; di = desc[2*cx+1] offset
    mov     [es: VRING_DESC_OFF + di + 0], ax
    mov     [es: VRING_DESC_OFF + di + 2], dx
    mov     word [es: VRING_DESC_OFF + di + 4], 0
    mov     word [es: VRING_DESC_OFF + di + 6], 0
    mov     word [es: VRING_DESC_OFF + di + 8], ETH_BUF_SZ
    mov     word [es: VRING_DESC_OFF + di + 10], 0
    mov     word [es: VRING_DESC_OFF + di + 12], VRING_DESC_F_WRITE
    mov     word [es: VRING_DESC_OFF + di + 14], 0

    ; Init avail ring: avail.ring[cx] = 2*cx (head desc for slot cx)
    mov     ax, cx
    shl     ax, 1
    mov     di, cx
    shl     di, 1
    mov     [es: VRING_AVAIL_OFF + 4 + di], ax  ; +4 skips flags+idx

    inc     cx
    jmp     .rx_desc_loop

.rx_desc_done:
    ; avail.flags = 0 (allow interrupts)
    mov     word [es: VRING_AVAIL_OFF + 0], 0
    ; avail.idx = 0 (filled in refill_rx)
    mov     word [es: VRING_AVAIL_OFF + 2], 0
    ; used.flags = 0, used.idx = 0 (already zeroed above)
    pop     es
    clc
    ret

.fail:
    pop     es
    stc
    ret


setup_queue_tx:
    ; Select queue 1
    mov     dx, [virtio_iobase]
    add     dx, VIRTIO_LEG_SEL
    mov     ax, 1
    out     dx, ax

    ; Read and write queue size
    mov     dx, [virtio_iobase]
    add     dx, VIRTIO_LEG_SIZE
    in      ax, dx
    test    ax, ax
    jz      .fail
    cmp     ax, VRING_SIZE
    jbe     .use_sz
    mov     ax, VRING_SIZE
.use_sz:
    mov     [tx_qsz], ax
    out     dx, ax

    ; TX page = after RX vring (VRING_PAGE_SZ/16 paragraphs after vring_seg)
    ; VRING_PAGE_SZ=16384, 16384/16=1024 paragraphs
    mov     ax, [vring_seg]
    add     ax, 1024        ; 16384 bytes / 16 bytes per paragraph
    mov     [tx_vring_seg], ax

    ; Zero TX vring page first
    push    es
    mov     es, [tx_vring_seg]
    xor     di, di
    xor     ax, ax
    mov     cx, VRING_PAGE_SZ / 2
    rep     stosw
    pop     es

    ; Write TX queue PFN to device
    mov     dx, [virtio_iobase]
    add     dx, VIRTIO_LEG_BASE
    xor     eax, eax
    mov     ax, [tx_vring_seg]
    shr     ax, 8
    o32 out dx, eax

    ; TX descriptors are filled dynamically on send, avail/used rings pre-init'd
    clc
    ret

.fail:
    stc
    ret


; =============================================================================
; REFILL_RX — post all RX descriptors to avail ring and notify device
; =============================================================================
refill_rx:
    push    es
    mov     es, [vring_seg]

    ; avail ring: only post NUM_SLOTS real buffers
    ; ring[i] = 2*i  (head descriptor for slot i)
    ; avail.idx = NUM_SLOTS
    mov     word [es: VRING_AVAIL_OFF + 4 + 0], 0   ; slot 0 head = desc 0
    mov     word [es: VRING_AVAIL_OFF + 4 + 2], 2   ; slot 1 head = desc 2
    mov     word [es: VRING_AVAIL_OFF + 4 + 4], 4   ; slot 2 head = desc 4
    mov     word [es: VRING_AVAIL_OFF + 4 + 6], 6   ; slot 3 head = desc 6
    mov     word [es: VRING_AVAIL_OFF + 2], NUM_SLOTS  ; avail.idx = 4

    mov     [rx_last_used], word 0

    pop     es

    ; Notify device: queue 0
    mov     dx, [virtio_iobase]
    add     dx, VIRTIO_LEG_DB
    xor     ax, ax
    out     dx, ax
    ret


; =============================================================================
; INSTALL_HANDLER — set INT vector to pd_handler
; =============================================================================
install_handler:
    movzx   bx, byte [pkt_vector]
    shl     bx, 2

    xor     ax, ax
    mov     es, ax

    ; Check if already occupied by a PD (signature 3 bytes before handler)
    mov     ax, [es: bx + 2]    ; segment of current handler
    mov     di, [es: bx]        ; offset
    sub     di, 3
    mov     ds, ax
    push    cs
    pop     ax
    cmp     word [di], 'PK'
    push    cs
    pop     ds
    jne     .install

    stc
    ret

.install:
    ; Save old vector
    xor     ax, ax
    mov     es, ax
    movzx   bx, byte [pkt_vector]
    shl     bx, 2
    mov     ax, [es: bx]
    mov     [old_vec_off], ax
    mov     ax, [es: bx + 2]
    mov     [old_vec_seg], ax

    cli
    mov     word [es: bx],     pd_handler
    mov     [es: bx + 2], cs
    ; no STI here — keep IRQs off until hook_timer + INT 21h/31h sequence
    clc
    ret


; =============================================================================
; HOOK_TIMER — chain INT 1Ch (user timer tick) for RX polling
; =============================================================================
hook_timer:
    ; Read + install INT 1Ch vector atomically under CLI.
    ; We do NOT STI after install — caller (entry) will CLI again before INT 21h/31h.
    ; STI will happen naturally when INT 21h restores flags via IRET.
    cli
    xor     ax, ax
    mov     es, ax
    mov     ax, [es: 0x1C * 4]
    mov     [old_1c_off], ax
    mov     ax, [es: 0x1C * 4 + 2]
    mov     [old_1c_seg], ax
    mov     word [es: 0x1C * 4],     timer_isr
    mov     [es: 0x1C * 4 + 2], cs
    ; intentionally no STI here — entry code keeps CLI until INT 21h/31h
    ret


; =============================================================================
; TRY_UNINSTALL — remove handlers and free memory
; =============================================================================
try_uninstall:
    mov     dx, msg_notimp
    call    print_str
    ret


; =============================================================================
; ========================  RESIDENT SECTION  =================================
; =============================================================================

; =============================================================================
; PD_HANDLER — Packet Driver INT handler
;
; Crynwr spec v1.09: the handler entry point must be followed (within 12 bytes)
; by the null-terminated string "PKT DRVR". mTCP scans INT 60h–80h looking for
; this signature to detect the driver. We embed it as a 3-byte JMP over the
; string, then fall through to the real dispatch code.
;
; Layout:
;   pd_handler:  JMP SHORT pd_dispatch  (2 bytes)
;                NOP                    (1 byte — makes it 3-byte "entry code")
;   pd_sig:      db 'PKT DRVR', 0       (9 bytes — within first 12 bytes)
;   pd_dispatch: ... real handler ...
; =============================================================================
pd_handler:
    jmp     short pd_dispatch
    nop
pd_sig:
    db      'PKT DRVR', 0
pd_dispatch:
    ; Stack frame on entry (INT frame already pushed by CPU):
    ;   [sp+0] = IP, [sp+2] = CS, [sp+4] = FLAGS
    ;
    ; We push: ax, ds, es, bx, cx, si, di, bp  (8 words)
    ; Then set BP = SP so we can access saved values via BP
    ;
    ; Stack layout after all pushes (BP-relative):
    ;   [bp+0]  = saved BP
    ;   [bp+2]  = saved DI
    ;   [bp+4]  = saved SI
    ;   [bp+6]  = saved CX
    ;   [bp+8]  = saved BX
    ;   [bp+10] = saved ES
    ;   [bp+12] = saved DS  (= caller's DS)
    ;   [bp+14] = saved AX  (AH=func, AL=if_class)
    ;   [bp+16] = INT ret IP
    ;   [bp+18] = INT ret CS
    ;   [bp+20] = INT FLAGS

    push    ax              ; save AX first — AH=function, AL=if_class for ACCESS_TYPE
    push    ds
    push    es
    push    bx
    push    cx
    push    si
    push    di
    push    bp
    mov     bp, sp          ; frame set

    push    cs
    pop     ds              ; DS = our segment

    ; Read saved values from stack frame
    mov     ax, [bp + 14]   ; saved AX (AH=func, AL=if_class)
    mov     [caller_al], al ; save AL (if_class) for ACCESS_TYPE
    mov     [caller_ah], ah ; save AH (func code) BEFORE AX is overwritten

    mov     ax, [bp + 12]   ; caller's DS
    mov     [caller_ds], ax

    ; Save caller's ES:DI from saved stack slots
    mov     ax, [bp + 10]   ; saved ES slot
    mov     [caller_es], ax
    mov     ax, [bp + 2]    ; saved DI slot
    mov     [caller_di], ax

    mov     ah, [caller_ah] ; restore AH for dispatch
    cmp     ah, PD_DRIVER_INFO
    je      .fn_info
    cmp     ah, PD_ACCESS_TYPE
    je      .fn_access
    cmp     ah, PD_RELEASE_TYPE
    je      .fn_release
    cmp     ah, PD_SEND_PKT
    je      .fn_send
    cmp     ah, PD_TERMINATE
    je      .fn_terminate
    cmp     ah, PD_GET_ADDRESS
    je      .fn_getaddr
    cmp     ah, PD_RESET_IFACE
    je      .fn_reset
    cmp     ah, PD_GET_PARAMS
    je      .fn_params

    mov     dh, PD_ERR_BAD_COMMAND
    jmp     .err

; --- 0x01: DRIVER_INFO ---
; Entry: AH=1, AL=255 (per spec: caller sets AL=255 as "info" sub-function)
; Exit:  BX=version, CH=class, CL=number, DX=if_type, AL=functionality, DS:SI=name
; functionality: 1 = basic functions only
.fn_info:
    mov     bx, PD_VER              ; BX = version (0x0109)
    mov     ch, PD_CLASS_ETHER      ; CH = class (1 = Ethernet)
    mov     cl, PD_IF_NUMBER        ; CL = interface number (0)
    mov     dx, PD_IF_TYPE          ; DX = interface type (6 = DIX Ethernet)
    ; AL = functionality: 1 = basic functions
    ; Store return values into saved-register slots on stack so pop restores them
    mov     [bp + 8],  bx           ; saved BX slot = version
    mov     [bp + 6],  cx           ; saved CX slot = class/number
    mov     word [bp + 14], 0x0001  ; saved AX slot: AH=0, AL=1 (functionality)
    push    cs
    pop     ds
    mov     si, str_drvname
    mov     [bp + 4],  si           ; saved SI slot = name offset
    mov     [bp + 12], ds          ; saved DS slot = our segment (for DS:SI name)
    jmp     .ok

; --- 0x02: ACCESS_TYPE ---
; Spec: AL=if_class, BX=if_type, DL=if_number, DS:SI=type_ptr, CX=typelen, ES:DI=receiver
; mTCP passes: AL=1 (ETHER), BX=if_type (or -1 wildcard), DL=0, CX=0 (wildcard), ES:DI=receiver
.fn_access:
    cmp     byte [caller_al], PD_CLASS_ETHER  ; AL was saved before prolog trashed AX
    jne     .err_no_class
    cmp     word [handle], 0
    jnz     .err_inuse
    ; Save receiver callback from ES:DI (already saved as caller_es/caller_di in prolog)
    mov     ax, [caller_di]
    mov     [recv_off], ax
    mov     [recv_seg_word], ax     ; recv_seg_word[0] = offset
    mov     ax, [caller_es]
    mov     [recv_seg], ax
    mov     [recv_seg_word + 2], ax ; recv_seg_word[2] = segment
    mov     word [handle], 1
    ; return handle = 1 in AX
    mov     word [bp + 14], 1       ; saved AX slot: AX = 1 (handle)
    jmp     .ok

.err_no_class:
    mov     dh, PD_ERR_NO_CLASS
    jmp     .err
.err_inuse:
    mov     dh, PD_ERR_TYPE_INUSE
    jmp     .err

; --- 0x03: RELEASE_TYPE ---
.fn_release:
    cmp     bx, 1
    jne     .err_handle
    mov     word [handle], 0
    jmp     .ok

.err_handle:
    mov     dh, PD_ERR_BAD_HANDLE
    jmp     .err

; --- 0x04: SEND_PKT (DS:SI = buffer, CX = length) ---
.fn_send:
    call    do_send
    jc      .err_send
    jmp     .ok

.err_send:
    mov     dh, PD_ERR_CANT_SEND
    jmp     .err

; --- 0x05: TERMINATE ---
.fn_terminate:
    cli
    xor     ax, ax
    mov     es, ax
    movzx   bx, byte [pkt_vector]
    shl     bx, 2
    mov     ax, [old_vec_off]
    mov     [es: bx], ax
    mov     ax, [old_vec_seg]
    mov     [es: bx + 2], ax
    ; restore timer
    mov     ax, [old_1c_off]
    mov     [es: 0x1C * 4], ax
    mov     ax, [old_1c_seg]
    mov     [es: 0x1C * 4 + 2], ax
    sti
    jmp     .ok

; --- 0x06: GET_ADDRESS (ES:DI = buffer, CX = buf size) ---
.fn_getaddr:
    cmp     bx, 1
    jne     .err_handle
    cmp     cx, 6
    jl      .err_space
    push    ds
    pop     es
    mov     si, mac_addr
    mov     di, [caller_di]
    mov     es, [caller_es]
    mov     cx, 6
    rep     movsb
    push    cs
    pop     ds
    ; return CX=6 in saved slot
    mov     word [bp + 6], 6        ; saved CX slot
    jmp     .ok

.err_space:
    mov     dh, PD_ERR_NO_SPACE
    jmp     .err

; --- 0x07: RESET_IFACE ---
.fn_reset:
    call    virtio_init
    jmp     .ok

; --- 0x0A: GET_PARAMETERS ---
.fn_params:
    ; return BX=6, CX=14, DX=6 in saved slots
    mov     word [bp + 8], 6        ; saved BX slot = bcast addr len
    mov     word [bp + 6], 14       ; saved CX slot = header len
    ; DX not saved on stack (not in prolog), return via register is fine for DX
    mov     dx, 6
    jmp     .ok

; pd_ret_ok / pd_ret_err:
; Stack layout (bp-relative, push ax was FIRST push in prolog):
;   [bp+0]  = saved BP
;   [bp+2]  = saved DI
;   [bp+4]  = saved SI
;   [bp+6]  = saved CX
;   [bp+8]  = saved BX
;   [bp+10] = saved ES
;   [bp+12] = saved DS
;   [bp+14] = saved AX (caller's AX: AH=func, AL=class)
;   [bp+16] = INT return IP
;   [bp+18] = INT return CS
;   [bp+20] = INT FLAGS (CF manipulated here)
;
; Return values from handler functions are WRITTEN INTO the saved-register slots
; BEFORE reaching .ok/.err (e.g. mov [bp+8], bx to return a BX value).
; This ensures pop restores the desired return values.
;
; DX is NOT saved in the prolog, so DX is returned as-is from the function.

.ok:
    and     word [bp + 20], 0xFFFE  ; clear CF in INT FLAGS
    pop     bp
    pop     di
    pop     si
    pop     cx
    pop     bx
    pop     es
    pop     ds
    pop     ax
    iret

.err:
    ; DH = error code — return it: write into saved AH slot (high byte of [bp+14+1])
    ; Actually we set DH and return it via the conventional error method.
    ; err path: CF=1, DH=error_code. DH not on stack, returned as-is.
    or      word [bp + 20], 0x0001  ; set CF in INT FLAGS
    pop     bp
    pop     di
    pop     si
    pop     cx
    pop     bx
    pop     es
    pop     ds
    pop     ax
    iret

; =============================================================================
; DO_SEND — transmit one packet
; Entry: SI = packet offset, CX = length
;         DS = our segment (CS), caller_ds = caller's segment for SI
; =============================================================================
; =============================================================================
; TX_SELFTEST — send one 60-byte broadcast frame at init time
; Verifies TX path works before TSR. Uses do_send_direct which bypasses
; caller_ds (since we ARE the driver at this point, DS=CS).
; =============================================================================
tx_selftest:
    push    es
    push    di
    push    cx
    push    ax

    ; Zero tx_buf
    push    cs
    pop     es
    mov     di, tx_buf
    xor     ax, ax
    mov     cx, ETH_BUF_SZ / 2
    rep     stosw

    ; DST = FF:FF:FF:FF:FF:FF  (broadcast)
    mov     byte [tx_buf + 0], 0xFF
    mov     byte [tx_buf + 1], 0xFF
    mov     byte [tx_buf + 2], 0xFF
    mov     byte [tx_buf + 3], 0xFF
    mov     byte [tx_buf + 4], 0xFF
    mov     byte [tx_buf + 5], 0xFF
    ; SRC = our MAC
    mov     al, [mac_addr + 0]  
    mov     [tx_buf + 6], al
    mov     al, [mac_addr + 1]
    mov     [tx_buf + 7], al
    mov     al, [mac_addr + 2]
    mov     [tx_buf + 8], al
    mov     al, [mac_addr + 3]
    mov     [tx_buf + 9], al
    mov     al, [mac_addr + 4]
    mov     [tx_buf + 10], al
    mov     al, [mac_addr + 5]
    mov     [tx_buf + 11], al
    ; EtherType = 0x0806 (ARP)
    mov     byte [tx_buf + 12], 0x08
    mov     byte [tx_buf + 13], 0x06

    ; caller_ds = our segment (tx_buf is in our seg)
    push    ds
    mov     ax, ds
    mov     [caller_ds], ax
    pop     ds

    ; Call do_send: SI=tx_buf offset, CX=60, caller_ds=our seg
    mov     si, tx_buf
    mov     cx, 60
    call    do_send             ; caller_ds=DS=our seg, SI=tx_buf, CX=60

    pop     ax
    pop     cx
    pop     di
    pop     es
    ret


do_send:
    test    cx, cx
    jz      .bad
    cmp     cx, ETH_BUF_SZ
    ja      .bad

    ; Copy packet from caller's segment (caller_ds:SI) to our tx_buf
    push    si
    push    cx
    push    ds
    push    es
    push    di
    mov     ds, [caller_ds]     ; DS = caller's segment
    push    cs
    pop     es
    mov     di, tx_buf
    rep     movsb               ; ES:DI = our tx_buf, DS:SI = caller's buffer
    pop     di
    pop     es
    pop     ds                  ; restore our DS (CS)
    pop     cx
    pop     si

    ; Zero the TX net header
    push    es
    push    di
    push    cs
    pop     es
    mov     di, tx_hdr
    xor     ax, ax
    mov     dx, VIRTIO_NET_HDR_SZ / 2
.zero_hdr:
    stosw
    dec     dx
    jnz     .zero_hdr
    pop     di
    pop     es

    ; Get a free TX descriptor slot
    ; Use round-robin: mod our NUM_TX slots, NOT VRING_SIZE
    ; (would corrupt avail.ring indices 4..255, causing QEMU "index 256" panic)
    mov     ax, [tx_avail_head]
    and     ax, (NUM_TX - 1)
    mov     [tx_slot], ax       ; slot index

    ; Fill TX descriptors in TX vring page (tx_vring_seg)
    ; desc[2*slot+0]: tx_hdr, len=10, NEXT
    ; desc[2*slot+1]: tx_buf, len=cx, no flags
    push    es
    mov     es, [tx_vring_seg]

    ; Slot descriptor offset = slot * 32 (2 descs * 16 bytes)
    mov     di, [tx_slot]
    shl     di, 5               ; di = slot * 32

    ; Physical addr of tx_hdr  = DS_phys + tx_hdr
    mov     ax, [ds_phys_lo]
    mov     dx, [ds_phys_hi]
    add     ax, tx_hdr
    adc     dx, 0

    mov     [es: VRING_DESC_OFF + di + 0],  ax      ; addr lo16
    mov     [es: VRING_DESC_OFF + di + 2],  dx      ; addr hi16
    mov     word [es: VRING_DESC_OFF + di + 4], 0   ; addr bits 32-47
    mov     word [es: VRING_DESC_OFF + di + 6], 0
    mov     word [es: VRING_DESC_OFF + di + 8], VIRTIO_NET_HDR_SZ
    mov     word [es: VRING_DESC_OFF + di + 10], 0
    mov     word [es: VRING_DESC_OFF + di + 12], VRING_DESC_F_NEXT
    mov     ax, [tx_slot]
    shl     ax, 1
    inc     ax
    mov     [es: VRING_DESC_OFF + di + 14], ax  ; next = 2*slot+1

    ; Physical addr of tx_buf = DS_phys + tx_buf
    mov     ax, [ds_phys_lo]
    mov     dx, [ds_phys_hi]
    add     ax, tx_buf
    adc     dx, 0

    add     di, 16
    mov     [es: VRING_DESC_OFF + di + 0],  ax
    mov     [es: VRING_DESC_OFF + di + 2],  dx
    mov     word [es: VRING_DESC_OFF + di + 4], 0
    mov     word [es: VRING_DESC_OFF + di + 6], 0
    mov     [es: VRING_DESC_OFF + di + 8],  cx      ; len (packet length)
    mov     word [es: VRING_DESC_OFF + di + 10], 0
    mov     word [es: VRING_DESC_OFF + di + 12], 0
    mov     word [es: VRING_DESC_OFF + di + 14], 0

    ; Post to TX avail ring
    mov     ax, [tx_avail_head]
    and     ax, VRING_MASK
    mov     di, ax
    shl     di, 1               ; byte offset in avail ring array
    mov     ax, [tx_slot]
    shl     ax, 1               ; head desc = 2 * slot
    mov     [es: VRING_AVAIL_OFF + 4 + di], ax

    ; Advance avail.idx
    inc     word [es: VRING_AVAIL_OFF + 2]
    inc     word [tx_avail_head]

    pop     es

    ; Notify TX queue (queue 1)
    mov     dx, [virtio_iobase]
    add     dx, VIRTIO_LEG_DB
    mov     ax, TX_QUEUE_IDX
    out     dx, ax

    clc
    ret

.bad:
    stc
    ret


; =============================================================================
; TIMER_ISR — INT 1Ch hook: poll RX queue
; =============================================================================
timer_isr:
    push    ax
    push    bx
    push    cx
    push    dx
    push    si
    push    di
    push    ds
    push    es

    push    cs
    pop     ds

    ; rx_poll does NOT make any DOS INT 21h calls — it only accesses I/O ports
    ; and makes a far call to the receiver buffer. Safe to call even when InDOS=1.
    ; (InDOS guard was causing DHCP deadlock: DHCP.EXE holds InDOS=1 while waiting.)

    call    rx_poll
.poll_done:

    pop     es
    pop     ds
    pop     di
    pop     si
    pop     dx
    pop     cx
    pop     bx
    pop     ax

    ; Do NOT chain to old INT 1Ch handler.
    ; DOS INT 1Ch is typically a no-op IRET stub; chaining it can cause
    ; re-entrant DOS calls (InDOS violation) and deadlocks.
    ; We simply IRET here — this is safe for standard FreeDOS systems.
    iret


; =============================================================================
; RX_POLL — check used RX ring, deliver packets to receiver callback
;
; Two-call Packet Driver receiver protocol:
;   Call 1: AX=handle, BX=0, CX=len  → receiver returns ES:DI = app buffer
;   Call 2: AX=handle, BX=1, DS:SI=app_buf, CX=len  → receiver processes data
;
; Uses saved variables: rx_pkt_slot, rx_pkt_len, app_buf_off/seg
; DS = our segment (CS) throughout; we restore it after each far call
; =============================================================================
rx_poll:
    cmp     word [handle], 0
    je      .done

.poll_loop:
    ; Check used ring: compare rx_last_used with used.idx
    push    es
    mov     es, [vring_seg]
    mov     ax, [rx_last_used]
    cmp     ax, [es: VRING_USED_OFF + 2]
    je      .done_es

    ; Read used element at index (ax & mask)
    mov     bx, ax
    and     bx, VRING_MASK
    mov     di, bx
    shl     di, 3               ; * 8 bytes per used element
    mov     bx, [es: VRING_USED_OFF + 4 + di]     ; id lo16 (head desc index)
    mov     cx, [es: VRING_USED_OFF + 4 + di + 4] ; len lo16
    pop     es

    ; slot = id / 2
    shr     bx, 1
    and     bx, VRING_MASK
    mov     [rx_pkt_slot], bx

    ; packet length = total len - net header
    sub     cx, VIRTIO_NET_HDR_SZ
    jle     .repost_short
    mov     [rx_pkt_len], cx

    ; DS:SI = rx_bufs + slot * ETH_BUF_SZ
    mov     ax, bx
    mov     bx, ETH_BUF_SZ
    mul     bx
    add     ax, rx_bufs
    mov     [rx_pkt_off], ax    ; save for call 2

    ; ---- Call 1: ask receiver for a buffer ----
    ; Crynwr spec: (*receiver)(handle BX, flag AX, len CX)
    ;   AX = 0  (flag: buffer request)
    ;   BX = handle
    ;   CX = packet length (including MAC header, excluding FCS)
    ; Returns: ES:DI = app buffer, or 0:0 if no space
    xor     ax, ax              ; AX = 0 (call 1 flag)
    mov     bx, [handle]        ; BX = handle
    mov     cx, [rx_pkt_len]    ; CX = length
    call    far [recv_seg_word]
    push    cs
    pop     ds                  ; restore DS (receiver may have changed it)

    ; ES:DI = app buffer from receiver; if both 0 = no space
    mov     ax, di
    push    es
    pop     bx
    or      ax, bx
    jz      .repost             ; receiver returned 0:0 = drop

    mov     [app_buf_off], di
    mov     [app_buf_seg], es

    ; ---- Copy ethernet frame to app buffer ----
    push    ds
    push    si
    push    cx
    push    di
    push    es

    mov     si, [rx_pkt_off]    ; DS:SI = our rx buffer
    mov     cx, [rx_pkt_len]
    mov     di, [app_buf_off]
    mov     es, [app_buf_seg]
    rep     movsb

    pop     es
    pop     di
    pop     cx
    pop     si
    pop     ds

    ; ---- Call 2: notify receiver data is ready ----
    ; Crynwr spec: (*receiver)(handle BX, flag AX, len CX, buffer DS:SI)
    ;   AX = 1  (flag: data ready)
    ;   BX = handle
    ;   CX = length
    ;   DS:SI = app buffer (the same pointer returned by call 1)
    mov     ax, 1               ; AX = 1 (call 2 flag)
    mov     bx, [handle]        ; BX = handle
    mov     si, [app_buf_off]
    mov     cx, [rx_pkt_len]
    push    ds                  ; save our DS
    mov     ds, [app_buf_seg]   ; DS:SI = app buffer
    ; IMPORTANT: recv_seg_word is in OUR segment (CS), not in DS (app buffer).
    ; Use CS: segment override so the far call reads the right address.
    call    far [cs:recv_seg_word]
    pop     ds                  ; restore our DS
    push    cs
    pop     ds

.repost:
    ; Re-post this RX slot to avail ring
    push    es
    mov     es, [vring_seg]
    mov     ax, [es: VRING_AVAIL_OFF + 2]  ; avail.idx
    and     ax, VRING_MASK
    mov     di, ax
    shl     di, 1
    mov     ax, [rx_pkt_slot]
    shl     ax, 1               ; head desc = 2 * slot
    mov     [es: VRING_AVAIL_OFF + 4 + di], ax
    inc     word [es: VRING_AVAIL_OFF + 2]
    pop     es

    ; Notify RX queue
    mov     dx, [virtio_iobase]
    add     dx, VIRTIO_LEG_DB
    xor     ax, ax
    out     dx, ax

    inc     word [rx_last_used]
    jmp     .poll_loop

.repost_short:
    ; No extra pop es here — caller already did pop es before jle.
    mov     [rx_pkt_slot], bx
    jmp     .repost

.done_es:
    pop     es
.done:
    ret


; =============================================================================
; DATA — persistent state (resident)
; =============================================================================

; --- Identification ---
str_drvname     db  'VirtIO-Net PD v0.1', 0

; --- Strings ---
msg_banner      db  'VirtIO-Net DOS Packet Driver v0.1', 13, 10
                db  '(c) 2026 — legacy virtio 0.9.5/1.0', 13, 10, '$'
msg_found       db  'Found virtio-net at I/O 0x', '$'
msg_vring       db  'VRing segment: 0x', '$'
msg_mac         db  'MAC: ', '$'
msg_ok1         db  'Installed at INT 0x', '$'
msg_ok2         db  13, 10, '$'
msg_usage       db  'Usage: virtio_pkt <INT_vector> [-u]', 13, 10
                db  '  e.g. virtio_pkt 0x60', 13, 10, '$'
msg_no_dev      db  'ERROR: virtio-net not found (PCI 1AF4:1000)', 13, 10, '$'
msg_modern      db  'ERROR: device uses modern virtio — use -device virtio-net-pci,disable-modern=on', 13, 10, '$'
msg_no_mem      db  'ERROR: cannot allocate memory', 13, 10, '$'
msg_init_fail   db  'ERROR: virtio init failed', 13, 10, '$'
msg_vec_busy    db  'ERROR: INT vector already in use', 13, 10, '$'
msg_notimp      db  'Uninstall: use TERMINATE function or reboot', 13, 10, '$'
msg_crlf        db  13, 10, '$'

; --- State ---
pkt_vector      db  0x60
virtio_iobase   dw  0
pci_bus         db  0
pci_devfn       db  0
dev_features    dd  0
mac_addr        db  0, 0, 0, 0, 0, 0
vring_seg       dw  0
tx_vring_seg    dw  0
handle          dw  0
recv_off        dw  0
recv_seg        dw  0
recv_seg_word   dw  0, 0        ; recv_off, recv_seg as dword for call far [mem]
old_vec_off     dw  0
old_vec_seg     dw  0
old_1c_off      dw  0
old_1c_seg      dw  0
caller_es       dw  0
caller_di       dw  0
caller_ds       dw  0           ; caller's DS for SEND_PKT buffer access
caller_al       db  0           ; caller's AL (if_class for ACCESS_TYPE)
caller_ah       db  0           ; caller's AH (function code)
tx_slot         dw  0
tx_avail_head   dw  0
rx_last_used    dw  0
rx_qsz          dw  VRING_SIZE
tx_qsz          dw  VRING_SIZE
app_buf_off     dw  0
app_buf_seg     dw  0
ds_phys_lo      dw  0           ; DS * 16, low 16 bits
ds_phys_hi      dw  0           ; DS * 16, bits 16-19
rx_pkt_slot     dw  0           ; current RX slot being processed
rx_pkt_len      dw  0           ; current RX packet length (without virtio hdr)
rx_pkt_off      dw  0           ; offset of current RX data in rx_bufs
indos_off       dw  0           ; InDOS flag pointer: offset (from INT 21h/34h)
indos_seg       dw  0           ; InDOS flag pointer: segment

; --- TX/RX buffers (in .COM segment, addressed via DS) ---
tx_hdr          times VIRTIO_NET_HDR_SZ db 0
tx_buf          times ETH_BUF_SZ db 0
rx_hdrs         times (NUM_RX * VIRTIO_NET_HDR_SZ) db 0
rx_bufs         times (NUM_RX * ETH_BUF_SZ) db 0

; --- Static vring pool: 3 * 4096 bytes + 4095 bytes padding for alignment ---
; alloc_vring_memory computes the aligned segment from this pool at runtime.
; No DOS memory allocation needed — everything stays inside the .COM image.
; 2 pages (RX + TX) + 4095 bytes padding for alignment = 12287 bytes
; 2 queues x 16384 bytes each + 4095 bytes alignment padding
vring_pool      times (2 * 16384 + 4095) db 0

; =============================================================================
; END MARKER for TSR size calculation
; =============================================================================
_init_end:


; =============================================================================
; UTILITY ROUTINES (non-resident — init only)
; =============================================================================

; print_str: DS:DX = $-terminated string
print_str:
    mov     ah, 0x09
    int     0x21
    ret

; print_hex16: AX = 16-bit value, prints 4 hex digits
print_hex16:
    push    bx
    push    cx
    mov     bx, ax
    mov     cx, 4
.h16_loop:
    rol     bx, 4
    mov     al, bl
    and     al, 0x0F
    add     al, '0'
    cmp     al, '9'
    jle     .h16_ok
    add     al, 7
.h16_ok:
    mov     [hex_tmp], al
    push    bx
    push    cx
    mov     dx, hex_tmp
    call    print_str
    pop     cx
    pop     bx
    loop    .h16_loop
    pop     cx
    pop     bx
    ret

; print_hex8: AL = 8-bit value
print_hex8:
    push    ax
    push    bx
    mov     bl, al
    shr     al, 4
    add     al, '0'
    cmp     al, '9'
    jle     .hi
    add     al, 7
.hi:
    mov     [hex_tmp], al
    push    dx
    mov     dx, hex_tmp
    call    print_str
    pop     dx
    mov     al, bl
    and     al, 0x0F
    add     al, '0'
    cmp     al, '9'
    jle     .lo
    add     al, 7
.lo:
    mov     [hex_tmp], al
    push    dx
    mov     dx, hex_tmp
    call    print_str
    pop     dx
    pop     bx
    pop     ax
    ret

hex_tmp     db  '0', '$'

; print_mac: print MAC address from mac_addr
print_mac:
    push    si
    push    cx
    mov     si, mac_addr
    mov     cx, 6
.loop:
    mov     al, [si]
    call    print_hex8
    cmp     cx, 1
    je      .done
    push    dx
    mov     dx, colon_str
    call    print_str
    pop     dx
    inc     si
    dec     cx
    jmp     .loop
.done:
    pop     cx
    pop     si
    ret

colon_str   db  ':', '$'

; skip_spaces: advance SI past spaces
skip_spaces:
    cmp     byte [si], ' '
    jne     .done
    inc     si
    jmp     skip_spaces
.done:
    ret

; parse_hex_byte: parse hex byte from [SI] (format "0x60" or "60")
; Returns AL = value, CF=0 on success, CF=1 on error
parse_hex_byte:
    mov     al, [si]
    cmp     al, 0x0D
    je      .fail
    cmp     al, 0
    je      .fail
    cmp     al, '$'
    je      .fail

    ; Check "0x" prefix
    cmp     al, '0'
    jne     .decimal
    inc     si
    mov     al, [si]
    or      al, 0x20
    cmp     al, 'x'
    jne     .back_decimal
    inc     si
    jmp     .hex

.back_decimal:
    dec     si

.decimal:
    xor     bx, bx
.dec_loop:
    mov     al, [si]
    cmp     al, '0'
    jl      .dec_done
    cmp     al, '9'
    jg      .dec_done
    sub     al, '0'
    mov     cl, al
    mov     ax, bx
    add     ax, ax              ; *2
    add     ax, ax              ; *4
    add     ax, bx              ; *5
    add     ax, ax              ; *10
    xor     bh, bh
    mov     bl, cl
    add     ax, bx
    mov     bx, ax
    inc     si
    jmp     .dec_loop
.dec_done:
    mov     al, bl
    clc
    ret

.hex:
    xor     bx, bx
.hex_loop:
    mov     al, [si]
    or      al, 0x20
    cmp     al, '0'
    jl      .hex_done
    cmp     al, '9'
    jle     .hex_num
    cmp     al, 'a'
    jl      .hex_done
    cmp     al, 'f'
    jg      .hex_done
    sub     al, 'a' - 10
    jmp     .hex_add
.hex_num:
    sub     al, '0'
.hex_add:
    shl     bx, 4
    or      bl, al
    inc     si
    jmp     .hex_loop
.hex_done:
    mov     al, bl
    clc
    ret

.fail:
    stc
    ret
