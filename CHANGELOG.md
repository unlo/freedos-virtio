# Changelog

All notable changes to the freedos-virtio project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project uses a pragmatic versioning scheme matching driver iterations.

---

## [v1.0.1] — 2026-05-25

### Fixed
- **TX slot mask** (`virtio_pkt.asm:do_send`) — changed `and ax, VRING_MASK`
  to `and ax, (NUM_TX - 1)`. QEMU no longer crashes with
  *"Guest says index 256 is available"* after the TX queue wraps.
- **Stack balance** (`virtio_pkt.asm:.repost_short`) — removed stray `pop es`
  that broke the stack when handling short RX packets (`jle` path).
- **Directory listing** — `MTCP.CFG` now includes `DirectoryIndex INDEX.HTM`,
  and `FDAUTO.BAT` passes `-dir_indexes` to HTTPServ.EXE so `GET /` works.
- **Rebuild disk image** — `fdos_hdd.qcow2` rebuilt from fixed sources.

### Changed
- `virtio_pkt.com` rebuilt from fixed source (47295 bytes vs previous 47296).

---

## [v1.0.0] — 2026-05-25

### Added
- Initial release: VirtIO-Net packet driver (v13k-style) for DOS.
- QEMU FreeDOS image with built-in web server:
  - Bootable `fdos_hdd.qcow2` (256 MB, virtio HDD, virtio-net-pci).
  - `HTTPserv.EXE` on port 80 serving `INDEX.HTM`.
  - MTCP networking stack configured.
- Source: `virtio_pkt.asm` (NASM `bin` format → `.COM` TSR).
