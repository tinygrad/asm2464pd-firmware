# Immutable boot ABI

Boot ABI version 1 fixes the following contract:

- Bootstub CODE is `0x0000-0x2FFF`; application CODE is `0x3000-0xFFFF`.
- Reset is the only CPU vector owned by the bootstub; every interrupt vector
  trampolines to the corresponding application vector at `0x3000`.
- Application flash begins at `0x4000`; its body is at most `0xD000` bytes.
- XRAM `0x5FF8-0x5FFB` is the DFU cookie and `0x5FFC-0x5FFF` is boot tracking.
- Application startup clears IDATA, sets linker-derived `SP`, clears `DPX` and
  `IE`, and jumps to `main`. It does not run SDCC GSINIT/XINIT, so writable
  initializers are forbidden.
- The bootstub restores MEMSEL before entry. All other peripheral state is
  unspecified and must be initialized by the application.
- A healthy application clears boot tracking. Recovery writes
  `DFU_COOKIE_MAGIC`, completes the host request, and triggers `REG_CPU_RESET`.

The build-time layout checker enforces CODE bounds, vectors, reserved XRAM,
stack margin, and absence of unsupported initialization sections.

## Application image

| Offset | Size | Field |
| --- | ---: | --- |
| `0x00` | 4 | `A24F` |
| `0x04` | 23 | NUL-padded git version |
| `0x1B` | 1 | Required boot ABI |
| `0x1C` | 4 | Body length, little-endian |
| `0x20` | 4 | CRC-32/IEEE of bytes `0x00-0x1F` and the body |
| `0x24` | 28 | Reserved zero |
| `0x40` | body length | CODE image loaded at `0x3000` |

## DFU protocol v1

The bootstub enumerates as `ADD1:B007`. All commands are vendor EP0 requests:

| Request | Direction/length | Operation |
| --- | --- | --- |
| `B0` | OUT/8 | Erase: `u32 address, u32 length`, sector-aligned |
| `B1` | OUT/0 | Set 24-bit cursor from `wIndex.low:wValue` |
| `B2`, `B3` | OUT/1-max, IN/1-max | Write/read and advance |
| `B4` | IN/20 | `A24D`, protocol, ABI, flags, max write, sector and bounds |
| `B5` | IN/4 | Protocol, status, last request, pending operation |
| `B6`, `EB` | OUT/0 | Abort/reset |

Status 0 is success; 1-8 are request, range, USB DMA, unlock, erase, program,
read, and aborted errors. Invalid requests stall EP0. New SETUP or bus reset
cancels pending work; bus reset also clears address, configuration, and cursor.
Erase is idempotent. A write retry must set its address again and replay the
identical bytes.
