# Wire-level capture plan: handmade USB4 control-transport bring-up failure

## The precisely-localized wall (what every software method converged on)
Handmade fw trains the USB4 link to `CL0/CL0` **byte-true to stock** (`SB[0xA0]=SB[0xA1]=0x02`,
`E710=0x04`, every firmware register identical at the bond), and the host's TB4 controller sees the
port reach **UP**. But the host's first in-band config read then **times out**:

```
host: TB_CFG_PKG_READ route=1, config=0x2 (TB_CFG_SWITCH), port=0, offset=0   (= ROUTER_CS_0, vid/did)
stock device:    answers in ~140us  (tb_rx = 0x2463174c)  -> tb_switch_alloc -> ... -> GPU 1002:7590
handmade device: SILENT, 4 retries over ~414ms, zero tb_rx -> -110 -> no switch -> no GPU
```

Device-side at that moment (handmade, every run): `1201=00 1203=00 1407=00 EC06=00 C80A=20`.
The width/control-transport event `P1[0x1407].0` never fires, so the in-band **control transport
never goes "config-capable"**, so the chip's transport-RX never delivers the route=1 TLP to the
(boot-armed, firmware-arm-able via `e56f`) router-op engine, so `EC06.0` never sets.

## Why this is below firmware (proven 3 independent ways)
1. **Pure-emulation execution-path differential** (`emulate/diff_trace.py`): drove both firmwares
   against an identical mock; found + fixed 3 real omitted-path bugs (boot `c35b`, steady `d7cd`,
   responder `c0a5` stub), after which the firmware execution-path is byte-and-path-true to stock —
   and route=1 still fails.
2. **Full INT1 dispatch RE**: `EC06` (= `REG_NVME_EVENT_STATUS`) is read-only, HW-set; no firmware
   writer/enable in the path. `c0a5` only runs *after* the HW delivers the TLP.
3. **Engine-arming test**: `e56f` arms the router-op RX engine (`EC00.0`, speed `EA88/89`, DMA ctrl);
   forced on, `EC06` still stays 0 — engine armed, **no TLP ever arrives**.

Conclusion: the divergence is the chip's USB4 **link-layer / transport-layer bring-up after CL0**
(the step that makes the control adapter config-capable and raises `P1[0x1407].0`). It is identical
in every firmware-observable register, so it is either (a) a wire-level/analog PHY difference, or
(b) a firmware *timing* difference (compiled-C vs hand-asm response cadence) that the emulation
cannot model. Only a wire capture distinguishes these.

## Capture goal
Find where, between `CL0/CL0` and the host's route=1 read, stock's link establishes the in-band
control transport and handmade's does not — i.e. the divergent LMP / transport-init exchange, or the
timing window handmade misses.

## What to probe (in priority order)
1. **USB4 sideband (SB / LSB low-speed channel)** — HIGHEST VALUE, lowest-speed, probeable with a
   good LA or a TB/USB4 protocol analyzer. This carries the lane-init + the link/transport
   management exchange the firmware drives (the `af38`/connect handshake the fw traces as
   `[SB Con]/[ConnRout]`). Capture both directions device<->host.
2. **The high-speed USB4 lanes (TX/RX, 2 lanes, 10-20 Gbps per lane)** — requires a **Thunderbolt/USB4
   protocol analyzer** (e.g. Teledyne LeCroy T4/USB4 exerciser-analyzer), NOT a generic LA. This is
   where the LMPs (Link Management Packets) and the Transport-Layer init / the control-adapter
   bring-up live. This is the definitive capture if available.
3. **PERST / link control sidebands** (`B402`/`B480` device-side correlate) — generic LA, cheap,
   confirms reset/PERST timing isn't re-asserting.

## Trigger
Trigger on the **`CL0/CL0` bond** (both lanes reach CL0). Device-side correlate for cross-timing:
flash a build with a one-shot GPIO/UART marker at the `A0=02 && A1=02` point (the `ring_log.h`
infra + a spare GPIO). Capture the window from that trigger through the host's 4 route=1 retries
(~414ms).

## What to compare (stock vs handmade) — the differential on the wire
Run the SAME capture twice: once with **stock `fw_tinygrad.bin`** (GPU enumerates) and once with
**handmade** (current build). Diff the post-`CL0` exchange:
- Does stock exchange **LMPs / a transport-layer init** that handmade does not (or handmade sends a
  malformed / mistimed one)?
- Does the **control adapter (adapter 0)** get brought to a config-capable state in stock (a specific
  ordered set / LMP) that's missing/different in handmade?
- **Timing**: does the host expect a device response within a window after some event, and does
  handmade respond *too late* (the compiled-C-vs-asm cadence hypothesis)? Measure the device's
  response latency to each host packet, stock vs handmade.
- At the route=1 read itself: does the host's TLP physically reach the device on the wire in BOTH
  cases (confirming the host sends it identically), and does the device's transport-RX drop it
  (no link-layer ACK) for handmade?

## Success oracle (host-side, already wired)
`kretprobe tb_cfg_get_upstream_port`: returns **-110** on handmade, **0** on stock. The wire capture
explains *why* the -110; any firmware change that flips it to 0 => route=1 answered => GPU.

## If the capture shows a firmware-reachable cause
- A **mistimed/missing device response** (timing hypothesis): identify the timing-critical
  link-event handler and hand-optimize that specific path (asm/cycle-tighten) to match stock's
  response window. This is the only firmware lever the emulation could not test.
- A **malformed LMP / transport bytes**: that's a firmware composer bug -> fix byte-true (the
  differential can then verify convergence once the missing event is modeled).
- If the capture shows handmade's PHY/link does **not** even initiate the post-CL0 transport
  exchange that stock does, with identical firmware register state: that is a genuine
  analog/silicon difference and the firmware truly cannot cross it.

## Reusable assets from this investigation
- `emulate/diff_trace.py` (+`emulate/ghidra_funcs.json`): pure-emulation execution-path differential
  (modes: base / `--steady` / `--inject-route1`). Re-run after any fix to confirm firmware-path
  convergence.
- `app/patch_stock_*.py`: ~30 non-intrusive stock code-cave tracers (SB regs, width seq, the
  route=1 window, the PwrOn->Deassert window, etc.).
- Host: `~/flash_cap.sh`, the `tb_cfg_get_upstream_port` kretprobe oracle, the `tb_tx`/`tb_rx`
  tracepoints.
- `project_inband_routercs_read_wall.md` (memory): the full dated investigation log.
