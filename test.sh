#!/bin/bash
# test.sh [binary] [capture_seconds] [target]
#
# Ship a wrapped firmware binary to the NUC HITL rigs, flash it, reset each card,
# print dmesg from reset release, and run a tinygrad smoke test.
#
#   binary           - wrapped firmware .bin (default: handmade/build/firmware_wrapped.bin)
#   capture_seconds  - UART capture length after reset (default: 1)
#   target           - both|all|usb3|usb4 (default: both)
#
# Default target map on nuc:
#   usb3 -> FTDI DK0CG1XJ
#   usb4 -> FTDI DK0CGSKL
#
# USB bus is auto-detected after reset. USB3_BUS/USB4_BUS can be set as hints
# if cabling changes or both cards enumerate at the same time.
# Useful knobs: SMOKE_TIMEOUT=45s FLASH_TIMEOUT=45s RUN_TIMEOUT=180s VERBOSE=1
# USB4_ENUM_TRIES=16 shortens the optional ADD1 wait on native USB4 tests.
# USB4_ROUTER_WAIT_S=25 bounds how long native USB4 sideband waits for debugfs.
# STOCK_FW=1 skips custom ADD1/sideband/tinygrad checks and times AMD PCI enum.
# STOCK_USBDEV=ADD1:0001 is the USB3 stock runtime enumeration marker for fw_tinygrad.bin.
# USB4 native 0xC0 sideband status should report the local monitor rail
# around 2.9V. A ~20V reading usually means the USB3 board was read instead.
set -u

NUC="${NUC:-nuc}"
BIN="${1:-handmade/build/firmware_wrapped.bin}"
SECS="${2:-${CAPTURE_SECONDS:-1}}"
TARGET="${3:-both}"

USB3_FTDI="${USB3_FTDI:-ftdi://ftdi:ft-x:DK0CG1XJ/1}"
USB4_FTDI="${USB4_FTDI:-ftdi://ftdi:ft-x:DK0CGSKL/1}"
USB3_BUS="${USB3_BUS:-auto}"
USB4_BUS="${USB4_BUS:-auto}"

SMOKE_TIMEOUT="${SMOKE_TIMEOUT:-45s}"
SIDEBAND_TIMEOUT="${SIDEBAND_TIMEOUT:-30s}"
USB4_SIDEBAND_MIN_MV="${USB4_SIDEBAND_MIN_MV:-2500}"
USB4_SIDEBAND_MAX_MV="${USB4_SIDEBAND_MAX_MV:-3500}"
FLASH_TIMEOUT="${FLASH_TIMEOUT:-45s}"
RUN_TIMEOUT="${RUN_TIMEOUT:-180s}"
VERBOSE="${VERBOSE:-0}"
STOCK_FW="${STOCK_FW:-0}"
AMD_PCI_ID="${AMD_PCI_ID:-1002:7590}"
STOCK_USBDEV="${STOCK_USBDEV:-ADD1:0001}"
PCI_ENUM_TRIES="${PCI_ENUM_TRIES:-240}"
PCI_ENUM_DELAY="${PCI_ENUM_DELAY:-0.125}"

case "$TARGET" in
  both|all) TARGETS="usb3 usb4" ;;
  usb3|USB3) TARGETS="usb3" ;;
  usb4|USB4) TARGETS="usb4" ;;
  *) echo "ERROR: target must be both, usb3, or usb4"; exit 1 ;;
esac

if [ ! -f "$BIN" ]; then echo "ERROR: binary '$BIN' not found"; exit 1; fi

SZ=$(stat -c%s "$BIN")
LMD5=$(md5sum "$BIN" | cut -d' ' -f1)
echo "[test] ship $BIN ($SZ bytes, md5 ${LMD5:0:8}) -> $NUC:~/hm_fw.bin"
scp -q "$BIN" "$NUC:hm_fw.bin" || { echo "ERROR: scp failed"; exit 1; }
RMD5=$(ssh "$NUC" 'md5sum ~/hm_fw.bin 2>/dev/null | cut -d" " -f1')
if [ "$LMD5" != "$RMD5" ]; then
  echo "ERROR: md5 mismatch (local $LMD5 != nuc $RMD5)"
  exit 1
fi

cat > /tmp/_flash_cap_multi.sh <<'REMOTE_EOF'
#!/bin/bash
set -u

cd ~/asm2464pd-firmware || exit 1

SECS="${1:-1}"
TARGETS="${2:-both}"
USB3_FTDI="${3:?missing USB3_FTDI}"
USB4_FTDI="${4:?missing USB4_FTDI}"
USB3_BUS="${5:-auto}"
USB4_BUS="${6:-auto}"
SMOKE_TIMEOUT="${7:-45s}"
FLASH_TIMEOUT="${8:-45s}"
SIDEBAND_TIMEOUT="${9:-30s}"
USB4_SIDEBAND_MIN_MV="${10:-2500}"
USB4_SIDEBAND_MAX_MV="${11:-3500}"
VERBOSE="${12:-0}"

BOOT_USBDEV="${BOOT_USBDEV:-174C:2463}"
TINY_USBDEV="${TINY_USBDEV:-ADD1:0001}"
STOCK_USBDEV="${STOCK_USBDEV:-$TINY_USBDEV}"
TG="${TG:-/home/batman/openpilot/tinygrad_repo}"
RESET_HOLD_S="${RESET_HOLD_S:-0.5}"
BOOT_HOLD_S="${BOOT_HOLD_S:-2.0}"
ENUM_TRIES="${ENUM_TRIES:-80}"
ENUM_DELAY="${ENUM_DELAY:-0.125}"
USB4_ENUM_TRIES="${USB4_ENUM_TRIES:-16}"
DMESG_LINES="${DMESG_LINES:-160}"
USB4_ROUTER_WAIT_S="${USB4_ROUTER_WAIT_S:-25}"
STOCK_FW="${STOCK_FW:-0}"
AMD_PCI_ID="${AMD_PCI_ID:-1002:7590}"
STOCK_USBDEV="${STOCK_USBDEV:-$TINY_USBDEV}"
PCI_ENUM_TRIES="${PCI_ENUM_TRIES:-240}"
PCI_ENUM_DELAY="${PCI_ENUM_DELAY:-0.125}"

case "$TARGETS" in
  both|all) TARGET_LIST="usb3 usb4" ;;
  usb3|USB3) TARGET_LIST="usb3" ;;
  usb4|USB4) TARGET_LIST="usb4" ;;
  *) echo "ERROR: remote target must be both, usb3, or usb4"; exit 1 ;;
esac

ftdi_for_target() {
  case "$1" in
    usb3) echo "$USB3_FTDI" ;;
    usb4) echo "$USB4_FTDI" ;;
    *) return 1 ;;
  esac
}

other_target() {
  case "$1" in
    usb3) echo "usb4" ;;
    usb4) echo "usb3" ;;
    *) return 1 ;;
  esac
}

bus_for_target() {
  case "$1" in
    usb3) echo "$USB3_BUS" ;;
    usb4) echo "$USB4_BUS" ;;
    *) return 1 ;;
  esac
}

uppercase() {
  printf '%s' "$1" | tr '[:lower:]' '[:upper:]'
}

cleanup_ftdi_readers() {
  local url="$1"
  sudo pkill -f "python3 - ${url}" 2>/dev/null || true
  sudo pkill -f "ftdi_debug.py.*${url}" 2>/dev/null || true
}

cleanup_all_readers() {
  cleanup_ftdi_readers "$USB3_FTDI"
  cleanup_ftdi_readers "$USB4_FTDI"
}

trap cleanup_all_readers EXIT INT TERM

reset_ftdi() {
  local url="$1"
  local mode="$2"
  sudo python3 - "$url" "$mode" "$RESET_HOLD_S" "$BOOT_HOLD_S" <<'PY'
import sys, time
from ftdi_debug import USBGPUDebug

url, mode, reset_hold, boot_hold = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4])
with USBGPUDebug(url) as dbg:
  if mode == "bootloader":
    dbg.ftdi.set_cbus_gpio(dbg.CBUS_RESET | dbg.CBUS_BOOTLOADER)
    time.sleep(reset_hold)
    dbg.ftdi.set_cbus_gpio(dbg.CBUS_BOOTLOADER)
    time.sleep(boot_hold)
    dbg.ftdi.set_cbus_gpio(0)
  elif mode == "run":
    dbg.ftdi.set_cbus_gpio(dbg.CBUS_RESET)
    time.sleep(reset_hold)
    dbg.ftdi.set_cbus_gpio(0)
  elif mode == "hold":
    dbg.ftdi.set_cbus_gpio(dbg.CBUS_RESET)
  elif mode == "release":
    dbg.ftdi.set_cbus_gpio(0)
  else:
    raise SystemExit(f"unknown reset mode {mode}")
PY
}

run_and_capture() {
  local url="$1"
  local secs="$2"
  local out="$3"
  local dmesg_marker="$4"
  sudo python3 - "$url" "$secs" "$dmesg_marker" "$RESET_HOLD_S" > "$out" 2>&1 <<'PY'
import subprocess, sys, time
from ftdi_debug import USBGPUDebug

def dmesg_count():
  try:
    return subprocess.check_output(["dmesg"], text=True, stderr=subprocess.DEVNULL).count("\n")
  except Exception:
    return -1

url, secs, marker, reset_hold = sys.argv[1], float(sys.argv[2]), sys.argv[3], float(sys.argv[4])
with USBGPUDebug(url) as dbg:
  dbg.ftdi.set_cbus_gpio(dbg.CBUS_RESET)
  time.sleep(reset_hold)
  start = dmesg_count()
  with open(marker, "w", encoding="ascii") as f:
    f.write(f"dmesg_lines={start}\n")
    f.write(f"epoch_ms={time.time_ns() // 1000000}\n")
  dbg.ftdi.set_cbus_gpio(0)

  end = time.perf_counter() + secs
  while time.perf_counter() < end:
    data = dbg.read()
    if data:
      sys.stdout.write(data)
      sys.stdout.flush()
    time.sleep(0.001)
PY
}

bus3() {
  printf '%03d' "$((10#$1))"
}

wait_usbdev_any() {
  local vidpid="$1"
  local tries="${2:-$ENUM_TRIES}"
  local delay="${3:-$ENUM_DELAY}"
  local i
  for i in $(seq 1 "$tries"); do
    if lsusb -d "$vidpid" >/dev/null 2>&1; then return 0; fi
    sleep "$delay"
  done
  return 1
}

list_usbdev_paths() {
  local vidpid="$1"
  lsusb -d "$vidpid" 2>/dev/null | awk '$1 == "Bus" { dev = $4; sub(":", "", dev); print $2 "-" dev }'
}

is_bus_hint() {
  case "$1" in
    ''|auto|AUTO) return 1 ;;
    *[!0-9]*) return 1 ;;
    *) return 0 ;;
  esac
}

choose_new_usbdev_path() {
  local before="$1"
  local hint="$2"
  local current="$3"
  local new
  local hinted

  new="$(printf '%s\n' "$current" | awk 'FILENAME == ARGV[1] { seen[$0] = 1; next } $0 != "" && !($0 in seen)' "$before" -)"
  if [ -z "$new" ]; then return 1; fi

  if is_bus_hint "$hint"; then
    hinted="$(printf '%s\n' "$new" | awk -F- -v bus="$(bus3 "$hint")" '$1 == bus { print; exit }')"
    if [ -n "$hinted" ]; then
      echo "$hinted"
      return 0
    fi
  fi

  printf '%s\n' "$new" | head -1
}

wait_new_usbdev_bus() {
  local vidpid="$1"
  local before="$2"
  local hint="$3"
  local tries="${4:-$ENUM_TRIES}"
  local delay="${5:-$ENUM_DELAY}"
  local i
  local current
  local path
  local count

  for i in $(seq 1 "$tries"); do
    current="$(list_usbdev_paths "$vidpid")"
    if path="$(choose_new_usbdev_path "$before" "$hint" "$current")"; then
      echo "$((10#${path%%-*}))"
      return 0
    fi
    if is_bus_hint "$hint"; then
      path="$(printf '%s\n' "$current" | awk -F- -v bus="$(bus3 "$hint")" '$1 == bus { print; exit }')"
      if [ -n "$path" ]; then
        echo "$((10#${path%%-*}))"
        return 0
      fi
    fi
    sleep "$delay"
  done

  current="$(list_usbdev_paths "$vidpid")"
  count="$(printf '%s\n' "$current" | awk 'NF { c++ } END { print c + 0 }')"
  if [ "$count" = "1" ]; then
    path="$(printf '%s\n' "$current" | awk 'NF { print; exit }')"
    echo "$((10#${path%%-*}))"
    return 0
  fi

  return 1
}

now_ms() {
  date +%s%3N
}

reset_epoch_ms() {
  awk -F= '$1 == "epoch_ms" { print $2; exit }' "$1" 2>/dev/null
}

wait_reset_marker() {
  local marker="$1"
  local i
  for i in $(seq 1 200); do
    if [ -n "$(reset_epoch_ms "$marker")" ]; then
      return 0
    fi
    sleep 0.01
  done
  return 1
}

elapsed_since_reset_ms() {
  local marker="$1"
  local start
  start="$(reset_epoch_ms "$marker")"
  if [ -z "$start" ]; then
    echo ""
    return 1
  fi
  echo "$(( $(now_ms) - start ))"
}

wait_new_usbdev_bus_timed() {
  local vidpid="$1"
  local before="$2"
  local hint="$3"
  local tries="${4:-$ENUM_TRIES}"
  local delay="${5:-$ENUM_DELAY}"
  local marker="$6"
  local timing_out="$7"
  local i
  local current
  local path
  local count
  local bus
  local elapsed

  if ! wait_reset_marker "$marker"; then
    echo "status=missing-reset-marker" > "$timing_out"
    return 1
  fi

  for i in $(seq 1 "$tries"); do
    current="$(list_usbdev_paths "$vidpid")"
    if path="$(choose_new_usbdev_path "$before" "$hint" "$current")"; then
      bus="$((10#${path%%-*}))"
      elapsed="$(elapsed_since_reset_ms "$marker" || true)"
      echo "status=found source=new path=$path bus=$(bus3 "$bus") elapsed_ms=$elapsed poll=$i" > "$timing_out"
      echo "$bus"
      return 0
    fi
    if is_bus_hint "$hint"; then
      path="$(printf '%s\n' "$current" | awk -F- -v bus="$(bus3 "$hint")" '$1 == bus { print; exit }')"
      if [ -n "$path" ]; then
        bus="$((10#${path%%-*}))"
        elapsed="$(elapsed_since_reset_ms "$marker" || true)"
        echo "status=found source=hint path=$path bus=$(bus3 "$bus") elapsed_ms=$elapsed poll=$i" > "$timing_out"
        echo "$bus"
        return 0
      fi
    fi
    sleep "$delay"
  done

  current="$(list_usbdev_paths "$vidpid")"
  count="$(printf '%s\n' "$current" | awk 'NF { c++ } END { print c + 0 }')"
  if [ "$count" = "1" ]; then
    path="$(printf '%s\n' "$current" | awk 'NF { print; exit }')"
    bus="$((10#${path%%-*}))"
    elapsed="$(elapsed_since_reset_ms "$marker" || true)"
    echo "status=found source=single path=$path bus=$(bus3 "$bus") elapsed_ms=$elapsed poll=final" > "$timing_out"
    echo "$bus"
    return 0
  fi

  elapsed="$(elapsed_since_reset_ms "$marker" || true)"
  echo "status=timeout elapsed_ms=$elapsed tries=$tries delay_s=$delay visible=$count" > "$timing_out"
  return 1
}

list_pci_paths() {
  local pci_id="$1"
  lspci -Dnnd "$pci_id" 2>/dev/null | awk 'NF { print $1 }'
}

wait_new_pci_timed() {
  local pci_id="$1"
  local before="$2"
  local marker="$3"
  local timing_out="$4"
  local tries="${5:-$PCI_ENUM_TRIES}"
  local delay="${6:-$PCI_ENUM_DELAY}"
  local i
  local current
  local path
  local count
  local elapsed

  if ! wait_reset_marker "$marker"; then
    echo "status=missing-reset-marker pci_id=$pci_id" > "$timing_out"
    return 1
  fi

  for i in $(seq 1 "$tries"); do
    current="$(list_pci_paths "$pci_id")"
    path="$(printf '%s\n' "$current" | awk 'FILENAME == ARGV[1] { seen[$0] = 1; next } $0 != "" && !($0 in seen) { print; exit }' "$before" -)"
    if [ -n "$path" ]; then
      elapsed="$(elapsed_since_reset_ms "$marker" || true)"
      echo "status=found source=new bdf=$path pci_id=$pci_id elapsed_ms=$elapsed poll=$i" > "$timing_out"
      echo "$path"
      return 0
    fi
    sleep "$delay"
  done

  current="$(list_pci_paths "$pci_id")"
  count="$(printf '%s\n' "$current" | awk 'NF { c++ } END { print c + 0 }')"
  if [ "$count" != "0" ]; then
    path="$(printf '%s\n' "$current" | awk 'NF { print; exit }')"
    elapsed="$(elapsed_since_reset_ms "$marker" || true)"
    if grep -qxF "$path" "$before" 2>/dev/null; then
      echo "status=present source=existing bdf=$path pci_id=$pci_id elapsed_ms=$elapsed poll=final" > "$timing_out"
    else
      echo "status=found source=final bdf=$path pci_id=$pci_id elapsed_ms=$elapsed poll=final" > "$timing_out"
    fi
    echo "$path"
    return 0
  fi

  elapsed="$(elapsed_since_reset_ms "$marker" || true)"
  echo "status=timeout pci_id=$pci_id elapsed_ms=$elapsed tries=$tries delay_s=$delay visible=0" > "$timing_out"
  return 1
}

print_dmesg_since() {
  local label="$1"
  local marker="$2"
  local start
  local lines
  if [ ! -s "$marker" ]; then
    echo "[dmesg][$label] <missing reset marker>"
    return
  fi
  start="$(awk -F= 'NR == 1 { if ($1 == "dmesg_lines") print $2; else print $0; exit }' "$marker" 2>/dev/null || echo -1)"
  if [ "$start" -lt 0 ] 2>/dev/null; then
    echo "[dmesg][$label] <dmesg unavailable>"
    return
  fi
  lines="$(sudo dmesg --time-format=iso 2>/dev/null | tail -n +"$((start + 1))" | tail -n "$DMESG_LINES")"
  echo "[dmesg][$label] since reset release:"
  if [ -n "$lines" ]; then
    printf '%s\n' "$lines" | sed "s/^/[dmesg][$label] /"
  else
    echo "[dmesg][$label] <no new messages>"
  fi
}

run_tinygrad_smoke() {
  local label="$1"
  local bus="$2"
  local out="$3"
  local mode="${4:-usb}"

  if [ ! -d "$TG" ]; then
    echo "[test][$label] tinygrad smoke: FAIL (missing $TG)"
    return 1
  fi

  if [ "$mode" = "pci" ]; then
    (
      cd "$TG" || exit 1
      timeout --foreground "$SMOKE_TIMEOUT" sudo env PYTHONPATH="." DEBUG=2 DEV=PCI+AMD \
        python3 test/test_tiny.py TestTiny.test_plus
    ) > "$out" 2>&1
    return $?
  fi

  if [ -z "$bus" ]; then
    echo "[test][$label] tinygrad smoke: FAIL (no ADD1 bus for USB smoke)"
    return 1
  fi

  (
    cd "$TG" || exit 1
    timeout --foreground "$SMOKE_TIMEOUT" env PYTHONPATH="." DEBUG=2 DEV=USB+AMD TINYGRAD_USB_BUS="$bus" python3 - <<'PY'
import os, runpy, sys
from tinygrad.runtime.support.usb import USB3

target_bus = str(int(os.environ["TINYGRAD_USB_BUS"]))
orig_list_devices = USB3.list_devices

def list_devices_on_bus(cls, vendor, dev):
  ret = []
  for item in orig_list_devices(vendor, dev):
    _, path = item
    bus = path.split(":", 1)[1].split("-", 1)[0]
    if bus == target_bus:
      ret.append(item)
  return ret

USB3.list_devices = classmethod(list_devices_on_bus)
sys.argv = ["test/test_tiny.py", "TestTiny.test_plus"]
runpy.run_path("test/test_tiny.py", run_name="__main__")
PY
  ) > "$out" 2>&1
}

run_usb_control_sideband_read() {
  local label="$1"
  local bus="$2"
  local out="$3"

  if [ -z "$bus" ]; then
    echo "[test][$label] USB 0xC0: FAIL (no ADD1 bus for USB fallback)"
    return 1
  fi

  if [ ! -d "$TG" ]; then
    echo "[test][$label] USB 0xC0: FAIL (missing $TG)"
    return 1
  fi

  (
    cd "$TG" || exit 1
    timeout --foreground "$SIDEBAND_TIMEOUT" env PYTHONPATH="." TINYGRAD_USB_BUS="$bus" \
      USB4_SIDEBAND_MIN_MV="$USB4_SIDEBAND_MIN_MV" USB4_SIDEBAND_MAX_MV="$USB4_SIDEBAND_MAX_MV" python3 - <<'PY'
import ctypes, os, struct
from tinygrad.runtime.autogen import libusb
from tinygrad.runtime.support.usb import USB3

target_bus = str(int(os.environ["TINYGRAD_USB_BUS"]))
min_mv = int(os.environ["USB4_SIDEBAND_MIN_MV"])
max_mv = int(os.environ["USB4_SIDEBAND_MAX_MV"])
matches = []
for dev, path in USB3.list_devices(0xADD1, 0x0001):
  bus = path.split(":", 1)[1].split("-", 1)[0]
  if bus == target_bus:
    matches.append((dev, path))

if not matches:
  raise SystemExit(f"no ADD1:0001 device found on bus {target_bus}")

usb = USB3(matches[0][0], 0x81, 0x83, 0x02, 0x04, use_bot=True)
buf = (ctypes.c_ubyte * 4)()
ret = libusb.libusb_control_transfer(usb.handle, 0xC0, 0xC0, 0, 0, buf, 4, 1000)
if ret != 4:
  raise SystemExit(f"0xC0 sideband read returned {ret}, expected 4")

voltage_mv, current_ma = struct.unpack("<Hh", bytes(buf))
if not (min_mv <= voltage_mv <= max_mv):
  raise SystemExit(f"USB 0xC0 voltage {voltage_mv}mV outside expected USB4 range {min_mv}..{max_mv}mV")
print(f"sideband_0xC0 source=usb-control bus={target_bus} voltage_mv={voltage_mv} current_ma={current_ma}")
PY
  ) > "$out" 2>&1
}

run_usb4_router_sideband_read() {
  local label="$1"
  local out="$2"
  local wait_s="${3:-0}"
  local marker="${4:-}"
  local reset_ms=""

  if [ -n "$marker" ]; then
    reset_ms="$(reset_epoch_ms "$marker" || true)"
  fi

  timeout --foreground "$SIDEBAND_TIMEOUT" sudo env \
    USB4_SIDEBAND_MIN_MV="$USB4_SIDEBAND_MIN_MV" USB4_SIDEBAND_MAX_MV="$USB4_SIDEBAND_MAX_MV" \
    USB4_ROUTER_WAIT_S="$wait_s" RESET_EPOCH_MS="$reset_ms" python3 - <<'PY' > "$out" 2>&1
import glob, os, subprocess, sys, time

MIN_MV = int(os.environ["USB4_SIDEBAND_MIN_MV"])
MAX_MV = int(os.environ["USB4_SIDEBAND_MAX_MV"])
ROUTER_WAIT_S = float(os.environ.get("USB4_ROUTER_WAIT_S", "0"))
RESET_EPOCH_MS = int(os.environ.get("RESET_EPOCH_MS", "0") or "0")
OP_OV = 0x80000000
OP_UNSUPPORTED = 0x40000000
OP_TINY_HW_STATUS = 0xC0
SUDO = [] if os.geteuid() == 0 else ["sudo"]

def elapsed_ms():
  if not RESET_EPOCH_MS:
    return -1
  return int(time.time() * 1000) - RESET_EPOCH_MS

def thunderbolt_type(name):
  try:
    with open(f"/sys/bus/thunderbolt/devices/{name}/uevent", "r", encoding="ascii") as f:
      return f.read()
  except OSError:
    return ""

def router_candidates():
  candidates = []
  for path in glob.glob("/sys/kernel/debug/thunderbolt/*/regs"):
    name = os.path.basename(os.path.dirname(path))
    if ":" in name:
      continue
    uevent = thunderbolt_type(name)
    if "USB4_TYPE=device" in uevent:
      score = 0
    elif not name.endswith("-0"):
      score = 1
    else:
      score = 2
    candidates.append((score, name, path))
  candidates.sort()
  return candidates

def wait_device_router_candidates():
  deadline = time.monotonic() + ROUTER_WAIT_S
  while True:
    # Host routers report unsupported for the firmware 0xC0 op; only issue it
    # to the device router once the USB4 connection has enumerated.
    candidates = [c for c in router_candidates() if c[0] < 2]
    if candidates:
      return candidates, elapsed_ms()
    if time.monotonic() >= deadline:
      return [], elapsed_ms()
    time.sleep(0.25)

def write_reg(path, offset, value):
  data = f"0x{offset:04x} 0x{value:08x}\n"
  subprocess.run(SUDO + ["tee", path], input=data, text=True, stdout=subprocess.DEVNULL,
                 stderr=subprocess.PIPE, check=True)

def read_regs(path):
  out = subprocess.check_output(SUDO + ["grep", "-E", "^0x0009|^0x0019|^0x001a", path],
                                text=True, stderr=subprocess.STDOUT)
  vals = {}
  for line in out.splitlines():
    cols = line.split()
    if len(cols) >= 5:
      vals[int(cols[0], 16)] = int(cols[4], 16)
  return vals

def s16(v):
  v &= 0xffff
  return v - 0x10000 if v & 0x8000 else v

candidates, router_enum_ms = wait_device_router_candidates()
if not candidates:
  print(f"no USB4 device router after {router_enum_ms}ms", file=sys.stderr)
  raise SystemExit(2)

errors = []
for _, name, path in candidates:
  try:
    write_reg(path, 0x0019, 0x00000000)
    write_reg(path, 0x001a, OP_OV | OP_TINY_HW_STATUS)
    deadline = time.monotonic() + 2.0
    vals = {}
    while time.monotonic() < deadline:
      vals = read_regs(path)
      op = vals.get(0x001a, OP_OV | OP_TINY_HW_STATUS)
      if not (op & OP_OV):
        if op & OP_UNSUPPORTED:
          raise RuntimeError(f"{name}: native 0xC0 unsupported, op=0x{op:08x}")
        data = vals.get(0x0009, 0)
        meta = vals.get(0x0019, 0)
        voltage_mv = data & 0xffff
        current_ma = s16(data >> 16)
        if meta < 1:
          raise RuntimeError(f"{name}: native 0xC0 returned metadata={meta}, expected >=1")
        if not (MIN_MV <= voltage_mv <= MAX_MV):
          raise RuntimeError(
            f"{name}: native 0xC0 voltage {voltage_mv}mV outside expected USB4 range {MIN_MV}..{MAX_MV}mV"
          )
        print(
          f"sideband_0xC0 source=usb4-router router={name} "
          f"router_enum_ms={router_enum_ms} voltage_mv={voltage_mv} current_ma={current_ma}"
        )
        raise SystemExit(0)
      time.sleep(0.05)
    errors.append(f"{name}: native 0xC0 timed out, regs={vals}")
  except Exception as exc:
    errors.append(str(exc))

for err in errors:
  print(err, file=sys.stderr)
raise SystemExit(1)
PY
}

run_sideband_read() {
  local label="$1"
  local bus="$2"
  local out="$3"
  local marker="${4:-}"
  local rc
  local router_wait_s

  if [ "$label" = "usb4" ]; then
    router_wait_s="$USB4_ROUTER_WAIT_S"
    if [ -n "$bus" ]; then
      router_wait_s=0
    fi
    run_usb4_router_sideband_read "$label" "$out" "$router_wait_s" "$marker"
    rc=$?
    if [ "$rc" = "0" ]; then
      return 0
    fi
    if [ "$rc" != "2" ]; then
      return "$rc"
    fi
    echo "[test][$label] no USB4 router debugfs device; falling back to USB control 0xC0" >> "$out"
  fi

  run_usb_control_sideband_read "$label" "$bus" "$out"
}

flash_target() {
  local label="$1"
  local ftdi="$2"
  local flash_log="$3"

  exec {boot_fd}>/tmp/asm2464pd-bootloader.lock
  flock "$boot_fd"

  echo "[test][$label] bootloader reset"
  reset_ftdi "$ftdi" bootloader
  if ! wait_usbdev_any "$BOOT_USBDEV"; then
    echo "[test][$label] bootloader $BOOT_USBDEV did not enumerate"
    return 1
  fi

  echo "[test][$label] flash"
  if ! sudo env PYTHONPATH="$TG" USBDEV="$BOOT_USBDEV" timeout --foreground "$FLASH_TIMEOUT" python3 flash.py ~/hm_fw.bin > "$flash_log" 2>&1; then
    echo "[test][$label] flash failed; tail of $flash_log:"
    tail -40 "$flash_log" | sed "s/^/[flash][$label] /"
    return 1
  fi

  flock -u "$boot_fd" || true
  exec {boot_fd}>&-
}

run_one() {
  local label="$1"
  local ftdi
  local other
  local other_ftdi
  local bus_hint
  local enum_tries
  local usb_enum_dev
  local bus=""
  local rc=0
  local flashed=0
  local smoke_mode="usb"
  local cap="$HOME/cap_${label}.txt"
  local smoke_log="$HOME/smoke_${label}.log"
  local sideband_log="$HOME/sideband_${label}.log"
  local flash_log="$HOME/flash_${label}.log"
  local enum_timing="$HOME/enum_${label}.txt"
  local enum_bus_file="$HOME/enum_${label}.bus"
  local pci_timing="$HOME/pci_${label}.txt"
  local pci_bdf_file="$HOME/pci_${label}.bdf"
  local dmesg_marker="$HOME/dmesg_${label}.start"
  local before_paths="$HOME/tinydev_${label}.before"
  local before_pci="$HOME/pci_${label}.before"
  local before_pci_snapshot="$HOME/pci_${label}.before.visible"
  local enum_pid=""
  local pci_pid=""
  local pci_ok=0
  local pci_pre_count=0
  local upper

  ftdi="$(ftdi_for_target "$label")"
  other="$(other_target "$label")"
  other_ftdi="$(ftdi_for_target "$other")"
  bus_hint="$(bus_for_target "$label")"
  enum_tries="$ENUM_TRIES"
  usb_enum_dev="$TINY_USBDEV"
  upper="$(uppercase "$label")"
  if [ "$STOCK_FW" != "0" ] && [ "$label" = "usb3" ]; then
    usb_enum_dev="$STOCK_USBDEV"
  fi
  if [ "$label" = "usb4" ]; then
    smoke_mode="pci"
    if ! is_bus_hint "$bus_hint"; then
      enum_tries="$USB4_ENUM_TRIES"
    fi
  fi

  exec {usb3_fd}>"/tmp/asm2464pd-usb3.lock"
  exec {usb4_fd}>"/tmp/asm2464pd-usb4.lock"
  echo "[test][$label] waiting for USB3+USB4 hardware locks"
  flock "$usb3_fd"
  flock "$usb4_fd"

  echo "[test][$label] start FTDI=$ftdi bus_hint=$bus_hint"
  cleanup_ftdi_readers "$ftdi"
  cleanup_ftdi_readers "$other_ftdi"

  echo "[test][$label] holding $other in reset while testing $label"
  if ! reset_ftdi "$other_ftdi" hold; then
    echo "[test][$label] failed to hold $other in reset"
    return 1
  fi

  if flash_target "$label" "$ftdi" "$flash_log"; then
    flashed=1
  else
    rc=1
  fi

  if [ "$flashed" = "1" ]; then
    : > "$cap"
    rm -f "$dmesg_marker" "$enum_timing" "$enum_bus_file" "$pci_timing" "$pci_bdf_file" "$before_pci_snapshot"
    list_usbdev_paths "$usb_enum_dev" > "$before_paths"
    wait_new_usbdev_bus_timed "$usb_enum_dev" "$before_paths" "$bus_hint" "$enum_tries" "$ENUM_DELAY" \
      "$dmesg_marker" "$enum_timing" > "$enum_bus_file" &
    enum_pid=$!
    if [ "$label" = "usb4" ]; then
      list_pci_paths "$AMD_PCI_ID" > "$before_pci_snapshot"
      pci_pre_count="$(awk 'NF { c++ } END { print c + 0 }' "$before_pci_snapshot")"
      : > "$before_pci"
      wait_new_pci_timed "$AMD_PCI_ID" "$before_pci" "$dmesg_marker" "$pci_timing" \
        "$PCI_ENUM_TRIES" "$PCI_ENUM_DELAY" > "$pci_bdf_file" &
      pci_pid=$!
    fi
    echo "[test][$label] run reset + UART capture ${SECS}s"
    run_and_capture "$ftdi" "$SECS" "$cap" "$dmesg_marker"
    echo "[test][$label] UART capture: $cap ($(wc -l < "$cap") lines)"

    if wait "$enum_pid" && [ -s "$enum_bus_file" ]; then
      bus="$(cat "$enum_bus_file")"
      echo "[test][$label] $usb_enum_dev enumerated on bus $(bus3 "$bus") ($(cat "$enum_timing"))"
      print_dmesg_since "$label" "$dmesg_marker"
    else
      print_dmesg_since "$label" "$dmesg_marker"
      echo "[test][$label] $usb_enum_dev did not enumerate after reset ($(cat "$enum_timing" 2>/dev/null || echo status=unknown))"
      if [ "$label" != "usb4" ]; then
        rc=1
      fi
    fi

  fi

  if [ "$flashed" = "1" ] && [ "$label" = "usb4" ] && [ "$STOCK_FW" != "0" ]; then
    if [ -n "$pci_pid" ]; then
      if wait "$pci_pid" && [ -s "$pci_bdf_file" ]; then
        pci_ok=1
        echo "[test][$label] AMD PCI visible at $(cat "$pci_bdf_file") ($(cat "$pci_timing") pre_visible=$pci_pre_count)"
      else
        echo "[test][$label] AMD PCI $AMD_PCI_ID did not enumerate after reset ($(cat "$pci_timing" 2>/dev/null || echo status=unknown) pre_visible=$pci_pre_count)"
        rc=1
      fi
      pci_pid=""
    fi
    echo "[test][$label] STOCK_FW=1: using AMD PCI enumeration as completion signal"
  fi

  if [ "$flashed" = "1" ] && [ "$label" = "usb4" ] && [ "$STOCK_FW" = "0" ]; then
    echo "[test][$label] USB4 sideband 0xC0 read"
    if run_sideband_read "$label" "$bus" "$sideband_log" "$dmesg_marker"; then
      echo "[test][$label] USB4 sideband 0xC0: PASS ($(tail -1 "$sideband_log"))"
      if grep -q 'source=usb4-router' "$sideband_log"; then
        smoke_mode="pci"
      fi
    else
      echo "[test][$label] USB4 sideband 0xC0: FAIL; tail of $sideband_log:"
      tail -40 "$sideband_log" | sed "s/^/[sideband][$label] /"
      rc=1
    fi
    if [ -n "$pci_pid" ]; then
      if wait "$pci_pid" && [ -s "$pci_bdf_file" ]; then
        pci_ok=1
        echo "[test][$label] AMD PCI visible at $(cat "$pci_bdf_file") ($(cat "$pci_timing") pre_visible=$pci_pre_count)"
      else
        echo "[test][$label] AMD PCI $AMD_PCI_ID did not enumerate after reset ($(cat "$pci_timing" 2>/dev/null || echo status=unknown) pre_visible=$pci_pre_count)"
      fi
      pci_pid=""
    fi
  fi

  if [ "$rc" = "0" ] && [ "$STOCK_FW" = "0" ]; then
    echo "[test][$label] tinygrad smoke ($smoke_mode)"
    if run_tinygrad_smoke "$label" "$bus" "$smoke_log" "$smoke_mode"; then
      echo "[test][$label] tinygrad smoke: PASS"
    else
      echo "[test][$label] tinygrad smoke: FAIL; tail of $smoke_log:"
      tail -60 "$smoke_log" | sed "s/^/[smoke][$label] /"
      rc=1
    fi
  fi

  if [ "$VERBOSE" != "0" ]; then
    echo "[test][$label] UART tail:"
    tail -30 "$cap" | sed "s/^/[uart][$label] /"
  fi

  echo "[test][$label] releasing $other reset"
  reset_ftdi "$other_ftdi" release >/dev/null 2>&1 || true
  return "$rc"
}

run_job() {
  local label="$1"
  local log="$HOME/test_${label}.log"
  local status="$HOME/status_${label}.txt"

  rm -f "$status"
  if run_one "$label" > "$log" 2>&1; then
    echo PASS > "$status"
  else
    echo FAIL > "$status"
  fi
}

PIDS=""
for label in $TARGET_LIST; do
  run_job "$label" &
  PIDS="$PIDS $label:$!"
done

FAIL=0
for item in $PIDS; do
  label="${item%%:*}"
  pid="${item##*:}"
  wait "$pid" || true
  log="$HOME/test_${label}.log"
  echo "================================================================"
  cat "$log"
  if [ "$(cat "$HOME/status_${label}.txt" 2>/dev/null || echo FAIL)" != "PASS" ]; then
    FAIL=1
  fi
done

echo "================================================================"
echo "[summary]"
for label in usb3 usb4; do
  case " $TARGET_LIST " in
    *" $label "*)
      status="$(cat "$HOME/status_${label}.txt" 2>/dev/null || echo FAIL)"
      printf '[summary] %-4s %s\n' "$(uppercase "$label")" "$status"
      ;;
    *)
      printf '[summary] %-4s SKIP\n' "$(uppercase "$label")"
      ;;
  esac
done

exit "$FAIL"
REMOTE_EOF

scp -q /tmp/_flash_cap_multi.sh "$NUC:flash_cap.sh"
echo "[test] targets: $TARGETS  capture=${SECS}s  smoke_timeout=$SMOKE_TIMEOUT  stock_fw=$STOCK_FW"
ssh_rc=0
ssh "$NUC" "chmod +x ~/flash_cap.sh && env STOCK_FW='$STOCK_FW' AMD_PCI_ID='$AMD_PCI_ID' STOCK_USBDEV='$STOCK_USBDEV' PCI_ENUM_TRIES='$PCI_ENUM_TRIES' PCI_ENUM_DELAY='$PCI_ENUM_DELAY' timeout --foreground '$RUN_TIMEOUT' ~/flash_cap.sh '$SECS' '$TARGET' '$USB3_FTDI' '$USB4_FTDI' '$USB3_BUS' '$USB4_BUS' '$SMOKE_TIMEOUT' '$FLASH_TIMEOUT' '$SIDEBAND_TIMEOUT' '$USB4_SIDEBAND_MIN_MV' '$USB4_SIDEBAND_MAX_MV' '$VERBOSE'" || ssh_rc=$?

FAIL="$ssh_rc"
for label in usb3 usb4; do
  case " $TARGETS " in
    *" $label "*)
      status_file="/tmp/nuc_${label}_status.txt"
      log_file="/tmp/nuc_${label}_test.log"
      cap_file="/tmp/nuc_${label}_cap.txt"
      smoke_file="/tmp/nuc_${label}_smoke.log"
      sideband_file="/tmp/nuc_${label}_sideband.log"
      pci_file="/tmp/nuc_${label}_pci.txt"
      scp -q "$NUC:status_${label}.txt" "$status_file" 2>/dev/null || echo FAIL > "$status_file"
      scp -q "$NUC:test_${label}.log" "$log_file" 2>/dev/null || true
      scp -q "$NUC:cap_${label}.txt" "$cap_file" 2>/dev/null || true
      scp -q "$NUC:smoke_${label}.log" "$smoke_file" 2>/dev/null || true
      scp -q "$NUC:sideband_${label}.log" "$sideband_file" 2>/dev/null || true
      scp -q "$NUC:pci_${label}.txt" "$pci_file" 2>/dev/null || true
      status="$(cat "$status_file" 2>/dev/null || echo FAIL)"
      echo "[test][$label] logs: $log_file $cap_file $smoke_file $sideband_file $pci_file"
      [ "$status" = "PASS" ] || FAIL=1
      ;;
  esac
done

echo "================ FINAL ================"
for label in usb3 usb4; do
  case " $TARGETS " in
    *" $label "*)
      status="$(cat "/tmp/nuc_${label}_status.txt" 2>/dev/null || echo FAIL)"
      ;;
    *)
      status="SKIP"
      ;;
  esac
  printf '%-4s %s\n' "$(printf '%s' "$label" | tr '[:lower:]' '[:upper:]')" "$status"
done

exit "$FAIL"
