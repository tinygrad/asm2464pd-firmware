#!/bin/bash
# test_bootstub.sh [target]
#
# Verify the bootstub two-stage firmware on a NUC HITL rig:
#   1. Flash the bootstub itself (one-time, via FTDI bootloader strap)
#   2. Flash the userfw image through bootstub DFU (nflash)
#   3. App boots and enumerates as ADD1:0001
#   4. 0xEC enter-DFU from app → bootstub DFU enumerates as ADD1:B007
#   5. 0xEB reboot from bootstub → app re-enumerates as ADD1:0001
#   6. tinygrad smoke test
#
#   target - usb3|usb4 (default: usb3)
#
# The bootstub is only re-flashed if BOOTSTUB_INSTALL=1 is set; otherwise
# the existing bootstub is reused and only the userfw image is updated.
#
# Useful knobs: SMOKE_TIMEOUT=45s FLASH_TIMEOUT=45s RUN_TIMEOUT=120s
# VERBOSE=1

set -u

NUC="${NUC:-nuc}"
TARGET="${1:-usb3}"
BOOTSTUB_BIN="handmade/build/bootstub_wrapped.bin"
APP_IMAGE="handmade/build/firmware_image.bin"
BOOTSTUB_INSTALL="${BOOTSTUB_INSTALL:-0}"
SMOKE_TIMEOUT="${SMOKE_TIMEOUT:-45s}"
FLASH_TIMEOUT="${FLASH_TIMEOUT:-45s}"
RUN_TIMEOUT="${RUN_TIMEOUT:-120s}"
VERBOSE="${VERBOSE:-0}"

case "$TARGET" in
  usb3|USB3) FTDI="ftdi://ftdi:ft-x:DK0CG1XJ/1" ;;
  usb4|USB4) FTDI="ftdi://ftdi:ft-x:DK0CGSKL/1" ;;
  *) echo "ERROR: target must be usb3 or usb4"; exit 1 ;;
esac

TG="${TG:-/home/batman/openpilot/tinygrad_repo}"
BOOT_USBDEV="${BOOT_USBDEV:-174C:2463}"
TINY_USBDEV="${TINY_USBDEV:-ADD1:0001}"
DFU_USBDEV="${DFU_USBDEV:-ADD1:B007}"

echo "[bootstub-test] target=$TARGET  bootstub_install=$BOOTSTUB_INSTALL"

# ---- build locally ----
echo "[bootstub-test] building..."
make -C handmade clean >/dev/null 2>&1
if ! make -C handmade all 2>&1 | grep -E "error|FAIL"; then
  echo "[bootstub-test] build OK"
else
  echo "ERROR: build failed"
  exit 1
fi

for f in "$BOOTSTUB_BIN" "$APP_IMAGE"; do
  if [ ! -f "$f" ]; then echo "ERROR: $f not found"; exit 1; fi
done

# ---- ship to NUC ----
echo "[bootstub-test] shipping binaries to $NUC"
scp -q "$BOOTSTUB_BIN" "$NUC:hm_bootstub.bin" || { echo "ERROR: scp bootstub failed"; exit 1; }
scp -q "$APP_IMAGE"    "$NUC:hm_userfw.bin"   || { echo "ERROR: scp userfw failed"; exit 1; }

# ---- remote test script ----
cat > /tmp/_bootstub_test.sh <<'REMOTE_EOF'
#!/bin/bash
set -u

cd ~/asm2464pd-firmware || exit 1

FTDI="${1:?missing FTDI}"
BOOTSTUB_INSTALL="${2:-0}"
SMOKE_TIMEOUT="${3:-45s}"
FLASH_TIMEOUT="${4:-45s}"
TG="${TG:-/home/batman/openpilot/tinygrad_repo}"
BOOT_USBDEV="${BOOT_USBDEV:-174C:2463}"
TINY_USBDEV="${TINY_USBDEV:-ADD1:0001}"
DFU_USBDEV="${DFU_USBDEV:-ADD1:B007}"
VERBOSE="${VERBOSE:-0}"
FAIL=0

reset_ftdi() {
  local url="$1"
  local mode="$2"
  sudo python3 - "$url" "$mode" <<'PY'
import sys, time
from ftdi_debug import USBGPUDebug
url, mode = sys.argv[1], sys.argv[2]
with USBGPUDebug(url) as dbg:
  if mode == "bootloader":
    dbg.ftdi.set_cbus_gpio(dbg.CBUS_RESET | dbg.CBUS_BOOTLOADER)
    time.sleep(0.5)
    dbg.ftdi.set_cbus_gpio(dbg.CBUS_BOOTLOADER)
    time.sleep(2.0)
    dbg.ftdi.set_cbus_gpio(0)
  elif mode == "run":
    dbg.ftdi.set_cbus_gpio(dbg.CBUS_RESET)
    time.sleep(0.5)
    dbg.ftdi.set_cbus_gpio(0)
  elif mode == "hold":
    dbg.ftdi.set_cbus_gpio(dbg.CBUS_RESET)
  elif mode == "release":
    dbg.ftdi.set_cbus_gpio(0)
PY
}

wait_usbdev() {
  local vidpid="$1"
  local tries="${2:-80}"
  local delay="${3:-0.125}"
  local i
  for i in $(seq 1 "$tries"); do
    if lsusb -d "$vidpid" >/dev/null 2>&1; then return 0; fi
    sleep "$delay"
  done
  return 1
}

wait_usbdev_gone() {
  local vidpid="$1"
  local tries="${2:-40}"
  local delay="${3:-0.125}"
  local i
  for i in $(seq 1 "$tries"); do
    if ! lsusb -d "$vidpid" >/dev/null 2>&1; then return 0; fi
    sleep "$delay"
  done
  return 1
}

get_usb_bus() {
  lsusb -d "$TINY_USBDEV" 2>/dev/null | awk '{print $2}' | head -1
}

# ---- step 1: (optional) flash bootstub via FTDI strap ----
if [ "$BOOTSTUB_INSTALL" = "1" ]; then
  echo "[bootstub-test] flashing bootstub via FTDI bootloader strap"
  reset_ftdi "$FTDI" bootloader
  if ! wait_usbdev "$BOOT_USBDEV" 60 0.25; then
    echo "FAIL: bootloader $BOOT_USBDEV did not enumerate"
    exit 1
  fi
  # Pad bootstub to 64 KB with 0xFF so the FTDI flasher erases the entire
  # SPI flash (including any stale userfw at 0x4000 from a previous layout).
  python3 -c "
import sys
body = open('$HOME/hm_bootstub.bin','rb').read()
padded = body + b'\xff' * (0x10000 - len(body))
open('$HOME/hm_bootstub_padded.bin','wb').write(padded)
"
  if ! sudo env PYTHONPATH="$TG" USBDEV="$BOOT_USBDEV" timeout --foreground "$FLASH_TIMEOUT" \
       python3 flash.py ~/hm_bootstub_padded.bin; then
    echo "FAIL: bootstub flash failed"
    exit 1
  fi
  reset_ftdi "$FTDI" run
  echo "[bootstub-test] bootstub installed"
  # Bootstub should enter DFU (no valid userfw) — wait for it
  if ! wait_usbdev "$DFU_USBDEV" 30 0.25; then
    echo "FAIL: bootstub DFU did not enumerate after install"
    exit 1
  fi
else
  echo "[bootstub-test] skipping bootstub install (set BOOTSTUB_INSTALL=1 to re-flash)"
fi

# ---- step 2: flash userfw via bootstub DFU ----
echo "[bootstub-test] flashing userfw via bootstub DFU"
# If app is running, handmade/flash.py will send 0xEC to enter DFU first
if ! sudo env PYTHONPATH="$TG" timeout --foreground "$FLASH_TIMEOUT" \
     python3 handmade/flash.py ~/hm_userfw.bin; then
  echo "FAIL: userfw flash failed"
  exit 1
fi

# ---- step 3: verify app boots and enumerates as ADD1:0001 ----
echo "[bootstub-test] waiting for app to enumerate ($TINY_USBDEV)"
if ! wait_usbdev "$TINY_USBDEV" 80 0.125; then
  echo "FAIL: app did not enumerate after userfw flash"
  exit 1
fi
BUS="$(get_usb_bus)"
echo "[bootstub-test] app enumerated on bus $BUS"

# ---- step 4: test 0xEC enter-DFU from app ----
echo "[bootstub-test] testing 0xEC enter-DFU from app"
sudo env PYTHONPATH="$TG" timeout --foreground 10 python3 handmade/flash.py enter-dfu 2>/dev/null || true

if ! wait_usbdev_gone "$TINY_USBDEV" 20 0.25; then
  echo "FAIL: app did not disappear after 0xEC"
  exit 1
fi
if ! wait_usbdev "$DFU_USBDEV" 30 0.25; then
  echo "FAIL: bootstub DFU ($DFU_USBDEV) did not enumerate after 0xEC"
  exit 1
fi
echo "[bootstub-test] bootstub DFU up after 0xEC"

# ---- step 5: test 0xEB reboot from bootstub → app re-enumerates ----
echo "[bootstub-test] testing 0xEB reboot from bootstub"
sudo env PYTHONPATH="$TG" timeout --foreground 10 python3 handmade/flash.py reboot 2>/dev/null || true

if ! wait_usbdev_gone "$DFU_USBDEV" 20 0.25; then
  echo "FAIL: bootstub DFU did not disappear after 0xEB"
  exit 1
fi
if ! wait_usbdev "$TINY_USBDEV" 80 0.125; then
  echo "FAIL: app did not re-enumerate after 0xEB reboot"
  exit 1
fi
BUS="$(get_usb_bus)"
echo "[bootstub-test] app re-enumerated on bus $BUS after reboot"

# ---- step 6: tinygrad smoke test ----
if [ -d "$TG" ]; then
  echo "[bootstub-test] tinygrad smoke test (DEV=USB+AMD)"
  if [ -n "$BUS" ]; then
    ( cd "$TG" && timeout --foreground "$SMOKE_TIMEOUT" env PYTHONPATH="." DEBUG=2 DEV=USB+AMD \
      TINYGRAD_USB_BUS="$BUS" python3 - <<'PY'
import os, runpy, sys
from tinygrad.runtime.support.usb import USB3
target_bus = str(int(os.environ["TINYGRAD_USB_BUS"]))
orig = USB3.list_devices
def filtered(cls, vendor, dev):
  return [item for item in orig(vendor, dev) if item[1].split(":")[1].split("-")[0] == target_bus]
USB3.list_devices = classmethod(filtered)
sys.argv = ["test/test_tiny.py", "TestTiny.test_plus"]
runpy.run_path("test/test_tiny.py", run_name="__main__")
PY
    ) > /tmp/bootstub_smoke.log 2>&1
    rc=$?
    if [ "$rc" = "0" ]; then
      echo "[bootstub-test] tinygrad smoke: PASS"
    else
      echo "[bootstub-test] tinygrad smoke: FAIL"
      tail -30 /tmp/bootstub_smoke.log | sed 's/^/[smoke] /'
      FAIL=1
    fi
  else
    echo "[bootstub-test] tinygrad smoke: SKIP (no bus)"
  fi
else
  echo "[bootstub-test] tinygrad smoke: SKIP (no $TG)"
fi

if [ "$FAIL" = "0" ]; then
  echo "[bootstub-test] ALL PASS"
else
  echo "[bootstub-test] FAIL"
fi
exit "$FAIL"
REMOTE_EOF

scp -q /tmp/_bootstub_test.sh "$NUC:bootstub_test.sh"
ssh "$NUC" "chmod +x ~/bootstub_test.sh"
echo "[bootstub-test] running on $NUC..."
ssh_rc=0
ssh "$NUC" "timeout --foreground '$RUN_TIMEOUT' ~/bootstub_test.sh '$FTDI' '$BOOTSTUB_INSTALL' '$SMOKE_TIMEOUT' '$FLASH_TIMEOUT'" || ssh_rc=$?

echo "================ FINAL ================"
if [ "$ssh_rc" = "0" ]; then
  echo "BOOTSTUB  PASS"
else
  echo "BOOTSTUB  FAIL"
fi
exit "$ssh_rc"
