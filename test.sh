#!/bin/bash
# test.sh [binary] [capture_seconds]
#
# Ship a wrapped firmware binary to the NUC test rig, flash it, and capture UART.
#   binary           - path to a wrapped firmware .bin (default: handmade/build/firmware_wrapped.bin,
#                      i.e. the last `make -C handmade wrapped` output)
#   capture_seconds  - how long to capture UART after reset (default: 10)
#
# Gotchas this encodes:
#  * The NUC's /tmp is periodically WIPED (systemd-tmpfiles) -- earlier "flash didn't take" bugs were
#    really the binary/scripts vanishing from /tmp. Stage into the persistent HOME dir and VERIFY md5.
#  * flash.py needs tinygrad on PYTHONPATH + USBDEV; ftdi_debug.py needs sudo for the FTDI.
#  * Run the flash+capture from a staged remote SCRIPT (not an inline ssh command) -- nested quoting and
#    `sudo timeout ... > file` redirects are unreliable inline.
#  * The device reboot-loops (~1.8s/boot); the USB4 connect is cycle-variable (only ~1 in 5 is the
#    20G/819=01 cycle that reaches the state-5 lane walker), so a 10s capture catches a few boots --
#    re-run if you need a specific cycle.
set -u
NUC=nuc
BIN="${1:-handmade/build/firmware_wrapped.bin}"
SECS="${2:-10}"

if [ ! -f "$BIN" ]; then echo "ERROR: binary '$BIN' not found"; exit 1; fi
SZ=$(stat -c%s "$BIN"); LMD5=$(md5sum "$BIN" | cut -d' ' -f1)
echo "[test] ship  $BIN  ($SZ bytes, md5 ${LMD5:0:8}) -> $NUC:~/hm_fw.bin"
scp -q "$BIN" "$NUC:hm_fw.bin" || { echo "ERROR: scp failed"; exit 1; }
RMD5=$(ssh "$NUC" 'md5sum ~/hm_fw.bin 2>/dev/null | cut -d" " -f1')
if [ "$LMD5" != "$RMD5" ]; then echo "ERROR: md5 mismatch (local $LMD5 != nuc $RMD5) -- copy did not land"; exit 1; fi
echo "[test] verified on NUC (md5 ${RMD5:0:8})"

# stage the remote flash+capture helper (persistent home dir)
cat > /tmp/_flash_cap.sh <<REMOTE_EOF
#!/bin/bash
cd ~/asm2464pd-firmware
SECS=\${1:-10}
sudo pkill -f ftdi_debug 2>/dev/null; sleep 1
echo -n "[test]  bootloader: "; sudo python3 ftdi_debug.py -bn 2>&1 | tail -1
echo -n "[test]  flash:      "; sudo PYTHONPATH=/home/batman/openpilot/tinygrad_repo USBDEV=174C:2463 python3 flash.py ~/hm_fw.bin 2>&1 | tail -1
echo -n "[test]  run:        "; sudo python3 ftdi_debug.py -rn 2>&1 | tail -1
: > ~/cap.txt
sudo timeout \$SECS python3 ftdi_debug.py >> ~/cap.txt 2>&1
echo "[test]  captured \$(wc -l < ~/cap.txt) lines"
REMOTE_EOF
scp -q /tmp/_flash_cap.sh "$NUC:flash_cap.sh"
echo "[test] flash + run + capture ${SECS}s ..."
ssh "$NUC" "chmod +x ~/flash_cap.sh && ~/flash_cap.sh $SECS"

scp -q "$NUC:cap.txt" /tmp/nuc_cap.txt
echo "[test] capture -> /tmp/nuc_cap.txt  (tail below)"
echo "----------------------------------------------------------------"
tail -40 /tmp/nuc_cap.txt
