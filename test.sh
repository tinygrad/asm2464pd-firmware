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
DMESG_PRE=\$(sudo dmesg 2>/dev/null | wc -l)   # marker: host kernel-log line count before this run
echo -n "[test]  bootloader: "; sudo python3 ftdi_debug.py -bn 2>&1 | tail -1
echo -n "[test]  flash:      "; sudo PYTHONPATH=/home/batman/openpilot/tinygrad_repo USBDEV=174C:2463 python3 flash.py ~/hm_fw.bin 2>&1 | tail -1
echo -n "[test]  run:        "; sudo python3 ftdi_debug.py -rn 2>&1 | tail -1
: > ~/cap.txt
sudo timeout \$SECS python3 ftdi_debug.py >> ~/cap.txt 2>&1
echo "[test]  captured \$(wc -l < ~/cap.txt) lines"
# End-to-end ENUMERATION + HOST-SIDE THUNDERBOLT view. The firmware change only truly works when
# the AMD eGPU (1002:7590) is tunneled onto the NUC's PCIe bus -- the UART [Lane Bonded] alone is
# NOT sufficient. This kernel has no boltctl/tbtools/debugfs, so the HOST's view of the bond is:
#  (a) dmesg thunderbolt events during this run -- the device connecting as 1-N, and the per-lane
#      "new retimer found"/"retimer disconnected" churn = lane-training stability (stock stabilizes;
#      a partial bond churns: found==disconnected, device repeatedly connects/disconnects);
#  (b) the connected device's sysfs rx_lanes/tx_lanes/*_speed = the negotiated/bonded lane count+gen.
echo "1" | sudo tee /sys/bus/pci/rescan >/dev/null 2>&1 || true
sleep 1
GPU=\$(lspci -nn 2>/dev/null | grep -iE "1002:7590|Radeon RX 9060" | head -1)
echo "[test]  GPU(lspci 1002:7590): \${GPU:-<absent>}"
# (a) host kernel-log TB events that occurred during this flash+capture window
echo "[tbhost] dmesg thunderbolt events this run:"
sudo dmesg 2>/dev/null | tail -n +\$((DMESG_PRE+1)) | grep -i thunderbolt | sed 's/^/[tbhost]   /' | tail -25
RTF=\$(sudo dmesg 2>/dev/null | tail -n +\$((DMESG_PRE+1)) | grep -c "new retimer found")
RTD=\$(sudo dmesg 2>/dev/null | tail -n +\$((DMESG_PRE+1)) | grep -c "retimer disconnected")
DEVF=\$(sudo dmesg 2>/dev/null | tail -n +\$((DMESG_PRE+1)) | grep -c "device found\|device connected")
DEVD=\$(sudo dmesg 2>/dev/null | tail -n +\$((DMESG_PRE+1)) | grep -c "device disconnected")
echo "[tbhost] churn: retimer found=\$RTF disc=\$RTD  device found=\$DEVF disc=\$DEVD  (stable bond = found>>disc; churn = lanes never latch)"
# (b) connected-device routers (skip host routers x-0 and retimers x-y:z) -> bonded lane count/speed
for d in /sys/bus/thunderbolt/devices/[0-9]-[0-9]*; do
  bn=\$(basename "\$d"); case "\$bn" in *:*) continue;; *-0) continue;; esac
  [ -d "\$d" ] || continue
  line="[tbhost] dev \$bn:"
  for a in device_name rx_lanes tx_lanes rx_speed tx_speed generation authorized; do
    [ -e "\$d/\$a" ] && line="\$line \$a=\$(cat "\$d/\$a" 2>/dev/null)"
  done
  echo "\$line"
done
TB=\$(ls /sys/bus/thunderbolt/devices/ 2>/dev/null | grep -E "^[0-9]+-[0-9]" | tr '\n' ' ')
echo "[test]  thunderbolt routers: \${TB:-<none>}"
REMOTE_EOF
scp -q /tmp/_flash_cap.sh "$NUC:flash_cap.sh"
echo "[test] flash + run + capture ${SECS}s ..."
ssh "$NUC" "chmod +x ~/flash_cap.sh && ~/flash_cap.sh $SECS"

scp -q "$NUC:cap.txt" /tmp/nuc_cap.txt
echo "[test] capture -> /tmp/nuc_cap.txt  (tail below)"
echo "----------------------------------------------------------------"
tail -30 /tmp/nuc_cap.txt
echo "----------------------------------------------------------------"
# Local verdict from the captured UART (the remote GPU/thunderbolt lines above are the
# ground-truth end-to-end check; these are the on-device milestones leading up to it).
BOND=$(grep -c "Lane Bonded" /tmp/nuc_cap.txt 2>/dev/null); BOND=${BOND:-0}
CL0=$(grep -cE "L0:CL0 02|L1:CL0 02" /tmp/nuc_cap.txt 2>/dev/null); CL0=${CL0:-0}
PCIE=$(grep -cE "PCIE Gen|PcieLinkUp|Bus#" /tmp/nuc_cap.txt 2>/dev/null); PCIE=${PCIE:-0}
echo "[verdict] Lane Bonded=$BOND  Lx:CL0-02=$CL0  PCIe-enum=$PCIE"
if grep -qE "1002:7590|Radeon RX 9060" /tmp/nuc_cap.txt 2>/dev/null; then
  echo "[verdict] *** GPU ENUMERATED — FULL SUCCESS ***"
elif [ "$BOND" -gt 0 ]; then
  echo "[verdict] Lane Bonded but GPU not yet enumerated (see remote GPU line above for the real check)"
else
  echo "[verdict] NOT bonded — see remote 'GPU(lspci ...)' line above for the end-to-end ground truth"
fi
