#!/usr/bin/env python3
"""Test: BOT E5 pokes to set readiness, then switch to UAS for E4 reads."""
import sys, ctypes, struct, time
sys.path.insert(0, '/home/geohot/tinygrad')
from tinygrad.runtime.autogen import libusb

ctx = ctypes.POINTER(libusb.struct_libusb_context)()
libusb.libusb_init(ctypes.byref(ctx))
handle = libusb.libusb_open_device_with_vid_pid(ctx, 0x174c, 0x2463)
if not handle: print('not found'); sys.exit(1)
if libusb.libusb_kernel_driver_active(handle, 0):
    libusb.libusb_detach_kernel_driver(handle, 0)
    libusb.libusb_reset_device(handle)
libusb.libusb_set_configuration(handle, 1)
libusb.libusb_claim_interface(handle, 0)

transferred = ctypes.c_int(0)

def bot_cmd(cdb, tag, data_in_len=0):
    """Send a SCSI command via BOT. Returns (csw_status, data_or_None)."""
    flags = 0x80 if data_in_len > 0 else 0x00
    cbw = struct.pack('<IIIBBB', 0x43425355, tag, data_in_len, flags, 0, len(cdb)) + cdb + b'\x00' * (16 - len(cdb))
    buf = (ctypes.c_ubyte * 31)(*cbw)
    csw = (ctypes.c_ubyte * 13)()

    for attempt in range(3):
        rc = libusb.libusb_bulk_transfer(handle, 0x02, buf, 31, ctypes.byref(transferred), 2000)
        if rc == 0: break
        libusb.libusb_clear_halt(handle, 0x02)
        libusb.libusb_clear_halt(handle, 0x81)
    else:
        return -1, None

    data = None
    if data_in_len > 0:
        dbuf = (ctypes.c_ubyte * data_in_len)()
        rc = libusb.libusb_bulk_transfer(handle, 0x81, dbuf, data_in_len, ctypes.byref(transferred), 2000)
        if rc == 0 and transferred.value > 0:
            data = bytes(dbuf[:transferred.value])
        elif rc != 0:
            libusb.libusb_clear_halt(handle, 0x81)

    for attempt in range(3):
        rc = libusb.libusb_bulk_transfer(handle, 0x81, csw, 13, ctypes.byref(transferred), 2000)
        if rc == 0 and transferred.value == 13: break
        libusb.libusb_clear_halt(handle, 0x81)
    else:
        return -2, data

    sig, rtag, residue, status = struct.unpack('<IIIB', bytes(csw))
    return status, data

def bot_e5(addr, val, tag):
    usb_addr = (addr & 0x1FFFF) | 0x500000
    cdb = struct.pack('>BBBHB', 0xE5, val, usb_addr >> 16, usb_addr & 0xFFFF, 0)
    return bot_cmd(cdb, tag)

def bot_e4(addr, size, tag):
    usb_addr = (addr & 0x1FFFF) | 0x500000
    cdb = struct.pack('>BBBHB', 0xE4, size, usb_addr >> 16, usb_addr & 0xFFFF, 0)
    return bot_cmd(cdb, tag, data_in_len=size)

# Phase 1: BOT mode pokes
print('=== Phase 1: BOT E5 writes to set readiness globals ===')
tag = 1
pokes = [
    (0x07EF, 0x00, 'G_SYS_FLAGS_07EF = 0'),
    (0x0B2F, 0x01, 'G_INTERFACE_READY = 1'),
    (0x0002, 0x01, 'G_IO_CMD_STATE != 0'),
]
for addr, val, desc in pokes:
    st, _ = bot_e5(addr, val, tag)
    print(f'  E5 0x{addr:04X}=0x{val:02X}: CSW status={st} ({desc})')
    tag += 1

# Try a BOT E4 read to see if it works now
print('\n=== Phase 1b: BOT E4 read test ===')
st, data = bot_e4(0xf000, 4, tag)
tag += 1
print(f'  E4 read 0xf000 (4 bytes): CSW status={st}, data={data.hex() if data else "none"}')

# Phase 2: Switch to UAS
print('\n=== Phase 2: Switch to UAS ===')
rc = libusb.libusb_set_interface_alt_setting(handle, 0, 1)
print(f'  set_alt_setting(1): rc={rc}')
for ep in [0x02, 0x81, 0x83, 0x04]:
    libusb.libusb_clear_halt(handle, ep)
stream_eps = (ctypes.c_uint8 * 3)(0x02, 0x81, 0x83)
ns = libusb.libusb_alloc_streams(handle, 93, stream_eps, 3)
print(f'  alloc_streams: {ns}')

# Phase 3: UAS E5 write
def uas_cmd(cdb, utag, data_in_len=0):
    """Send UAS command. Returns (scsi_status, data, debug_info)."""
    cmd_iu = bytearray(32)
    cmd_iu[0] = 0x01
    cmd_iu[3] = utag
    cmd_iu[16:16+len(cdb)] = cdb

    cmd_tr = libusb.libusb_alloc_transfer(0)
    stat_tr = libusb.libusb_alloc_transfer(0)
    cmd_buf = (ctypes.c_uint8 * 32)(*cmd_iu)
    stat_buf = (ctypes.c_uint8 * 64)()

    trs = []
    for tr, ep, sz, buf, stream, ttype in [
        (cmd_tr, 0x04, 32, cmd_buf, None, libusb.LIBUSB_TRANSFER_TYPE_BULK),
        (stat_tr, 0x83, 64, stat_buf, utag, libusb.LIBUSB_TRANSFER_TYPE_BULK_STREAM),
    ]:
        tr.contents.dev_handle = handle
        tr.contents.endpoint = ep
        tr.contents.length = sz
        tr.contents.buffer = buf
        tr.contents.status = 0xFF
        tr.contents.flags = 0
        tr.contents.timeout = 5000
        tr.contents.type = ttype
        if stream: libusb.libusb_transfer_set_stream_id(tr, stream)
        trs.append(tr)

    data_tr = None
    data_buf = None
    if data_in_len > 0:
        data_tr = libusb.libusb_alloc_transfer(0)
        data_buf = (ctypes.c_uint8 * max(data_in_len, 64))()
        data_tr.contents.dev_handle = handle
        data_tr.contents.endpoint = 0x81
        data_tr.contents.length = data_in_len
        data_tr.contents.buffer = data_buf
        data_tr.contents.status = 0xFF
        data_tr.contents.flags = 0
        data_tr.contents.timeout = 5000
        data_tr.contents.type = libusb.LIBUSB_TRANSFER_TYPE_BULK_STREAM
        libusb.libusb_transfer_set_stream_id(data_tr, utag)
        trs.append(data_tr)

    for tr in trs:
        libusb.libusb_submit_transfer(tr)

    names = {0: 'OK', 1: 'ERROR', 2: 'TIMED_OUT', 3: 'CANCELLED', 4: 'STALL', 5: 'NO_DEVICE', 6: 'OVERFLOW', 0xFF: 'pending'}
    t0 = time.monotonic()
    while time.monotonic() - t0 < 7:
        libusb.libusb_handle_events(ctx)
        if all(t.contents.status != 0xFF for t in trs):
            break
        if any(t.contents.status not in (0, 0xFF) for t in trs):
            break

    # Cancel pending
    for tr in trs:
        if tr.contents.status == 0xFF:
            libusb.libusb_cancel_transfer(tr)
    time.sleep(0.1)
    libusb.libusb_handle_events(ctx)

    al = stat_tr.contents.actual_length if stat_tr.contents.status == 0 else 0
    resp = bytes(stat_buf[:al]) if al else b''
    scsi_st = resp[6] if len(resp) > 6 else -1

    data = None
    if data_tr and data_tr.contents.status == 0:
        dal = data_tr.contents.actual_length
        data = bytes(data_buf[:dal])

    info_parts = []
    info_parts.append(f'cmd={names.get(cmd_tr.contents.status, cmd_tr.contents.status)}')
    info_parts.append(f'stat={names.get(stat_tr.contents.status, stat_tr.contents.status)}')
    if data_tr:
        info_parts.append(f'data={names.get(data_tr.contents.status, data_tr.contents.status)}')
    return scsi_st, data, ' '.join(info_parts)

print('\n=== Phase 3: UAS E5 write test ===')
usb_addr = (0x07EF & 0x1FFFF) | 0x500000
cdb = struct.pack('>BBBHB', 0xE5, 0x00, usb_addr >> 16, usb_addr & 0xFFFF, 0)
st, _, info = uas_cmd(cdb, 1)
print(f'  E5 0x07EF=0x00: scsi_status={st} [{info}]')

print('\n=== Phase 4: UAS E4 read test ===')
usb_addr = (0xf000 & 0x1FFFF) | 0x500000
cdb = struct.pack('>BBBHB', 0xE4, 4, usb_addr >> 16, usb_addr & 0xFFFF, 0)
st, data, info = uas_cmd(cdb, 1, data_in_len=4)
dstr = data.hex() if data else 'none'
print(f'  E4 read 0xf000: scsi_status={st} data={dstr} [{info}]')

libusb.libusb_close(handle)
libusb.libusb_exit(ctx)
