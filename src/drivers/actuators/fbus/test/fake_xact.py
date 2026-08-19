#!/usr/bin/env python3
"""Software Xact W56xx servo for FBUS driver loopback testing.

Creates a pty, symlinks it to /tmp/fbus_pty, and answers the PX4 fbus
master like a real servo: telemetry (3.5 A / 7.4 V / 42 degC, the frame
captured on the Teensy bench) for DATA polls to physical ID 0x0C, and
config RESPONSEs for READ requests. Prints a summary on exit.
"""

import os
import pty
import sys
import time
import tty

CAPTURED_UPLINK = bytes([0x08, 0xAC, 0x10, 0x00, 0x68, 0x23, 0x4A, 0x2A, 0x00, 0x43])

# Xact config fields the fake servo reports on READ
CONFIG_VALUES = {
    0x00: 0x0C,  # physical id
    0x01: 0x00,  # servo id
    0x02: 100,   # data rate ms
    0x04: 0,     # range enum
    0x05: 0,     # direction
    0x06: 0,     # pulse type 1520us/333Hz
    0x07: 0,     # followed channel
    0x08: 0,     # center
}


def frsky_checksum(data):
    checksum = sum(data)
    while checksum > 0xFF:
        checksum = (checksum & 0xFF) + (checksum >> 8)
    return 0xFF - checksum


def build_uplink(phy_id_with_bits, prim, app_id, data_u32):
    frame = bytearray(10)
    frame[0] = 0x08
    frame[1] = phy_id_with_bits
    frame[2] = prim
    frame[3] = app_id & 0xFF
    frame[4] = (app_id >> 8) & 0xFF
    frame[5] = data_u32 & 0xFF
    frame[6] = (data_u32 >> 8) & 0xFF
    frame[7] = (data_u32 >> 16) & 0xFF
    frame[8] = (data_u32 >> 24) & 0xFF
    frame[9] = frsky_checksum(frame[1:9])
    return bytes(frame)


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0

    master_fd, slave_fd = pty.openpty()
    tty.setraw(master_fd)
    slave_name = os.ttyname(slave_fd)

    link = "/tmp/fbus_pty"
    if os.path.islink(link) or os.path.exists(link):
        os.unlink(link)
    os.symlink(slave_name, link)
    print(f"fake Xact on {slave_name} -> {link}", flush=True)

    buf = bytearray()
    frames = 0
    polls_answered = 0
    config_answered = 0
    crc_errors = 0
    first_frame_t = None
    last_frame_t = None

    deadline = time.monotonic() + duration
    os.set_blocking(master_fd, False)

    while time.monotonic() < deadline:
        try:
            chunk = os.read(master_fd, 256)
            if chunk:
                buf.extend(chunk)
        except (BlockingIOError, OSError):
            time.sleep(0.0002)

        # Hunt for a 37-byte master frame: control [0x18 0xFF ... crc] + downlink [0x08 ...]
        while len(buf) >= 37:
            if buf[0] != 0x18 or buf[1] != 0xFF or buf[27] != 0x08:
                buf.pop(0)
                continue

            control = bytes(buf[:27])
            downlink = bytes(buf[27:37])
            del buf[:37]

            if frsky_checksum(control[1:26]) != control[26]:
                crc_errors += 1
                continue

            frames += 1
            now = time.monotonic()
            first_frame_t = first_frame_t or now
            last_frame_t = now

            phy = downlink[1]
            prim = downlink[2]
            app_id = downlink[3] | (downlink[4] << 8)
            data = downlink[5] | (downlink[6] << 8) | (downlink[7] << 16) | (downlink[8] << 24)

            if phy != 0xAC:  # only answer polls addressed to physical ID 0x0C
                continue

            if prim == 0x10:  # DATA poll -> telemetry
                os.write(master_fd, CAPTURED_UPLINK)
                polls_answered += 1

            elif prim == 0x30:  # READ -> RESPONSE with field | value << 8
                field = data & 0xFF
                value = CONFIG_VALUES.get(field, 0)
                os.write(master_fd, build_uplink(0xAC, 0x32, app_id, field | (value << 8)))
                config_answered += 1

            elif prim == 0x31:  # WRITE -> store + echo back as RESPONSE
                field = data & 0xFF
                if field != 0x30:  # save has no stored value
                    CONFIG_VALUES[field] = (data >> 8) & 0xFFFFFF
                    os.write(master_fd, build_uplink(0xAC, 0x32, app_id, data))
                config_answered += 1

    rate = 0.0
    if frames > 1 and last_frame_t and first_frame_t and last_frame_t > first_frame_t:
        rate = (frames - 1) / (last_frame_t - first_frame_t)

    print(f"SUMMARY frames={frames} rate={rate:.0f}Hz polls_answered={polls_answered} "
          f"config_answered={config_answered} crc_errors={crc_errors}", flush=True)


if __name__ == "__main__":
    main()
