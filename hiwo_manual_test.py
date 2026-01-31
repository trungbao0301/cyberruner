#!/usr/bin/env python3
import os
import glob
import subprocess
import time


VID = "0483"
PID = "5750"


def find_hiwo_hidraw(vid=VID, pid=PID) -> str:
    """Find /dev/hidraw* that matches USB vid/pid using udevadm."""
    for dev in sorted(glob.glob("/dev/hidraw*")):
        try:
            info = subprocess.check_output(
                ["udevadm", "info", "-a", "-n", dev],
                text=True,
                stderr=subprocess.DEVNULL,
            )
            if f'ATTRS{{idVendor}}=="{vid}"' in info and f'ATTRS{{idProduct}}=="{pid}"' in info:
                return dev
        except Exception:
            pass
    raise RuntimeError(f"Hiwonder controller {vid}:{pid} not found. Plug it in and retry.")


def checksum(pkt_wo_ck: bytes) -> int:
    """
    LewanSoul/Hiwonder checksum:
    checksum = (~(sum(bytes[2:]) & 0xFF)) & 0xFF
    (sum from ID onward, not including the two 0x55 headers)
    """
    s = sum(pkt_wo_ck[2:]) & 0xFF
    return (~s) & 0xFF


def build_move_time_write(servo_id: int, pos: int, time_ms: int, len_override=None) -> bytes:
    """
    LX-16A / LewanSoul:
    55 55 ID LEN CMD POS_L POS_H TIME_L TIME_H CHECKSUM
    CMD=1 (Move Time Write)

    LEN normally = 6 (CMD + 4 params + checksum).
    Some controllers/firmwares may expect LEN=7; you can override to test.
    """
    pos = max(0, min(1000, int(pos)))
    time_ms = max(0, min(30000, int(time_ms)))

    pos_l = pos & 0xFF
    pos_h = (pos >> 8) & 0xFF
    t_l = time_ms & 0xFF
    t_h = (time_ms >> 8) & 0xFF

    ID = servo_id & 0xFF
    LEN = 6 if len_override is None else int(len_override)
    CMD = 1

    pkt_wo_ck = bytes([0x55, 0x55, ID, LEN, CMD, pos_l, pos_h, t_l, t_h])
    ck = checksum(pkt_wo_ck)
    return pkt_wo_ck + bytes([ck])


def write64(fd: int, payload: bytes):
    if len(payload) != 64:
        raise ValueError(f"Need exactly 64 bytes, got {len(payload)}")
    os.write(fd, payload)


def send_packet(fd: int, pkt: bytes, mode: str):
    """
    mode:
      - 'no_report_id': put packet at byte 0 (pad zeros after)
      - 'report_id_0':  byte0=0, packet starts at byte1
    """
    if mode == "no_report_id":
        if len(pkt) > 64:
            raise ValueError("Packet too long")
        report = pkt + bytes(64 - len(pkt))
        write64(fd, report)
        return report
    elif mode == "report_id_0":
        if len(pkt) > 63:
            raise ValueError("Packet too long for report_id_0")
        report = bytes([0]) + pkt + bytes(63 - len(pkt))
        write64(fd, report)
        return report
    else:
        raise ValueError("mode must be 'no_report_id' or 'report_id_0'")


def hexdump(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)


def main():
    dev = find_hiwo_hidraw()
    print(f"Using HID device: {dev} (VID={VID} PID={PID})")

    # Open non-blocking hidraw
    fd = os.open(dev, os.O_RDWR | os.O_NONBLOCK)

    try:
        # Choose send mode (many HID devices want report_id_0)
        mode = input("Send mode? [1]=no_report_id, [2]=report_id_0 (recommended): ").strip()
        mode = "no_report_id" if mode == "1" else "report_id_0"

        # LEN override test (some firmwares want LEN=7)
        len_override = input("LEN override? press Enter for default (6), or type 7: ").strip()
        len_override = None if len_override == "" else int(len_override)

        servo_id = int(input("Servo ID (e.g. 1): ").strip() or "1")

        print("\nType positions 0..1000. Example: 200, 500, 800. Type 'q' to quit.\n")
        while True:
            s = input("pos> ").strip().lower()
            if s in ("q", "quit", "exit"):
                break

            pos = int(s)
            time_ms = int(input("time_ms (e.g. 300): ").strip() or "300")

            pkt = build_move_time_write(servo_id, pos, time_ms, len_override=len_override)
            report = send_packet(fd, pkt, mode)

            print(f"Sent pkt ({len(pkt)} bytes): {hexdump(pkt)}")
            print(f"Sent report (64 bytes): {hexdump(report[:min(32, len(report))])} ...")

            # small delay so servo has time
            time.sleep(0.05)

    finally:
        os.close(fd)
        print("Closed.")


if __name__ == "__main__":
    main()
