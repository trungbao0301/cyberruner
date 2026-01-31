#!/usr/bin/env python3
import time
import hid

VID = 0x0483
PID = 0x5750

def itos_u16(v: int):
    v = int(v) & 0xFFFF
    return v & 0xFF, (v >> 8) & 0xFF

def pick_device_path():
    devs = hid.enumerate(VID, PID)
    if not devs:
        raise RuntimeError(f"No HID device found for VID:PID = {VID:04x}:{PID:04x}")
    print("Found devices:")
    for i, d in enumerate(devs):
        print(f" [{i}] path={d['path']} man={d.get('manufacturer_string','')} prod={d.get('product_string','')} serial={d.get('serial_number','')}")
    return devs[0]["path"]

def write_report(dev: hid.device, payload: bytes):
    """
    HIDAPI expects: first byte = Report ID.
    For these boards, Report ID is 0x00, and report length is 64 bytes.
    So we send 65 bytes: [0x00] + 64-byte report.
    """
    if len(payload) > 64:
        raise ValueError("payload too long; must be <= 64 bytes")
    report64 = payload + bytes([0] * (64 - len(payload)))
    out = bytes([0x00]) + report64
    n = dev.write(out)
    return n

def read_report(dev: hid.device, timeout_ms=200):
    # Reads up to 64 bytes (no report-id byte returned here)
    return dev.read(64, timeout_ms)

def cmd_move_one(servo_id: int, pos: int, time_ms: int):
    # 55 55 len=8 cmd=0x03 count=1 [timeL timeH id posL posH]
    tL, tH = itos_u16(time_ms)
    pL, pH = itos_u16(pos)
    return bytes([0x55, 0x55, 0x08, 0x03, 0x01, tL, tH, servo_id & 0xFF, pL, pH])

def cmd_servos_off(ids):
    # From common xArm HID examples: 55 55 len=9 cmd=20 count=6 ids...
    ids = list(ids)
    return bytes([0x55, 0x55, 0x03 + len(ids), 20, len(ids)] + ids)

def main():
    path = pick_device_path()

    dev = hid.device()
    dev.open_path(path)
    print(f"\nOpened HID path={path}\n")

    # Some boards need nonblocking off for reads to work as expected
    dev.set_nonblocking(0) ss

    servo_id = int(input("Servo ID (default 1): ") or "1")
    print("\nType target positions 0..1000. 'off' to torque-off. 'q' to quit.\n")

    while True:
        s = input("pos> ").strip().lower()
        if s in ("q", "quit", "exit"):
            break
        if s == "off":
            pkt = cmd_servos_off([servo_id])
            n = write_report(dev, pkt)
            print(f"[TX] servos_off bytes={n}  pkt={pkt.hex(' ')}")
            continue

        pos = int(s)
        time_ms = int(input("time_ms (default 300): ") or "300")

        pkt = cmd_move_one(servo_id, pos, time_ms)
        n = write_report(dev, pkt)
        print(f"[TX] move bytes={n}  pkt={pkt.hex(' ')}")

        # optional readback (board may or may not reply immediately)
        rx = read_report(dev, timeout_ms=200)
        if rx:
            print("[RX]", " ".join(f"{b:02X}" for b in rx))
        else:
            print("[RX] (no data)")

        time.sleep(0.05)

    dev.close()
    print("Closed.")

if __name__ == "__main__":
    main()

