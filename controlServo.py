#!/usr/bin/env python3
import time
import argparse
import hid

VID = 0x0483
PID = 0x5750

# ---------- HID helpers ----------

def open_hid_device():
    dev = hid.Device(vid=VID, pid=PID)
    # this hid package uses a property, not a method
    dev.nonblocking = True
    return dev

def _pad64(b: bytes) -> bytes:
    if len(b) > 64:
        raise ValueError(f"payload too long: {len(b)} > 64")
    return b + bytes(64 - len(b))

def hid_write64(dev, payload: bytes) -> None:
    """
    Many HID devices want a leading report-id byte (0x00), making 65 bytes total.
    Others want exactly 64 bytes. We'll try both.
    """
    pkt64 = _pad64(payload)

    # Try "report id + 64"
    try:
        dev.write(bytes([0x00]) + pkt64)
        return
    except TypeError:
        # Some wrappers want list[int]
        try:
            dev.write([0x00] + list(pkt64))
            return
        except Exception:
            pass
    except Exception:
        pass

    # Fallback: write 64 only
    try:
        dev.write(pkt64)
        return
    except TypeError:
        dev.write(list(pkt64))

def hid_read64(dev, timeout_ms: int = 120) -> bytes:
    """
    Nonblocking read loop with timeout. Returns b"" on timeout.
    """
    t0 = time.time()
    while (time.time() - t0) * 1000.0 < timeout_ms:
        data = dev.read(64)
        if data:
            # sometimes it comes as list[int]
            if isinstance(data, list):
                return bytes(data)
            return bytes(data)
        time.sleep(0.001)
    return b""

# ---------- LX-16A packet helpers ----------

def lx_checksum(core: bytes) -> int:
    # core = [ID, LEN, CMD, params...]
    return (~(sum(core) & 0xFF)) & 0xFF

def lx_packet(servo_id: int, cmd: int, params: bytes) -> bytes:
    # LEN = CMD(1) + params(N) + checksum(1) + ID/LEN already exist? (protocol defines LEN from CMD to checksum)
    # For LX-16A: packet = 55 55 ID LEN CMD params... checksum
    length = 3 + len(params)  # CMD + params + checksum + ??? (this matches known examples: move_time has LEN=7 for 4 params)
    core = bytes([servo_id & 0xFF, length & 0xFF, cmd & 0xFF]) + params
    chk = lx_checksum(core)
    return bytes([0x55, 0x55]) + core + bytes([chk])

# ---------- Commands you need ----------

def lx_move_time_write(dev, servo_id: int, position: int, move_time_ms: int) -> None:
    """
    CMD 1: Move in time.
    position: 0..1000
    time: 0..30000 ms typically
    """
    position = max(0, min(1000, int(position)))
    move_time_ms = max(0, min(30000, int(move_time_ms)))

    params = bytes([
        position & 0xFF, (position >> 8) & 0xFF,
        move_time_ms & 0xFF, (move_time_ms >> 8) & 0xFF
    ])
    pkt = lx_packet(servo_id, cmd=1, params=params)
    hid_write64(dev, pkt)

def lx_set_id(dev, old_id: int, new_id: int) -> None:
    """
    CMD 13 (0x0D): Set servo ID.
    IMPORTANT: connect ONLY ONE servo when running this.
    """
    new_id = int(new_id) & 0xFF
    pkt = lx_packet(old_id, cmd=13, params=bytes([new_id]))
    hid_write64(dev, pkt)

# ---------- CLI ----------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--move", action="store_true", help="send a move command")
    ap.add_argument("--id", type=int, default=2, help="servo id to command")
    ap.add_argument("--pos", type=int, default=500, help="0..1000")
    ap.add_argument("--time", type=int, default=800, help="move time ms")

    ap.add_argument("--set-id", action="store_true", help="set ID (ONLY ONE SERVO CONNECTED)")
    ap.add_argument("--old", type=int, default=2, help="old id")
    ap.add_argument("--new", type=int, default=1, help="new id")

    ap.add_argument("--test12", action="store_true", help="move id=1 then id=2 (quick check)")
    args = ap.parse_args()

    dev = open_hid_device()

    if args.set_id:
        print(f"Setting ID {args.old} -> {args.new}  (ONLY one servo should be connected)")
        lx_set_id(dev, args.old, args.new)
        print("Done. Power-cycle servo/controller now.")
        return

    if args.test12:
        print("Move ID=1 then ID=2")
        lx_move_time_write(dev, 1, 300, 600)
        time.sleep(1.0)
        lx_move_time_write(dev, 2, 700, 600)
        return

    if args.move:
        print(f"Move ID={args.id} pos={args.pos} time={args.time}ms")
        lx_move_time_write(dev, args.id, args.pos, args.time)
        return

    # default action
    print("No action selected. Examples:")
    print("  python3 controlServo.py --move --id 2 --pos 700 --time 800")
    print("  python3 controlServo.py --set-id --old 2 --new 1")

if __name__ == "__main__":
    main()

