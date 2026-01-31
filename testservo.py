import time
import hid

VID = 0x0483
PID = 0x5750

# ----------------------------
# LX-16A checksum
# ----------------------------
def lx_checksum(pkt):
    s = sum(pkt[2:]) & 0xFF
    return (~s) & 0xFF

# ----------------------------
# Build MOVE packet
# ----------------------------
def lx_move_packet(servo_id, pos, ms):
    pos = max(0, min(1000, int(pos)))
    ms  = max(0, min(30000, int(ms)))

    pkt = bytearray([
        0x55, 0x55,
        0x07,
        servo_id,
        0x01,
        pos & 0xFF, (pos >> 8) & 0xFF,
        ms & 0xFF,  (ms >> 8) & 0xFF
    ])
    pkt.append(lx_checksum(pkt))
    return pkt

# ----------------------------
# Write 64-byte HID report
# ----------------------------
def hid_write64(dev, payload):
    # Most STM32 HID firmwares expect report ID = 0
    report = bytes([0x00]) + payload
    report = report.ljust(64, b'\x00')
    dev.write(report)

def move_servo(dev, servo_id, pos, ms):
    pkt = lx_move_packet(servo_id, pos, ms)
    hid_write64(dev, pkt)

# ----------------------------
# MAIN
# ----------------------------
if __name__ == "__main__":
    dev = hid.device()
    dev.open(VID, PID)
    dev.set_nonblocking(1)

    print(f"Opened HID device VID={hex(VID)} PID={hex(PID)}")

    move_servo(dev, 1, 500, 800)
    time.sleep(1)

    move_servo(dev, 2, 300, 800)
    time.sleep(1)

    move_servo(dev, 2, 700, 800)
    time.sleep(1)

    dev.close()
    print("Done.")
