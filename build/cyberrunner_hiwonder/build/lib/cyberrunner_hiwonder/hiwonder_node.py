import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from std_srvs.srv import Trigger

import hid  # pip package "hid" (works for your board)

VID = 0x0483
PID = 0x5750


def _u16(v: int):
    v &= 0xFFFF
    return v & 0xFF, (v >> 8) & 0xFF


class HiwonderHID:
    """
    Hiwonder USB-HID bus control board using the 'hid' python package.

    Payload (<=64):
      55 55 08 03 01 timeL timeH id posL posH

    Write style:
      Try 65 bytes: [0x00] + 64-byte payload (report-id style)
      Fallback to 64 bytes if needed
    """

    def __init__(self, vid=VID, pid=PID):
        self.vid = vid
        self.pid = pid
        self.dev = hid.Device(vid=vid, pid=pid)
        self.dev.nonblocking = True

        # Cache which write method works so we don't "try both" every time
        self._use_65 = None  # None unknown, True use 65, False use 64

    def close(self):
        try:
            self.dev.close()
        except Exception:
            pass

    def _write64(self, payload: bytes):
        if len(payload) > 64:
            raise ValueError("payload too long (>64)")

        pkt64 = payload + bytes(64 - len(payload))

        # If we already discovered the correct mode, use it directly
        if self._use_65 is True:
            self.dev.write(bytes([0x00]) + pkt64)
            return
        if self._use_65 is False:
            self.dev.write(pkt64)
            return

        # Otherwise auto-detect on first write
        try:
            self.dev.write(bytes([0x00]) + pkt64)
            self._use_65 = True
        except Exception:
            self.dev.write(pkt64)
            self._use_65 = False

    def move(self, servo_id: int, pos: int, time_ms: int = 300):
        pos = max(0, min(1000, int(pos)))
        time_ms = max(0, min(30000, int(time_ms)))

        tL, tH = _u16(time_ms)
        pL, pH = _u16(pos)

        pkt = bytes([0x55, 0x55, 0x08, 0x03, 0x01, tL, tH, servo_id & 0xFF, pL, pH])
        self._write64(pkt)


class HiwonderNode(Node):
    def __init__(self):
        super().__init__("cyberrunner_hiwonder")

        self.declare_parameter("servo1_id", 1)
        self.declare_parameter("servo2_id", 2)
        self.declare_parameter("home_pos", 500)

        self.servo1_id = int(self.get_parameter("servo1_id").value)
        self.servo2_id = int(self.get_parameter("servo2_id").value)
        self.home_pos = int(self.get_parameter("home_pos").value)

        self.hid = HiwonderHID()
        self.get_logger().info(f"Opened Hiwonder HID {VID:04x}:{PID:04x} using python 'hid'")

        self.sub = self.create_subscription(Int32MultiArray, "/hiwonder/cmd", self.on_cmd, 10)
        self.srv = self.create_service(Trigger, "/hiwonder/reset", self.on_reset)

        time.sleep(0.1)
        self.go_home(600)

    def go_home(self, time_ms: int = 300):
        self.hid.move(self.servo1_id, self.home_pos, time_ms)
        self.hid.move(self.servo2_id, self.home_pos, time_ms)

    def on_cmd(self, msg: Int32MultiArray):
        data = list(msg.data)
        if len(data) < 2:
            self.get_logger().warn("Expected Int32MultiArray: [pos1, pos2, time_ms(optional)]")
            return

        pos1 = int(data[0])
        pos2 = int(data[1])
        time_ms = int(data[2]) if len(data) >= 3 else 80

        # Helpful log so you can see it is receiving commands
        self.get_logger().info(
            f"CMD: id1={self.servo1_id} pos1={pos1}  id2={self.servo2_id} pos2={pos2}  t={time_ms}ms"
        )

        self.hid.move(self.servo1_id, pos1, time_ms)
        self.hid.move(self.servo2_id, pos2, time_ms)

    def on_reset(self, request, response):
        self.go_home(600)
        response.success = True
        response.message = "Reset -> moved to home position"
        return response


def main():
    rclpy.init()
    node = HiwonderNode()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.hid.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

