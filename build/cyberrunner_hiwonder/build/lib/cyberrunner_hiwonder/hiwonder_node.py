import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from std_srvs.srv import Trigger

import hid

VID = 0x0483
PID = 0x5750


def _u16(v: int):
    v &= 0xFFFF
    return v & 0xFF, (v >> 8) & 0xFF


class HiwonderHID:
    def __init__(self, vid=VID, pid=PID):
        self.vid = vid
        self.pid = pid
        self.dev = None
        self._use_65 = None
        self._connect()

    def _connect(self):
        try:
            self.dev = hid.Device(vid=self.vid, pid=self.pid)
            self.dev.nonblocking = True
            self._use_65 = None
            print("[HID] Connected to Hiwonder board")
            return True
        except Exception as e:
            print(f"[HID] Connection failed: {e}")
            self.dev = None
            return False

    def close(self):
        try:
            if self.dev:
                self.dev.close()
        except Exception:
            pass

    def _write64(self, payload: bytes):
        if len(payload) > 64:
            raise ValueError("payload too long (>64)")
        pkt64 = payload + bytes(64 - len(payload))

        if self._use_65 is True:
            self.dev.write(bytes([0x00]) + pkt64)
            return
        if self._use_65 is False:
            self.dev.write(pkt64)
            return

        try:
            self.dev.write(bytes([0x00]) + pkt64)
            self._use_65 = True
        except Exception:
            self.dev.write(pkt64)
            self._use_65 = False

    def move(self, servo_id: int, pos: int, time_ms: int = 300):
        if self.dev is None:
            return False
        pos     = max(0,   min(1000,  int(pos)))
        time_ms = max(0,   min(30000, int(time_ms)))
        tL, tH  = _u16(time_ms)
        pL, pH  = _u16(pos)
        pkt = bytes([0x55, 0x55, 0x08, 0x03, 0x01,
                     tL, tH, servo_id & 0xFF, pL, pH])
        try:
            self._write64(pkt)
            return True
        except Exception as e:
            print(f"[HID] Write error: {e} — will reconnect")
            self.dev = None
            return False


class HiwonderNode(Node):
    def __init__(self):
        super().__init__("cyberrunner_hiwonder")

        self.declare_parameter("servo1_id", 1)
        self.declare_parameter("servo2_id", 2)
        self.declare_parameter("home_pos",  500)
        self.declare_parameter("reconnect_interval", 2.0)  # seconds

        self.servo1_id = int(self.get_parameter("servo1_id").value)
        self.servo2_id = int(self.get_parameter("servo2_id").value)
        self.home_pos  = int(self.get_parameter("home_pos").value)
        self.reconnect_interval = float(
            self.get_parameter("reconnect_interval").value)

        self.hid = HiwonderHID()

        if self.hid.dev:
            self.get_logger().info(
                f"Opened Hiwonder HID {VID:04x}:{PID:04x}")
            time.sleep(0.1)
            self.go_home(600)
        else:
            self.get_logger().error(
                "Could not open Hiwonder HID — will retry every " +
                str(self.reconnect_interval) + "s")

        self.sub = self.create_subscription(
            Int32MultiArray, "/hiwonder/cmd", self.on_cmd, 10)
        self.srv = self.create_service(
            Trigger, "/hiwonder/reset", self.on_reset)

        # Reconnect timer
        self.create_timer(self.reconnect_interval, self._check_connection)

    def _check_connection(self):
        """Auto-reconnect if HID device was lost."""
        if self.hid.dev is None:
            self.get_logger().warn(
                "HID disconnected — attempting reconnect...")
            if self.hid._connect():
                self.get_logger().info("HID reconnected!")
                time.sleep(0.1)
                self.go_home(600)
            else:
                self.get_logger().warn(
                    "Reconnect failed — check USB cable")

    def go_home(self, time_ms: int = 300):
        self.hid.move(self.servo1_id, self.home_pos, time_ms)
        self.hid.move(self.servo2_id, self.home_pos, time_ms)

    def on_cmd(self, msg: Int32MultiArray):
        data = list(msg.data)
        if len(data) < 2:
            self.get_logger().warn(
                "Expected [pos1, pos2, time_ms(optional)]")
            return

        # Silently drop if disconnected (reconnect timer will fix it)
        if self.hid.dev is None:
            return

        pos1    = int(data[0])
        pos2    = int(data[1])
        time_ms = int(data[2]) if len(data) >= 3 else 80

        self.get_logger().info(
            f"CMD: id1={self.servo1_id} pos1={pos1}  "
            f"id2={self.servo2_id} pos2={pos2}  t={time_ms}ms",
            throttle_duration_sec=0.5)

        ok1 = self.hid.move(self.servo1_id, pos1, time_ms)
        ok2 = self.hid.move(self.servo2_id, pos2, time_ms)

        # If write failed, mark as disconnected
        if not ok1 or not ok2:
            self.hid.dev = None

    def on_reset(self, request, response):
        self.go_home(600)
        response.success = True
        response.message = "Reset -> home position"
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