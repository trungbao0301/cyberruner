"""
calibration_node
----------------
Interactive calibration tool. Runs three automated tests:

  Test 4 — Marble detection noise  → suggests kalman_r_meas
  Test 5 — Pixel/mm scale check    → verifies estimator calibration
  Test 6 — Max ball speed          → suggests kalman_q_vel bounds

Usage:
  ros2 run cyberrunner_vision calibration_node

Prerequisites (must be running):
  camera_node, estimator_node (8 points clicked), marble_node
"""

import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray


class CalibrationNode(Node):
    def __init__(self):
        super().__init__("calibration_node")
        self.marble_samples = []
        self.est_state = None
        self.collecting = False

        self.sub_marble = self.create_subscription(
            Point, "/marble/position", self._on_marble, 2)
        self.sub_state = self.create_subscription(
            Float32MultiArray, "/estimator/state",
            lambda m: setattr(self, "est_state", list(m.data)), 2)

    def _on_marble(self, msg):
        if self.collecting and msg.z >= 0:
            self.marble_samples.append((time.time(), float(msg.x), float(msg.y)))

    # ── Test 4 ────────────────────────────────────────────────────────────────
    def run_test_4(self):
        print("\n" + "=" * 60)
        print("TEST 4 — Marble Detection Noise  →  kalman_r_meas")
        print("=" * 60)
        print("Place the marble on the board and hold it PERFECTLY STILL.")
        print("The board must be level and the marble must not move.")
        input("Press ENTER when ready...")

        print("Collecting 200 samples (~7 seconds)...")
        self.marble_samples.clear()
        self.collecting = True

        deadline = time.time() + 15.0
        while len(self.marble_samples) < 200 and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            print(f"\r  Samples: {len(self.marble_samples)}/200", end="", flush=True)

        self.collecting = False
        print()

        if len(self.marble_samples) < 50:
            print(f"ERROR: Only {len(self.marble_samples)} samples received.")
            print("       Is marble_node running and detecting the marble?")
            return None

        xs = np.array([s[1] for s in self.marble_samples])
        ys = np.array([s[2] for s in self.marble_samples])

        var_x = float(np.var(xs))
        var_y = float(np.var(ys))
        std_x = float(np.std(xs))
        std_y = float(np.std(ys))
        r_meas = float((var_x + var_y) / 2.0)

        print(f"\nResults ({len(self.marble_samples)} samples):")
        print(f"  X  std={std_x:.2f} px   var={var_x:.2f} px²")
        print(f"  Y  std={std_y:.2f} px   var={var_y:.2f} px²")
        print()
        print(f"  >> Recommended  kalman_r_meas = {r_meas:.1f}")
        print(f"     Conservative (smoother):    {r_meas * 2.0:.1f}")
        print(f"     Aggressive   (faster):      {r_meas * 0.5:.1f}")
        return r_meas

    # ── Test 5 ────────────────────────────────────────────────────────────────
    def run_test_5(self):
        print("\n" + "=" * 60)
        print("TEST 5 — Pixel/mm Scale Verification")
        print("=" * 60)
        print("Ensure estimator_node is running and all 8 points are clicked.")
        input("Press ENTER when ready...")

        print("Waiting for estimator state...")
        deadline = time.time() + 5.0
        while self.est_state is None and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.est_state is None:
            print("ERROR: No /estimator/state received.")
            print("       Is estimator_node running?")
            return None

        board_valid  = self.est_state[5]
        scale_px_mm  = self.est_state[2]   # px/mm
        tilt_x       = self.est_state[3]
        tilt_y       = self.est_state[4]
        topdown_size = 1000.0              # default TOP param

        print(f"\nRaw state[2] (scale):  {scale_px_mm:.4f} px/mm")
        print(f"board_valid:           {board_valid:.1f}  (1.0 = fully calibrated)")

        if board_valid < 0.5:
            print("WARNING: Board not fully calibrated — click all 8 points first.")

        if scale_px_mm > 0:
            back_w_mm = topdown_size / scale_px_mm
            expected  = 320.0
            diff_mm   = abs(back_w_mm - expected)

            print(f"\nBack-calculated board width: {back_w_mm:.1f} mm")
            print(f"Expected (board_width_mm):   {expected:.1f} mm")
            print(f"Difference:                  {diff_mm:.1f} mm")

            if diff_mm < 5.0:
                print("  >> GOOD — scale is accurate")
            elif diff_mm < 15.0:
                print("  >> ACCEPTABLE — small discrepancy, re-check ruler measurement")
            else:
                print("  >> BAD — large error, update board_width_mm param or re-calibrate")
        else:
            print("WARNING: scale is 0 — calibration not complete.")

        print(f"\nCurrent board tilt:")
        print(f"  Tilt X: {tilt_x:.2f} deg")
        print(f"  Tilt Y: {tilt_y:.2f} deg")

        if abs(tilt_x) < 1.0 and abs(tilt_y) < 1.0:
            print("  >> Board is level — good baseline for Test 4")
        else:
            print("  >> Board is tilted — level it before running Test 4")

        return scale_px_mm

    # ── Test 6 ────────────────────────────────────────────────────────────────
    def run_test_6(self):
        print("\n" + "=" * 60)
        print("TEST 6 — Maximum Ball Speed  →  kalman_q_vel bounds")
        print("=" * 60)
        print("Roll the marble freely around the board for 15 seconds.")
        print("Include fast straight runs, sharp corners, and flicks.")
        input("Press ENTER to start recording...")

        print("Recording for 15 seconds — move the marble now!")
        self.marble_samples.clear()
        self.collecting = True

        start = time.time()
        while time.time() - start < 15.0:
            rclpy.spin_once(self, timeout_sec=0.05)
            remaining = int(15 - (time.time() - start))
            print(f"\r  {remaining:2d}s remaining   samples: {len(self.marble_samples)}   ",
                  end="", flush=True)

        self.collecting = False
        print()

        if len(self.marble_samples) < 20:
            print("ERROR: Not enough samples. Is marble_node running?")
            return None

        ts  = np.array([s[0] for s in self.marble_samples])
        xs  = np.array([s[1] for s in self.marble_samples])
        ys  = np.array([s[2] for s in self.marble_samples])
        # Use actual inter-sample deltas — avoids errors when marble goes lost
        # mid-test and creates large gaps that inflate uniform-dt estimates.
        dt_arr = np.diff(ts)
        dt_arr = np.where(dt_arr > 1e-6, dt_arr, 1e-3)  # guard zero/negative
        avg_dt = float(np.mean(dt_arr))

        vx     = np.diff(xs) / dt_arr
        vy     = np.diff(ys) / dt_arr
        speeds = np.sqrt(vx ** 2 + vy ** 2)

        max_speed = float(np.max(speeds))
        p95_speed = float(np.percentile(speeds, 95))
        p50_speed = float(np.percentile(speeds, 50))

        print(f"\nResults ({len(self.marble_samples)} samples, avg dt={avg_dt*1000:.1f} ms):")
        print(f"  Median speed:   {p50_speed:.1f} px/s")
        print(f"  95th pct speed: {p95_speed:.1f} px/s")
        print(f"  Max speed:      {max_speed:.1f} px/s")
        print()
        print(f"  >> Recommended  kalman_q_vel = {p95_speed * 0.5:.1f}  (start here)")
        print(f"     If Kalman lags fast moves, raise toward {p95_speed:.1f}")
        print(f"     If Kalman is jittery at rest, lower toward {p50_speed * 0.2:.1f}")

        return max_speed, p95_speed


# ── Entry point ───────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()

    print()
    print("=" * 60)
    print("  CyberRunner Calibration Tool")
    print("=" * 60)
    print("Required nodes (start before running tests):")
    print("  ros2 run cyberrunner_vision camera_node")
    print("  ros2 run cyberrunner_vision estimator_node")
    print("  ros2 run cyberrunner_vision marble_node")
    print()
    print("Tests:")
    print("  4 — Marble detection noise  (ball held still)")
    print("  5 — Pixel/mm scale check    (estimator calibrated)")
    print("  6 — Max ball speed          (move ball freely for 15s)")
    print("  all — run 5 → 4 → 6 in order")
    print("  q — quit")
    print()

    while True:
        choice = input("Run test [4 / 5 / 6 / all / q]: ").strip().lower()
        if choice == 'q':
            break
        elif choice == '4':
            node.run_test_4()
        elif choice == '5':
            node.run_test_5()
        elif choice == '6':
            node.run_test_6()
        elif choice == 'all':
            node.run_test_5()
            node.run_test_4()
            node.run_test_6()
        else:
            print("  Enter 4, 5, 6, all, or q")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
