"""
tuner_node  (real-time parameter tuning GUI)
---------------------------------------------
Tkinter GUI with sliders for all controller_node parameters.
Uses ROS2 SetParameters service to push values live.

Run:
  source /opt/ros/humble/setup.bash
  python3 tuner_node.py
"""

import json
import os
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import SetParameters
from std_srvs.srv import Trigger

PARAMS_FILE = os.path.expanduser("~/cyberrunner_tuner_params.json")


# ── Parameter definitions ─────────────────────────────────────────────────────
# (name, min, max, default, step, is_bool)
PARAMS = [
    # PD gains
    ("kp_x",        0.0,  20.0,  0.3,  0.01,  False),
    ("kp_y",        0.0,  20.0,  0.3,  0.01,  False),
    ("kd_x",        0.0,  1.0,   0.08, 0.005, False),
    ("kd_y",        0.0,  1.0,   0.08, 0.005, False),

    # Output / path
    ("max_output",   10,   300,   80,   1,     False),
    ("arrival_px",   5.0,  150.0, 35.0, 1.0,   False),
    ("lookahead_px", 5.0,  150.0, 25.0, 1.0,   False),

    # Misc
    ("cmd_time_ms",       5,    200,   20,    1,      False),
    ("ff_gain",           0.0,  2.0,   0.0,   0.05,   False),
    ("invert_x",          0,    1,     0,     1,      True),
    ("invert_y",          0,    1,     1,     1,      True),

    # Kalman filter
    ("kalman_q_pos",      0.1,  50.0,  1.0,   0.1,    False),
    ("kalman_q_vel",      1.0,  500.0, 50.0,  1.0,    False),
    ("kalman_r_meas",     1.0,  200.0, 10.0,  1.0,    False),

    # Danger zones
    ("danger_zone_gain",   0.0,  100.0, 0.0,   1.0,   False),
    ("danger_zone_radius", 10.0, 200.0, 80.0,  5.0,   False),

    # Timing / latency
    ("waypoint_pause_s",   0.0,  2.0,   0.3,   0.05,  False),
    ("predict_latency_s",  0.0,  1.0,   0.05,  0.005, False),

    # Corner gain scheduling
    ("corner_kp_scale",     1.0,  5.0,  2.0,  0.1,   False),
    ("corner_kd_scale",     1.0,  8.0,  2.5,  0.1,   False),
    ("corner_angle_thresh", 5.0, 90.0, 25.0,  5.0,   False),
    ("corner_preview_px",  20.0, 300.0, 100.0, 5.0,   False),

    # Auto-start / settling
    ("auto_start",        0,    1,     0,    1,     True),
    ("settle_speed_px",   2.0,  100.0, 15.0, 1.0,   False),
    ("settle_frames",     3,    60,    10,   1,     False),
    ("settle_timeout_s",  1.0,  15.0,  5.0,  0.5,   False),

    # ILC
    ("ilc_enabled",       0,    1,     1,    1,     True),
    ("ilc_gain",          0.01, 0.5,   0.1,  0.01,  False),

    # Speed ceiling
    ("max_speed_px",     0.0,  500.0, 0.0,  5.0,   False),
    ("speed_brake_gain", 0.0,  5.0,   0.5,  0.05,  False),
]

GROUPS = {
    "── PD Gains ──":      ["kp_x", "kp_y", "kd_x", "kd_y"],
    "── Output / Path ──": ["max_output", "arrival_px", "lookahead_px"],
    "── Kalman ──":        ["kalman_q_pos", "kalman_q_vel", "kalman_r_meas"],
    "── Danger Zones ──":  ["danger_zone_gain", "danger_zone_radius"],
    "── Timing ──":        ["waypoint_pause_s", "predict_latency_s"],
    "── Corners ──":       ["corner_kp_scale", "corner_kd_scale",
                            "corner_angle_thresh", "corner_preview_px"],
    "── Auto-Start ──":    ["auto_start", "settle_speed_px",
                            "settle_frames", "settle_timeout_s"],
    "── ILC ──":           ["ilc_enabled", "ilc_gain"],
    "── Speed ──":         ["max_speed_px", "speed_brake_gain"],
    "── Misc ──":          ["cmd_time_ms", "ff_gain", "invert_x", "invert_y"],
}

INT_PARAMS  = {"max_output", "cmd_time_ms", "settle_frames"}


# ── ROS2 helpers ──────────────────────────────────────────────────────────────
def build_param_msg(name, value, is_bool):
    from rcl_interfaces.msg import Parameter as RclParam, ParameterValue

    p = RclParam()
    p.name = name

    pv = ParameterValue()
    if is_bool:
        pv.type = ParameterType.PARAMETER_BOOL
        pv.bool_value = bool(int(value))
    elif name in INT_PARAMS:
        pv.type = ParameterType.PARAMETER_INTEGER
        pv.integer_value = int(value)
    else:
        pv.type = ParameterType.PARAMETER_DOUBLE
        pv.double_value = float(value)

    p.value = pv
    return p


class TunerNode(Node):
    def __init__(self):
        super().__init__("tuner_node")
        self._client = self.create_client(
            SetParameters, "/controller_node/set_parameters"
        )
        self._svc_start      = self.create_client(Trigger, "/controller/start")
        self._svc_stop       = self.create_client(Trigger, "/controller/stop")
        self._svc_calibrate  = self.create_client(Trigger, "/controller/calibrate")
        self._svc_reset_ilc  = self.create_client(Trigger, "/controller/reset_ilc")
        self._svc_clear_dz   = self.create_client(Trigger, "/controller/clear_danger_zones")
        self._svc_draw       = self.create_client(Trigger, "/path/draw")
        self._svc_path_load  = self.create_client(Trigger, "/path/load")
        self._svc_path_save  = self.create_client(Trigger, "/path/save")
        self.get_logger().info("TunerNode ready — waiting for controller_node...")

    def call_trigger(self, client, label):
        if not client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warn(f"{label}: service not available",
                                   throttle_duration_sec=2.0)
            return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda f, l=label: self._on_trigger_done(f, l))

    def _on_trigger_done(self, future, label):
        try:
            res = future.result()
            if res is None:
                self.get_logger().warn(f"{label}: no response")
            elif not res.success:
                self.get_logger().warn(f"{label}: {res.message}")
            else:
                self.get_logger().info(f"{label}: {res.message}")
        except Exception as e:
            self.get_logger().error(f"{label} error: {e}")

    def set_param(self, name, value, is_bool):
        if not self._client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warn(
                "controller_node not available",
                throttle_duration_sec=2.0
            )
            return

        req = SetParameters.Request()
        req.parameters = [build_param_msg(name, value, is_bool)]
        future = self._client.call_async(req)
        future.add_done_callback(lambda f, n=name: self._on_done(f, n))

    def reset_all_params(self):
        """Send all parameters in a single batched SetParameters request."""
        if not self._client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warn(
                "controller_node not available",
                throttle_duration_sec=2.0
            )
            return

        req = SetParameters.Request()
        req.parameters = [
            build_param_msg(name, default, is_bool)
            for name, _, _, default, _, is_bool in PARAMS
        ]
        future = self._client.call_async(req)
        future.add_done_callback(lambda f: self._on_done(f, "reset_all"))

    def set_params_batch(self, params):
        """Send an arbitrary list of (name, value, is_bool) in one request."""
        if not self._client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warn(
                "controller_node not available",
                throttle_duration_sec=2.0
            )
            return
        req = SetParameters.Request()
        req.parameters = [
            build_param_msg(name, value, is_bool)
            for name, value, is_bool in params
        ]
        future = self._client.call_async(req)
        future.add_done_callback(lambda f: self._on_done(f, "load_all"))

    def _on_done(self, future, name):
        try:
            result = future.result()
            if result is None or len(result.results) == 0:
                self.get_logger().warn(f"No response when setting {name}")
                return

            if not result.results[0].successful:
                self.get_logger().warn(
                    "Failed to set " + name + ": " + result.results[0].reason
                )
        except Exception as e:
            self.get_logger().error("set_param error: " + str(e))


# ── GUI ───────────────────────────────────────────────────────────────────────
BG = "#1e1e2e"
BG2 = "#313244"
FG = "#cdd6f4"
FG_HEAD = "#89b4fa"
FG_VAL = "#a6e3a1"
FG_DIM = "#6c7086"
FONT = ("Consolas", 10)
FONT_B = ("Consolas", 10, "bold")


class TunerGUI:
    def __init__(self, node: TunerNode):
        self.node = node
        self.vars = {}
        self.value_labels = {}
        self.param_map = {p[0]: p for p in PARAMS}
        self.saved_vals = self._load_params()

        self.root = tk.Tk()
        self.root.title("Controller Tuner")
        self.root.resizable(True, True)
        self.root.minsize(420, 320)
        self.root.configure(bg=BG)

        self._build()

    def _build(self):
        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure(
            "TScale",
            background=BG,
            troughcolor=BG2,
            sliderlength=16,
            sliderrelief="flat",
        )
        style.configure("TLabel", background=BG, foreground=FG, font=FONT)
        style.configure("H.TLabel", background=BG, foreground=FG_HEAD, font=FONT_B)
        style.configure("V.TLabel", background=BG, foreground=FG_VAL, font=FONT)
        style.configure("TCheckbutton", background=BG, foreground=FG, font=FONT)

        canvas = tk.Canvas(self.root, bg=BG, highlightthickness=0)
        scroll = tk.Scrollbar(self.root, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=scroll.set)

        scroll.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)

        inner = tk.Frame(canvas, bg=BG, padx=14, pady=10)
        inner_id = canvas.create_window((0, 0), window=inner, anchor="nw")

        def _on_inner_configure(_e):
            canvas.configure(scrollregion=canvas.bbox("all"))

        def _on_canvas_configure(e):
            canvas.itemconfig(inner_id, width=e.width)

        inner.bind("<Configure>", _on_inner_configure)
        canvas.bind("<Configure>", _on_canvas_configure)
        canvas.bind_all(
            "<MouseWheel>",
            lambda e: canvas.yview_scroll(int(-1 * (e.delta / 120)), "units")
        )

        for group_label, names in GROUPS.items():
            ttk.Label(inner, text=group_label, style="H.TLabel").pack(
                anchor="w", pady=(12, 2)
            )
            for name in names:
                _, lo, hi, default, step, is_bool = self.param_map[name]
                initial = self.saved_vals.get(name, default)
                self._add_row(inner, name, lo, hi, initial, step, is_bool)

        ctrl_row = tk.Frame(inner, bg=BG)
        ctrl_row.pack(fill="x", pady=(14, 2))

        ttk.Label(ctrl_row, text="Controller:", style="TLabel").pack(side="left", padx=(0, 6))

        tk.Button(
            ctrl_row, text="▶ Start",
            bg="#40a02b", fg="#ffffff",
            relief="flat", font=FONT_B, cursor="hand2",
            command=lambda: self.node.call_trigger(self.node._svc_start, "start"),
        ).pack(side="left", padx=(0, 4))

        tk.Button(
            ctrl_row, text="■ Stop",
            bg="#d20f39", fg="#ffffff",
            relief="flat", font=FONT_B, cursor="hand2",
            command=lambda: self.node.call_trigger(self.node._svc_stop, "stop"),
        ).pack(side="left", padx=(0, 4))

        tk.Button(
            ctrl_row, text="Calibrate",
            bg=BG2, fg=FG,
            relief="flat", font=FONT, cursor="hand2",
            command=lambda: self.node.call_trigger(self.node._svc_calibrate, "calibrate"),
        ).pack(side="left", padx=(0, 4))

        tk.Button(
            ctrl_row, text="Reset ILC",
            bg=BG2, fg=FG,
            relief="flat", font=FONT, cursor="hand2",
            command=lambda: self.node.call_trigger(self.node._svc_reset_ilc, "reset_ilc"),
        ).pack(side="left", padx=(0, 4))

        tk.Button(
            ctrl_row, text="Clear DZ",
            bg=BG2, fg=FG,
            relief="flat", font=FONT, cursor="hand2",
            command=lambda: self.node.call_trigger(self.node._svc_clear_dz, "clear_dz"),
        ).pack(side="left")

        path_row = tk.Frame(inner, bg=BG)
        path_row.pack(fill="x", pady=(4, 2))

        ttk.Label(path_row, text="Path:      ", style="TLabel").pack(side="left", padx=(0, 6))

        tk.Button(
            path_row, text="✎ Draw",
            bg="#7287fd", fg="#ffffff",
            relief="flat", font=FONT_B, cursor="hand2",
            command=lambda: self.node.call_trigger(self.node._svc_draw, "draw"),
        ).pack(side="left", padx=(0, 4))

        tk.Button(
            path_row, text="Load",
            bg=BG2, fg=FG,
            relief="flat", font=FONT, cursor="hand2",
            command=lambda: self.node.call_trigger(self.node._svc_path_load, "path_load"),
        ).pack(side="left", padx=(0, 4))

        tk.Button(
            path_row, text="Save",
            bg=BG2, fg=FG,
            relief="flat", font=FONT, cursor="hand2",
            command=lambda: self.node.call_trigger(self.node._svc_path_save, "path_save"),
        ).pack(side="left")

        btn_row = tk.Frame(inner, bg=BG)
        btn_row.pack(fill="x", pady=(6, 4))

        tk.Button(
            btn_row,
            text="Load saved",
            bg=BG2,
            fg=FG,
            relief="flat",
            font=FONT,
            cursor="hand2",
            command=self._load_all,
        ).pack(side="left", padx=(0, 6))

        tk.Button(
            btn_row,
            text="Reset defaults",
            bg=BG2,
            fg=FG,
            relief="flat",
            font=FONT,
            cursor="hand2",
            command=self._reset_all,
        ).pack(side="left")

        tk.Label(
            btn_row,
            text="Changes apply instantly",
            bg=BG,
            fg=FG_DIM,
            font=("Consolas", 9),
        ).pack(side="right")

    @staticmethod
    def _load_params():
        try:
            with open(PARAMS_FILE) as f:
                return json.load(f)
        except (FileNotFoundError, json.JSONDecodeError):
            return {}

    def _save_params(self):
        data = {}
        for name, var in self.vars.items():
            data[name] = var.get()
        with open(PARAMS_FILE, "w") as f:
            json.dump(data, f, indent=2)

    def _add_row(self, parent, name, lo, hi, default, step, is_bool):
        row = tk.Frame(parent, bg=BG)
        row.pack(fill="x", pady=2)

        ttk.Label(row, text=name.ljust(22), style="TLabel", width=22).pack(side="left")

        if is_bool:
            var = tk.IntVar(value=int(default))
            self.vars[name] = var

            lbl_text = tk.StringVar(value="ON" if default else "OFF")
            var.trace_add(
                "write",
                lambda *a, v=var, l=lbl_text: l.set("ON" if v.get() else "OFF")
            )

            ttk.Checkbutton(
                row,
                variable=var,
                command=lambda n=name, v=var: (
                    self.node.set_param(n, v.get(), True),
                    self._save_params(),
                ),
            ).pack(side="left")

            ttk.Label(
                row,
                textvariable=lbl_text,
                style="V.TLabel",
                width=4,
            ).pack(side="left", padx=4)

        else:
            if name in INT_PARAMS:
                var = tk.IntVar(value=int(default))
            else:
                var = tk.DoubleVar(value=float(default))

            self.vars[name] = var

            val_lbl = ttk.Label(
                row,
                text=self._fmt(default),
                style="V.TLabel",
                width=8
            )
            val_lbl.pack(side="right")
            self.value_labels[name] = val_lbl

            scale = ttk.Scale(
                row,
                from_=lo,
                to=hi,
                orient="horizontal",
                variable=var,
                command=lambda raw, n=name, v=var, lbl=val_lbl, s=step:
                    self._on_slide(raw, n, v, lbl, s),
            )
            scale.pack(side="left", fill="x", expand=True)

    def _on_slide(self, raw_val, name, var, lbl, step):
        snapped = round(float(raw_val) / step) * step

        if name in INT_PARAMS:
            snapped = int(round(snapped))
            var.set(snapped)
        else:
            var.set(float(snapped))

        lbl.config(text=self._fmt(snapped))

        _, _, _, _, _, is_bool = self.param_map[name]
        self.node.set_param(name, snapped, is_bool)
        self._save_params()

    def _load_all(self):
        vals = self._load_params()
        if not vals:
            return
        batch = []
        for name, _, _, _, step, is_bool in PARAMS:
            if name not in vals:
                continue
            raw = vals[name]
            if is_bool:
                v = int(raw)
                self.vars[name].set(v)
            elif name in INT_PARAMS:
                v = int(raw)
                self.vars[name].set(v)
                if name in self.value_labels:
                    self.value_labels[name].config(text=self._fmt(v))
            else:
                v = float(round(float(raw) / step) * step)
                self.vars[name].set(v)
                if name in self.value_labels:
                    self.value_labels[name].config(text=self._fmt(v))
            batch.append((name, v, is_bool))
        self.node.set_params_batch(batch)

    def _reset_all(self):
        for name, _, _, default, _, is_bool in PARAMS:
            if is_bool:
                self.vars[name].set(int(default))
            elif name in INT_PARAMS:
                self.vars[name].set(int(default))
                if name in self.value_labels:
                    self.value_labels[name].config(text=self._fmt(default))
            else:
                self.vars[name].set(float(default))
                if name in self.value_labels:
                    self.value_labels[name].config(text=self._fmt(default))

        # Send all defaults in a single batched RPC instead of N individual calls
        self.node.reset_all_params()
        self._save_params()

    @staticmethod
    def _fmt(v):
        if isinstance(v, int):
            return str(v)
        s = f"{float(v):.3f}"
        return s.rstrip("0").rstrip(".") if "." in s else s

    def run(self):
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.mainloop()

    def _on_close(self):
        self.root.destroy()


# ── Entry point ───────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = TunerNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    gui = TunerGUI(node)
    gui.run()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()