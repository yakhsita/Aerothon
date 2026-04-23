"""
Flight Controller GUI - MAVLink/Mission Planner Interface
Connects via MAVLink protocol (same as Mission Planner) to send
continuous Roll, Pitch, Yaw, and Throttle commands to a flight controller.

Requirements:
    pip install pymavlink

Usage:
    - Connect via USB/Serial or UDP (Mission Planner passthrough on UDP 14550)
    - Select connection type and port, then click Connect
    - Use sliders or keyboard to send continuous RC override commands
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import threading
import time
import queue
import sys

try:
    from pymavlink import mavutil
    MAVLINK_AVAILABLE = True
except ImportError:
    MAVLINK_AVAILABLE = False


# ── Constants ──────────────────────────────────────────────────────────────────
RC_MIN   = 1000
RC_MID   = 1500
RC_MAX   = 2000

CHANNEL_NAMES = [
    "Roll (CH1)", "Pitch (CH2)", "Throttle (CH3)", "Yaw (CH4)",
    "Aux 5 (CH5)", "Aux 6 (CH6)", "Aux 7 (CH7)", "Aux 8 (CH8)"
]

COLORS = {
    "bg":        "#1a1a2e",
    "panel":     "#16213e",
    "accent":    "#0f3460",
    "highlight": "#e94560",
    "green":     "#00b894",
    "yellow":    "#fdcb6e",
    "text":      "#eaeaea",
    "subtext":   "#a0a0b0",
    "slider_bg": "#0f3460",
}

CHANNEL_COLORS = [
    "#e94560", "#6c5ce7", "#00b894", "#fdcb6e",
    "#74b9ff", "#fd79a8", "#55efc4", "#b2bec3"
]


# ── MAVLink Worker Thread ──────────────────────────────────────────────────────
class MAVLinkWorker(threading.Thread):
    def __init__(self, connection_string, log_queue, status_callback):
        super().__init__(daemon=True)
        self.connection_string = connection_string
        self.log_queue = log_queue
        self.status_callback = status_callback
        self.mav = None
        self.connected = False
        self.running = True
        self.rc_values = [RC_MID] * 8
        self.rc_values[2] = RC_MIN          # throttle starts LOW
        self.send_rc = False
        self.send_interval = 0.05           # 20 Hz default
        self._lock = threading.Lock()
        self.telemetry = {}

    def log(self, msg):
        self.log_queue.put(msg)

    def run(self):
        self.log(f"Connecting to {self.connection_string} ...")
        try:
            self.mav = mavutil.mavlink_connection(
                self.connection_string, baud=57600)
            self.log("Waiting for heartbeat ...")
            hb = self.mav.wait_heartbeat(timeout=10)
            if hb is None:
                self.log("ERROR: No heartbeat received. Check connection.")
                self.status_callback("disconnected")
                return
            self.connected = True
            self.log(f"Heartbeat OK — System {self.mav.target_system}, "
                     f"Component {self.mav.target_component}")
            self.status_callback("connected")
            self._main_loop()
        except Exception as e:
            self.log(f"Connection error: {e}")
            self.status_callback("disconnected")

    def _main_loop(self):
        last_send = 0.0
        while self.running and self.connected:
            now = time.time()
            if self.send_rc and (now - last_send) >= self.send_interval:
                with self._lock:
                    vals = list(self.rc_values)
                try:
                    self.mav.mav.rc_channels_override_send(
                        self.mav.target_system,
                        self.mav.target_component,
                        *vals
                    )
                    last_send = now
                except Exception as e:
                    self.log(f"RC send error: {e}")

            try:
                msg = self.mav.recv_match(blocking=False)
                if msg:
                    t = msg.get_type()
                    if t == "ATTITUDE":
                        self.telemetry.update({
                            "roll_deg":  round(msg.roll  * 57.2958, 1),
                            "pitch_deg": round(msg.pitch * 57.2958, 1),
                            "yaw_deg":   round(msg.yaw   * 57.2958, 1),
                        })
                    elif t == "VFR_HUD":
                        self.telemetry.update({
                            "altitude": round(msg.alt, 1),
                            "airspeed": round(msg.airspeed, 1),
                            "throttle": msg.throttle,
                        })
                    elif t == "SYS_STATUS":
                        self.telemetry["battery_v"] = round(
                            msg.voltage_battery / 1000.0, 2)
                    elif t == "HEARTBEAT":
                        modes = {
                            0:"STABILIZE", 2:"ALT_HOLD", 3:"AUTO",
                            4:"GUIDED",    5:"LOITER",   6:"RTL",
                            9:"LAND"
                        }
                        self.telemetry["mode"] = modes.get(
                            msg.custom_mode, str(msg.custom_mode))
            except Exception:
                pass
            time.sleep(0.01)

    def set_rc(self, channel_index, value):
        with self._lock:
            self.rc_values[channel_index] = int(
                max(RC_MIN, min(RC_MAX, value)))

    def stop(self):
        self.running = False
        self.send_rc = False
        if self.mav:
            try:
                self.mav.mav.rc_channels_override_send(
                    self.mav.target_system, self.mav.target_component,
                    0, 0, 0, 0, 0, 0, 0, 0)
            except Exception:
                pass


# ── Main GUI ───────────────────────────────────────────────────────────────────
class FlightControllerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Flight Controller — MAVLink RC Override")
        self.root.configure(bg=COLORS["bg"])
        self.root.resizable(True, True)
        self.root.minsize(920, 720)

        self.worker = None
        self.log_queue = queue.Queue()
        self.slider_vars = []
        self.slider_labels = []
        self.armed = False

        self._build_ui()
        self._poll_logs()
        self._poll_telemetry()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._bind_keyboard()

    # ── Build UI ───────────────────────────────────────────────────────────────
    def _build_ui(self):
        # accent bar
        tk.Frame(self.root, bg=COLORS["highlight"], height=4).pack(fill="x")

        # title row
        tf = tk.Frame(self.root, bg=COLORS["bg"], pady=8)
        tf.pack(fill="x", padx=16)
        tk.Label(tf, text="✈  Flight Controller RC Override",
                 font=("Helvetica", 18, "bold"),
                 fg=COLORS["text"], bg=COLORS["bg"]).pack(side="left")
        self.status_dot = tk.Label(tf, text="●  Disconnected",
                                    font=("Helvetica", 11),
                                    fg=COLORS["highlight"], bg=COLORS["bg"])
        self.status_dot.pack(side="right", padx=8)

        # connection bar
        cf = tk.LabelFrame(self.root, text=" Connection ",
                            bg=COLORS["panel"], fg=COLORS["subtext"],
                            font=("Helvetica", 10), bd=1, relief="flat",
                            padx=10, pady=8)
        cf.pack(fill="x", padx=16, pady=(0, 8))

        tk.Label(cf, text="Type:", bg=COLORS["panel"],
                 fg=COLORS["text"]).grid(row=0, column=0, padx=4)
        self.conn_type = ttk.Combobox(cf, width=8,
                                       values=["UDP", "TCP", "Serial"],
                                       state="readonly")
        self.conn_type.set("UDP")
        self.conn_type.grid(row=0, column=1, padx=4)
        self.conn_type.bind("<<ComboboxSelected>>", self._on_conn_type)

        tk.Label(cf, text="Address:", bg=COLORS["panel"],
                 fg=COLORS["text"]).grid(row=0, column=2, padx=4)
        self.conn_addr = tk.Entry(cf, width=26,
                                   bg=COLORS["accent"], fg=COLORS["text"],
                                   insertbackground=COLORS["text"],
                                   bd=0, font=("Courier", 11))
        self.conn_addr.insert(0, "udpin:0.0.0.0:14550")
        self.conn_addr.grid(row=0, column=3, padx=4)

        tk.Label(cf, text="Rate (Hz):", bg=COLORS["panel"],
                 fg=COLORS["text"]).grid(row=0, column=4, padx=4)
        self.rate_var = tk.IntVar(value=20)
        ttk.Spinbox(cf, from_=1, to=50, textvariable=self.rate_var,
                    width=5).grid(row=0, column=5, padx=4)

        self.conn_btn = tk.Button(cf, text="Connect",
                                   bg=COLORS["green"], fg="white",
                                   font=("Helvetica", 10, "bold"),
                                   relief="flat", padx=12, pady=4,
                                   cursor="hand2",
                                   command=self._toggle_connection)
        self.conn_btn.grid(row=0, column=6, padx=10)

        # body
        body = tk.Frame(self.root, bg=COLORS["bg"])
        body.pack(fill="both", expand=True, padx=16, pady=4)
        body.columnconfigure(0, weight=3)
        body.columnconfigure(1, weight=1)
        body.rowconfigure(0, weight=1)

        # sliders
        sf = tk.LabelFrame(body, text=" RC Channel Override ",
                            bg=COLORS["panel"], fg=COLORS["subtext"],
                            font=("Helvetica", 10), bd=1, relief="flat",
                            padx=10, pady=8)
        sf.grid(row=0, column=0, sticky="nsew", padx=(0, 8))

        for i, name in enumerate(CHANNEL_NAMES):
            row = tk.Frame(sf, bg=COLORS["panel"])
            row.pack(fill="x", pady=3)

            tk.Label(row, text=name, width=14, anchor="w",
                      fg=CHANNEL_COLORS[i], bg=COLORS["panel"],
                      font=("Helvetica", 10, "bold")).pack(side="left", padx=4)

            default = RC_MIN if i == 2 else RC_MID
            var = tk.IntVar(value=default)
            self.slider_vars.append(var)

            tk.Scale(row, variable=var,
                      from_=RC_MIN, to=RC_MAX, orient="horizontal",
                      length=370, resolution=1,
                      bg=COLORS["slider_bg"], fg=COLORS["text"],
                      troughcolor=COLORS["accent"],
                      highlightthickness=0, bd=0,
                      activebackground=CHANNEL_COLORS[i],
                      command=lambda v, idx=i: self._on_slider(idx, v)
                      ).pack(side="left", padx=4)

            val_lbl = tk.Label(row, text=str(default), width=5,
                                fg=COLORS["text"], bg=COLORS["panel"],
                                font=("Courier", 10))
            val_lbl.pack(side="left", padx=2)
            self.slider_labels.append(val_lbl)

            tk.Button(row, text="CTR",
                       bg=COLORS["accent"], fg=COLORS["text"],
                       relief="flat", padx=6, font=("Helvetica", 8),
                       cursor="hand2",
                       command=lambda idx=i: self._centre_channel(idx)
                       ).pack(side="left", padx=2)
            tk.Button(row, text="MIN",
                       bg=COLORS["accent"], fg=COLORS["text"],
                       relief="flat", padx=6, font=("Helvetica", 8),
                       cursor="hand2",
                       command=lambda idx=i: self._min_channel(idx)
                       ).pack(side="left", padx=2)

        # action row
        ar = tk.Frame(sf, bg=COLORS["panel"])
        ar.pack(fill="x", pady=(10, 2))

        self.send_var = tk.BooleanVar(value=False)
        tk.Checkbutton(ar, text="  ● Send RC Override",
                        variable=self.send_var,
                        bg=COLORS["panel"], fg=COLORS["green"],
                        selectcolor=COLORS["panel"],
                        activeforeground=COLORS["green"],
                        activebackground=COLORS["panel"],
                        font=("Helvetica", 11, "bold"),
                        cursor="hand2",
                        command=self._toggle_send).pack(side="left", padx=8)

        tk.Button(ar, text="Centre All",
                   bg=COLORS["accent"], fg=COLORS["text"],
                   relief="flat", padx=10, pady=4,
                   cursor="hand2",
                   command=self._centre_all).pack(side="left", padx=6)

        self.arm_btn = tk.Button(ar, text="ARM",
                                  bg="#e17055", fg="white",
                                  font=("Helvetica", 10, "bold"),
                                  relief="flat", padx=14, pady=4,
                                  cursor="hand2",
                                  command=self._arm_disarm)
        self.arm_btn.pack(side="right", padx=8)

        hint = ("Keys:  W/S=Pitch   A/D=Roll   Q/E=Yaw   "
                "↑/↓=Throttle   Space=Centre All")
        tk.Label(sf, text=hint, fg=COLORS["subtext"], bg=COLORS["panel"],
                  font=("Helvetica", 9)).pack(anchor="w", padx=6, pady=(4, 0))

        # right panel
        right = tk.Frame(body, bg=COLORS["bg"])
        right.grid(row=0, column=1, sticky="nsew")

        telem_frame = tk.LabelFrame(right, text=" Live Telemetry ",
                                     bg=COLORS["panel"], fg=COLORS["subtext"],
                                     font=("Helvetica", 10),
                                     bd=1, relief="flat", padx=10, pady=8)
        telem_frame.pack(fill="x", pady=(0, 8))

        self.telem_labels = {}
        for key, label, unit in [
            ("roll_deg",  "Roll",     "°"),
            ("pitch_deg", "Pitch",    "°"),
            ("yaw_deg",   "Yaw",      "°"),
            ("altitude",  "Altitude", "m"),
            ("airspeed",  "Airspeed", "m/s"),
            ("throttle",  "Throttle", "%"),
            ("battery_v", "Battery",  "V"),
            ("mode",      "Mode",     ""),
        ]:
            r = tk.Frame(telem_frame, bg=COLORS["panel"])
            r.pack(fill="x", pady=1)
            tk.Label(r, text=label + ":", width=10, anchor="w",
                      fg=COLORS["subtext"], bg=COLORS["panel"],
                      font=("Helvetica", 9)).pack(side="left")
            v = tk.Label(r, text="—", anchor="w",
                          fg=COLORS["yellow"], bg=COLORS["panel"],
                          font=("Courier", 10, "bold"))
            v.pack(side="left")
            if unit:
                tk.Label(r, text=" " + unit, fg=COLORS["subtext"],
                          bg=COLORS["panel"],
                          font=("Helvetica", 9)).pack(side="left")
            self.telem_labels[key] = v

        log_frame = tk.LabelFrame(right, text=" Console ",
                                   bg=COLORS["panel"], fg=COLORS["subtext"],
                                   font=("Helvetica", 10), bd=1, relief="flat")
        log_frame.pack(fill="both", expand=True)
        self.log_box = scrolledtext.ScrolledText(
            log_frame, bg="#0a0a1a", fg="#00ff88",
            font=("Courier", 9), bd=0, wrap="word",
            state="disabled", height=14)
        self.log_box.pack(fill="both", expand=True, padx=4, pady=4)

    # ── Logic ──────────────────────────────────────────────────────────────────
    def _log(self, msg):
        self.log_box.configure(state="normal")
        self.log_box.insert("end", f"[{time.strftime('%H:%M:%S')}] {msg}\n")
        self.log_box.see("end")
        self.log_box.configure(state="disabled")

    def _on_slider(self, idx, value):
        self.slider_labels[idx].config(text=str(int(float(value))))
        if self.worker and self.worker.connected:
            self.worker.set_rc(idx, int(float(value)))

    def _centre_channel(self, idx):
        self.slider_vars[idx].set(RC_MID)
        self._on_slider(idx, RC_MID)

    def _min_channel(self, idx):
        self.slider_vars[idx].set(RC_MIN)
        self._on_slider(idx, RC_MIN)

    def _centre_all(self):
        for i in range(8):
            v = RC_MIN if i == 2 else RC_MID
            self.slider_vars[i].set(v)
            self._on_slider(i, v)

    def _toggle_send(self):
        if not (self.worker and self.worker.connected):
            self.send_var.set(False)
            messagebox.showwarning("Not Connected",
                                    "Connect to the flight controller first.")
            return
        self.worker.send_rc = self.send_var.get()
        self._log("RC Override: " + ("ACTIVE" if self.worker.send_rc else "STOPPED"))

    def _arm_disarm(self):
        if not (self.worker and self.worker.connected):
            messagebox.showwarning("Not Connected", "Connect first.")
            return
        self.armed = not self.armed
        cmd = 1 if self.armed else 0
        try:
            self.worker.mav.mav.command_long_send(
                self.worker.mav.target_system,
                self.worker.mav.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, cmd, 0, 0, 0, 0, 0, 0)
            self.arm_btn.config(
                text="DISARM" if self.armed else "ARM",
                bg=COLORS["green"] if self.armed else "#e17055")
            self._log("ARM command sent." if self.armed else "DISARM command sent.")
        except Exception as e:
            self._log(f"Arm error: {e}")
            self.armed = not self.armed

    def _on_conn_type(self, _=None):
        presets = {
            "UDP":    "udpin:0.0.0.0:14550",
            "TCP":    "tcp:127.0.0.1:5760",
            "Serial": "COM3" if sys.platform == "win32" else "/dev/ttyUSB0",
        }
        self.conn_addr.delete(0, "end")
        self.conn_addr.insert(0, presets.get(self.conn_type.get(), ""))

    def _toggle_connection(self):
        if self.worker and self.worker.connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        if not MAVLINK_AVAILABLE:
            messagebox.showerror("Missing library",
                                  "pymavlink not installed.\n"
                                  "Run:  pip install pymavlink")
            return
        addr = self.conn_addr.get().strip()
        if not addr:
            messagebox.showerror("Error", "Enter a connection address.")
            return
        self.conn_btn.config(text="Connecting…", state="disabled",
                              bg=COLORS["yellow"])
        self.worker = MAVLinkWorker(addr, self.log_queue, self._status_callback)
        self.worker.send_interval = 1.0 / max(1, self.rate_var.get())
        self.worker.start()

    def _disconnect(self):
        if self.worker:
            self.worker.stop()
            self.worker = None
        self.send_var.set(False)
        self.armed = False
        self.arm_btn.config(text="ARM", bg="#e17055")
        self._status_callback("disconnected")
        self._log("Disconnected.")

    def _status_callback(self, status):
        self.root.after(0, self._apply_status, status)

    def _apply_status(self, status):
        if status == "connected":
            self.status_dot.config(text="●  Connected", fg=COLORS["green"])
            self.conn_btn.config(text="Disconnect", state="normal",
                                  bg=COLORS["highlight"])
            self._log("Connected successfully.")
        else:
            self.status_dot.config(text="●  Disconnected",
                                    fg=COLORS["highlight"])
            self.conn_btn.config(text="Connect", state="normal",
                                  bg=COLORS["green"])

    def _poll_logs(self):
        try:
            while True:
                self._log(self.log_queue.get_nowait())
        except queue.Empty:
            pass
        self.root.after(100, self._poll_logs)

    def _poll_telemetry(self):
        if self.worker and self.worker.connected:
            for key, lbl in self.telem_labels.items():
                lbl.config(text=str(self.worker.telemetry.get(key, "—")))
        self.root.after(200, self._poll_telemetry)

    # ── Keyboard ───────────────────────────────────────────────────────────────
    def _bind_keyboard(self):
        step = 20
        self.root.bind("<w>",     lambda e: self._nudge(1, -step))
        self.root.bind("<s>",     lambda e: self._nudge(1,  step))
        self.root.bind("<a>",     lambda e: self._nudge(0, -step))
        self.root.bind("<d>",     lambda e: self._nudge(0,  step))
        self.root.bind("<q>",     lambda e: self._nudge(3, -step))
        self.root.bind("<e>",     lambda e: self._nudge(3,  step))
        self.root.bind("<Up>",    lambda e: self._nudge(2,  step))
        self.root.bind("<Down>",  lambda e: self._nudge(2, -step))
        self.root.bind("<space>", lambda e: self._centre_all())

    def _nudge(self, ch, delta):
        new_val = max(RC_MIN, min(RC_MAX, self.slider_vars[ch].get() + delta))
        self.slider_vars[ch].set(new_val)
        self._on_slider(ch, new_val)

    def _on_close(self):
        self._disconnect()
        self.root.destroy()


# ── Entry ──────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    if not MAVLINK_AVAILABLE:
        print("WARNING: pymavlink not installed.")
        print("Run:  pip install pymavlink\n")

    root = tk.Tk()
    style = ttk.Style(root)
    try:
        style.theme_use("clam")
    except Exception:
        pass
    style.configure("TCombobox",
                     fieldbackground=COLORS["accent"],
                     background=COLORS["accent"],
                     foreground=COLORS["text"])
    style.configure("TSpinbox",
                     fieldbackground=COLORS["accent"],
                     foreground=COLORS["text"])

    FlightControllerGUI(root)
    root.mainloop()
