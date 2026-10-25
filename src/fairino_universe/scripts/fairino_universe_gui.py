#!/usr/bin/env python3
"""
fairino_universe_gui.py

Small Tkinter launcher for `ros2 launch fairino_universe fairino_universe.launch.py`.

Lets you pick (partial configuration):
  - robot_model        (fairino3 / 5 / 10 / 16 / 20 / 30)
  - gripper            (none, dh_ag95, or type another name)
  - robot_ip_address   (validated IPv4)

When the IP differs from the one the hardware plugin was last built with,
`rebuild_plugin:=true` is passed automatically (you can override the checkbox).

Settings are saved to ~/.config/fairino_universe_gui/config.json.
Launch output is streamed into the window; Stop sends Ctrl+C (SIGINT) to the
whole launch, like pressing Ctrl+C in a terminal.

Requires: python3-tk  (sudo apt install -y python3-tk)
"""
import ipaddress
import json
import os
import queue
import re
import shlex
import signal
import subprocess
import threading
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

# --------------------------------------------------------------------------
# Constants (match `ros2 launch fairino_universe fairino_universe.launch.py --show-args`)
# --------------------------------------------------------------------------
PACKAGE = "fairino_universe"
LAUNCH_FILE = "fairino_universe.launch.py"
ROS_SETUP = "/opt/ros/humble/setup.bash"

ROBOT_MODELS = ["fairino3", "fairino5", "fairino10", "fairino16", "fairino20", "fairino30"]
GRIPPERS = ["none", "dh_ag95"]

CONFIG_PATH = Path.home() / ".config" / "fairino_universe_gui" / "config.json"
DEFAULTS = {
    "robot_model": "fairino5",
    "gripper": "none",
    "robot_ip_address": "192.168.58.2",
    "workspace": str(Path.home() / "github" / "fairino-universe"),
    # IP the hardware plugin was last built with (CONTROLLER_IP macro patch)
    "last_built_ip": "192.168.58.2",
}

MAX_LOG_LINES = 5000
ANSI_RE = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")


# --------------------------------------------------------------------------
# Helpers (no Tk dependency, easy to test)
# --------------------------------------------------------------------------
def load_config() -> dict:
    cfg = dict(DEFAULTS)
    try:
        with open(CONFIG_PATH) as f:
            cfg.update({k: v for k, v in json.load(f).items() if k in DEFAULTS})
    except (OSError, ValueError):
        pass
    return cfg


def save_config(cfg: dict) -> None:
    CONFIG_PATH.parent.mkdir(parents=True, exist_ok=True)
    with open(CONFIG_PATH, "w") as f:
        json.dump(cfg, f, indent=2)


def validate_ip(text: str):
    """Return None if `text` is a usable IPv4 address, else an error message."""
    text = text.strip()
    if not text:
        return "IP address is empty"
    try:
        ip = ipaddress.IPv4Address(text)
    except ipaddress.AddressValueError:
        return "Not a valid IPv4 address"
    if ip.is_unspecified or ip.is_multicast or ip.is_loopback:
        return "Not a usable controller address"
    return None


def build_launch_args(robot_model: str, gripper: str, ip: str, rebuild: bool) -> list:
    return [
        f"robot_model:={robot_model}",
        f"gripper:={gripper.strip() or 'none'}",
        f"robot_ip_address:={ip.strip()}",
        f"rebuild_plugin:={'true' if rebuild else 'false'}",
    ]


def build_shell_command(workspace: str, launch_args: list) -> str:
    ws_setup = Path(workspace).expanduser() / "install" / "setup.bash"
    launch = " ".join(shlex.quote(a) for a in ["ros2", "launch", PACKAGE, LAUNCH_FILE, *launch_args])
    return (
        f"source {shlex.quote(ROS_SETUP)} && "
        f"source {shlex.quote(str(ws_setup))} && "
        f"exec {launch}"
    )


# --------------------------------------------------------------------------
# GUI
# --------------------------------------------------------------------------
class LauncherApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.cfg = load_config()
        self.proc = None
        self.log_queue = queue.Queue()

        root.title("FAIRINO Universe Launcher")
        root.minsize(720, 520)
        root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.robot_var = tk.StringVar(value=self.cfg["robot_model"])
        self.gripper_var = tk.StringVar(value=self.cfg["gripper"])
        self.ip_var = tk.StringVar(value=self.cfg["robot_ip_address"])
        self.ws_var = tk.StringVar(value=self.cfg["workspace"])
        self.rebuild_var = tk.BooleanVar(value=False)
        self.cmd_var = tk.StringVar()
        self.ip_msg_var = tk.StringVar()
        self.status_var = tk.StringVar(value="Idle")

        self._build_ui()

        for var in (self.robot_var, self.gripper_var, self.ws_var, self.rebuild_var):
            var.trace_add("write", lambda *_: self.update_preview())
        self.ip_var.trace_add("write", lambda *_: self.on_ip_changed())
        self.on_ip_changed()

        self.root.after(100, self.poll)

    # ---------------- layout ----------------
    def _build_ui(self):
        pad = {"padx": 8, "pady": 4}
        main = ttk.Frame(self.root, padding=10)
        main.pack(fill="both", expand=True)
        main.columnconfigure(0, weight=1)
        main.rowconfigure(4, weight=1)

        # Robot configuration
        cfg_box = ttk.LabelFrame(main, text="Robot configuration", padding=8)
        cfg_box.grid(row=0, column=0, sticky="ew")
        cfg_box.columnconfigure(1, weight=1)

        ttk.Label(cfg_box, text="Robot model").grid(row=0, column=0, sticky="w", **pad)
        ttk.Combobox(cfg_box, textvariable=self.robot_var, values=ROBOT_MODELS,
                     state="readonly", width=18).grid(row=0, column=1, sticky="w", **pad)

        ttk.Label(cfg_box, text="Gripper").grid(row=1, column=0, sticky="w", **pad)
        ttk.Combobox(cfg_box, textvariable=self.gripper_var, values=GRIPPERS,
                     width=18).grid(row=1, column=1, sticky="w", **pad)

        ttk.Label(cfg_box, text="Robot IP address").grid(row=2, column=0, sticky="w", **pad)
        ip_row = ttk.Frame(cfg_box)
        ip_row.grid(row=2, column=1, sticky="w", **pad)
        ttk.Entry(ip_row, textvariable=self.ip_var, width=20).pack(side="left")
        self.ip_msg = ttk.Label(ip_row, textvariable=self.ip_msg_var)
        self.ip_msg.pack(side="left", padx=8)

        ttk.Checkbutton(
            cfg_box, variable=self.rebuild_var,
            text="Rebuild hardware plugin (auto-checked when the IP changes)",
        ).grid(row=3, column=1, sticky="w", **pad)

        # Workspace
        ws_box = ttk.LabelFrame(main, text="Workspace", padding=8)
        ws_box.grid(row=1, column=0, sticky="ew", pady=(8, 0))
        ws_box.columnconfigure(0, weight=1)
        ttk.Entry(ws_box, textvariable=self.ws_var).grid(row=0, column=0, sticky="ew", padx=(0, 8))
        ttk.Button(ws_box, text="Browse…", command=self.browse_ws).grid(row=0, column=1)

        # Command preview
        prev_box = ttk.LabelFrame(main, text="Command", padding=8)
        prev_box.grid(row=2, column=0, sticky="ew", pady=(8, 0))
        prev_box.columnconfigure(0, weight=1)
        ttk.Entry(prev_box, textvariable=self.cmd_var, state="readonly").grid(row=0, column=0, sticky="ew")

        # Buttons + status
        btns = ttk.Frame(main)
        btns.grid(row=3, column=0, sticky="ew", pady=8)
        ttk.Button(btns, text="Save", command=self.on_save).pack(side="left")
        self.launch_btn = ttk.Button(btns, text="Launch", command=self.on_launch)
        self.launch_btn.pack(side="left", padx=6)
        self.stop_btn = ttk.Button(btns, text="Stop", command=self.on_stop, state="disabled")
        self.stop_btn.pack(side="left")
        ttk.Button(btns, text="Clear log", command=self.clear_log).pack(side="left", padx=6)
        ttk.Label(btns, textvariable=self.status_var).pack(side="right")

        # Log
        log_box = ttk.LabelFrame(main, text="Launch output", padding=4)
        log_box.grid(row=4, column=0, sticky="nsew")
        log_box.rowconfigure(0, weight=1)
        log_box.columnconfigure(0, weight=1)
        self.log = tk.Text(log_box, wrap="none", height=14, font=("monospace", 9), state="disabled")
        self.log.grid(row=0, column=0, sticky="nsew")
        ysb = ttk.Scrollbar(log_box, orient="vertical", command=self.log.yview)
        ysb.grid(row=0, column=1, sticky="ns")
        xsb = ttk.Scrollbar(log_box, orient="horizontal", command=self.log.xview)
        xsb.grid(row=1, column=0, sticky="ew")
        self.log.configure(yscrollcommand=ysb.set, xscrollcommand=xsb.set)
        self.log.tag_configure("err", foreground="#c62828")
        self.log.tag_configure("warn", foreground="#b26a00")
        self.log.tag_configure("gui", foreground="#1565c0")

    # ---------------- state / preview ----------------
    def current_args(self) -> list:
        return build_launch_args(self.robot_var.get(), self.gripper_var.get(),
                                 self.ip_var.get(), self.rebuild_var.get())

    def update_preview(self):
        self.cmd_var.set(" ".join(["ros2", "launch", PACKAGE, LAUNCH_FILE, *self.current_args()]))

    def on_ip_changed(self):
        err = validate_ip(self.ip_var.get())
        if err:
            self.ip_msg_var.set(f"✗ {err}")
            self.ip_msg.configure(foreground="#c62828")
        else:
            changed = self.ip_var.get().strip() != self.cfg.get("last_built_ip")
            self.ip_msg_var.set("✓ (plugin will be rebuilt)" if changed else "✓")
            self.ip_msg.configure(foreground="#2e7d32")
            self.rebuild_var.set(changed)
        self.update_preview()

    def browse_ws(self):
        d = filedialog.askdirectory(initialdir=self.ws_var.get() or str(Path.home()))
        if d:
            self.ws_var.set(d)

    def collect_cfg(self) -> dict:
        cfg = dict(self.cfg)
        cfg.update(
            robot_model=self.robot_var.get(),
            gripper=self.gripper_var.get().strip() or "none",
            robot_ip_address=self.ip_var.get().strip(),
            workspace=self.ws_var.get().strip(),
        )
        return cfg

    def validate_all(self) -> bool:
        err = validate_ip(self.ip_var.get())
        if err:
            messagebox.showerror("Invalid IP", err)
            return False
        ws_setup = Path(self.ws_var.get()).expanduser() / "install" / "setup.bash"
        if not ws_setup.is_file():
            messagebox.showerror(
                "Workspace not built",
                f"Can't find:\n{ws_setup}\n\nBuild the workspace first (colcon build).")
            return False
        if not Path(ROS_SETUP).is_file():
            messagebox.showerror("ROS 2 not found", f"Can't find {ROS_SETUP}")
            return False
        return True

    # ---------------- actions ----------------
    def on_save(self):
        if validate_ip(self.ip_var.get()):
            messagebox.showerror("Invalid IP", validate_ip(self.ip_var.get()))
            return
        self.cfg = self.collect_cfg()
        save_config(self.cfg)
        self.append(f"[gui] Saved settings to {CONFIG_PATH}\n", "gui")

    def on_launch(self):
        if self.proc is not None or not self.validate_all():
            return
        self.cfg = self.collect_cfg()
        rebuild = self.rebuild_var.get()
        cmd = build_shell_command(self.cfg["workspace"], self.current_args())

        env = dict(os.environ, PYTHONUNBUFFERED="1",
                   RCUTILS_COLORIZED_OUTPUT="0", RCUTILS_LOGGING_BUFFERED_STREAM="0")
        try:
            self.proc = subprocess.Popen(
                ["bash", "-c", cmd],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1, env=env, start_new_session=True,
                cwd=os.path.expanduser(self.cfg["workspace"]),
            )
        except OSError as exc:
            messagebox.showerror("Launch failed", str(exc))
            self.proc = None
            return

        if rebuild:
            # Plugin gets rebuilt with this IP during this launch
            self.cfg["last_built_ip"] = self.cfg["robot_ip_address"]
        save_config(self.cfg)

        self.append(f"[gui] {self.cmd_var.get()}\n", "gui")
        threading.Thread(target=self._reader, args=(self.proc,), daemon=True).start()
        self.launch_btn.configure(state="disabled")
        self.stop_btn.configure(state="normal")
        self.status_var.set(f"Running (pid {self.proc.pid})")
        self.on_ip_changed()  # refresh rebuild checkbox now that last_built_ip moved

    def on_stop(self):
        if self.proc is None or self.proc.poll() is not None:
            return
        self.status_var.set("Stopping…")
        self.append("[gui] Sending Ctrl+C to launch…\n", "gui")
        self._signal(signal.SIGINT)
        # escalate if it doesn't exit
        self.root.after(10000, lambda p=self.proc: self._escalate(p, signal.SIGTERM))
        self.root.after(15000, lambda p=self.proc: self._escalate(p, signal.SIGKILL))

    def _escalate(self, proc, sig):
        if proc is self.proc and proc.poll() is None:
            self.append(f"[gui] Launch still running, sending {sig.name}\n", "gui")
            self._signal(sig)

    def _signal(self, sig):
        try:
            os.killpg(os.getpgid(self.proc.pid), sig)
        except (ProcessLookupError, PermissionError):
            pass

    def _reader(self, proc):
        for line in proc.stdout:
            self.log_queue.put(ANSI_RE.sub("", line))
        proc.stdout.close()

    def poll(self):
        try:
            while True:
                line = self.log_queue.get_nowait()
                tag = None
                if "[ERROR]" in line or "[FATAL]" in line or "process has died" in line:
                    tag = "err"
                elif "[WARN]" in line:
                    tag = "warn"
                self.append(line, tag)
        except queue.Empty:
            pass

        if self.proc is not None and self.proc.poll() is not None:
            code = self.proc.returncode
            self.append(f"[gui] Launch exited with code {code}\n", "gui")
            self.proc = None
            self.launch_btn.configure(state="normal")
            self.stop_btn.configure(state="disabled")
            self.status_var.set(f"Exited ({code})")

        self.root.after(100, self.poll)

    def append(self, text, tag=None):
        at_bottom = self.log.yview()[1] >= 0.999
        self.log.configure(state="normal")
        self.log.insert("end", text, tag)
        extra = int(self.log.index("end-1c").split(".")[0]) - MAX_LOG_LINES
        if extra > 0:
            self.log.delete("1.0", f"{extra + 1}.0")
        self.log.configure(state="disabled")
        if at_bottom:
            self.log.see("end")

    def clear_log(self):
        self.log.configure(state="normal")
        self.log.delete("1.0", "end")
        self.log.configure(state="disabled")

    def on_close(self):
        if self.proc is not None and self.proc.poll() is None:
            if not messagebox.askyesno("Launch running", "Stop the running launch and quit?"):
                return
            self._signal(signal.SIGINT)
            try:
                self.proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self._signal(signal.SIGKILL)
        self.root.destroy()


def main():
    root = tk.Tk()
    try:
        ttk.Style().theme_use("clam")
    except tk.TclError:
        pass
    LauncherApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()