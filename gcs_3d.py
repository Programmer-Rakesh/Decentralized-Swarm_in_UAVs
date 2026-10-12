"""gcs_3d.py -- the 3D Map button: launches / tracks / kills the drone_3d.py process.
Keep drone_3d.py in the same folder. (Mixed into the dashboard by gcs5.py.)
"""
import os
import subprocess
import sys
import time
import tkinter as tk
from tkinter import messagebox

try:
    import psutil
except ImportError:
    psutil = None

from gcs_ui import (
    COL_BORDER,
    COL_BTN,
    COL_BTN_HOVER,
    COL_DOT_OFF,
    COL_DOT_ON,
    COL_TEXT,
    COL_VALUE,
    COL_WARN,
    PROJECT_ROOT,
)


##############################################################################
# SECTION: map3d.py   (was gcs/app/map/map3d.py)
##############################################################################
class Map3DMixin:
    """3D Map button: launches / tracks / kills the drone_3d.py process."""

    def _build_3d_map_button(self, toolbar):
        """The '3D' button itself doubles as its own status indicator:
        a status dot (grey when idle, green and gently pulsing while
        drone_3d.py is running) and a small x live inside the button, right
        after the label. Clicking the label/dot area launches the script;
        clicking the x (only "live" once something is running) kills it."""
        btn = tk.Frame(toolbar, bg=COL_BTN, highlightthickness=1,
                        highlightbackground=COL_BORDER, highlightcolor=COL_BORDER,
                        cursor="hand2")
        self.map3d_btn = btn

        self.map3d_label = tk.Label(btn, text="3D", font=("Segoe UI", 9),
                                     fg=COL_TEXT, bg=COL_BTN, cursor="hand2")
        self.map3d_label.pack(side="left", padx=(10, 6), pady=6)

        self.map3d_dot = tk.Canvas(btn, width=10, height=10, bg=COL_BTN,
                                    highlightthickness=0, cursor="hand2")
        self.map3d_dot.pack(side="left", padx=(0, 6), pady=6)
        self.map3d_dot_id = self.map3d_dot.create_oval(1, 1, 9, 9,
                                                         fill=COL_DOT_OFF, outline="")

        self.map3d_close = tk.Canvas(btn, width=16, height=16, bg=COL_BTN,
                                      highlightthickness=0, cursor="hand2")
        self.map3d_close.pack(side="left", padx=(0, 10), pady=6)
        self.map3d_close_bg = self.map3d_close.create_oval(1, 1, 15, 15,
                                                             fill=COL_BTN, outline="")
        self.map3d_close_l1 = self.map3d_close.create_line(5, 5, 11, 11,
                                                             fill=COL_DOT_OFF, width=1.6)
        self.map3d_close_l2 = self.map3d_close.create_line(11, 5, 5, 11,
                                                             fill=COL_DOT_OFF, width=1.6)

        hover_targets = (btn, self.map3d_label, self.map3d_dot, self.map3d_close)

        def _on_enter(_e):
            for w in hover_targets:
                w.config(bg=COL_BTN_HOVER)
            self.map3d_dot.itemconfig(
                self.map3d_dot_id,
                fill=(COL_DOT_ON if self.map3d_process is not None else COL_DOT_OFF))

        def _on_leave(_e):
            for w in hover_targets:
                w.config(bg=COL_BTN)

        def _on_close_enter(_e):
            if self.map3d_process is not None:
                self.map3d_close.itemconfig(self.map3d_close_bg, fill=COL_WARN)

        def _on_close_leave(_e):
            self.map3d_close.itemconfig(self.map3d_close_bg, fill=COL_BTN_HOVER)

        for w in (btn, self.map3d_label, self.map3d_dot):
            w.bind("<Enter>", _on_enter)
            w.bind("<Leave>", _on_leave)
            w.bind("<Button-1>", lambda e: self._launch_3d_map())

        self.map3d_close.bind("<Enter>", _on_close_enter)
        self.map3d_close.bind("<Leave>", _on_close_leave)
        self.map3d_close.bind("<Button-1>", lambda e: self._terminate_3d_map())

        btn.pack(fill="x", pady=3)

    def _launch_3d_map(self):
        if self.map3d_process is not None and self.map3d_process.poll() is None:
            # Already running -- flash the button border instead of
            # spawning a second copy.
            self._flash_3d_map_button()
            return

        drone_dir = PROJECT_ROOT
        script_path = os.path.join(drone_dir, "drone_3d.py")
        if not os.path.isfile(script_path):
            messagebox.showerror(
                "3D",
                f"Could not find drone_3d.py in:\n{drone_dir}"
            )
            return

        # A copy left over from an earlier click/session (before this
        # button tracked/killed its own children) can still be holding
        # a window, port, or GL context open, which silently blocks a
        # fresh one from opening. Clear those out first.
        self._kill_stray_3dmap_processes()

        try:
            self.map3d_process = subprocess.Popen(
                [sys.executable, script_path], cwd=drone_dir,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
            )
        except Exception as e:
            messagebox.showerror("3D", f"Failed to launch drone_3d.py:\n{e}")
            return

        self.map3d_pulse_on = False
        self._pulse_3d_map_dot()
        # Give it a beat to crash-on-startup before the first poll, so a
        # bad launch surfaces as an error dialog instead of just "nothing
        # happened".
        self.map3d_poll_after_id = self.root.after(700, self._poll_3d_map_process)

    def _kill_stray_3dmap_processes(self):
        """Best-effort cleanup of any drone_3d.py process already running --
        including ones from before this dashboard session existed -- so
        the new launch isn't silently blocked by a leftover instance."""
        if psutil is not None:
            def _matches(proc):
                try:
                    cmdline = proc.info.get("cmdline") or []
                except Exception:
                    return False
                return (proc.pid != os.getpid()
                        and any("drone_3d.py" in str(part) for part in cmdline))

            for proc in psutil.process_iter(["pid", "cmdline"]):
                if _matches(proc):
                    try:
                        proc.terminate()
                    except Exception:
                        pass

            deadline = time.time() + 0.8
            while time.time() < deadline:
                if not any(_matches(p) for p in psutil.process_iter(["pid", "cmdline"])):
                    break
                time.sleep(0.1)
        else:
            # No psutil available -- fall back to a blunt OS-level sweep.
            try:
                if os.name == "nt":
                    subprocess.run(
                        ["wmic", "process", "where",
                         "CommandLine like '%drone_3d.py%'", "delete"],
                        capture_output=True
                    )
                else:
                    subprocess.run(["pkill", "-f", "drone_3d.py"], capture_output=True)
                time.sleep(0.3)
            except Exception:
                pass

    def _set_3d_map_idle_visuals(self):
        self.map3d_dot.itemconfig(self.map3d_dot_id, fill=COL_DOT_OFF)
        self.map3d_close.itemconfig(self.map3d_close_l1, fill=COL_DOT_OFF)
        self.map3d_close.itemconfig(self.map3d_close_l2, fill=COL_DOT_OFF)

    def _set_3d_map_running_visuals(self):
        self.map3d_close.itemconfig(self.map3d_close_l1, fill=COL_TEXT)
        self.map3d_close.itemconfig(self.map3d_close_l2, fill=COL_TEXT)

    def _pulse_3d_map_dot(self):
        # Smooth two-tone breathing effect -- alternates between the
        # bright "on" green and a dimmer shade of it, only while running.
        if self.map3d_process is None:
            return
        self._set_3d_map_running_visuals()
        self.map3d_pulse_on = not self.map3d_pulse_on
        color = COL_DOT_ON if self.map3d_pulse_on else "#1f6b45"
        self.map3d_dot.itemconfig(self.map3d_dot_id, fill=color)
        self.map3d_pulse_after_id = self.root.after(650, self._pulse_3d_map_dot)

    def _flash_3d_map_button(self):
        # Briefly brighten the button border to draw the eye back to it
        # when the user clicks "3D Map" while it's already running.
        self.map3d_btn.config(highlightbackground=COL_VALUE, highlightcolor=COL_VALUE)
        self.root.after(350, lambda: self.map3d_btn.config(
            highlightbackground=COL_BORDER, highlightcolor=COL_BORDER))

    def _poll_3d_map_process(self):
        # Detects the case where the user closed the 3D map window
        # themselves (or it crashed on startup) -- the subprocess exits
        # and the button should fall back to its idle look on its own
        # rather than lying about it still running.
        if self.map3d_process is None:
            return
        ret = self.map3d_process.poll()
        if ret is not None:
            proc = self.map3d_process
            self.map3d_process = None
            if self.map3d_pulse_after_id is not None:
                self.root.after_cancel(self.map3d_pulse_after_id)
                self.map3d_pulse_after_id = None
            self._set_3d_map_idle_visuals()
            if ret != 0:
                err_text = ""
                try:
                    err_text = (proc.stderr.read() or "").strip()
                except Exception:
                    pass
                tail = "\n".join(err_text.splitlines()[-15:]) if err_text else \
                    "(no error output captured)"
                messagebox.showerror(
                    "3D",
                    f"drone_3d.py exited unexpectedly (code {ret}):\n\n{tail}"
                )
            return
        self.map3d_poll_after_id = self.root.after(1000, self._poll_3d_map_process)

    def _terminate_3d_map(self):
        proc = self.map3d_process
        self.map3d_process = None
        if self.map3d_pulse_after_id is not None:
            self.root.after_cancel(self.map3d_pulse_after_id)
            self.map3d_pulse_after_id = None
        if self.map3d_poll_after_id is not None:
            self.root.after_cancel(self.map3d_poll_after_id)
            self.map3d_poll_after_id = None
        self._set_3d_map_idle_visuals()
        if proc is not None and proc.poll() is None:
            try:
                proc.terminate()
                try:
                    proc.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    proc.kill()
            except Exception:
                pass
