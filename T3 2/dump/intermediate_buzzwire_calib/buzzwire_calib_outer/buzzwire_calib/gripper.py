"""Non-blocking gripper open/close toggle (SPACE key in every teaching tool).

Preferred path: a direct rclpy service client (fast, ~20 ms). Fallback: the same `ros2 service call`
CLI trick your heal_dynamixel_gripper.py uses (needs the workspace setup.bash, ~1 s per call).
Calls run in a worker thread so the control / recording loop never stalls. Failures only print a warning.
"""
from __future__ import annotations
import os, subprocess, threading, time


class GripperHelper:
    def __init__(self, cfg: dict, hub=None, log=print):
        g = cfg.get("gripper", {})
        self.enabled = bool(g.get("enabled", True))
        self.service_name = g.get("service_name", "/gripper_controller/command")
        self.service_type = g.get("service_type", "addverb_cobot_msgs/srv/Gripper")
        self.open_pos, self.close_pos = float(g.get("open_position", 1.0)), float(g.get("close_position", 0.0))
        self.force = float(g.get("force", 100.0))
        self.ws = g.get("workspace_path", "") or os.environ.get("WS", "")
        self.state = g.get("initial_state", "open")      # "open" | "closed"
        self.log, self._busy, self._warned = log, threading.Lock(), False
        self._client, self._req_cls = None, None
        if hub is not None and self.enabled:
            try:
                mod, _, name = self.service_type.replace("/srv/", "/").partition("/")
                srv = getattr(__import__(f"{mod}.srv", fromlist=[name]), name)
                self._req_cls = srv.Request
                self._client = hub.node.create_client(srv, self.service_name)
            except Exception as e:      # message package not importable -> CLI fallback
                self.log(f"[gripper] rclpy client unavailable ({e.__class__.__name__}); using CLI fallback")

    @property
    def value(self) -> float:
        return 1.0 if self.state == "open" else 0.0

    def toggle(self):
        self.set("closed" if self.state == "open" else "open")

    def set(self, state: str):
        if not self.enabled:
            return
        if not self._busy.acquire(blocking=False):
            self.log("[gripper] busy - ignored")
            return
        threading.Thread(target=self._do, args=(state,), daemon=True).start()

    def _do(self, state):
        try:
            pos = self.open_pos if state == "open" else self.close_pos
            ok = self._call_client(pos) if self._client is not None else self._call_cli(pos)
            if ok:
                self.state = state
                self.log(f"\n[gripper] {state.upper()}")
            elif not self._warned:
                self._warned = True
                self.log("\n[gripper] command FAILED (service not available?). Teaching continues without gripper control.")
        finally:
            self._busy.release()

    def _call_client(self, pos) -> bool:
        if not self._client.wait_for_service(timeout_sec=1.0):
            return False
        req = self._req_cls()
        req.position, req.grasp_force = float(pos), float(self.force)
        fut = self._client.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 5.0:
            time.sleep(0.01)
        return fut.done() and fut.exception() is None

    def _call_cli(self, pos) -> bool:
        setup = os.path.join(self.ws, "install/setup.bash") if self.ws else ""
        src = f"source {setup} && " if setup and os.path.isfile(setup) else ""
        cmd = f'{src}ros2 service call {self.service_name} {self.service_type} "{{position: {pos}, grasp_force: {self.force}}}"'
        try:
            r = subprocess.run(cmd, shell=True, executable="/bin/bash", capture_output=True, text=True, timeout=8)
            return r.returncode == 0
        except Exception:
            return False
