"""UR3e motion through the lab's stack (github.com/alexhjkim/ur3e_experiments): every joint goal
goes through ur3e_plan_runner's PlanSender -> /ur3e/command (CommandUR3e) -> the ENME480 bridge
(ur3e_mrc) -> scaled_joint_trajectory_controller.

What this reuses from ur3e_plan_runner.move_ur3e, unchanged:
  - PlanSender.safety_error(): robot mode RUNNING, safety mode NORMAL/REDUCED, External Control
    running, pendant speed slider in (0, 30 %], fresh driver state; refused otherwise
  - PlanSender.send(): exactly one official bridge, publish, wait for the joint goal, 1.2 s bridge
    settle (the bridge drops commands that arrive while it is busy)
  - MAX_STEP (0.35 rad per command), the ENME480 joint conversion (pan - pi/2, wrist 1 + pi/2)
  - the arming phrase MOVE_REAL_UR3E
What it adds for probing:
  - a tighter final wait (joints within `settle_tol_rad` and still), since grasps need mm accuracy
    and PlanSender's endpoint tolerance is 0.03 rad
  - Cartesian moves (descend, lift) as joint goals spaced so the TCP stays within `line_tol_m` of
    the straight line (the bridge sends one 2 s point-to-point goal per command)
  - the wrist wrench, averaged at rest
This node is spun from the calling thread (spin_once), like PlanSender itself.
"""

import time

import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from ur3e_plan_runner.move_ur3e import JOINTS, MAX_STEP, PlanSender, expected_enme480

from slatphys_real import ur_kinematics as K

assert list(JOINTS) == list(K.JOINT_NAMES)


class EnmeArm(PlanSender):
    def __init__(self, cfg, skip_driver_checks=False):
        super().__init__()
        self.cfg = cfg
        self.skip_driver_checks = skip_driver_checks
        self._wrench = []  # (t, [Fx Fy Fz Tx Ty Tz])
        self.create_subscription(WrenchStamped, cfg["topics"]["wrench"], self._on_wrench, 500)
        self._n = 0
        self.spin_until(lambda: self.q is not None, 15.0, "/joint_states")
        self.spin_until(lambda: self.official_bridge_count() == 1, 15.0, "official ur3e_mrc command bridge")
        self.spin_until(lambda: self.safety_error() is None, 15.0, "safe UR driver state")

    # -- state ---------------------------------------------------------------------------------
    def safety_error(self):
        if self.skip_driver_checks:  # ur_robot_driver's mock hardware reports no robot / safety mode
            return None if self.q is not None else "no joint states"
        return super().safety_error()

    def _on_wrench(self, msg):
        f, t = msg.wrench.force, msg.wrench.torque
        self._wrench.append((time.monotonic(), np.array([f.x, f.y, f.z, t.x, t.y, t.z])))
        if len(self._wrench) > 20000:
            del self._wrench[:10000]

    def q_now(self):
        rclpy.spin_once(self, timeout_sec=0.01)
        return np.array(self.q, float)

    def tcp(self, tcp_z):
        T = K.fk(self.q_now())
        return T[:3, 3] + tcp_z * T[:3, 2]

    def _spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wrench(self, settle, seconds):
        """Mean / std of the wrist wrench over `seconds` after `settle` s at rest. Only differences
        at the same pose are used downstream, so the published frame (tool0_controller) is fine."""
        self._spin_for(settle)
        t0 = time.monotonic()
        self._spin_for(seconds)
        w = np.array([x for t, x in self._wrench if t >= t0])
        if len(w) < 10:
            raise RuntimeError(f"only {len(w)} wrench samples in {seconds} s on {self.cfg['topics']['wrench']}")
        if np.isnan(w).any() and not self.cfg.get("allow_nan_wrench", False):
            raise RuntimeError(f"NaN wrench on {self.cfg['topics']['wrench']}")
        return {"mean": w.mean(0), "std": w.std(0), "n": len(w)}

    # -- motion --------------------------------------------------------------------------------
    def _goal(self, q):
        q = [float(x) for x in q]
        self._n += 1
        self.send({"q_ros": q, "q_enme": expected_enme480(q), "closed": False, "source_index": f"probe {self._n}"},
                  io_enabled=False, force=True, number=self._n, total=0)  # force: small steps are < 0.03 rad
        # Tighter than PlanSender's 0.03 rad: wait until on target and still.
        tol, deadline, last = self.cfg["safety"]["settle_tol_rad"], time.monotonic() + 10.0, None
        while time.monotonic() < deadline:
            q_now = self.q_now()
            if np.abs(q_now - np.array(q)).max() <= tol and last is not None and np.abs(q_now - last).max() < 1e-4:
                return
            last = q_now
            self._spin_for(0.1)
        raise RuntimeError(f"joint goal not reached within {tol} rad: error "
                           f"{np.abs(self.q_now() - np.array(q)).max():.4f} rad")

    def move_joints(self, q_goal, seconds=None):
        """Joint-space move split into goals of at most MAX_STEP (PlanSender's limit) per joint."""
        q0, q_goal = self.q_now(), np.asarray(q_goal, float)
        n = max(1, int(np.ceil(np.abs(q_goal - q0).max() / (MAX_STEP * 0.95))))
        for i in range(1, n + 1):
            self._goal(q0 + (q_goal - q0) * i / n)

    def move_tcp(self, p_tcp, yaw, seconds=None, linear=True):
        tcp_z = self.cfg["gripper"]["tcp_z"]
        self.check_workspace(p_tcp)
        q0 = self.q_now()
        q1 = K.solve_tcp(p_tcp, yaw, tcp_z, q0)
        if q1 is None:
            raise RuntimeError(f"no IK for TCP {np.round(p_tcp, 3)} yaw {yaw:.2f}")
        if not linear:
            return self.move_joints(q1)
        for q in self.line_goals(self.tcp(tcp_z), np.asarray(p_tcp, float), yaw, q0):
            if np.abs(q - self.q_now()).max() > MAX_STEP:
                raise RuntimeError("Cartesian segment exceeds MAX_STEP; IK branch flip?")
            self._goal(q)

    def line_goals(self, p0, p1, yaw, q_ref):
        """Joint goals along the segment p0 -> p1, subdivided until the joint-space interpolation
        between consecutive goals keeps the TCP within line_tol_m of the line."""
        tcp_z, tol = self.cfg["gripper"]["tcp_z"], self.cfg["safety"]["line_tol_m"]
        d = p1 - p0
        L = max(np.linalg.norm(d), 1e-9)

        def q_at(s, ref):
            q = K.solve_tcp(p0 + s * d, yaw, tcp_z, ref)
            if q is None:
                raise RuntimeError(f"no IK along the line at {np.round(p0 + s * d, 3)}")
            return q

        def dev(qa, qb):
            worst = 0.0
            for a in (0.25, 0.5, 0.75):
                T = K.fk(qa + a * (qb - qa))
                p = T[:3, 3] + tcp_z * T[:3, 2] - p0
                worst = max(worst, np.linalg.norm(p - (p @ d / L ** 2) * d))
            return worst

        goals, s, q_prev = [], 0.0, np.asarray(q_ref, float)
        while s < 1.0 - 1e-9:
            step = 1.0 - s
            while True:
                q_next = q_at(s + step, q_prev)
                if dev(q_prev, q_next) <= tol or step * L < 0.002:
                    break
                step /= 2
            goals.append(q_next)
            q_prev, s = q_next, s + step
        return goals

    def check_workspace(self, p):
        s = self.cfg["safety"]
        if p[2] < s["min_tcp_z"]:
            raise RuntimeError(f"TCP z {p[2]:.3f} below the safety floor {s['min_tcp_z']:.3f}")
        if np.hypot(p[0], p[1]) > s["max_reach"]:
            raise RuntimeError(f"TCP {np.round(p, 3)} beyond max reach {s['max_reach']}")

    def zero_ft(self):
        pass

    def release(self):
        self.destroy_node()

    def close(self):
        self.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
