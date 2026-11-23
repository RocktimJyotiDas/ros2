"""UR3e access through ur_robot_driver (ROS 2): joint states, wrist wrench, trajectory goals.

Interfaces (ur_robot_driver defaults, all configurable in configs/real.yaml):
  /joint_states                                                   sensor_msgs/JointState
  /force_torque_sensor_broadcaster/wrench                         geometry_msgs/WrenchStamped
  /scaled_joint_trajectory_controller/follow_joint_trajectory     control_msgs/FollowJointTrajectory
  /io_and_status_controller/zero_ftsensor                         std_srvs/Trigger

Scripts are written procedurally (move, read, move ...): the node spins in a background thread
and every call blocks until its result arrives.
"""

import collections
import threading
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import WrenchStamped
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from slatphys_real import ur_kinematics as K


def _wait(future, timeout):
    t0 = time.time()
    while not future.done():
        if time.time() - t0 > timeout:
            raise TimeoutError("ROS call timed out")
        time.sleep(0.005)
    return future.result()


class Arm(Node):
    def __init__(self, cfg, mock=False):
        super().__init__("slatphys_real_arm")
        self.cfg = cfg
        self.mock = mock
        self._q = None
        self._wrench = collections.deque(maxlen=5000)  # (t, [Fx Fy Fz Tx Ty Tz])
        self._lock = threading.Lock()
        self.create_subscription(JointState, cfg["topics"]["joint_states"], self._on_js, 50)
        self.create_subscription(WrenchStamped, cfg["topics"]["wrench"], self._on_wrench, 500)
        self._traj = ActionClient(self, FollowJointTrajectory, cfg["topics"]["trajectory_action"])
        self._zero = self.create_client(Trigger, cfg["topics"]["zero_ft"])
        self._exec = MultiThreadedExecutor()
        self._exec.add_node(self)
        threading.Thread(target=self._exec.spin, daemon=True).start()

        if mock:  # no driver: start at HOME, trajectories just set the joints, the wrench reads zero
            self._q = np.array(cfg["home_q"], float)
            return
        t0 = time.time()
        while self._q is None:
            if time.time() - t0 > 10:
                raise RuntimeError(f"no {cfg['topics']['joint_states']}: is ur_robot_driver running?")
            time.sleep(0.05)
        if not self._traj.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(f"no action server {cfg['topics']['trajectory_action']}: is the External Control"
                               " program playing on the pendant and the controller active?")

    # -- state ---------------------------------------------------------------------------------
    def _on_js(self, msg):
        lookup = dict(zip(msg.name, msg.position))
        if all(n in lookup for n in K.JOINT_NAMES):
            with self._lock:
                self._q = np.array([lookup[n] for n in K.JOINT_NAMES])

    def _on_wrench(self, msg):
        f, t = msg.wrench.force, msg.wrench.torque
        with self._lock:
            self._wrench.append((time.time(), np.array([f.x, f.y, f.z, t.x, t.y, t.z])))

    def q(self):
        with self._lock:
            return self._q.copy()

    q_now = q

    def release(self):
        self._exec.shutdown()
        self.destroy_node()

    def tcp(self, tcp_z):
        T = K.fk(self.q())
        return T[:3, 3] + tcp_z * T[:3, 2]

    def wrench(self, settle, seconds):
        """Mean and std of the wrist wrench over `seconds`, after `settle` s of holding still.
        The frame is whatever ur_robot_driver publishes in (tool0 or base depending on the
        driver version); the scoring only uses differences taken at the same pose, and their
        magnitude, so the frame doesn't matter."""
        time.sleep(settle)
        t0 = time.time()
        time.sleep(seconds)
        with self._lock:
            w = np.array([x for t, x in self._wrench if t >= t0])
        if self.mock:
            return {"mean": np.zeros(6), "std": np.zeros(6), "n": 0}
        if len(w) < 10:
            raise RuntimeError(f"only {len(w)} wrench samples in {seconds} s on {self.cfg['topics']['wrench']}")
        if np.isnan(w).any() and not self.cfg.get("allow_nan_wrench", False):  # ur_robot_driver's mock hardware
            raise RuntimeError(f"NaN wrench on {self.cfg['topics']['wrench']}")
        return {"mean": w.mean(0), "std": w.std(0), "n": len(w)}

    def zero_ft(self):
        if not self._zero.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("zero_ftsensor service unavailable; continuing without zeroing")
            return
        _wait(self._zero.call_async(Trigger.Request()), 5.0)
        time.sleep(0.3)

    # -- motion --------------------------------------------------------------------------------
    def _check_step(self, qa, qb):
        jump = np.abs(qb - qa).max()
        if jump > self.cfg["safety"]["max_joint_step_rad"]:
            raise RuntimeError(f"IK branch flip: {jump:.2f} rad between waypoints; aborting")

    def run(self, waypoints, times):
        """Send a joint trajectory (list of q, times from start in s) and wait for it."""
        q0 = self.q()
        vmax = self.cfg["safety"]["max_joint_speed_rad_s"]
        prev_q, prev_t = q0, 0.0
        for q, t in zip(waypoints, times):
            if np.abs(q - prev_q).max() / max(t - prev_t, 1e-3) > vmax:
                raise RuntimeError(f"trajectory exceeds {vmax} rad/s; slow it down")
            prev_q, prev_t = q, t
        if self.mock:
            with self._lock:
                self._q = np.asarray(waypoints[-1], float).copy()
            return
        traj = JointTrajectory()
        traj.joint_names = list(K.JOINT_NAMES)
        for q, t in zip(waypoints, times):
            p = JointTrajectoryPoint()
            p.positions = [float(x) for x in q]
            p.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            traj.points.append(p)
        goal = FollowJointTrajectory.Goal(trajectory=traj)
        handle = _wait(self._traj.send_goal_async(goal), 10.0)
        if not handle.accepted:
            raise RuntimeError("trajectory goal rejected")
        res = _wait(handle.get_result_async(), times[-1] + 30.0)
        if res.result.error_code != 0:
            raise RuntimeError(f"trajectory failed: error {res.result.error_code} {res.result.error_string}")

    def move_joints(self, q_goal, seconds):
        q0 = self.q()
        # Cosine profile peaks at pi/2 * distance / time: stretch long moves to stay under the limit.
        dq = np.abs(np.asarray(q_goal) - q0).max()
        seconds = max(seconds, 1.7 * dq / self.cfg["safety"]["max_joint_speed_rad_s"])
        n = max(2, int(seconds * 10))
        s = 0.5 - 0.5 * np.cos(np.pi * np.arange(1, n + 1) / n)  # same cosine profile as the simulation
        wps = [q0 + (np.asarray(q_goal) - q0) * si for si in s]
        self.run(wps, [seconds * i / n for i in range(1, n + 1)])

    def move_tcp(self, p_tcp, yaw, seconds, linear=True):
        """Move the TCP to p_tcp (base frame) with a top-down orientation at `yaw`; `linear`
        interpolates the TCP in Cartesian space (straight descents and lifts)."""
        tcp_z = self.cfg["gripper"]["tcp_z"]
        self.check_workspace(p_tcp)
        q0 = self.q()
        if not linear:
            q = K.solve_tcp(p_tcp, yaw, tcp_z, q0)
            if q is None:
                raise RuntimeError(f"no IK for TCP {np.round(p_tcp, 3)} yaw {yaw:.2f}")
            return self.move_joints(q, seconds)
        p0 = self.tcp(tcp_z)
        n = max(2, int(seconds * 10))
        wps, q_prev = [], q0
        for i in range(1, n + 1):
            s = 0.5 - 0.5 * np.cos(np.pi * i / n)
            q = K.solve_tcp(p0 + (np.asarray(p_tcp) - p0) * s, yaw, tcp_z, q_prev)
            if q is None:
                raise RuntimeError(f"no IK along the line to {np.round(p_tcp, 3)}")
            self._check_step(q_prev, q)
            wps.append(q)
            q_prev = q
        self.run(wps, [seconds * i / n for i in range(1, n + 1)])

    def check_workspace(self, p):
        s = self.cfg["safety"]
        if p[2] < s["min_tcp_z"]:
            raise RuntimeError(f"TCP z {p[2]:.3f} below the safety floor {s['min_tcp_z']}")
        if np.hypot(p[0], p[1]) > s["max_reach"]:
            raise RuntimeError(f"TCP {np.round(p, 3)} beyond max reach {s['max_reach']}")


    def close(self):
        self._exec.shutdown()
        self.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def start(cfg, mock=False, skip_driver_checks=False):
    """The arm backend: `enme480` (default; the lab's ur3e_plan_runner + ENME480 bridge) or
    `trajectory` (direct FollowJointTrajectory). --mock / --dry_run use the trajectory class's mock."""
    if not rclpy.ok():
        rclpy.init()
    if mock:
        return Arm(cfg, mock=True)
    if cfg.get("motion_backend", "enme480") == "enme480":
        from slatphys_real.arm_enme480 import EnmeArm

        return EnmeArm(cfg, skip_driver_checks=skip_driver_checks)
    return Arm(cfg)
