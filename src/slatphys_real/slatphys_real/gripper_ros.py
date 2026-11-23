"""A force-controlled parallel gripper over ros2_control (the Gazebo test bench, slatphys_gazebo),
with the same interface as the Hand-E client (hande.HandE), so probe.py runs unchanged.

  command  std_msgs/Float64MultiArray [f_left, f_right] on <command_topic> (an
           effort_controllers/JointGroupEffortController): positive opens, so gripping at F N
           is [-F, -F] -- a constant closing force, as the Hand-E's force setting.
  state    the finger joints in /joint_states: joint position = each finger face's distance
           from the centre, so the opening is q_left + q_right.

close_on() returns the Hand-E's OBJ codes: 2 = stopped on an object while closing,
3 = reached the end of travel (nothing between the fingers).
"""

import collections
import threading
import time

import numpy as np
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

STROKE_M = 0.050
OBJ_CONTACT, OBJ_AT_END = 2, 3


class RosGripper(Node):
    def __init__(self, cfg):
        super().__init__("slatphys_gripper")
        g = cfg["gripper"]
        self.joints = list(g.get("joints", ["finger_left_joint", "finger_right_joint"]))
        self.open_force = float(g.get("open_force_N", 20.0))
        self.contact_min = float(g.get("contact_min_width_m", 0.002))  # opening below this = nothing held
        self.settle_tol = float(g.get("settle_tol_m", 0.0002))     # opening change that counts as stopped
        self.settle_window = float(g.get("settle_window_s", 0.5))  # in message (sim) time
        self.timeout = float(g.get("motion_timeout_s", 120.0))     # wall clock
        self._q, self._lock = None, threading.Lock()
        self._hist = collections.deque(maxlen=20000)  # (message stamp s, opening m)
        self._pub = self.create_publisher(Float64MultiArray, g.get("command_topic", "/gripper_effort_controller/commands"), 10)
        self.create_subscription(JointState, cfg["topics"]["joint_states"], self._on_js, 50)
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self)
        threading.Thread(target=self._exec.spin, daemon=True).start()
        t0 = time.time()
        while self._q is None:
            if time.time() - t0 > 10:
                raise RuntimeError(f"no finger joints {self.joints} in {cfg['topics']['joint_states']}")
            time.sleep(0.05)
        t0 = time.time()
        while self._pub.get_subscription_count() == 0:
            if time.time() - t0 > 10:
                raise RuntimeError(f"nobody listens on {self._pub.topic_name}: is gripper_effort_controller active?")
            time.sleep(0.05)

    def _on_js(self, msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        if all(j in idx for j in self.joints):
            with self._lock:
                self._q = np.array([msg.position[idx[j]] for j in self.joints])
                self._hist.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, float(self._q.sum())))

    def _command(self, f_each):
        self._pub.publish(Float64MultiArray(data=[float(f_each)] * len(self.joints)))

    def _settle(self):
        """Wait until the opening stops changing (fingers on an object or at a limit): less than
        settle_tol over settle_window. Judged on the /joint_states stamps -- simulation time in
        Gazebo, which may run slower than real time -- and over a window, because a held object
        can make the fingers jitter by microns. On timeout (wall clock) it warns and goes on,
        like the Hand-E, which just reports where it stopped."""
        with self._lock:
            t_cmd = self._hist[-1][0] if self._hist else 0.0
        t0 = time.time()
        while time.time() - t0 < self.timeout:
            with self._lock:
                h = [(t, w) for t, w in self._hist if t >= t_cmd + 0.1]  # give the command 0.1 s to act
            if h and h[-1][0] - h[0][0] > self.settle_window:
                recent = [w for t, w in h if t >= h[-1][0] - self.settle_window]
                if max(recent) - min(recent) < self.settle_tol:
                    return True
            time.sleep(0.02)
        self.get_logger().warn(f"gripper opening still changing after {self.timeout} s")
        return False

    # -- the Hand-E interface used by probe.py -----------------------------------------------
    def width(self, samples=5):
        vals = []
        for _ in range(samples):
            with self._lock:
                vals.append(float(self._q.sum()))
            time.sleep(0.01)
        return float(np.mean(vals)), vals

    def open(self, force_n=None, speed=None):
        self._command(+(force_n or self.open_force))
        self._settle()
        return OBJ_AT_END

    def close_on(self, force_n, speed=0):
        self._command(-float(force_n))
        self._settle()
        return OBJ_CONTACT if self.width(1)[0] > self.contact_min else OBJ_AT_END

    def shutdown(self):
        self._exec.shutdown()
        self.destroy_node()
