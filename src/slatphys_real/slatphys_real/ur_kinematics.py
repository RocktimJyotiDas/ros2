"""UR3e forward / analytic inverse kinematics (nominal DH), pure numpy.

Frames follow ur_robot_driver: IK and FK are between the ROS `base` frame (the controller's
base, = the pendant's "Base" feature; `base_link` is this rotated 180 deg about z) and `tool0`
(the flange). The robot's factory kinematic calibration shifts the real flange by a few mm from
this nominal model; that's fine here because camera calibration (calibrate_camera.py) also
measures points with this same FK, so commanded and measured positions are self-consistent.
"""

import numpy as np

# UR3e DH parameters (Universal Robots, "DH parameters for calculations of kinematics and dynamics").
D1, A2, A3, D4, D5, D6 = 0.15185, -0.24355, -0.2132, 0.13105, 0.08535, 0.0921
DH_A = np.array([0.0, A2, A3, 0.0, 0.0, 0.0])
DH_D = np.array([D1, 0.0, 0.0, D4, D5, D6])
DH_ALPHA = np.array([np.pi / 2, 0.0, 0.0, np.pi / 2, -np.pi / 2, 0.0])
JOINT_NAMES = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
               "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


def dh(i, theta):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(DH_ALPHA[i]), np.sin(DH_ALPHA[i])
    return np.array([[ct, -st * ca, st * sa, DH_A[i] * ct],
                     [st, ct * ca, -ct * sa, DH_A[i] * st],
                     [0.0, sa, ca, DH_D[i]],
                     [0.0, 0.0, 0.0, 1.0]])


def fk(q):
    """4x4 pose of tool0 in `base`."""
    T = np.eye(4)
    for i in range(6):
        T = T @ dh(i, q[i])
    return T


def ik(T, eps=1e-9):
    """All analytic solutions (up to 8) for the tool0 pose T in `base`, each verified by FK."""
    sols = []
    p05 = T @ np.array([0.0, 0.0, -D6, 1.0])
    r = np.hypot(p05[0], p05[1])
    if r < abs(D4):
        return sols
    phi1 = np.arctan2(p05[1], p05[0])
    phi2 = np.arccos(np.clip(D4 / r, -1, 1))
    for t1 in (phi1 + phi2 + np.pi / 2, phi1 - phi2 + np.pi / 2):
        s1, c1 = np.sin(t1), np.cos(t1)
        c5 = (T[0, 3] * s1 - T[1, 3] * c1 - D4) / D6
        if abs(c5) > 1 + 1e-9:
            continue
        for t5 in (np.arccos(np.clip(c5, -1, 1)), -np.arccos(np.clip(c5, -1, 1))):
            s5 = np.sin(t5)
            if abs(s5) < eps:  # wrist singularity: theta6 is free, keep it at 0
                t6 = 0.0
            else:
                # From T06^-1: its x / y columns' components give theta6.
                Ti = np.linalg.inv(T)
                t6 = np.arctan2((-Ti[1, 0] * s1 + Ti[1, 1] * c1) / s5, (Ti[0, 0] * s1 - Ti[0, 1] * c1) / s5)
            T01, T45, T56 = dh(0, t1), dh(4, t5), dh(5, t6)
            T14 = np.linalg.inv(T01) @ T @ np.linalg.inv(T45 @ T56)
            px, pz = T14[0, 3], T14[1, 3]  # frame-1 x / (rotated) planar coordinates
            d = np.hypot(px, pz)
            c3 = (d ** 2 - A2 ** 2 - A3 ** 2) / (2 * A2 * A3)
            if abs(c3) > 1 + 1e-9:
                continue
            for t3 in (np.arccos(np.clip(c3, -1, 1)), -np.arccos(np.clip(c3, -1, 1))):
                t2 = np.arctan2(pz, px) - np.arctan2(A3 * np.sin(t3), A2 + A3 * np.cos(t3))
                T13 = dh(1, t2) @ dh(2, t3)
                T34 = np.linalg.inv(T13) @ T14
                t4 = np.arctan2(T34[1, 0], T34[0, 0])
                q = np.array([t1, t2, t3, t4, t5, t6])
                q = (q + np.pi) % (2 * np.pi) - np.pi
                Tq = fk(q)
                if np.abs(Tq[:3, 3] - T[:3, 3]).max() < 1e-6 and np.abs(Tq[:3, :3] - T[:3, :3]).max() < 1e-6:
                    sols.append(q)
    return sols


def nearest(sols, q_ref, limit=2 * np.pi):
    """Solution closest to q_ref, each joint unwrapped by 2*pi to the copy nearest q_ref
    (UR joints span +-2pi), or None. Joints are kept inside +-limit."""
    best, best_d = None, np.inf
    q_ref = np.asarray(q_ref, float)
    for q in sols:
        q = q_ref + (q - q_ref + np.pi) % (2 * np.pi) - np.pi
        if np.any(np.abs(q) > limit):
            continue
        d = np.abs(q - q_ref).sum()
        if d < best_d:
            best, best_d = q, d
    return best


def tool_pose(p_tcp, yaw, tcp_z):
    """tool0 pose for a top-down grasp: tool z points down, x (the Hand-E closing axis) along
    `yaw` in the base xy plane, and the TCP tcp_z along tool z from the flange."""
    x = np.array([np.cos(yaw), np.sin(yaw), 0.0])
    z = np.array([0.0, 0.0, -1.0])
    T = np.eye(4)
    T[:3, :3] = np.stack([x, np.cross(z, x), z], axis=1)
    T[:3, 3] = np.asarray(p_tcp, float) - tcp_z * z
    return T


def solve_tcp(p_tcp, yaw, tcp_z, q_ref):
    sols = ik(tool_pose(p_tcp, yaw, tcp_z))
    return nearest(sols, q_ref)
