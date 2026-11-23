"""Config / calibration loading shared by the nodes."""

import json
import os

import numpy as np
import yaml

DEFAULT_CONFIG = os.environ.get("SLATPHYS_REAL_CONFIG", "configs/real.yaml")


def load_config(path=None):
    """configs/real.yaml; relative paths inside it resolve against the repo root (the directory
    holding configs/), so the nodes can be run from anywhere."""
    path = os.path.abspath(path or DEFAULT_CONFIG)
    cfg = yaml.safe_load(open(path))
    root = os.path.dirname(os.path.dirname(path))
    cfg["_root"] = root
    cfg["calib_dir"] = os.path.join(root, cfg["calib_dir"])
    cfg["gripper"]["calibration"] = os.path.join(root, cfg["gripper"]["calibration"])
    return cfg


def load_camera_calib(cfg):
    p = os.path.join(cfg["calib_dir"], "camera.json")
    if not os.path.exists(p):
        raise FileNotFoundError(f"{p} missing: run `ros2 run slatphys_real calibrate_camera` first")
    return json.load(open(p))


def table_z(cfg):
    """Table height in `base`: from the camera calibration, or fixed in the config (simulation)."""
    if cfg.get("table_z_base") is not None:
        return float(cfg["table_z_base"])
    return float(load_camera_calib(cfg)["table_z_base"])


def width_model(cfg):
    p = cfg["gripper"]["calibration"]
    if os.path.exists(p):
        g = json.load(open(p))
        return g["w0_m"], g["slope_m_per_count"]
    return 0.050, -0.050 / 255  # nominal: 50 mm stroke over 255 counts


def make_gripper(cfg, mock=False, activate=False):
    """Hand-E via the lab's Robotiq client. Like ur3e_plan_runner, activation is opt-in
    (--activate_gripper): activation moves the fingers, so clear them first."""
    from slatphys_real.hande import MockHandE

    if mock:
        return MockHandE()
    if cfg["gripper"].get("backend", "hande") == "ros":  # Gazebo (slatphys_gazebo)
        from slatphys_real.gripper_ros import RosGripper

        return RosGripper(cfg)
    from slatphys_real.hande import HandE

    return HandE(cfg["robot_ip"], activate=activate, width_model=width_model(cfg))


# -- frames ----------------------------------------------------------------------------------
# The simulation (SLAT-Phys-Scene) puts the robot base at world (robot_base_xy, table_height) with
# the objects toward world -x. On the real table the objects lie in the direction
# `world.workspace_yaw_deg` (in `base`, measured from +x). So
#     p_world = Rz(theta) p_base + [robot_base_xy, table_height - table_z],  theta = pi - workspace_yaw
# and yaws transform as yaw_world = yaw_base + theta.


def world_theta(cfg):
    return np.pi - np.radians(cfg["world"]["workspace_yaw_deg"])


def rot_z(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def T_world_base(cfg, tz):
    T = np.eye(4)
    T[:3, :3] = rot_z(world_theta(cfg))
    T[:3, 3] = [*cfg["world"]["robot_base_xy"], cfg["world"]["table_height"] - tz]
    return T


def world_xy_to_base(cfg, xy_world):
    """A world xy position -> base xy (z is handled through the table height)."""
    d = np.asarray(xy_world, float) - np.asarray(cfg["world"]["robot_base_xy"], float)
    return (rot_z(-world_theta(cfg))[:2, :2] @ d)


def hold_point_base(cfg, tz):
    """The protocol's hold point (world offset from the robot, as in the simulation) in base."""
    off = cfg["protocol"]["hold_offset"]
    xy = rot_z(-world_theta(cfg))[:2, :2] @ np.asarray(off[:2], float)
    return np.array([xy[0], xy[1], tz + off[2]])


def to_jsonable(x):
    if isinstance(x, dict):
        return {k: to_jsonable(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [to_jsonable(v) for v in x]
    if isinstance(x, np.ndarray):
        return to_jsonable(x.tolist())
    if isinstance(x, (np.floating, float)):
        return round(float(x), 6)
    if isinstance(x, np.integer):
        return int(x)
    return x
