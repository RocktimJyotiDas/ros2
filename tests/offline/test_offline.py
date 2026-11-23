"""Offline checks (no robot, no GPU), in the slatphys python env with SLAT-Phys-Scene importable:
    export SLATPHYS_SCENE=/path/to/SLAT-Phys-Scene; source $SLATPHYS_SCENE/setup/sam3d_env.sh
    PYTHONPATH=$SLATPHYS_SCENE python tests/offline/test_offline.py
  1. UR3e IK round trip
  2. camera calibration (solvePnP) recovers a known pose
  3. make_scene on a simulated top-down scene used as a "real" photo: masks, matching, camera
  4. correlate on the 30 simulated eval scenes, with the GT twin standing in for the real robot
  5. the sim-probe launcher's substitutions apply to SLAT-Phys-Scene's probe_objects.py
"""
import glob
import json
import os
import shutil
import subprocess
import sys
import tempfile

import cv2
import numpy as np
from PIL import Image

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # repo root
SCENE_REPO = os.environ.get("SLATPHYS_SCENE") or sys.exit("set SLATPHYS_SCENE to your SLAT-Phys-Scene checkout")
sys.path.insert(0, os.path.join(ROOT, "src/slatphys_real"))
sys.path.insert(0, os.path.join(ROOT, "real2sim"))
TMP = tempfile.mkdtemp(prefix="slatphys_real_test_")


def test_ik():
    from slatphys_real import ur_kinematics as K
    rng = np.random.default_rng(0)
    for _ in range(2000):
        q = rng.uniform(-np.pi, np.pi, 6)
        qn = K.nearest(K.ik(K.fk(q)), q)
        assert qn is not None and np.abs(qn - q).max() < 1e-6
    # a grasp pose in the workspace is reachable from HOME without a branch flip
    home = np.array([0.0, -1.57, 1.57, -1.57, -1.57, 0.0])
    q = K.solve_tcp([-0.30, 0.05, 0.03], 0.3, 0.131, home)
    assert q is not None and np.abs(q - home).max() < 1.6, q
    print("ik ok")


def test_calibration():
    from slatphys_real.calibrate_camera import solve
    Kmat = np.array([[900.0, 0, 640], [0, 900, 360], [0, 0, 1]])
    cam2base = np.eye(4)
    cam2base[:3, :3] = cv2.Rodrigues(np.array([np.pi - 0.05, 0.03, 0.02]))[0]
    cam2base[:3, 3] = [-0.31, 0.01, 0.45]
    pts = np.array([[-0.42, -0.18, 0.0], [-0.42, 0.18, 0.0], [-0.20, 0.18, 0.0], [-0.20, -0.18, 0.0],
                    [-0.31, 0.0, 0.0], [-0.25, -0.1, 0.0]])
    b2c = np.linalg.inv(cam2base)
    c = pts @ b2c[:3, :3].T + b2c[:3, 3]
    uv = c @ Kmat.T
    uv = uv[:, :2] / uv[:, 2:] + np.random.default_rng(0).normal(0, 0.3, (len(pts), 2))
    est, err = solve(pts, uv, Kmat, np.zeros(5))
    assert np.abs(est[:3, 3] - cam2base[:3, 3]).max() < 0.003, est[:3, 3]
    print(f"calibration ok (pose error {np.abs(est[:3, 3] - cam2base[:3, 3]).max() * 1000:.2f} mm, reproj {err.mean():.2f} px)")


def test_make_scene():
    src = os.path.join(SCENE_REPO, "data/topdown_scenes/task_sort")
    scene = json.load(open(f"{src}/scene.json"))
    cam = json.load(open(f"{src}/camera.json"))
    rgb = np.array(Image.open(f"{src}/rgb.png").convert("RGB"))
    gt_masks = np.load(f"{src}/masks.npz")["masks"].astype(bool)
    import yaml
    from slatphys_real import common
    cfg = yaml.safe_load(open(os.path.join(ROOT, "configs/real.yaml")))
    tz = -0.011  # the lab table: 11 mm below the base (ur3e_experiments plans)
    T_wb = common.T_world_base(cfg, tz)
    cam2base = np.linalg.inv(T_wb) @ np.array(cam["cam2world_opencv"])
    calib = os.path.join(TMP, "camera_calib.json")
    json.dump({"K": cam["K"], "D": [0.0] * 5, "cam2base_opencv": cam2base.tolist(), "table_z_base": tz}, open(calib, "w"))
    cap, emp = os.path.join(TMP, "cap"), os.path.join(TMP, "empty")
    os.makedirs(cap), os.makedirs(emp)
    Image.fromarray(rgb).save(f"{cap}/rgb.png")
    hole = cv2.dilate(gt_masks.any(0).astype(np.uint8) * 255, np.ones((25, 25), np.uint8))
    Image.fromarray(cv2.inpaint(rgb, hole, 15, cv2.INPAINT_TELEA)).save(f"{emp}/rgb.png")
    # plan in shuffled order, positions in base
    order = [2, 0, 1]
    plan = {"scene_id": "sort_test", "task": "sort", "instruction": scene["instruction"], "objects": [
        {"id": scene["objects"][i]["obj_id"][:6], "name": scene["objects"][i]["name"], "mass_g": 10.0 * (i + 1),
         "size_mm": [scene["objects"][i]["scale"] * 1000] * 3, "soft": False,
         "xy_base": common.world_xy_to_base(cfg, scene["objects"][i]["position"][:2]).round(3).tolist()} for i in order]}
    plan_p = os.path.join(TMP, "plan.json")
    json.dump(plan, open(plan_p, "w"))
    out = os.path.join(TMP, "scenes")
    subprocess.run([sys.executable, os.path.join(ROOT, "real2sim/make_scene.py"), "--config", os.path.join(ROOT, "configs/real.yaml"),
                    "--plan", plan_p, "--capture", cap, "--empty", emp, "--calib", calib, "--out", out], check=True)
    d = os.path.join(out, "sort_test")
    m = np.load(f"{d}/masks.npz")["masks"]
    for k, i in enumerate(order):
        iou = (m[k] & gt_masks[i]).sum() / (m[k] | gt_masks[i]).sum()
        assert iou > 0.85, (k, i, iou)
    c2w = np.array(json.load(open(f"{d}/camera.json"))["cam2world_opencv"])
    assert np.allclose(c2w, cam["cam2world_opencv"], atol=1e-9)
    s = json.load(open(f"{d}/scene.json"))
    for k, i in enumerate(order):
        assert np.linalg.norm(np.array(s["objects"][k]["position"][:2]) - scene["objects"][i]["position"][:2]) < 0.012
    inst = np.array(Image.open(f"{d}/instance_mask.png"))[..., 0]
    assert (inst == 255).mean() > 0.5
    # Ground truth not measured yet (as in data/plans/*.json before weighing): must still work.
    for o in plan["objects"]:
        o["mass_g"] = o["size_mm"] = o["soft"] = None
    plan["scene_id"] = "sort_unmeasured"
    json.dump(plan, open(plan_p, "w"))
    subprocess.run([sys.executable, os.path.join(ROOT, "real2sim/make_scene.py"), "--config", os.path.join(ROOT, "configs/real.yaml"),
                    "--plan", plan_p, "--capture", cap, "--empty", emp, "--calib", calib, "--out", out], check=True)
    s = json.load(open(os.path.join(out, "sort_unmeasured", "scene.json")))
    assert all(o["real"]["mass_g"] is None and o["real"]["soft"] is None for o in s["objects"])
    print("make_scene ok (mask IoU > 0.85, camera exact, positions < 1.2 cm; unmeasured plan accepted)")


def test_frames_and_reach():
    """World <-> base round trip; the lab home, the hold point and the whole planning band are
    reachable top-down from the lab's home configuration without IK branch flips."""
    import yaml
    from slatphys_real import common, ur_kinematics as K
    cfg = yaml.safe_load(open(os.path.join(ROOT, "configs/real.yaml")))
    tz = -0.011
    T = common.T_world_base(cfg, tz)
    p = np.array([0.2, -0.3, tz])
    pw = T[:3, :3] @ p + T[:3, 3]
    assert np.allclose(common.world_xy_to_base(cfg, pw[:2]), p[:2]) and abs(pw[2] - 0.75) < 1e-12
    home = np.array(cfg["home_q"])
    hold = common.hold_point_base(cfg, tz)
    tcp_z = cfg["gripper"]["tcp_z"]
    assert K.solve_tcp(hold, 0.0, tcp_z, home) is not None
    worst = 0.0
    for dx in np.linspace(-0.38, -0.24, 5):
        for dy in np.linspace(-0.15, 0.15, 5):
            xy = common.world_xy_to_base(cfg, np.array(cfg["world"]["robot_base_xy"]) + [dx, dy])
            for z in (tz + 0.025, tz + 0.15):
                for yaw in np.linspace(-np.pi / 2, np.pi / 2, 5):
                    q = K.solve_tcp([*xy, z], yaw, tcp_z, home)
                    assert q is not None, (xy, z, yaw)
                    worst = max(worst, np.abs(q[1:4] - home[1:4]).max())
    print(f"frames ok (hold {np.round(hold, 3).tolist()} in base; whole band reachable, "
          f"max shoulder/elbow/wrist1 change from home {np.degrees(worst):.0f} deg)")


def test_correlate():
    """The GT twin plays 'real' and the SLAT-Phys twin 'sim' on SLAT-Phys-Scene's 30 eval scenes."""
    scenes_dir, real_dir = os.path.join(TMP, "corr_scenes"), os.path.join(TMP, "corr_real")
    n = 0
    for sp in glob.glob(os.path.join(SCENE_REPO, "data/eval_scenes/*/scene.json")):
        s = json.load(open(sp))
        sid = s["scene_id"]
        rp = os.path.join(SCENE_REPO, "outputs/eval_sim/gt", sid, "readings.json")
        if not os.path.exists(rp):
            continue
        for o in s["objects"]:
            o["real"] = {"mass_g": o["physics"]["mass_kg"] * 1000, "soft": bool(o["material"]["deformable"])}
        os.makedirs(os.path.join(scenes_dir, sid))
        json.dump(s, open(os.path.join(scenes_dir, sid, "scene.json"), "w"))
        os.makedirs(os.path.join(real_dir, sid))
        shutil.copy(rp, os.path.join(real_dir, sid, "readings.json"))
        n += 1
    assert n >= 20, n
    out = os.path.join(TMP, "results")
    subprocess.run([sys.executable, os.path.join(ROOT, "real2sim/correlate.py"), "--scenes", scenes_dir,
                    "--sim", os.path.join(SCENE_REPO, "outputs/eval_sim/slatphys"), "--real", real_dir, "--out", out], check=True)
    m = json.load(open(os.path.join(out, "metrics.json")))
    assert m["objects"]["load"]["n"] > 30
    print("correlate ok")


def test_launcher():
    import yaml
    import sim_probe_launcher as L
    src = L.patched_source(yaml.safe_load(open(os.path.join(ROOT, "configs/real.yaml")))["protocol"])
    compile(src, L.PROBE, "exec")
    assert "SQUEEZE_N = (20.0, 60.0)" in src and "default=0.005," in src
    print("launcher ok")


if __name__ == "__main__":
    test_ik()
    test_calibration()
    test_frames_and_reach()
    test_make_scene()
    test_correlate()
    test_launcher()
    print("all offline tests passed; scratch:", TMP)
