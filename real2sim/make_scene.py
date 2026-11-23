"""Turn a real top-down photo into a SLAT-Phys-Scene task scene, so SLAT-Phys-Scene's
recon_task_scenes.py reconstructs it unchanged.

Inputs: the scene photo and an empty-table photo (ros2 run slatphys_real capture), the camera
calibration (calibrate_camera), the scene plan (plan_scenes.py) and the inventory.

  1. undistort both photos (the reconstruction assumes a pinhole camera)
  2. object masks: colour difference to the empty table (Lab, lightness down-weighted so
     shadows drop out), cleaned, the N largest blobs; or hand-made masks (--masks_dir,
     obj_1.png ... white = object) when differencing fails (glare, table-coloured objects)
  3. match the blobs to the planned objects by their positions on the table
  4. write <out>/<scene_id>/: rgb.png, masks.npz, instance_mask.png (table = 255),
     camera.json (K, cam2world_opencv in the simulation's world frame), scene.json, overlay.png

World frame (SLAT-Phys-Scene): common.T_world_base (the base rotated so the workspace lies toward
world -x, as in the simulation, and shifted so the table top is at z = table_height). Real ground truth (measured mass, soft) goes in scene.json.

    python real2sim/make_scene.py --plan data/real/plans/heavy_01.json \
        --capture data/real/captures/heavy_01 --empty data/real/captures/_empty --out data/real/scenes
"""

import argparse
import json
import os
import sys

import cv2
import numpy as np
import yaml
from PIL import Image
from scipy import ndimage
from scipy.optimize import linear_sum_assignment

# Frame conventions live in the ROS package (slatphys_real/common.py, no ROS imports) so the robot
# and these offline scripts share one definition.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src/slatphys_real"))
from slatphys_real import common  # noqa: E402

TABLE_ID = 255
NAN = float("nan")


def load_rgb(d):
    return np.array(Image.open(os.path.join(d, "rgb.png")).convert("RGB"))


def undistort(rgb, K, D):
    K, D = np.asarray(K, float), np.asarray(D, float)
    if D.size == 0 or np.allclose(D, 0):
        return rgb, K
    h, w = rgb.shape[:2]
    newK, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 0)
    return cv2.undistort(rgb, K, D, None, newK), newK


def diff_masks(rgb, empty, n, thresh, min_area, l_weight=0.35):
    a = cv2.cvtColor(rgb, cv2.COLOR_RGB2LAB).astype(np.float32)
    b = cv2.cvtColor(empty, cv2.COLOR_RGB2LAB).astype(np.float32)
    a, b = cv2.GaussianBlur(a, (5, 5), 0), cv2.GaussianBlur(b, (5, 5), 0)
    d = np.sqrt((l_weight * (a[..., 0] - b[..., 0])) ** 2 + (a[..., 1] - b[..., 1]) ** 2 + (a[..., 2] - b[..., 2]) ** 2)
    fg = d > thresh
    fg = cv2.morphologyEx(fg.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    fg = cv2.morphologyEx(fg, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    fg = ndimage.binary_fill_holes(fg)
    lab, k = ndimage.label(fg)
    areas = ndimage.sum(fg, lab, range(1, k + 1))
    order = [i + 1 for i in np.argsort(-areas) if areas[i] >= min_area]
    if len(order) < n:
        raise RuntimeError(f"found {len(order)} blobs >= {min_area} px for {n} objects: lower --thresh / --min_area,"
                           " or draw masks (--masks_dir)")
    if len(order) > n:
        print(f"  note: {len(order) - n} extra blob(s) ignored (areas {sorted(areas[np.array(order[n:]) - 1].astype(int))} px)")
    return [lab == i for i in order[:n]], d


def pixel_to_table(uv, K, cam2base, tz):
    ray = np.linalg.inv(K) @ np.array([uv[0], uv[1], 1.0])
    o, r = cam2base[:3, 3], cam2base[:3, :3] @ ray
    t = (tz - o[2]) / r[2]
    return o + t * r


def table_to_pixel(p, K, cam2base):
    base2cam = np.linalg.inv(cam2base)
    c = base2cam[:3, :3] @ p + base2cam[:3, 3]
    uv = K @ c
    return uv[:2] / uv[2]


def overlay(rgb, masks, names, path):
    out = rgb.astype(np.float32).copy()
    colors = [(230, 25, 75), (60, 180, 75), (0, 130, 200), (245, 130, 48)]
    for k, (m, name) in enumerate(zip(masks, names)):
        out[m] = 0.5 * out[m] + 0.5 * np.array(colors[k % 4])
        ys, xs = np.nonzero(m)
        cv2.putText(out, f"{k + 1} {name}", (int(xs.min()), max(int(ys.min()) - 6, 12)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, colors[k % 4], 2)
    Image.fromarray(out.clip(0, 255).astype(np.uint8)).save(path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="configs/real.yaml")
    ap.add_argument("--plan", required=True)
    ap.add_argument("--capture", required=True, help="scene capture dir (rgb.png)")
    ap.add_argument("--empty", required=True, help="empty-table capture dir (rgb.png), same camera pose")
    ap.add_argument("--calib", default=None, help="camera.json (default: <calib_dir>/camera.json)")
    ap.add_argument("--masks_dir", default=None, help="hand-made masks obj_1.png ... in plan order (skips differencing)")
    ap.add_argument("--thresh", type=float, default=18.0)
    ap.add_argument("--min_area", type=int, default=800)
    ap.add_argument("--max_match_cm", type=float, default=6.0, help="blob-to-plan distance that counts as misplaced")
    ap.add_argument("--out", default="data/real/scenes")
    args = ap.parse_args()

    cfg = yaml.safe_load(open(args.config))
    calib = json.load(open(args.calib or os.path.join(cfg["calib_dir"], "camera.json")))
    plan = json.load(open(args.plan))
    n = len(plan["objects"])
    cam2base = np.array(calib["cam2base_opencv"])
    tz = float(calib["table_z_base"])
    rgb, K = undistort(load_rgb(args.capture), calib["K"], calib["D"])
    empty, _ = undistort(load_rgb(args.empty), calib["K"], calib["D"])
    if rgb.shape != empty.shape:
        raise ValueError("scene and empty-table photos differ in size")

    if args.masks_dir:
        masks = [np.array(Image.open(os.path.join(args.masks_dir, f"obj_{k + 1}.png")).convert("L")) > 127 for k in range(n)]
        order = list(range(n))
    else:
        masks, _ = diff_masks(rgb, empty, n, args.thresh, args.min_area)
        # Match blobs to planned positions (Hungarian on the table-plane distance).
        cents = [pixel_to_table(np.array(ndimage.center_of_mass(m))[::-1], K, cam2base, tz)[:2] for m in masks]
        cost = np.array([[np.linalg.norm(c - np.array(o["xy_base"])) for c in cents] for o in plan["objects"]])
        rows, cols = linear_sum_assignment(cost)
        order = list(cols[np.argsort(rows)])
        for k, j in enumerate(order):
            if cost[k, j] > args.max_match_cm / 100:
                print(f"  WARNING: {plan['objects'][k]['id']} is {cost[k, j] * 100:.1f} cm from its planned position")
    masks = [masks[j] for j in order]

    T_world_base = common.T_world_base(cfg, tz)
    cam2world = T_world_base @ cam2base
    table_h = cfg["world"]["table_height"]

    objects = []
    for o, m in zip(plan["objects"], masks):
        uv = np.array(ndimage.center_of_mass(m))[::-1]
        p_world = (T_world_base @ np.append(pixel_to_table(uv, K, cam2base, tz), 1.0))[:3]
        # Ground truth may not be measured yet (null in the plan): NaN / None, filled in before scoring.
        mass = o["mass_g"] / 1000 if o.get("mass_g") is not None else NAN
        scale = max(o["size_mm"]) / 1000 if o.get("size_mm") else NAN
        soft = None if o.get("soft") is None else bool(o["soft"])
        objects.append({
            "obj_id": o["id"], "name": o["name"], "class": o.get("class", ""), "scale": scale,
            "position": p_world.tolist(),
            # The real objects' density / stiffness aren't measured: NaN keeps recon_task_scenes'
            # GT printout working. The real ground truth is mass_kg and soft.
            "material": {"name": o.get("class", ""), "density": NAN, "E": NAN, "nu": NAN, "deformable": soft},
            "physics": {"mass_kg": mass},
            "real": {"mass_g": o.get("mass_g"), "soft": soft, "size_mm": o.get("size_mm"),
                     "xy_base_planned": o["xy_base"], "mask_area_px": int(m.sum())},
        })

    sdir = os.path.join(args.out, plan["scene_id"])
    os.makedirs(sdir, exist_ok=True)
    Image.fromarray(rgb).save(os.path.join(sdir, "rgb.png"))
    np.savez_compressed(os.path.join(sdir, "masks.npz"), masks=np.stack(masks).astype(bool))
    inst = np.full(rgb.shape[:2], TABLE_ID, np.uint8)
    near = cv2.dilate(np.any(masks, 0).astype(np.uint8), np.ones((15, 15), np.uint8)) > 0
    inst[near] = 0  # object borders and shadows: neither table nor object
    for k, m in enumerate(masks):
        inst[m] = k + 1
    Image.fromarray(np.stack([inst] * 3, -1)).save(os.path.join(sdir, "instance_mask.png"))
    json.dump({"K": np.asarray(K).tolist(), "resolution": [rgb.shape[1], rgb.shape[0]], "cam2world_opencv": cam2world.tolist(),
               "cam2base_opencv": cam2base.tolist(), "table_z_base": tz, "source": "real camera, undistorted"},
              open(os.path.join(sdir, "camera.json"), "w"), indent=2)
    json.dump({"scene_id": plan["scene_id"], "task": plan["task"], "instruction": plan["instruction"],
               "robot_base_xy": cfg["world"]["robot_base_xy"],
               "table": {"size": [1.2, 0.8, 0.04], "height": table_h}, "objects": objects, "real": True},
              open(os.path.join(sdir, "scene.json"), "w"), indent=2)
    overlay(rgb, masks, [o["name"] for o in plan["objects"]], os.path.join(sdir, "overlay.png"))
    print(f"{plan['scene_id']}: wrote {sdir} (check overlay.png); camera {cam2world[2, 3] - table_h:.3f} m above the table")


if __name__ == "__main__":
    main()
