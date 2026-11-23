"""Find the objects in the calibrated top-down photo and write their grasps (grasps.json) for
run_tasks. No reconstruction, no simulation: the camera calibration (calibrate_camera) maps
pixels to the table, and the plan supplies each object's measured height (a camera looking
straight down cannot see height).

  1. undistort the scene photo and the empty-table photo (same camera pose, arm at home)
  2. object masks: change of colour against the empty table (chromaticity, so the objects' shadows,
     which only darken the table, drop out), cleaned, the N largest blobs; or hand-made masks (--masks_dir)
  3. every mask pixel -> a ray from the camera -> where it meets the plane at the object's
     mid-height, in the robot `base` frame
  4. per object, the minimum-area rectangle around those footprint points: the closing axis runs
     across its narrow side, the grasp width = that side, the centre = the rectangle's centre
  5. blobs -> plan names by nearest planned position (xy_base); a warning if one is > 6 cm off

    ros2 run slatphys_real capture --out data/real/captures/_empty --execute MOVE_REAL_UR3E   # empty table
    ros2 run slatphys_real capture --out data/real/captures/scene --execute MOVE_REAL_UR3E    # objects placed
    ros2 run slatphys_real locate --plan data/plans/scene.json --capture data/real/captures/scene \
        --empty data/real/captures/_empty --out data/real/grasps/scene.json    # check the *_overlay.png
"""

import argparse
import json
import os
import sys

import cv2
import numpy as np
from PIL import Image
from scipy import ndimage
from scipy.optimize import linear_sum_assignment

from slatphys_real import common

MATCH_WARN_M = 0.06


def undistort(rgb, K, D):
    K, D = np.asarray(K, float), np.asarray(D, float)
    if D.size == 0 or np.allclose(D, 0):
        return rgb, K
    h, w = rgb.shape[:2]
    newK, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 0)
    return cv2.undistort(rgb, K, D, None, newK), newK


def diff_masks(rgb, empty, n, thresh, min_area, bright=25.0):
    """The n largest blobs that differ from the empty table. A shadow makes the table darker but
    keeps its colour, so the test is a change of chromaticity (colour independent of brightness,
    r, g, b over their sum), or the pixel getting brighter; darkening alone is ignored."""
    a = cv2.GaussianBlur(rgb.astype(np.float32), (5, 5), 0) + 1.0
    b = cv2.GaussianBlur(empty.astype(np.float32), (5, 5), 0) + 1.0
    ca, cb = a / a.sum(2, keepdims=True), b / b.sum(2, keepdims=True)
    chroma = np.linalg.norm(ca - cb, axis=2) * 1000  # ~0 for shadows, tens for a coloured object
    brighter = a.mean(2) - b.mean(2) > bright
    fg = cv2.morphologyEx(((chroma > thresh) | brighter).astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    fg = ndimage.binary_fill_holes(cv2.morphologyEx(fg, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8)))
    lab, k = ndimage.label(fg)
    areas = ndimage.sum(fg, lab, range(1, k + 1))
    order = [i + 1 for i in np.argsort(-areas) if areas[i] >= min_area]
    if len(order) < n:
        raise RuntimeError(f"found {len(order)} blobs >= {min_area} px for {n} objects: lower --thresh / --min_area, "
                           "or draw masks (--masks_dir). Objects need a colour different from the table.")
    if len(order) > n:
        print(f"  note: {len(order) - n} smaller blob(s) ignored (areas {sorted(areas[np.array(order[n:]) - 1].astype(int))} px)")
    return [lab == i for i in order[:n]]


def pixels_to_plane(uv, K, cam2base, z):
    """Pixels (N, 2) -> points (N, 3) in base where their camera rays meet the plane z."""
    rays = np.linalg.inv(K) @ np.vstack([uv.T, np.ones(len(uv))])
    o, r = cam2base[:3, 3], cam2base[:3, :3] @ rays
    t = (z - o[2]) / r[2]
    return (o[:, None] + t * r).T


def footprint_grasp(xy):
    """Grasp from footprint points (N, 2), in metres: the minimum-area rectangle around them. The
    closing axis runs across its narrow side, the width is that side, the centre is its centre.
    (A principal-axis fit picks an arbitrary axis for square footprints and then measures nearly
    the diagonal.)"""
    (cx, cy), (w, h), ang = cv2.minAreaRect((xy * 1e4).astype(np.float32))  # 0.1 mm units
    ang = np.radians(ang)
    axis_w = np.array([np.cos(ang), np.sin(ang)])  # direction of the side of length w
    if w <= h:
        narrow, width, length = axis_w, w, h
    else:
        narrow, width, length = np.array([-axis_w[1], axis_w[0]]), h, w
    return np.array([cx, cy]) / 1e4, float(np.arctan2(narrow[1], narrow[0])), width / 1e4, length / 1e4


def overlay(rgb, masks, objs, K, cam2base, path):
    out = rgb.astype(np.float32).copy()
    colors = [(230, 25, 75), (0, 130, 200), (60, 180, 75), (245, 130, 48)]
    for k, (m, o) in enumerate(zip(masks, objs)):
        out[m] = 0.55 * out[m] + 0.45 * np.array(colors[k % 4])
    out = out.clip(0, 255).astype(np.uint8)
    base2cam = np.linalg.inv(cam2base)
    for k, o in enumerate(objs):
        g = o["grasp"]
        z = g["top_z"] - g["height"] / 2
        d = np.array([np.cos(g["yaw"]), np.sin(g["yaw"])]) * g["width"] / 2
        pts = [np.array([*np.array(g["centre_xy"]) + s * d, z]) for s in (-1, 0, 1)]
        uv = [K @ (base2cam[:3, :3] @ p + base2cam[:3, 3]) for p in pts]
        uv = [(int(u[0] / u[2]), int(u[1] / u[2])) for u in uv]
        cv2.line(out, uv[0], uv[2], (255, 255, 255), 2)   # the closing axis, fingers at its ends
        cv2.circle(out, uv[1], 4, (255, 255, 255), -1)
        cv2.putText(out, f"{k + 1} {o['name']} w={g['width'] * 1000:.0f}mm", (uv[1][0] + 8, uv[1][1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, colors[k % 4], 2)
    Image.fromarray(out).save(path)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=None)
    ap.add_argument("--plan", required=True, help="scene plan: names, xy_base, size_mm (height = size_mm[2])")
    ap.add_argument("--capture", required=True, help="scene capture dir (rgb.png)")
    ap.add_argument("--empty", required=True, help="empty-table capture dir (rgb.png), same camera pose")
    ap.add_argument("--calib", default=None, help="camera.json (default: <calib_dir>/camera.json)")
    ap.add_argument("--out", required=True, help="grasps.json to write (an *_overlay.png goes next to it)")
    ap.add_argument("--masks_dir", default=None, help="hand-made masks obj_1.png ... in plan order (skips differencing)")
    ap.add_argument("--thresh", type=float, default=20.0, help="chromaticity change x1000 that counts as object")
    ap.add_argument("--min_area", type=int, default=150)
    args = ap.parse_args(argv if argv is not None else sys.argv[1:])

    cfg = common.load_config(args.config)
    cal = json.load(open(args.calib)) if args.calib else common.load_camera_calib(cfg)
    plan = json.load(open(args.plan))
    tz = common.table_z(cfg) if cfg.get("table_z_base") is not None else float(cal["table_z_base"])
    cam2base = np.asarray(cal["cam2base_opencv"], float)
    rgb = np.array(Image.open(os.path.join(args.capture, "rgb.png")).convert("RGB"))
    empty = np.array(Image.open(os.path.join(args.empty, "rgb.png")).convert("RGB"))
    rgb, K = undistort(rgb, cal["K"], cal.get("D", []))
    empty, _ = undistort(empty, cal["K"], cal.get("D", []))
    n = len(plan["objects"])
    for o in plan["objects"]:
        if not o.get("size_mm") or o["size_mm"][2] is None:
            raise SystemExit(f"{o['name']}: size_mm [w, l, height] missing in the plan; the height is needed for the grasp")

    if args.masks_dir:
        masks = [np.array(Image.open(os.path.join(args.masks_dir, f"obj_{k + 1}.png")).convert("L")) > 127 for k in range(n)]
    else:
        masks = diff_masks(rgb, empty, n, args.thresh, args.min_area)

    # Blobs -> plan objects by position (centroid on the table plane vs the planned xy_base).
    cents = [pixels_to_plane(np.array(ndimage.center_of_mass(m))[None, ::-1], K, cam2base, tz)[0, :2] for m in masks]
    if args.masks_dir or not all("xy_base" in o for o in plan["objects"]):
        assign = list(range(n))
    else:
        cost = np.array([[np.linalg.norm(c - np.array(o["xy_base"])) for c in cents] for o in plan["objects"]])
        rows, cols = linear_sum_assignment(cost)
        assign = [int(cols[list(rows).index(k)]) for k in range(n)]
        for k in range(n):
            if cost[k, assign[k]] > MATCH_WARN_M:
                print(f"  WARNING: {plan['objects'][k]['name']} found {cost[k, assign[k]] * 100:.1f} cm from its planned position")
    masks = [masks[j] for j in assign]

    objs = []
    for o, m in zip(plan["objects"], masks):
        h = o["size_mm"][2] / 1000
        vs, us = np.nonzero(m)
        pts = pixels_to_plane(np.column_stack([us, vs]).astype(float), K, cam2base, tz + h / 2)
        centre, yaw, width, length = footprint_grasp(pts[:, :2])
        objs.append({"name": o["name"], "obj_id": o.get("id", o["name"]), "grasp": {
            "centre_xy": np.round(centre, 4).tolist(), "yaw": yaw, "width": width, "top_z": tz + h, "height": h},
            "footprint_length_m": length, "mask_area_px": int(m.sum())})
        print(f"  {o['name']:14s} centre {np.round(centre, 3).tolist()}  yaw {np.degrees(yaw):6.1f} deg  "
              f"width {width * 1000:5.1f} mm  length {length * 1000:5.1f} mm  height {h * 1000:5.1f} mm (plan)")

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    json.dump(common.to_jsonable({"scene_id": plan["scene_id"], "task": plan.get("task", "heavy_deformable"),
                                  "frame": "base", "table_z_base": tz, "source": "camera (locate)", "objects": objs}),
              open(args.out, "w"), indent=2)
    ov = os.path.splitext(args.out)[0] + "_overlay.png"
    overlay(rgb, masks, objs, K, cam2base, ov)
    print(f"wrote {args.out} and {ov} (check: each mask on its object, the white line across the narrow side)")


if __name__ == "__main__":
    main()
