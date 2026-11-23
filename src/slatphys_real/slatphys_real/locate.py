"""Find the objects in a calibrated camera photo and write their grasps (grasps.json) for run_tasks.
No reconstruction, no simulation. The camera may look at the table from the front at an angle
(the expected setup, e.g. 30-50 deg down) or straight down: every object is modelled as a box
of its measured size (the plan's size_mm, calipers), and its pose on the table is the one whose
projected box best covers the object in the photo.

  1. undistort the scene photo and the empty-table photo (same camera pose, same arm pose)
  2. object masks: change of colour against the empty table (chromaticity, so the objects'
     shadows, which only darken the table, drop out), cleaned, the N largest blobs; or
     hand-made masks (--masks_dir obj_1.png ..., in plan order)
  3. blobs -> plan objects by nearest planned position (xy_base); a warning if one is > 6 cm off
  4. per object, a search over (x, y, yaw) on the table: the box (narrow side, long side, height
     from size_mm) is projected through the calibrated camera, and the pose with the highest
     overlap (IoU) with the mask wins; coarse grid, then a fine one around the best
  5. the grasp: centre = the box centre, closing axis across the narrow side, width = the narrow
     side, top = table + height

Seen from the front, objects must stand side by side, not one behind the other (the one in front
would hide the other). Check the overlay: each box outline should sit on its object.

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
MIN_IOU = 0.70  # below this the box fit is suspect: check the overlay
COARSE = {"xy_range": 0.05, "xy_step": 0.004, "yaw_step": np.radians(5)}
FINE = {"xy_range": 0.004, "xy_step": 0.001, "yaw_range": np.radians(5), "yaw_step": np.radians(1)}


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


def box_corners(c, yaw, narrow, long_, z0, z1):
    """The 8 corners (8, 3) in base of a box standing on z0: `narrow` along yaw, `long_` across."""
    u, v = np.array([np.cos(yaw), np.sin(yaw)]), np.array([-np.sin(yaw), np.cos(yaw)])
    xy = [c + su * narrow / 2 * u + sv * long_ / 2 * v for su in (-1, 1) for sv in (-1, 1)]
    return np.array([[*p, z] for z in (z0, z1) for p in xy])


class BoxFitter:
    """Overlap (IoU) between an object's mask and the projection of a box at a given pose."""

    def __init__(self, mask, K, cam2base, tz, narrow, long_, height):
        ys, xs = np.nonzero(mask)
        pad = 40
        self.x0, self.y0 = max(xs.min() - pad, 0), max(ys.min() - pad, 0)
        x1, y1 = min(xs.max() + pad, mask.shape[1]), min(ys.max() + pad, mask.shape[0])
        self.m = mask[self.y0:y1, self.x0:x1]
        self.area = self.m.sum()
        self.P = np.asarray(K) @ np.linalg.inv(cam2base)[:3]  # base -> pixels
        self.dims, self.tz = (narrow, long_, height), tz
        self.buf = np.zeros(self.m.shape, np.uint8)

    def project(self, c, yaw):
        X = box_corners(c, yaw, self.dims[0], self.dims[1], self.tz, self.tz + self.dims[2])
        uvw = self.P @ np.vstack([X.T, np.ones(8)])
        return (uvw[:2] / uvw[2]).T

    def iou(self, c, yaw):
        uv = self.project(c, yaw) - [self.x0, self.y0]
        self.buf[:] = 0
        cv2.fillConvexPoly(self.buf, cv2.convexHull(np.round(uv).astype(np.int32)), 1)
        inter = np.logical_and(self.buf, self.m).sum()
        return inter / (self.area + self.buf.sum() - inter)

    def fit(self, c0):
        """Best (centre, yaw, iou): coarse grid around c0 over all distinct yaws, then a fine grid
        around the best."""
        def search(centre, xy_range, xy_step, yaws):
            best = (-1.0, None, None)
            offs = np.arange(-xy_range, xy_range + 1e-9, xy_step)
            for yaw in yaws:
                for dx in offs:
                    for dy in offs:
                        c = centre + [dx, dy]
                        s = self.iou(c, yaw)
                        if s > best[0]:
                            best = (s, c, yaw)
            return best
        # A square footprint repeats every 90 deg (the fingers should still meet its faces), a box every 180.
        period = np.pi / 2 if abs(self.dims[0] - self.dims[1]) < 0.002 else np.pi
        s, c, yaw = search(np.asarray(c0, float), COARSE["xy_range"], COARSE["xy_step"],
                           np.arange(0, period, COARSE["yaw_step"]))
        s, c, yaw = search(c, FINE["xy_range"], FINE["xy_step"],
                           yaw + np.arange(-FINE["yaw_range"], FINE["yaw_range"] + 1e-9, FINE["yaw_step"]))
        return c, float((yaw + np.pi / 2) % np.pi - np.pi / 2), float(s)


def overlay(rgb, masks, fits, objs, path):
    out = rgb.astype(np.float32).copy()
    colors = [(230, 25, 75), (0, 130, 200), (60, 180, 75), (245, 130, 48)]
    for k, m in enumerate(masks):
        out[m] = 0.6 * out[m] + 0.4 * np.array(colors[k % 4])
    out = np.ascontiguousarray(out.clip(0, 255).astype(np.uint8))
    for k, (f, o) in enumerate(zip(fits, objs)):
        g = o["grasp"]
        uv = np.round(f.project(np.array(g["centre_xy"]), g["yaw"])).astype(int)
        for i, j in [(0, 1), (1, 3), (3, 2), (2, 0), (4, 5), (5, 7), (7, 6), (6, 4), (0, 4), (1, 5), (2, 6), (3, 7)]:
            cv2.line(out, tuple(uv[i]), tuple(uv[j]), (255, 255, 255), 1, cv2.LINE_AA)  # the fitted box
        # the closing axis at the grasp height: the fingers meet the narrow faces
        z = g["top_z"] - g["height"] / 2
        d = np.array([np.cos(g["yaw"]), np.sin(g["yaw"])]) * g["width"] / 2
        pts = np.array([[*(np.array(g["centre_xy"]) + s * d), z, 1.0] for s in (-1, 0, 1)])
        p = f.P @ pts.T
        p = np.round((p[:2] / p[2]).T).astype(int)
        cv2.line(out, tuple(p[0]), tuple(p[2]), (255, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(out, f"{k + 1} {o['name']} IoU {o['fit_iou']:.2f}", (int(uv[:, 0].min()), int(uv[:, 1].min()) - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, colors[k % 4], 2)
    Image.fromarray(out).save(path)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=None)
    ap.add_argument("--plan", required=True, help="scene plan: names, xy_base, size_mm [w, l, height] (calipers)")
    ap.add_argument("--capture", required=True, help="scene capture dir (rgb.png)")
    ap.add_argument("--empty", required=True, help="empty-table capture dir (rgb.png), same camera and arm pose")
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
        if not o.get("size_mm") or len(o["size_mm"]) != 3 or None in o["size_mm"]:
            raise SystemExit(f"{o['name']}: size_mm [w, l, height] (mm, calipers) is missing in the plan; locate needs it")

    if args.masks_dir:
        masks = [np.array(Image.open(os.path.join(args.masks_dir, f"obj_{k + 1}.png")).convert("L")) > 127 for k in range(n)]
    else:
        masks = diff_masks(rgb, empty, n, args.thresh, args.min_area)

    # A first position per blob: its centroid cast onto the plane at the objects' typical mid-height.
    h_mid = np.median([o["size_mm"][2] for o in plan["objects"]]) / 2000
    cents = [pixels_to_plane(np.array(ndimage.center_of_mass(m))[None, ::-1], K, cam2base, tz + h_mid)[0, :2] for m in masks]
    if args.masks_dir or not all("xy_base" in o for o in plan["objects"]):
        assign = list(range(n))
    else:
        cost = np.array([[np.linalg.norm(c - np.array(o["xy_base"])) for c in cents] for o in plan["objects"]])
        rows, cols = linear_sum_assignment(cost)
        assign = [int(cols[list(rows).index(k)]) for k in range(n)]
        for k in range(n):
            if cost[k, assign[k]] > MATCH_WARN_M:
                print(f"  WARNING: {plan['objects'][k]['name']} found {cost[k, assign[k]] * 100:.1f} cm from its planned position")
    masks, cents = [masks[j] for j in assign], [cents[j] for j in assign]

    objs, fits = [], []
    for o, m, c0 in zip(plan["objects"], masks, cents):
        w, l, h = (v / 1000 for v in o["size_mm"])
        narrow, long_ = min(w, l), max(w, l)
        f = BoxFitter(m, K, cam2base, tz, narrow, long_, h)
        centre, yaw, iou = f.fit(c0)
        fits.append(f)
        objs.append({"name": o["name"], "obj_id": o.get("id", o["name"]), "grasp": {
            "centre_xy": np.round(centre, 4).tolist(), "yaw": yaw, "width": narrow, "top_z": tz + h, "height": h},
            "fit_iou": round(iou, 3), "mask_area_px": int(m.sum())})
        print(f"  {o['name']:14s} centre {np.round(centre, 3).tolist()}  yaw {np.degrees(yaw):6.1f} deg  "
              f"box {narrow * 1000:.0f} x {long_ * 1000:.0f} x {h * 1000:.0f} mm  fit IoU {iou:.2f}")
        if iou < MIN_IOU:
            print(f"  WARNING: {o['name']}: the box covers its mask poorly (IoU {iou:.2f} < {MIN_IOU}): check size_mm, "
                  "the mask and the overlay; is it hidden behind another object?")

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    json.dump(common.to_jsonable({"scene_id": plan["scene_id"], "task": plan.get("task", "heavy_deformable"),
                                  "frame": "base", "table_z_base": tz, "source": "camera (locate, box fit)", "objects": objs}),
              open(args.out, "w"), indent=2)
    ov = os.path.splitext(args.out)[0] + "_overlay.png"
    overlay(rgb, masks, fits, objs, ov)
    print(f"wrote {args.out} and {ov} (check: each white box on its object, the yellow closing axis across the narrow side)")


if __name__ == "__main__":
    main()
