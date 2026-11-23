"""One uncalibrated top-down phone photo -> a SLAT-Phys-Scene task scene (the paper's input format).

The paper's scenes (SLAT-Phys-Scene/data/eval_scenes/*) are a 1024x768 top-down image plus
masks.npz, instance_mask.png (table = 255), camera.json (K, cam2world_opencv) and scene.json.
recon_task_scenes.py needs the camera only for metric scale: it rescales SAM-3D's point map so
the camera-to-table distance matches camera.json, then fits each object's silhouette to its mask.

A phone photo has no calibration, so two things are assumed (both stored in camera.json):
  K           from the phone's focal length (iPhone main camera, 26 mm equivalent, by default)
  cam_height  camera-to-table distance in metres: THE metric scale of the whole reconstruction.
              Object sizes scale linearly with it and masses with its cube. Ratios between
              objects in the same photo do not depend on it, so the heavy / deformable answers
              only depend on it through graspability (the Hand-E opens 50 mm).
The camera is placed straight down above the table like the paper's eval scenes: image up =
world +x = toward the robot base, image right = world -y.

Masks: rembg (IS-Net) on a crop around one box per object (--boxes, original-image pixels), or
hand-made masks (--masks_dir obj_1.png ..., white = object).

    python real2sim/photo_scene.py --image data/IMG_4827.jpg --scene_id img4827 \
        --names yellow_toy green_disc --boxes 860,1000,1470,1740 1870,900,2760,1800
"""

import argparse
import json
import os

import cv2
import numpy as np
from PIL import Image, ImageOps
from scipy import ndimage

TABLE_ID = 255
OUT_WH = (1024, 768)  # the paper's eval-scene resolution
# The paper's world: robot base at (0.5, 0) on a 0.75 m table, objects toward -x.
ROBOT_BASE_XY = [0.5, 0.0]
TABLE = {"size": [1.2, 0.8, 0.04], "height": 0.75, "placement_margin": 0.12}
INSTRUCTIONS = {
    "heavy": "Pick up each object one at a time and tell me which one is heavier.",
    "deformable": "Pick up each object one at a time and tell me which one is deformable.",
}


def phone_K(w, h, focal_35mm, out_wh):
    """Pinhole K from a 35 mm-equivalent focal length (defined on the 43.27 mm diagonal)."""
    f = focal_35mm / 43.27 * np.hypot(w, h)
    sx, sy = out_wh[0] / w, out_wh[1] / h
    return np.array([[f * sx, 0, out_wh[0] / 2], [0, f * sy, out_wh[1] / 2], [0, 0, 1.0]])


def rembg_mask(img, box, session, pad=120):
    """Salient-object mask (rembg, IS-Net) of one object, from a padded crop around its box.
    Works at the original resolution; GrabCut leaked into the soft shadows of a phone photo."""
    from rembg import remove

    W, H = img.size
    x0, y0, x1, y1 = max(box[0] - pad, 0), max(box[1] - pad, 0), min(box[2] + pad, W), min(box[3] + pad, H)
    a = np.array(remove(img.crop((x0, y0, x1, y1)), session=session, only_mask=True)) > 127
    n, lab, stats, _ = cv2.connectedComponentsWithStats(a.astype(np.uint8))
    if n <= 1:
        raise RuntimeError(f"no object found in box {box}")
    a = ndimage.binary_fill_holes(lab == 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA])))
    full = np.zeros((H, W), bool)
    full[y0:y1, x0:x1] = a
    return full


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", required=True)
    ap.add_argument("--scene_id", required=True)
    ap.add_argument("--names", nargs="+", required=True, help="one name per object, in --boxes order")
    ap.add_argument("--boxes", nargs="*", default=[], help="x0,y0,x1,y1 per object, original-image pixels")
    ap.add_argument("--masks_dir", help="obj_1.png ... (original resolution, white = object) instead of --boxes")
    ap.add_argument("--tasks", nargs="+", default=["heavy", "deformable"])
    ap.add_argument("--focal_35mm", type=float, default=26.0, help="iPhone main camera: 26 mm equivalent")
    ap.add_argument("--cam_height", type=float, default=0.30, help="camera to table (m): sets the metric scale")
    ap.add_argument("--centre_xy", type=float, nargs=2, default=[0.22, 0.0],
                    help="world xy under the image centre (the paper's objects lie 0.25-0.31 m from the base)")
    ap.add_argument("--truth", help="json {name: {mass_g, soft}} with the measured ground truth (optional)")
    ap.add_argument("--out", default="data/photo/scenes")
    args = ap.parse_args()

    img = ImageOps.exif_transpose(Image.open(args.image)).convert("RGB")
    W, H = img.size
    rgb = np.array(img.resize(OUT_WH, Image.LANCZOS))
    if args.masks_dir:
        masks = [np.array(Image.open(os.path.join(args.masks_dir, f"obj_{k + 1}.png")).convert("L")
                          .resize(OUT_WH, Image.NEAREST)) > 127 for k in range(len(args.names))]
    else:
        assert len(args.boxes) == len(args.names), "one --boxes entry per --names entry"
        from rembg import new_session

        session = new_session("isnet-general-use")
        masks = [np.array(Image.fromarray(rembg_mask(img, [int(v) for v in b.split(",")], session))
                          .resize(OUT_WH, Image.NEAREST)) for b in args.boxes]
    masks = np.stack(masks)
    if (masks.sum(0) > 1).any():
        raise RuntimeError("masks overlap")

    K = phone_K(W, H, args.focal_35mm, OUT_WH)
    c2w = np.eye(4)
    c2w[:3, :3] = [[0, -1, 0], [-1, 0, 0], [0, 0, -1]]  # as data/eval_scenes: image up = +x (robot side)
    c2w[:3, 3] = [*args.centre_xy, TABLE["height"] + args.cam_height]

    truth = json.load(open(args.truth)) if args.truth else {}
    objects = []
    for name, m in zip(args.names, masks):
        ys, xs = np.nonzero(m)
        # Mask centroid on the table plane (for the scorer's position checks / overlays).
        ray = np.linalg.inv(K) @ np.array([xs.mean(), ys.mean(), 1.0])
        p = c2w[:3, :3] @ (ray * args.cam_height) + c2w[:3, 3]
        ext_px = max(xs.ptp(), ys.ptp())
        objects.append({"name": name, "obj_id": name, "position": [float(p[0]), float(p[1]), TABLE["height"]],
                        "mask_px": int(m.sum()),
                        "approx_extent_m": float(ext_px * args.cam_height / K[0, 0]),
                        "real": truth.get(name, {})})

    for task in args.tasks:
        sid = f"{task}_{args.scene_id}"
        d = os.path.join(args.out, sid)
        os.makedirs(d, exist_ok=True)
        Image.fromarray(rgb).save(os.path.join(d, "rgb.png"))
        np.savez_compressed(os.path.join(d, "masks.npz"), masks=masks)
        inst = np.full(rgb.shape[:2], TABLE_ID, np.uint8)
        for k, m in enumerate(masks):
            inst[m] = k + 1
        Image.fromarray(np.stack([inst] * 3, -1)).save(os.path.join(d, "instance_mask.png"))
        json.dump({"K": K.tolist(), "resolution": list(OUT_WH), "cam2world_opencv": c2w.tolist(),
                   "assumed": {"focal_35mm": args.focal_35mm, "cam_height_m": args.cam_height,
                               "source_image": os.path.abspath(args.image), "source_wh": [W, H]}},
                  open(os.path.join(d, "camera.json"), "w"), indent=2)
        json.dump({"scene_id": sid, "task": task, "instruction": INSTRUCTIONS[task], "robot_base_xy": ROBOT_BASE_XY,
                   "table": TABLE, "objects": objects}, open(os.path.join(d, "scene.json"), "w"), indent=2)

        out = rgb.astype(np.float32)
        colors = [(230, 25, 75), (0, 130, 200), (60, 180, 75), (245, 130, 48)]
        for k, (m, o) in enumerate(zip(masks, objects)):
            out[m] = 0.5 * out[m] + 0.5 * np.array(colors[k % 4])
            ys, xs = np.nonzero(m)
            cv2.putText(out, f"{k + 1} {o['name']} ~{o['approx_extent_m'] * 100:.1f} cm", (int(xs.min()), int(ys.min()) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, colors[k % 4], 2)
        Image.fromarray(out.clip(0, 255).astype(np.uint8)).save(os.path.join(d, "overlay.png"))
        print(f"{sid}: " + "  ".join(f"{o['name']} {o['mask_px']} px ~{o['approx_extent_m'] * 100:.1f} cm" for o in objects))


if __name__ == "__main__":
    main()
