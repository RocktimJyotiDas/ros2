"""Camera-to-robot calibration by touching points on the table.

Tape N >= 4 (6 recommended) ArUco markers (DICT_4X4_50, ids 0..N-1, ~4 cm, printed flat)
on the table around the workspace, all inside the camera's view. For each marker, free-drive
the robot (pendant) until the closed fingertips touch the marker's centre and press Enter: the
tip position comes from the joint angles and the same FK the robot is commanded with. Then the
arm goes home, the camera photographs the markers, and solvePnP gives the camera pose.

Also gives the table height in `base` (mean touched z), used for every grasp height.

    ros2 run slatphys_real calibrate_camera --markers 6
    ros2 run slatphys_real calibrate_camera --reuse_points   # re-photograph only (camera moved)

Writes <calib_dir>/camera.json: K, D, cam2base (OpenCV camera: x right, y down, z forward),
table_z_base, reprojection error. Re-run whenever the camera or the table moves.
"""

import argparse
import json
import os
import sys
import time

import numpy as np

from slatphys_real import common
from slatphys_real import ur_kinematics as K


def detect_markers(rgb, n):
    import cv2

    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco, "ArucoDetector"):  # OpenCV >= 4.7
        corners, ids, _ = cv2.aruco.ArucoDetector(dic, cv2.aruco.DetectorParameters()).detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dic, parameters=cv2.aruco.DetectorParameters_create())
    found = {int(i): c.reshape(4, 2).mean(0) for i, c in zip(ids.ravel(), corners)} if ids is not None else {}
    return {i: found[i] for i in range(n) if i in found}


def solve(points_base, pixels, Kmat, D):
    import cv2

    obj = np.asarray(points_base, np.float64)
    img = np.asarray(pixels, np.float64)
    flag = cv2.SOLVEPNP_IPPE if len(obj) >= 4 else cv2.SOLVEPNP_ITERATIVE
    ok, rvec, tvec = cv2.solvePnP(obj, img, np.asarray(Kmat), np.asarray(D), flags=flag)
    if not ok:
        raise RuntimeError("solvePnP failed")
    rvec, tvec = cv2.solvePnPRefineLM(obj, img, np.asarray(Kmat), np.asarray(D), rvec, tvec)
    proj, _ = cv2.projectPoints(obj, rvec, tvec, np.asarray(Kmat), np.asarray(D))
    err = np.linalg.norm(proj.reshape(-1, 2) - img, axis=1)
    R, _ = cv2.Rodrigues(rvec)
    base2cam = np.eye(4)
    base2cam[:3, :3], base2cam[:3, 3] = R, tvec.ravel()
    return np.linalg.inv(base2cam), err


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=None)
    ap.add_argument("--markers", type=int, default=6)
    ap.add_argument("--reuse_points", action="store_true", help="keep the touched points from the last run")
    ap.add_argument("--mock", action="store_true")
    ap.add_argument("--execute", metavar="MOVE_REAL_UR3E", help="arming phrase required to move the arm home")
    args = ap.parse_args(argv if argv is not None else sys.argv[1:])
    cfg = common.load_config(args.config)
    os.makedirs(cfg["calib_dir"], exist_ok=True)
    pts_path = os.path.join(cfg["calib_dir"], "touched_points.json")
    from slatphys_real.arm import start
    from slatphys_real.capture import grab

    if not args.mock and args.execute != "MOVE_REAL_UR3E":
        raise SystemExit("requires --execute MOVE_REAL_UR3E (the arm goes home before the photo)")
    arm = start(cfg, mock=args.mock)
    if args.reuse_points:
        touched = {int(k): np.array(v) for k, v in json.load(open(pts_path)).items()}
    else:
        touched = {}
        tip = cfg["gripper"]["tip_z"]
        print("Close the gripper fully. For each marker: stop the External Control program, free-drive the closed"
              " fingertips onto the marker centre (pendant free-drive button), press Enter; joint states keep"
              " streaming. Press Play again before the arm moves home.")
        for i in range(args.markers):
            input(f"  marker {i}: touch its centre, press Enter ")
            T = K.fk(arm.q_now())
            touched[i] = T[:3, 3] + tip * T[:3, 2]
            print(f"    tip at {np.round(touched[i], 4).tolist()} (base)")
        json.dump({k: v.tolist() for k, v in touched.items()}, open(pts_path, "w"), indent=2)
        tilt = np.ptp([p[2] for p in touched.values()])
        if tilt > 0.005:
            print(f"WARNING: touched heights differ by {tilt * 1000:.1f} mm: table not level or tip not touching")

    input("Free-drive away / clear the view; press Enter to send the arm home and photograph the markers ")
    arm.move_joints(np.array(cfg["home_q"]), cfg["protocol"]["durations"]["home"])
    time.sleep(1.0)
    arm.release()
    rgb, info = grab(cfg)
    px = detect_markers(rgb, args.markers)
    ids = [i for i in touched if i in px]
    if len(ids) < 4:
        raise RuntimeError(f"only markers {sorted(px)} detected; need >= 4 of {sorted(touched)}")
    cam2base, err = solve([touched[i] for i in ids], [px[i] for i in ids], info["K"], info["D"])
    tz = float(np.mean([touched[i][2] for i in ids]))
    cam_h = cam2base[2, 3] - tz
    down = -cam2base[2, 2]  # cos of the angle between the optical axis and straight down
    out = {"K": info["K"], "D": info["D"], "resolution": info["resolution"], "cam2base_opencv": cam2base.tolist(),
           "table_z_base": tz, "camera_height_above_table_m": cam_h, "optical_axis_down_cos": down,
           "reprojection_error_px": {"mean": float(err.mean()), "max": float(err.max())}, "markers_used": ids,
           "date": time.strftime("%Y-%m-%d %H:%M")}
    json.dump(out, open(os.path.join(cfg["calib_dir"], "camera.json"), "w"), indent=2)
    from PIL import Image

    Image.fromarray(rgb).save(os.path.join(cfg["calib_dir"], "calibration_view.png"))
    print(f"camera {cam_h:.3f} m above the table, {np.degrees(np.arccos(np.clip(down, -1, 1))):.1f} deg off vertical; "
          f"reprojection error mean {err.mean():.2f} px, max {err.max():.2f} px; table z {tz:.4f} m")
    if err.max() > 3.0:
        print("WARNING: reprojection error > 3 px: re-touch the markers or recheck tip_z / intrinsics")


if __name__ == "__main__":
    main()
