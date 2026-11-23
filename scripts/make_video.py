#!/usr/bin/env python3
"""A captioned MP4 of a recorded Gazebo run (RECORD=1 scripts/sim_demo.sh or sim_camera_pipeline.sh).

The camera views side by side, in simulation time, sped up (--speed). Under them: the probing step
being executed (from run_tasks_timed.log, matched to the frames through frames.csv's wall-clock
times), and a panel with the readings so far. A title card first, the answers last.

    python3 scripts/make_video.py runs/img4827_fit40 [--title "..."] [--speed 3]
    FFMPEG="apptainer exec isaacsim.sif ffmpeg" python3 scripts/make_video.py ...   # H.264 (else OpenCV mp4v)
Needs numpy + OpenCV.
"""

import argparse
import csv
import json
import os
import re
import shlex
import shutil
import subprocess

import cv2
import numpy as np

H = 480                  # every camera view is scaled to this height
BAR, PANEL = 84, 230     # caption bar height, readings panel width
FONT = cv2.FONT_HERSHEY_SIMPLEX
BG, FG, DIM, ACC = (32, 32, 32), (240, 240, 240), (150, 150, 150), (90, 200, 255)
LABELS = {"front": "front camera (the lab camera's view)", "overview": "overview", "closeup": "close-up"}


def text(img, s, xy, scale=0.55, color=FG, thick=1):
    cv2.putText(img, s, xy, FONT, scale, color, thick, cv2.LINE_AA)


def card(lines, size):
    img = np.full((size[1], size[0], 3), BG, np.uint8)
    y = size[1] // 2 - 22 * len(lines)
    for s, scale, color in lines:
        (w, _), _ = cv2.getTextSize(s, FONT, scale, 2 if scale > 0.8 else 1)
        text(img, s, ((size[0] - w) // 2, y), scale, color, 2 if scale > 0.8 else 1)
        y += int(46 * max(scale, 0.7))
    return img


def load_log(path):
    """(wall time, line) of every run_tasks line."""
    out = []
    for l in open(path):
        t, _, s = l.rstrip("\n").partition(" ")
        out.append((float(t), s))
    return out


def captions(log):
    """Step captions and readings over (wall) time."""
    steps, panel = [], []
    cur = None
    for t, s in log:
        m = re.match(r"\[probe\] (object \d+ \((\w+)\)): (.*)", s)
        if m:
            steps.append((t, f"{m.group(2)}: {m.group(3)}"))
            cur = m.group(2)
        m = re.search(r"squeeze (\d+) N: width ([\d.]+) mm", s)
        if m:
            panel.append((t, f"{cur}: {m.group(1)} N -> {m.group(2)} mm"))
        m = re.search(r"load ([\d.]+) N \(= (\d+) g\)", s)
        if m:
            panel.append((t, f"{cur}: load {float(m.group(1)) * 1000:.0f} mN"))
        m = re.match(r"=== \d\. (.*)", s)
        if m:
            steps.append((t, m.group(1)))
        m = re.match(r"\s+(\w+)\s+centre \[.*fit IoU ([\d.]+)", s)
        if m:
            panel.append((t, f"{m.group(1)}: located, IoU {m.group(2)}"))
        if s.startswith("preflight ok"):
            steps.append((t, "preflight ok: every grasp fits, is reachable and above the table"))
    return steps, panel


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run", help="runs/<world> (frames/, frames.csv, run_tasks_timed.log, answers.json)")
    ap.add_argument("--out", default=None, help="mp4 (default: <run>/<world>.mp4)")
    ap.add_argument("--title", default=None)
    ap.add_argument("--subtitle", default="")
    ap.add_argument("--speed", type=float, default=3.0, help="simulation seconds per video second")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--cams", nargs="+", default=None, help="camera views left to right (default: as recorded)")
    ap.add_argument("--overlay_s", type=float, default=4.0, help="show grasps_overlay.png this long when run_tasks starts")
    args = ap.parse_args()
    run = args.run.rstrip("/")
    name = os.path.basename(run)
    out = args.out or os.path.join(run, f"{name}.mp4")

    rows = list(csv.reader(open(os.path.join(run, "frames", "frames.csv"))))
    by_cam = {}
    for cam, t_sim, t_wall, f in rows:
        by_cam.setdefault(cam, []).append((float(t_sim), float(t_wall), f))
    for v in by_cam.values():
        v.sort()
    cams = args.cams or list(by_cam)
    ref = by_cam[cams[0]]
    sim_t = np.array([r[0] for r in ref])
    wall_t = np.array([r[1] for r in ref])
    log = load_log(os.path.join(run, "run_tasks_timed.log"))
    steps, panel = captions(log)
    sim_of = lambda tw: float(np.interp(tw, wall_t, sim_t))
    steps = [(sim_of(t), s) for t, s in steps]
    panel = [(sim_of(t), s) for t, s in panel]
    t_start, t_end = sim_t[0], sim_t[-1]
    widths = []
    for c in cams:
        im = cv2.imread(os.path.join(run, "frames", c, by_cam[c][0][2]))
        widths.append(int(round(im.shape[1] * H / im.shape[0])))
    t_overlay = next((ts for ts, s in steps if s.startswith("run_tasks")), None)
    overlay_png = os.path.join(run, "grasps_overlay.png")

    size = (sum(widths) + PANEL, H + BAR)
    tmp = os.path.join(run, "video_frames")
    shutil.rmtree(tmp, ignore_errors=True)
    os.makedirs(tmp)
    n = 0

    def emit(img, repeat=1):
        nonlocal n
        for _ in range(repeat):
            cv2.imwrite(os.path.join(tmp, f"{n:06d}.jpg"), img, [cv2.IMWRITE_JPEG_QUALITY, 92])
            n += 1

    ans = json.load(open(os.path.join(run, "answers.json")))["real"]
    title = args.title or f"SLAT-Phys real2sim: Gazebo run of {name}"
    emit(card([(title, 0.9, FG), (args.subtitle, 0.6, DIM),
               ("UR3e + Hand-E-sized gripper + wrist F/T, driven by the robot code (capture -> locate -> run_tasks)", 0.55, DIM),
               ("Tasks: which object is heavier?  which object is deformable?", 0.7, ACC),
               (f"objects: {', '.join(ans['objects'])}    video at {args.speed:g}x", 0.55, DIM)], size), args.fps * 4)

    cam_t = {c: np.array([r[0] for r in by_cam[c]]) for c in cams}
    t = t_start
    while t <= t_end:
        img = np.full((size[1], size[0], 3), BG, np.uint8)
        x = 0
        for c, w in zip(cams, widths):
            i = min(int(np.searchsorted(cam_t[c], t)), len(by_cam[c]) - 1)
            img[:H, x:x + w] = cv2.resize(cv2.imread(os.path.join(run, "frames", c, by_cam[c][i][2])), (w, H))
            text(img, LABELS.get(c, c), (x + 10, 22), 0.55, FG)
            x += w
        cur = [s for ts, s in steps if ts <= t]
        text(img, cur[-1] if cur else "starting", (12, H + 34), 0.65, ACC, 2)
        text(img, f"sim time {t - t_start:6.1f} s   ({args.speed:g}x)", (12, H + 66), 0.5, DIM)
        x0 = sum(widths) + 10
        text(img, "readings", (x0, 24), 0.6, FG, 2)
        y = 52
        for ts, s in panel:
            if ts <= t:
                text(img, s, (x0, y), 0.42, FG)
                y += 22
        if t_overlay is not None and t >= t_overlay and os.path.exists(overlay_png):
            # what locate found, from the lab camera's photo: shown once, before the robot moves
            ov_img = cv2.imread(overlay_png)
            still = np.full((size[1], size[0], 3), BG, np.uint8)
            w = int(round(ov_img.shape[1] * H / ov_img.shape[0]))
            still[:H, :w] = cv2.resize(ov_img, (w, H))
            # and a zoom on what locate drew: where the overlay differs from the captured photo
            cap = cv2.imread(os.path.join(run, "captures", "scene", "rgb.png"))
            if cap is not None and cap.shape == ov_img.shape and size[0] - w > 100:
                ys, xs = np.nonzero(np.abs(ov_img.astype(int) - cap.astype(int)).sum(2) > 60)
                if len(xs):
                    zw, zh = size[0] - w, H
                    cx, cy = (xs.min() + xs.max()) / 2, (ys.min() + ys.max()) / 2
                    half_w = max((xs.max() - xs.min()) * 0.65, 40)
                    half_h = half_w * zh / zw
                    x0, y0 = int(max(cx - half_w, 0)), int(max(cy - half_h, 0))
                    crop = ov_img[y0:int(cy + half_h), x0:int(cx + half_w)]
                    still[:H, w:] = cv2.resize(crop, (zw, zh), interpolation=cv2.INTER_CUBIC)
                    text(still, "zoom", (w + 10, 22), 0.55, FG)
            text(still, "locate: fitted boxes (white) and closing axes (yellow) in the captured photo", (12, H + 34), 0.65, ACC, 2)
            text(still, "grasps for run_tasks come from here", (12, H + 66), 0.5, DIM)
            emit(still, int(args.fps * args.overlay_s))
            t_overlay = None
        emit(img)
        t += args.speed / args.fps

    truth = {k: v.get("truth") for k, v in ans["answers"].items()}
    lines = [("Answers", 1.0, FG)]
    for task in ("heavy", "deformable"):
        a = ans["answers"][task]
        verdict = "" if truth[task] is None else ("  CORRECT" if a["answer"] == truth[task] else
                                                    f"  WRONG (expected {ans['objects'][truth[task] - 1]})")
        lines.append((f"{'heavier' if task == 'heavy' else 'deformable'}: {a['name']}{verdict}", 0.8,
                      (120, 220, 120) if "CORRECT" in verdict else ((90, 90, 240) if "WRONG" in verdict else FG)))
    for k, o in enumerate(ans["objects"]):
        lines.append((f"{o}: load {ans['load_N'][k] * 1000:.1f} mN, compression {ans['compression_m'][k] * 1000:.2f} mm",
                      0.55, DIM))
    emit(card(lines, size), args.fps * 5)

    ffmpeg = os.environ.get("FFMPEG")
    if ffmpeg:
        cmd = shlex.split(ffmpeg) + ["-y", "-loglevel", "error", "-framerate", str(args.fps), "-i",
                                     os.path.join(tmp, "%06d.jpg"), "-c:v", "libx264", "-pix_fmt", "yuv420p",
                                     "-crf", "23", "-movflags", "+faststart", out]
        subprocess.run(cmd, check=True)
    else:
        vw = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"mp4v"), args.fps, size)
        for k in range(n):
            vw.write(cv2.imread(os.path.join(tmp, f"{k:06d}.jpg")))
        vw.release()
    shutil.rmtree(tmp)
    print(f"{out}: {n} frames, {n / args.fps:.0f} s")


if __name__ == "__main__":
    main()
