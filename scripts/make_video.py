#!/usr/bin/env python3
"""A captioned MP4 of a recorded Gazebo run (scripts/sim_demo.sh with RECORD=1).

Both cameras side by side, in simulation time, sped up (--speed). Under them: the probing step
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

W, H = 640, 480          # each camera
BAR, PANEL = 84, 230     # caption bar height, readings panel width
FONT = cv2.FONT_HERSHEY_SIMPLEX
BG, FG, DIM, ACC = (32, 32, 32), (240, 240, 240), (150, 150, 150), (90, 200, 255)


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
    ov, cu = by_cam["overview"], by_cam["closeup"]
    sim_t = np.array([r[0] for r in ov])
    wall_t = np.array([r[1] for r in ov])
    log = load_log(os.path.join(run, "run_tasks_timed.log"))
    steps, panel = captions(log)
    sim_of = lambda tw: float(np.interp(tw, wall_t, sim_t))
    steps = [(sim_of(t), s) for t, s in steps]
    panel = [(sim_of(t), s) for t, s in panel]
    t_start, t_end = sim_t[0], sim_t[-1]

    size = (2 * W + PANEL, H + BAR)
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
               ("UR3e + Hand-E-sized gripper + wrist F/T, driven by the robot code (run_tasks -> probe)", 0.55, DIM),
               ("Tasks: which object is heavier?  which object is deformable?", 0.7, ACC),
               (f"objects: {', '.join(ans['objects'])}    video at {args.speed:g}x", 0.55, DIM)], size), args.fps * 4)

    cu_t = np.array([r[0] for r in cu])
    t = t_start
    while t <= t_end:
        i = min(int(np.searchsorted(sim_t, t)), len(ov) - 1)
        j = min(int(np.searchsorted(cu_t, t)), len(cu) - 1)
        a = cv2.imread(os.path.join(run, "frames", "overview", ov[i][2]))
        b = cv2.imread(os.path.join(run, "frames", "closeup", cu[j][2]))
        img = np.full((size[1], size[0], 3), BG, np.uint8)
        img[:H, :W], img[:H, W:2 * W] = a, b
        text(img, "overview", (10, 22), 0.55, FG)
        text(img, "close-up", (W + 10, 22), 0.55, FG)
        cur = [s for ts, s in steps if ts <= t]
        text(img, cur[-1] if cur else "starting", (12, H + 34), 0.65, ACC, 2)
        text(img, f"sim time {t - t_start:6.1f} s   ({args.speed:g}x)", (12, H + 66), 0.5, DIM)
        x0 = 2 * W + 10
        text(img, "readings", (x0, 24), 0.6, FG, 2)
        y = 52
        for ts, s in panel:
            if ts <= t:
                text(img, s, (x0, y), 0.42, FG)
                y += 22
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
