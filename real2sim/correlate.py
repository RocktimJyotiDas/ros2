"""Sim-to-real agreement of the SLAT-Phys twin, in the spirit of SimFoundry (arXiv 2606.28276),
which reports the Pearson correlation between simulated and real success rates and the mean
maximum rank violation (MMRV).

Inputs per scene: the real robot's readings (<real>/<scene>/readings.json), the simulated
robot's readings on the SLAT-Phys reconstruction (<sim>/<scene>/readings.json, same schema) and
the real ground truth (<scenes>/<scene>/scene.json: measured mass, soft).

Readings, identical for sim and real:
  load         |F_with - F_without| of the wrist force at the hold pose (N); magnitude, so the
               wrench frame doesn't matter. Real: F_without is the mean of the empty readings
               before and after the object when both exist (cancels sensor drift).
  compression  finger width at the lowest squeeze force - at the highest (m)
Answers: heavy = most load, deformable = most compression, sort = ascending load.

Reported:
  objects   Pearson r of log10 load and Spearman rho of load, sim vs real, over all objects;
            the same for compression; 95% bootstrap CIs over objects
  scenes    per-scene score (heavy / deformable: 1 if correct; sort: fraction of the 3 pairs
            ordered correctly), sim and real, against the real ground truth; Pearson r across
            scenes; sim/real answer agreement
  tasks     success rate per task, sim and real; Pearson r and MMRV across the tasks (SimFoundry's
            metrics; with three tasks, read them alongside the object-level numbers)
  sanity    real load vs measured mass x g (slope ~1 means the F/T reading is right)

    python real2sim/correlate.py --scenes data/real/scenes --sim data/real/sim/slatphys \
        --real data/real/robot --out results
"""

import argparse
import csv
import glob
import itertools
import json
import os

import numpy as np
from scipy import stats

G = 9.81


def load_N(r):
    w = r["wrist_force_torque"]
    f0 = np.array(w["without_object"][:3])
    if w.get("without_object_after") is not None:
        f0 = (f0 + np.array(w["without_object_after"][:3])) / 2
    return float(np.linalg.norm(np.array(w["with_object"][:3]) - f0))


def compression_m(r):
    sq = r["gripper_squeeze"]
    return float(sq[0]["width_m"] - sq[-1]["width_m"])


def answer(task, load, comp):
    if task == "heavy":
        return int(np.argmax(load)) + 1
    if task == "deformable":
        return int(np.argmax(comp)) + 1
    return [int(i) + 1 for i in np.argsort(load)]


def ground_truth(task, objs):
    mass = [o["real"]["mass_g"] for o in objs]
    if task == "heavy":
        return int(np.argmax(mass)) + 1
    if task == "deformable":
        return [o["real"]["soft"] for o in objs].index(True) + 1
    return [int(i) + 1 for i in np.argsort(mass)]


def score(task, ans, gt):
    """1/0 for heavy and deformable; for sort, the fraction of object pairs in the right order."""
    if task != "sort":
        return float(ans == gt)
    pos_a = {o: i for i, o in enumerate(ans)}
    pos_g = {o: i for i, o in enumerate(gt)}
    pairs = list(itertools.combinations(gt, 2))
    return float(np.mean([(pos_a[a] < pos_a[b]) == (pos_g[a] < pos_g[b]) for a, b in pairs]))


def mmrv(sim, real):
    """Mean maximum rank violation (SIMPLER, Li et al. 2024; used by SimFoundry): for each setting
    i, the largest real-performance gap |R_i - R_j| over settings j that sim ranks the other way."""
    sim, real = np.asarray(sim, float), np.asarray(real, float)
    n = len(sim)
    v = [max([abs(real[i] - real[j]) for j in range(n) if j != i and (sim[i] < sim[j]) != (real[i] < real[j])] or [0.0])
         for i in range(n)]
    return float(np.mean(v))


def boot_ci(f, x, y, n=2000, seed=0):
    rng = np.random.default_rng(seed)
    vals = []
    for _ in range(n):
        idx = rng.integers(0, len(x), len(x))
        if np.ptp(x[idx]) > 0 and np.ptp(y[idx]) > 0:
            vals.append(f(x[idx], y[idx]))
    return [float(np.percentile(vals, 2.5)), float(np.percentile(vals, 97.5))] if vals else [np.nan, np.nan]


def corr_block(sim, real, log=False):
    sim, real = np.asarray(sim, float), np.asarray(real, float)
    if len(sim) < 3:
        return {"n": len(sim)}
    xs, ys = (np.log10(np.maximum(sim, 1e-4)), np.log10(np.maximum(real, 1e-4))) if log else (sim, real)
    pear = lambda a, b: stats.pearsonr(a, b)[0]  # noqa: E731
    spear = lambda a, b: stats.spearmanr(a, b)[0]  # noqa: E731
    return {"n": len(sim), "pearson" + ("_log10" if log else ""): float(pear(xs, ys)),
            "pearson_ci95": boot_ci(pear, xs, ys), "spearman": float(spear(sim, real)), "spearman_ci95": boot_ci(spear, sim, real)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenes", default="data/real/scenes")
    ap.add_argument("--sim", default="data/real/sim/slatphys")
    ap.add_argument("--real", default="data/real/robot")
    ap.add_argument("--out", default="results")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    rows, scenes = [], []
    for sp in sorted(glob.glob(os.path.join(args.scenes, "*", "scene.json"))):
        scene = json.load(open(sp))
        sid, task = scene["scene_id"], scene["task"]
        paths = {s: os.path.join(d, sid, "readings.json") for s, d in (("sim", args.sim), ("real", args.real))}
        if not all(os.path.exists(p) for p in paths.values()):
            print(f"{sid}: missing {[s for s, p in paths.items() if not os.path.exists(p)]} readings, skipped")
            continue
        gt = ground_truth(task, scene["objects"])
        rec = {"scene_id": sid, "task": task, "ground_truth": gt}
        for s, p in paths.items():
            d = json.load(open(p))
            load = [load_N(r) for r in d["readings"]]
            comp = [compression_m(r) for r in d["readings"]]
            lifted = [x["weigh"]["lifted"] for x in d["diagnostics"]]
            ans = answer(task, load, comp)
            rec[s] = {"answer": ans, "score": score(task, ans, gt), "load_N": load, "compression_m": comp, "lifted": lifted}
        rec["agree"] = rec["sim"]["answer"] == rec["real"]["answer"]
        scenes.append(rec)
        for k, o in enumerate(scene["objects"]):
            rows.append({"scene_id": sid, "task": task, "object": k + 1, "name": o["name"], "mass_g": o["real"]["mass_g"],
                         "soft": o["real"]["soft"],
                         **{f"{s}_{m}": rec[s][key][k] for s in ("sim", "real")
                            for m, key in (("load_N", "load_N"), ("compression_mm", "compression_m"), ("lifted", "lifted"))}})
        for r in rows[-len(scene["objects"]):]:
            r["sim_compression_mm"] *= 1000
            r["real_compression_mm"] *= 1000
        print(f"{sid:14s} GT {gt}  sim {rec['sim']['answer']} ({rec['sim']['score']:.2f})  real {rec['real']['answer']}"
              f" ({rec['real']['score']:.2f})  {'agree' if rec['agree'] else 'DISAGREE'}")

    if not scenes:
        print("no scene has both sim and real readings")
        return
    both_lifted = [r for r in rows if r["sim_lifted"] and r["real_lifted"]]
    metrics = {
        "objects": {
            "load": corr_block([r["sim_load_N"] for r in both_lifted], [r["real_load_N"] for r in both_lifted], log=True),
            "compression": corr_block([r["sim_compression_mm"] for r in rows], [r["real_compression_mm"] for r in rows]),
            "excluded_not_lifted": len(rows) - len(both_lifted),
        },
        "sanity_real_load_vs_mass": {},
        "scenes": {"n": len(scenes), "answer_agreement": float(np.mean([s["agree"] for s in scenes]))},
        "tasks": {},
    }
    real_lift = [r for r in rows if r["real_lifted"]]
    if len(real_lift) >= 3:
        x = np.array([r["mass_g"] / 1000 * G for r in real_lift])
        y = np.array([r["real_load_N"] for r in real_lift])
        fit = stats.linregress(x, y)
        metrics["sanity_real_load_vs_mass"] = {"slope": float(fit.slope), "intercept_N": float(fit.intercept),
                                               "r2": float(fit.rvalue ** 2), "n": len(real_lift)}
    ss, rs = [s["sim"]["score"] for s in scenes], [s["real"]["score"] for s in scenes]
    if len(scenes) >= 3 and np.ptp(ss) > 0 and np.ptp(rs) > 0:
        metrics["scenes"]["score_pearson"] = float(stats.pearsonr(ss, rs)[0])
    sim_rate, real_rate = [], []
    for t in ("heavy", "deformable", "sort"):
        ts = [s for s in scenes if s["task"] == t]
        if not ts:
            continue
        m = {"n": len(ts), "sim_success": float(np.mean([s["sim"]["score"] == 1 for s in ts])),
             "real_success": float(np.mean([s["real"]["score"] == 1 for s in ts])),
             "sim_score": float(np.mean([s["sim"]["score"] for s in ts])),
             "real_score": float(np.mean([s["real"]["score"] for s in ts])),
             "agreement": float(np.mean([s["agree"] for s in ts]))}
        metrics["tasks"][t] = m
        sim_rate.append(m["sim_success"])
        real_rate.append(m["real_success"])
    if len(sim_rate) >= 2:
        metrics["tasks_summary"] = {"mmrv": mmrv(sim_rate, real_rate)}
        if len(sim_rate) >= 3 and np.ptp(sim_rate) > 0 and np.ptp(real_rate) > 0:
            metrics["tasks_summary"]["pearson"] = float(stats.pearsonr(sim_rate, real_rate)[0])

    with open(os.path.join(args.out, "per_object.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    json.dump(scenes, open(os.path.join(args.out, "per_scene.json"), "w"), indent=2)
    json.dump(metrics, open(os.path.join(args.out, "metrics.json"), "w"), indent=2)

    o = metrics["objects"]
    print(f"\nobjects: load  n={o['load'].get('n')}  Pearson(log) {o['load'].get('pearson_log10', np.nan):.3f} "
          f"{np.round(o['load'].get('pearson_ci95', [np.nan] * 2), 2).tolist()}  Spearman {o['load'].get('spearman', np.nan):.3f}"
          f"   ({o['excluded_not_lifted']} not lifted in sim or real, excluded)")
    print(f"         compression n={o['compression'].get('n')}  Pearson {o['compression'].get('pearson', np.nan):.3f} "
          f"{np.round(o['compression'].get('pearson_ci95', [np.nan] * 2), 2).tolist()}  Spearman {o['compression'].get('spearman', np.nan):.3f}")
    print(f"{'task':12s}{'sim':>8s}{'real':>8s}{'agree':>8s}")
    for t, m in metrics["tasks"].items():
        print(f"{t:12s}{m['sim_success']:8.2f}{m['real_success']:8.2f}{m['agreement']:8.2f}   (n={m['n']})")
    print(f"tasks: {metrics.get('tasks_summary')}; scenes: {metrics['scenes']}")
    print(f"sanity (real load vs mass*g): {metrics['sanity_real_load_vs_mass']}")
    print("wrote", args.out)


if __name__ == "__main__":
    main()
