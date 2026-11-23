"""Squeeze readings of the simulated robot from a linear-elastic contact model, instead of
Isaac Sim FEM.

Why: Isaac Sim 6.2's FEM bodies don't survive the Hand-E's real squeeze forces. At 20 N
and 60 N the pads tunnelled through a 1.8e7 Pa, 26.6 mm box (width 0 mm) in every solver
setting tried (logs/fem_check/, logs/fem_sweep/: more iterations, finer hex mesh, 1 ms steps,
larger contact offsets). The weigh pass is unaffected: it uses rigid bodies in Isaac.

Model: the pads (pad_width across, pad_len along the approach axis) close at force F on an
object of width L across the fingers (the reconstructed mesh's grasp width, the same one the
robot grasps) and the contact height h, i.e. the overlap of the pads with the object at the
grasp height. Uniaxial compression of the gripped section:
    delta(F) = F * L / (E * pad_width * h),   capped at max_strain * L
    width(F) = L - delta(F)
E is SLAT-Phys's median Young's modulus for the object. Stiff objects (wood, metal) give delta ~ 0.
The answer depends only on width(F_low) - width(F_high), exactly as for the real robot.

Writes <sim>/readings_squeeze.json in probe_objects.py's schema, so run_sim_probes.py merges it
with the Isaac weigh pass as before.
"""

import argparse
import json
import os

import numpy as np
import yaml


def squeeze_readings(sim, protocol, gripper):
    table = sim["table"]["height"]
    pad_half, pad_w = gripper["pad_half"], gripper["pad_width"]
    max_strain = protocol.get("model_max_strain", 0.8)
    readings, diags = [], []
    for k, o in enumerate(sim["objects"]):
        g = o["grasp"]
        z = max(table + pad_half + protocol["tip_clearance"], g["top_z"] - 0.5 * g["height"])
        h = max(min(g["top_z"], z + pad_half) - max(table, z - pad_half), 1e-3)
        L = g["width"]
        E = o["E"] if o.get("E") else 1e12  # no stiffness = rigid
        r = {"object_index": k + 1, "gripper_squeeze": []}
        for f in protocol["squeeze_forces_N"]:
            delta = min(f * L / (E * pad_w * h), max_strain * L)
            r["gripper_squeeze"].append({"commanded_force_N": f, "width_m": L - delta, "measured_finger_force_N": f})
        readings.append(r)
        diags.append({"name": o["name"], "body": "analytic", "mass_kg": o["mass_kg"], "E": E,
                      "contact_height_m": h, "grasp_width_m": L, "model": "linear elastic, uniaxial"})
    return readings, diags


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim", required=True, help="sim.json")
    ap.add_argument("--config", default="configs/real.yaml")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    cfg = yaml.safe_load(open(args.config))
    sim = json.load(open(args.sim))
    readings, diags = squeeze_readings(sim, cfg["protocol"], cfg["gripper"])
    out = args.out or os.path.join(os.path.dirname(args.sim), "readings_squeeze.json")
    json.dump({"scene_id": sim["scene_id"], "source": sim["source"], "phase": "squeeze", "squeeze_model": "analytic",
               "readings": readings, "diagnostics": diags}, open(out, "w"), indent=2)
    for r, d in zip(readings, diags):
        w = [s["width_m"] * 1000 for s in r["gripper_squeeze"]]
        print(f"obj {r['object_index']} ({d['name']}, E {d['E']:.1e}): width {w[0]:.1f} -> {w[-1]:.1f} mm "
              f"(compression {w[0] - w[-1]:.2f} mm)")


if __name__ == "__main__":
    main()
