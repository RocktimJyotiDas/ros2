"""Run SLAT-Phys-Scene's slatscene/sim/probe_objects.py with the real robot's protocol constants.

probe_objects.py hard-codes its forces (squeeze 0.5 / 2 N); the Hand-E can't go below 20 N, so
for sim-to-real the simulated robot must use the real protocol (configs/real.yaml). Rather than
fork or edit that file, this executes its source with the constants substituted; every
substitution must match exactly once, so a changed upstream file fails loudly instead of
silently running the old protocol.

    <isaac runner> python real2sim/sim_probe_launcher.py --protocol configs/real.yaml -- \
        --sim data/real/sim/slatphys/heavy_01/sim.json --phase weigh
"""

import argparse
import os
import sys

import yaml

PROBE = os.environ.get("SLATPHYS_SCENE_PROBE",
                       os.path.join(os.environ.get("SLATPHYS_SCENE", "<set SLATPHYS_SCENE>"), "slatscene/sim/probe_objects.py"))


def patched_source(protocol):
    src = open(PROBE).read()
    p = protocol
    subs = [
        ("SQUEEZE_N = (0.5, 2.0)", f"SQUEEZE_N = {tuple(float(f) for f in p['squeeze_forces_N'])!r}"),
        ("LIFT_N = 20.0", f"LIFT_N = {float(p['lift_force_N'])!r}"),
        ("HOLD_XYZ_OFFSET = (-0.25, 0.0, 0.22)", f"HOLD_XYZ_OFFSET = {tuple(float(x) for x in p['hold_offset'])!r}"),
        ('ap.add_argument("--tip_clearance", type=float, default=0.003,',
         f'ap.add_argument("--tip_clearance", type=float, default={float(p["tip_clearance"])!r},'),
    ]
    # Optional FEM solver settings (configs/real.yaml `sim_fem:`), for stable soft-body contact at
    # the Hand-E's 20-60 N: hex resolution, solver iterations, collision offsets, face budget.
    fem = protocol.get("sim_fem") or {}
    if "resolution" in fem:
        subs.append(('root.GetAttribute("physxDeformableBody:resolution").Set(12)',
                     f'root.GetAttribute("physxDeformableBody:resolution").Set({int(fem["resolution"])})'))
    if "iterations" in fem:
        subs.append(('root.GetAttribute("physxDeformableBody:solverPositionIterationCount").Set(40)',
                     f'root.GetAttribute("physxDeformableBody:solverPositionIterationCount").Set({int(fem["iterations"])})'))
    if "contact_offset" in fem or "rest_offset" in fem:
        subs.append(('small_offsets(stage.GetPrimAtPath(f"{path}/collMesh"), contact=0.006, rest=0.001)',
                     f'small_offsets(stage.GetPrimAtPath(f"{{path}}/collMesh"), contact={float(fem.get("contact_offset", 0.006))!r},'
                     f' rest={float(fem.get("rest_offset", 0.001))!r})'))
    if "max_faces" in fem:
        subs.append(("MAX_FACES_RIGID, MAX_FACES_FEM = 4000, 1500", f"MAX_FACES_RIGID, MAX_FACES_FEM = 4000, {int(fem['max_faces'])}"))
    if "dt" in fem:
        subs.append(('ap.add_argument("--dt", type=float, default=1 / 400)', f'ap.add_argument("--dt", type=float, default={float(fem["dt"])!r})'))
    for old, new in subs:
        n = src.count(old)
        if n != 1:
            raise RuntimeError(f"{PROBE}: expected exactly one {old!r}, found {n}; update sim_probe_launcher.py")
        src = src.replace(old, new)
    return src


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--protocol", default="configs/real.yaml")
    ap.add_argument("rest", nargs=argparse.REMAINDER)
    args = ap.parse_args()
    protocol = yaml.safe_load(open(args.protocol))["protocol"]
    src = patched_source(protocol)
    rest = args.rest[1:] if args.rest[:1] == ["--"] else args.rest
    sys.argv = [PROBE] + rest
    sys.path.insert(0, os.path.dirname(PROBE))
    exec(compile(src, PROBE, "exec"), {"__name__": "__main__", "__file__": PROBE})


if __name__ == "__main__":
    main()
