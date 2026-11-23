"""Simulated probing of every real scene's SLAT-Phys twin, with the real protocol, merged into
the same readings.json schema as the real robot's (SLAT-Phys-Scene/slatscene/sim/run_probes.py,
with the launcher in between). The weigh pass is one Isaac Sim process (skipped once done); the
squeeze pass is the analytic contact model unless the protocol says `sim_squeeze: fem`.

Run from SLAT-Phys-Real/ on a GPU node:
    python real2sim/run_sim_probes.py --task_sim data/real/sim
"""

import argparse
import glob
import json
import os
import subprocess
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
# SLAT-Phys-Scene (the reconstruction + Isaac Sim probe) is a separate checkout: SLATPHYS_SCENE points at it.
RUNNER = os.environ.get("SLATPHYS_ISAAC_RUNNER",
                        os.path.join(os.environ.get("SLATPHYS_SCENE", "<set SLATPHYS_SCENE>"), "setup/isaac62_run.sh"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task_sim", default="data/real/sim")
    ap.add_argument("--protocol", default="configs/real.yaml")
    ap.add_argument("--runner", default=RUNNER, help="wrapper that runs python with Isaac Sim")
    ap.add_argument("--logs", default="logs/sim_probes")
    ap.add_argument("--scene_prefix", default="")
    args = ap.parse_args()
    os.makedirs(args.logs, exist_ok=True)
    launcher = os.path.join(HERE, "sim_probe_launcher.py")
    protocol = os.path.abspath(args.protocol)
    squeeze_model = yaml.safe_load(open(protocol))["protocol"].get("sim_squeeze", "analytic")

    for sim_path in sorted(glob.glob(os.path.join(args.task_sim, "slatphys", "*", "sim.json"))):
        d = os.path.abspath(os.path.dirname(sim_path))
        sid = os.path.basename(d)
        if not sid.startswith(args.scene_prefix):
            continue
        phases = {}
        for phase in ("squeeze", "weigh"):
            out = os.path.join(d, f"readings_{phase}.json")
            if phase == "squeeze" and squeeze_model == "analytic":
                subprocess.run([sys.executable, os.path.join(HERE, "sim_squeeze_model.py"), "--sim",
                                os.path.join(d, "sim.json"), "--config", protocol, "--out", out], check=True)
            elif not os.path.exists(out):
                log = os.path.abspath(os.path.join(args.logs, f"{sid}_{phase}.log"))
                with open(log, "w") as f:
                    rc = subprocess.run([args.runner, "python", launcher, "--protocol", protocol, "--",
                                         "--sim", os.path.join(d, "sim.json"), "--phase", phase, "--out", out],
                                        stdout=f, stderr=subprocess.STDOUT).returncode
                if not os.path.exists(out):
                    print(f"{sid} {phase}: FAILED (exit {rc}, log {log})", flush=True)
                    break
            phases[phase] = json.load(open(out))
        else:
            merged = [{**a, **b} for a, b in zip(phases["squeeze"]["readings"], phases["weigh"]["readings"])]
            diag = [{"squeeze": a, "weigh": b} for a, b in
                    zip(phases["squeeze"]["diagnostics"], phases["weigh"]["diagnostics"])]
            sim = json.load(open(os.path.join(d, "sim.json")))
            json.dump({"scene_id": sid, "source": "slatphys_sim", "task": sim["task"], "readings": merged, "diagnostics": diag},
                      open(os.path.join(d, "readings.json"), "w"), indent=2)
            print(f"{sid}: done, lifted {[x['weigh']['lifted'] for x in diag]}", flush=True)


if __name__ == "__main__":
    main()
