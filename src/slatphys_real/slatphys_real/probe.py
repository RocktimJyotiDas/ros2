"""Scripted probing of every object in one real scene: the hardware twin of
SLAT-Phys-Scene/slatscene/sim/probe_objects.py, step for step.

Input: grasps.json (real2sim/materialize_real.py), the grasp of each object in the `base`
frame, computed from the SLAT-Phys reconstruction exactly as the simulated robot gets it.

  squeeze pass   approach open, descend, close at each squeeze force -> finger width at each
                 release, retreat
  weigh pass     empty gripper (closed) at the hold pose               -> wrist wrench, no load
                 approach, descend, grip at lift force, lift, hold      -> wrist wrench, loaded
                 put back, release, retreat
                 empty gripper at the hold pose again (ft_after)        -> drift check

Writes <out>/readings.json with the simulation's schema (score_readings / correlate read both).

    ros2 run slatphys_real probe --grasps data/real/sim/slatphys/heavy_01/grasps.json \
        --out data/real/robot/heavy_01            [--phase both] [--step] [--dry_run] [--mock]
"""

import argparse
import json
import os
import sys
import time

import numpy as np

from slatphys_real import common
from slatphys_real import ur_kinematics as K


def best_yaw(yaw, q_ref, p, tcp_z):
    """Grasp yaw or yaw + pi, whichever needs less wrist rotation (as in the simulation)."""
    best = None
    for y in (yaw, yaw + np.pi):
        y = (y + np.pi) % (2 * np.pi) - np.pi
        q = K.solve_tcp(p, y, tcp_z, q_ref)
        if q is not None and (best is None or abs(q[5] - q_ref[5]) < best[0]):
            best = (abs(q[5] - q_ref[5]), y)
    if best is None:
        raise RuntimeError(f"no IK above the object at {np.round(p, 3)}")
    return best[1]


class Prober:
    def __init__(self, cfg, arm, gripper, step=False, dry_run=False):
        self.cfg, self.arm, self.g = cfg, arm, gripper
        self.P = cfg["protocol"]
        self.D = self.P["durations"]
        self.step_mode, self.dry_run = step, dry_run
        self.tz = common.table_z(cfg)
        cfg["safety"]["min_tcp_z"] = self.tz + cfg["safety"]["min_tcp_above_table"]
        self.tcp_z = cfg["gripper"]["tcp_z"]
        # hold_offset is in the simulation's world frame (relative to the robot); rotate it into base.
        self.hold = common.hold_point_base(cfg, self.tz)

    def say(self, msg):
        print(f"[probe] {msg}", flush=True)
        if self.step_mode:
            input("    press Enter to continue (Ctrl-C aborts) ")

    def move(self, p, yaw, seconds, linear=True):
        if self.dry_run:
            print(f"    dry run: TCP -> {np.round(p, 3).tolist()} yaw {np.degrees(yaw):.0f} deg", flush=True)
            return
        self.arm.move_tcp(p, yaw, seconds, linear=linear)

    def wrench(self):
        if self.dry_run:
            return {"mean": np.zeros(6), "std": np.zeros(6), "n": 0}
        return self.arm.wrench(self.P["settle_s"], self.P["ft_window_s"])

    def geometry(self, o):
        g = o["grasp"]
        z_grasp = max(self.tz + self.cfg["gripper"]["pad_half"] + self.P["tip_clearance"], g["top_z"] - 0.5 * g["height"])
        above = np.array([*g["centre_xy"], g["top_z"] + self.P["above"]])
        at = np.array([*g["centre_xy"], z_grasp])
        yaw = best_yaw(g["yaw"], self.arm.q_now(), above, self.tcp_z)
        return above, at, yaw

    def go_home(self):
        if not self.dry_run:
            self.arm.move_joints(np.array(self.cfg["home_q"]), self.D["home"])

    def squeeze(self, k, o):
        above, at, yaw = self.geometry(o)
        name = f"object {k + 1} ({o['name']})"
        r, diag = {"object_index": k + 1, "gripper_squeeze": []}, {"name": o["name"], "grasp": o["grasp"]}
        self.say(f"{name}: squeeze - approach")
        self.g.open()
        self.move(above, yaw, self.D["approach"], linear=False)
        self.say(f"{name}: descend to z {at[2] - self.tz:.3f} above the table")
        self.move(at, yaw, self.D["descend"])
        for f in self.P["squeeze_forces_N"]:
            obj = self.g.close_on(f)
            time.sleep(self.P["settle_s"])
            w, raw = self.g.width()
            r["gripper_squeeze"].append({"commanded_force_N": f, "width_m": w, "pos_raw": raw, "obj_status": obj})
            print(f"    squeeze {f:g} N: width {w * 1000:.1f} mm (OBJ {obj})", flush=True)
        self.say(f"{name}: release, retreat")
        self.g.open()
        self.move(above, yaw, self.D["lift"])
        sq = r["gripper_squeeze"]
        diag["contact"] = all(s["obj_status"] == 2 for s in sq)
        return r, diag

    def weigh(self, k, o):
        above, at, yaw = self.geometry(o)
        name = f"object {k + 1} ({o['name']})"
        r, diag = {"object_index": k + 1}, {"name": o["name"], "grasp": o["grasp"]}
        self.say(f"{name}: weigh the empty gripper at the hold pose")
        self.g.close_on(self.P["lift_force_N"])
        self.move(self.hold, yaw, self.D["to_hold"], linear=False)
        empty = self.wrench()

        self.say(f"{name}: approach")
        self.g.open()
        self.move(above, yaw, self.D["approach"], linear=False)
        self.move(at, yaw, self.D["descend"])
        self.say(f"{name}: grip at {self.P['lift_force_N']:g} N, lift, hold")
        obj = self.g.close_on(self.P["lift_force_N"])
        time.sleep(self.P["settle_s"])
        diag["width_after_grip_m"], _ = self.g.width()
        self.move(above, yaw, self.D["lift"])
        self.move(self.hold, yaw, self.D["to_hold"], linear=False)
        load = self.wrench()
        w_hold, _ = self.g.width()
        # Still holding = the fingers stopped on something, not fully closed.
        diag["lifted"] = bool(obj == 2 and w_hold > 0.002)

        self.say(f"{name}: put back, release, retreat")
        self.move(above, yaw, self.D["to_hold"], linear=False)
        self.move(at + np.array([0, 0, 0.005]), yaw, self.D["descend"])
        self.g.open()
        self.move(above, yaw, self.D["lift"])

        after = None
        if self.P["ft_after"]:
            self.g.close_on(self.P["lift_force_N"])
            self.move(self.hold, yaw, self.D["to_hold"], linear=False)
            after = self.wrench()
            self.g.open()

        r["lift_grip_force_N"] = self.P["lift_force_N"]
        r["gripper_width_while_holding_m"] = w_hold
        r["wrist_force_torque"] = {
            "frame": "as published by ur_robot_driver (same pose for all three readings)",
            "components": ["Fx_N", "Fy_N", "Fz_N", "Tx_Nm", "Ty_Nm", "Tz_Nm"],
            "without_object": empty["mean"], "with_object": load["mean"],
            "without_object_std": empty["std"], "with_object_std": load["std"],
            "without_object_after": None if after is None else after["mean"],
            "samples": [empty["n"], load["n"], None if after is None else after["n"]],
        }
        dF = load["mean"][:3] - (empty["mean"][:3] if after is None else (empty["mean"][:3] + after["mean"][:3]) / 2)
        drift = None if after is None else float(np.linalg.norm(after["mean"][:3] - empty["mean"][:3]))
        diag["load_N"], diag["ft_drift_N"] = float(np.linalg.norm(dF)), drift
        print(f"    load {np.linalg.norm(dF):.3f} N (= {np.linalg.norm(dF) / 9.81 * 1000:.0f} g), "
              f"drift {drift if drift is None else round(drift, 3)} N, lifted {diag['lifted']}", flush=True)
        return r, diag


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=None)
    ap.add_argument("--grasps", required=True, help="grasps.json from real2sim/materialize_real.py")
    ap.add_argument("--out", required=True, help="output directory (readings.json)")
    ap.add_argument("--phase", choices=["squeeze", "weigh", "both"], default="both")
    ap.add_argument("--only", type=int, nargs="*", help="1-based object indices")
    ap.add_argument("--step", action="store_true", help="wait for Enter before every motion")
    ap.add_argument("--dry_run", action="store_true", help="print TCP targets, don't move the arm or gripper")
    ap.add_argument("--mock", action="store_true", help="no robot: mock arm and gripper (tests the script)")
    ap.add_argument("--execute", metavar="MOVE_REAL_UR3E", help="arming phrase required to move the robot")
    ap.add_argument("--activate_gripper", action="store_true", help="activate the Hand-E first (moves the fingers)")
    ap.add_argument("--mock_driver", action="store_true",
                    help="testing with ur_robot_driver mock hardware: skip robot/safety-mode and speed-slider checks")
    ap.add_argument("--mock_gripper", action="store_true",
                    help="real (or use_mock_hardware) arm, mock gripper: test motions without the Hand-E")
    args = ap.parse_args(argv if argv is not None else sys.argv[1:])

    cfg = common.load_config(args.config)
    spec = json.load(open(args.grasps))
    from slatphys_real.arm import start

    if not (args.mock or args.dry_run) and args.execute != "MOVE_REAL_UR3E":
        raise SystemExit("real motion requires --execute MOVE_REAL_UR3E (ur3e_plan_runner's arming phrase); "
                         "use --dry_run to audit the targets first")
    arm = start(cfg, mock=args.mock or args.dry_run, skip_driver_checks=args.mock_driver)
    gripper = common.make_gripper(cfg, mock=args.mock or args.dry_run or args.mock_gripper, activate=args.activate_gripper)
    pr = Prober(cfg, arm, gripper, step=args.step, dry_run=args.dry_run)
    os.makedirs(args.out, exist_ok=True)
    print(f"scene {spec['scene_id']} ({spec['task']}): {len(spec['objects'])} objects, table z {pr.tz:.4f} m in base")

    phases = ["squeeze", "weigh"] if args.phase == "both" else [args.phase]
    out = {ph: ([], []) for ph in phases}
    try:
        pr.go_home()
        for ph in phases:
            for k, o in enumerate(spec["objects"]):
                if args.only and k + 1 not in args.only:
                    continue
                r, d = getattr(pr, ph)(k, o)
                out[ph][0].append(r)
                out[ph][1].append(d)
            pr.go_home()
    finally:
        gripper.shutdown()
        arm.close()

    for ph in phases:
        json.dump(common.to_jsonable({"scene_id": spec["scene_id"], "source": "real", "phase": ph,
                                      "readings": out[ph][0], "diagnostics": out[ph][1]}),
                  open(os.path.join(args.out, f"readings_{ph}.json"), "w"), indent=2)
    if len(phases) == 2:  # merged, like SLAT-Phys-Scene's run_probes.py
        merged = [{**a, **b} for a, b in zip(*(out[ph][0] for ph in phases))]
        diag = [{"squeeze": a, "weigh": b} for a, b in zip(*(out[ph][1] for ph in phases))]
        json.dump(common.to_jsonable({"scene_id": spec["scene_id"], "source": "real", "task": spec["task"],
                                      "readings": merged, "diagnostics": diag}),
                  open(os.path.join(args.out, "readings.json"), "w"), indent=2)
    print("wrote", args.out)


if __name__ == "__main__":
    main()
