"""Help set up a planned scene: point the fingertips at each planned object position so you can
put the object directly under them (objects from real2sim/plan_scenes.py, positions in `base`).

    ros2 run slatphys_real place --plan data/real/plans/heavy_01.json
"""

import argparse
import json
import sys

import numpy as np

from slatphys_real import common


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=None)
    ap.add_argument("--plan", required=True)
    ap.add_argument("--height", type=float, default=0.06, help="fingertip height above the table while pointing")
    ap.add_argument("--mock", action="store_true")
    ap.add_argument("--execute", metavar="MOVE_REAL_UR3E", help="arming phrase required to move the robot")
    args = ap.parse_args(argv if argv is not None else sys.argv[1:])
    cfg = common.load_config(args.config)
    plan = json.load(open(args.plan))
    tz = common.table_z(cfg)
    cfg["safety"]["min_tcp_z"] = tz + cfg["safety"]["min_tcp_above_table"]
    from slatphys_real.arm import start

    if not args.mock and args.execute != "MOVE_REAL_UR3E":
        raise SystemExit("requires --execute MOVE_REAL_UR3E")
    arm = start(cfg, mock=args.mock)
    drop = cfg["gripper"]["tip_z"] - cfg["gripper"]["tcp_z"]  # TCP sits above the tip
    print(f"scene {plan['scene_id']} ({plan['task']})")
    for k, o in enumerate(plan["objects"]):
        x, y = o["xy_base"]
        arm.move_tcp(np.array([x, y, tz + args.height + drop]), np.radians(o.get("yaw_deg", 0.0)), 3.0, linear=False)
        input(f"  object {k + 1}: put the {o['name']} under the fingertips "
              f"(long side across the fingers: {o.get('yaw_deg', 0):.0f} deg); press Enter ")
    arm.move_joints(np.array(cfg["home_q"]), cfg["protocol"]["durations"]["home"])
    print("done; the arm is home. Next: capture.")
    arm.close()


if __name__ == "__main__":
    main()
