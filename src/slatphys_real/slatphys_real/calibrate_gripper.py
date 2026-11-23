"""Hand-E width calibration: close on gauge blocks of known width and fit width = w0 + slope*POS.

Hold each block (rigid, e.g. 10 / 20 / 30 / 40 mm; calipers to measure it) between the
fingers when asked; the gripper closes on it at the lowest force.

    ros2 run slatphys_real calibrate_gripper --widths_mm 10.0 20.0 30.0 40.0
"""

import argparse
import json
import os
import sys
import time

import numpy as np

from slatphys_real import common
from slatphys_real.hande import HandE


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=None)
    ap.add_argument("--widths_mm", type=float, nargs="+", required=True)
    args = ap.parse_args(argv if argv is not None else sys.argv[1:])
    cfg = common.load_config(args.config)
    g = HandE(cfg["robot_ip"], activate=True)
    pos = []
    for w in args.widths_mm:
        g.open()
        input(f"  hold the {w} mm block between the fingers, press Enter ")
        g.close_on(20.0)
        time.sleep(0.5)
        pos.append(float(np.mean([g.get("POS") for _ in range(10)])))
        print(f"    POS {pos[-1]:.1f}")
    g.open()
    g.shutdown()
    slope, w0 = np.polyfit(pos, np.array(args.widths_mm) / 1000, 1)
    res = np.array(args.widths_mm) / 1000 - (w0 + slope * np.array(pos))
    out = {"w0_m": float(w0), "slope_m_per_count": float(slope), "blocks_mm": args.widths_mm, "pos": pos,
           "residual_mm_max": float(np.abs(res).max() * 1000)}
    os.makedirs(os.path.dirname(cfg["gripper"]["calibration"]), exist_ok=True)
    json.dump(out, open(cfg["gripper"]["calibration"], "w"), indent=2)
    print(f"width = {w0 * 1000:.2f} mm + {slope * 1000:.4f} mm/count * POS (max residual {out['residual_mm_max']:.2f} mm)")


if __name__ == "__main__":
    main()
