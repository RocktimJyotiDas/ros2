"""SLAT-Phys reconstruction of a real scene -> the Isaac Sim scene (sim.json) and the real
robot's grasps (grasps.json), both from the same reconstructed meshes, so the simulated and the
real robot run the same policy on the same inputs.

Uses SLAT-Phys-Scene's materialize.py helpers (mass properties, envelope volume, grasp) with
the `slatphys` source only. One difference: SLAT-Phys-Scene models any object SLAT-Phys calls
metal as a 1 mm sheet-metal shell (PixieVerse's metal objects are cases); small real metal
objects are usually solid, so --metal_model defaults to solid here.

Writes <out>/slatphys/<scene>/{sim.json, obj_KK.obj, grasps.json}. Run from SLAT-Phys-Real/ with
SLAT-Phys-Scene on PYTHONPATH (scripts/env.sh sets it), in the `slatphys` env:
    python real2sim/materialize_real.py --scenes data/real/scenes --recon data/real/recon --out data/real/sim
"""

import argparse
import glob
import json
import os
import sys

import numpy as np
import trimesh
import yaml

# Frame conventions live in the ROS package (slatphys_real/common.py, no ROS imports) so the robot
# and these offline scripts share one definition.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src/slatphys_real"))
from slatphys_real import common  # noqa: E402
from slatscene.materialize import DEFAULT_FRICTION, FEM_MAX_E, envelope_volume, grasp_spec, mass_properties

SHELL_T = 0.001


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenes", default="data/real/scenes")
    ap.add_argument("--recon", default="data/real/recon")
    ap.add_argument("--out", default="data/real/sim")
    ap.add_argument("--metal_model", choices=["solid", "shell"], default="solid")
    ap.add_argument("--max_centre_err_cm", type=float, default=1.5)
    ap.add_argument("--config", default="configs/real.yaml")
    args = ap.parse_args()
    cfg = yaml.safe_load(open(args.config))
    theta = common.world_theta(cfg)

    for sdir in sorted(os.path.dirname(p) for p in glob.glob(os.path.join(args.scenes, "*", "scene.json"))):
        sid = os.path.basename(sdir)
        scene = json.load(open(os.path.join(sdir, "scene.json")))
        cam = json.load(open(os.path.join(sdir, "camera.json")))
        table_h, tz = scene["table"]["height"], cam["table_z_base"]
        dz = table_h - tz  # world z = base z + dz
        if not os.path.isdir(os.path.join(args.recon, sid)):
            print(f"{sid}: no reconstruction, skipped")
            continue
        out_dir = os.path.join(args.out, "slatphys", sid)
        os.makedirs(out_dir, exist_ok=True)
        objs, grasps = [], []
        for k, o in enumerate(scene["objects"]):
            odir = os.path.join(args.recon, sid, f"obj_{k:02d}")
            mesh = trimesh.load(os.path.join(odir, "mesh_world.glb"), force="mesh")
            sp = json.load(open(os.path.join(odir, "recon.json")))["slatphys"]
            rho, E, nu = sp["density_median"], sp["E_median"], sp["nu_median"]
            hollow = args.metal_model == "shell" and sp["majority_material"] == "metal"
            mass = rho * mesh.area * SHELL_T if hollow else rho * envelope_volume(mesh)
            soft = E < FEM_MAX_E
            mesh_path = os.path.join(out_dir, f"obj_{k:02d}.obj")
            mesh.export(mesh_path)
            com, inertia = mass_properties(mesh, mass, hollow)
            vol = envelope_volume(mesh)
            g = grasp_spec(mesh, table_h)
            objs.append({"name": o["name"], "mesh": os.path.abspath(mesh_path), "body": "fem" if soft else "rigid",
                         "mass_kg": float(mass), "com_world": com.tolist(), "inertia_world": inertia.tolist(),
                         "fem_density": float(mass / vol), "E": float(E), "nu": float(nu), "friction": DEFAULT_FRICTION,
                         "hollow": bool(hollow), "envelope_volume_m3": vol, "grasp": g,
                         "slatphys_material": sp["majority_material"]})
            err = np.linalg.norm(np.array(g["centre_xy"]) - np.array(o["position"][:2])) * 100
            if g["width"] > 0.045:
                print(f"  WARNING {sid} obj {k + 1} ({o['name']}): reconstructed grasp width {g['width'] * 1000:.0f} mm"
                      " exceeds the Hand-E's 50 mm stroke margin")
            if err > args.max_centre_err_cm:
                print(f"  WARNING {sid} obj {k + 1} ({o['name']}): grasp centre {err:.1f} cm from the mask centroid")
            grasps.append({"name": o["name"], "obj_id": o["obj_id"], "grasp": {
                "centre_xy": common.world_xy_to_base(cfg, g["centre_xy"]).tolist(),
                "yaw": float((g["yaw"] - theta + np.pi) % (2 * np.pi) - np.pi), "width": g["width"],
                "top_z": g["top_z"] - dz, "height": g["height"]}, "mask_centre_err_cm": round(float(err), 2)})
        sim = {"scene_id": sid, "source": "slatphys", "task": scene["task"], "instruction": scene["instruction"],
               "table": scene["table"], "robot_base_xy": scene["robot_base_xy"], "objects": objs}
        json.dump(sim, open(os.path.join(out_dir, "sim.json"), "w"), indent=2)
        json.dump({"scene_id": sid, "task": scene["task"], "frame": "base", "table_z_base": tz, "objects": grasps},
                  open(os.path.join(out_dir, "grasps.json"), "w"), indent=2)
        print(f"{sid:14s} " + "  ".join(
            f"{o['name']}: {o['mass_kg'] * 1000:.0f} g (real {so['real']['mass_g']:g}) {o['body']} E={o['E']:.0e}"
            f" w={o['grasp']['width'] * 1000:.0f}mm" for o, so in zip(objs, scene["objects"])))


if __name__ == "__main__":
    main()
