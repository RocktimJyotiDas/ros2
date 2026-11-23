"""The paper's simulated real2sim pipeline (SLAT-Phys Sec. "Simulated Real2Sim Robotic Evaluation")
on scenes made from a phone photo by photo_scene.py.

  recon        SAM-3D + SLAT-Phys per object, metric point map from the table plane, silhouette
               scale fit: the loop of SLAT-Phys-Scene/slatscene/recon_task_scenes.py, minus its
               comparison to the PixieVerse ground truth (a photo has none).
  materialize  the `slatphys` source of slatscene/materialize.py: SAM-3D mesh, SLAT-Phys median
               density / E / nu, thin metal shell when SLAT-Phys calls the object metal,
               FEM body when E < 5e7 Pa, grasp across the narrow axis of the footprint.
  score        slatscene/score_readings.answer on readings.json: heavy -> most wrist load,
               deformable -> most squeeze compression.

The probing itself is the paper's, unchanged (Isaac Sim 6.2, FEM squeeze at 0.5 N and 2 N,
rigid weigh pass with a 20 N grip), run with SLAT-Phys-Scene's runner on a GPU node:
    SLAT-Phys-Scene/setup/isaac62_run.sh python -m slatscene.sim.run_probes \
        --task_sim $PWD/data/photo/sim --sources slatphys --logs $PWD/logs/photo_probes

Run recon / materialize in the slatphys env with SLAT-Phys-Scene on PYTHONPATH
(source SLAT-Phys-Scene/setup/sam3d_env.sh; ATTN_BACKEND=xformers), from SLAT-Phys-Real/:
    python real2sim/photo_pipeline.py recon       --scenes data/photo/scenes --out data/photo/recon
    python real2sim/photo_pipeline.py materialize --scenes data/photo/scenes --recon data/photo/recon --out data/photo/sim
    python real2sim/photo_pipeline.py score       --scenes data/photo/scenes --sim data/photo/sim
"""

import argparse
import glob
import json
import os

import sys

import numpy as np
import yaml

# Frame conventions live in the ROS package (slatphys_real/common.py, no ROS imports).
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src/slatphys_real"))
from slatphys_real import common  # noqa: E402

FEM_MAX_E = 5e7          # slatscene/materialize.py
DEFAULT_FRICTION = 0.6   # slatscene/materialize.py
METAL_SHELL_T = 0.001    # SLAT-Phys-Scene/configs/task_scenes.yaml shell_thickness.metal_crates
HANDE_STROKE = 0.050


def scene_dirs(root):
    return sorted(os.path.dirname(p) for p in glob.glob(os.path.join(root, "*", "scene.json")))


def recon(args):
    import torch
    from PIL import Image

    from slatscene.recon_task_scenes import TABLE_ID, cam_to_world, fit_scale_to_mask, metric_pointmap
    from slatscene.slatphys_recon import SlatPhysReconstructor, spp

    rec = SlatPhysReconstructor()
    done = {}  # several task scenes may share one photo: reconstruct it once
    for sdir in scene_dirs(args.scenes):
        sid = os.path.basename(sdir)
        out_root = os.path.join(args.out, sid)
        cam = json.load(open(os.path.join(sdir, "camera.json")))
        key = (cam["assumed"]["source_image"], json.dumps(cam["cam2world_opencv"]), cam["assumed"]["cam_height_m"])
        if key in done:
            if not os.path.exists(out_root):
                os.symlink(os.path.relpath(done[key], args.out), out_root)
            print(f"{sid}: same photo as {os.path.basename(done[key])}, linked")
            continue
        scene = json.load(open(os.path.join(sdir, "scene.json")))
        c2w = np.array(cam["cam2world_opencv"])
        rgb = np.array(Image.open(os.path.join(sdir, "rgb.png")).convert("RGB"))
        table_mask = np.array(Image.open(os.path.join(sdir, "instance_mask.png")))[..., 0] == TABLE_ID
        masks = np.load(os.path.join(sdir, "masks.npz"))["masks"].astype(bool)

        pm, fit = metric_pointmap(rec, rgb, table_mask, c2w, scene["table"]["height"])
        os.makedirs(out_root, exist_ok=True)
        json.dump(fit, open(os.path.join(out_root, "scale_fit.json"), "w"), indent=2)
        print(f"{sid}: point-map scale x{fit['scale']:.3f} (table inliers {fit['inlier_frac']:.0%})", flush=True)
        for k, mask in enumerate(masks):
            out_dir = os.path.join(out_root, f"obj_{k:02d}")
            os.makedirs(out_dir, exist_ok=True)
            a = rec.reconstruct_object(rgb, mask, pointmap=torch.from_numpy(pm), seed=args.seed)
            mesh = a["mesh_cam"]
            mesh.vertices = cam_to_world(np.asarray(mesh.vertices), c2w)
            voxels = cam_to_world(a["voxels_cam"], c2w)
            s, anchor = fit_scale_to_mask(np.asarray(mesh.vertices), np.asarray(mesh.faces), mask, cam["K"], c2w)
            mesh.vertices = anchor + s * (np.asarray(mesh.vertices) - anchor)
            voxels = anchor + s * (voxels - anchor)
            mesh.export(os.path.join(out_dir, "mesh_world.glb"))
            np.savez_compressed(os.path.join(out_dir, "fields.npz"), voxels_world=voxels.astype(np.float32),
                                E=a["E"].astype(np.float32), rho=a["rho"].astype(np.float32),
                                nu=a["nu"].astype(np.float32), material_id=a["material_id"].astype(np.int16))
            ids, counts = np.unique(a["material_id"], return_counts=True)
            frac = {spp.MATERIAL_NAMES.get(int(i), str(i)): float(c / counts.sum()) for i, c in zip(ids, counts)}
            info = {"scale_m": a["pose"]["scale"] * s, "voxel_size_m": a["voxel_size_m"] * s, "silhouette_scale": s,
                    "solid_volume_m3": float(len(voxels) * (a["voxel_size_m"] * s) ** 3),
                    "shell_area_m2": float(mesh.area), "centroid_world": voxels.mean(0).tolist(),
                    "bbox_world": [voxels.min(0).tolist(), voxels.max(0).tolist()],
                    "slatphys": {"material_fractions": frac, "majority_material": max(frac, key=frac.get),
                                 "density_median": float(np.median(a["rho"])), "E_median": float(np.median(a["E"])),
                                 "nu_median": float(np.median(a["nu"]))}}
            json.dump(info, open(os.path.join(out_dir, "recon.json"), "w"), indent=2)
            ext = (np.array(info["bbox_world"][1]) - np.array(info["bbox_world"][0])) * 100
            sp = info["slatphys"]
            print(f"  obj {k + 1} ({scene['objects'][k]['name']}): silhouette scale x{s:.3f}, "
                  f"size {ext[0]:.1f} x {ext[1]:.1f} x {ext[2]:.1f} cm, material {sp['majority_material']} "
                  f"{frac[sp['majority_material']]:.0%}, rho {sp['density_median']:.0f} kg/m3, "
                  f"E {sp['E_median']:.1e} Pa, nu {sp['nu_median']:.2f}", flush=True)
        done[key] = out_root


def materialize(args):
    import trimesh

    from slatscene.materialize import envelope_volume, grasp_spec, mass_properties

    for sdir in scene_dirs(args.scenes):
        sid = os.path.basename(sdir)
        scene = json.load(open(os.path.join(sdir, "scene.json")))
        top = scene["table"]["height"]
        out_dir = os.path.join(args.out, "slatphys", sid)
        os.makedirs(out_dir, exist_ok=True)
        objs = []
        for k, o in enumerate(scene["objects"]):
            odir = os.path.join(args.recon, sid, f"obj_{k:02d}")
            mesh = trimesh.load(os.path.join(odir, "mesh_world.glb"), force="mesh")
            sp = json.load(open(os.path.join(odir, "recon.json")))["slatphys"]
            rho, E, nu = sp["density_median"], sp["E_median"], sp["nu_median"]
            hollow = sp["majority_material"] == "metal"
            mass = rho * mesh.area * METAL_SHELL_T if hollow else rho * envelope_volume(mesh)
            mesh_path = os.path.join(out_dir, f"obj_{k:02d}.obj")
            mesh.export(mesh_path)
            com, inertia = mass_properties(mesh, mass, hollow)
            vol = envelope_volume(mesh)
            g = grasp_spec(mesh, top)
            objs.append({"name": o["name"], "mesh": os.path.abspath(mesh_path), "body": "fem" if E < FEM_MAX_E else "rigid",
                         "mass_kg": float(mass), "com_world": com.tolist(), "inertia_world": inertia.tolist(),
                         "fem_density": float(mass / vol), "E": float(E), "nu": float(nu), "friction": DEFAULT_FRICTION,
                         "hollow": bool(hollow), "envelope_volume_m3": vol, "grasp": g,
                         "slatphys_material": sp["majority_material"]})
            if g["width"] > HANDE_STROKE - 0.005:
                print(f"  WARNING {sid} obj {k + 1} ({o['name']}): grasp width {g['width'] * 1000:.0f} mm, "
                      f"the Hand-E opens {HANDE_STROKE * 1000:.0f} mm")
        sim = {"scene_id": sid, "source": "slatphys", "task": scene["task"], "instruction": scene["instruction"],
               "table": scene["table"], "robot_base_xy": scene["robot_base_xy"], "objects": objs}
        json.dump(sim, open(os.path.join(out_dir, "sim.json"), "w"), indent=2)
        # The same grasps in the real robot's `base` frame (as real2sim/materialize_real.py writes
        # them), for dry runs of slatphys_real. A preview only: the photo is not registered to the
        # robot, so its positions are where photo_scene.py put the scene, not where the objects are.
        cfg = yaml.safe_load(open(args.config))
        theta, dz = common.world_theta(cfg), top - args.table_z_base
        json.dump({"scene_id": sid, "task": scene["task"], "frame": "base", "table_z_base": args.table_z_base,
                   "preview_from_uncalibrated_photo": True, "objects": [{
                       "name": o["name"], "obj_id": o["name"], "grasp": {
                           "centre_xy": np.round(common.world_xy_to_base(cfg, o["grasp"]["centre_xy"]), 4).tolist(),
                           "yaw": float((o["grasp"]["yaw"] - theta + np.pi) % (2 * np.pi) - np.pi),
                           "width": o["grasp"]["width"], "top_z": o["grasp"]["top_z"] - dz,
                           "height": o["grasp"]["height"]}} for o in objs]},
                  open(os.path.join(out_dir, "grasps.json"), "w"), indent=2)
        print(f"{sid:20s} " + "  ".join(f"{o['name']}: {o['mass_kg'] * 1000:.0f} g {o['body']} E={o['E']:.0e} "
                                        f"w={o['grasp']['width'] * 1000:.0f}mm h={o['grasp']['height'] * 1000:.0f}mm"
                                        for o in objs))


def score(args):
    from slatscene.score_readings import answer

    results = {}
    for sdir in scene_dirs(args.scenes):
        sid = os.path.basename(sdir)
        scene = json.load(open(os.path.join(sdir, "scene.json")))
        rp = os.path.join(args.sim, "slatphys", sid, "readings.json")
        if not os.path.exists(rp):
            print(f"{sid}: no readings.json")
            continue
        r = json.load(open(rp))
        ans, load, squeeze = answer(scene["task"], r["readings"])
        lifted = [d["weigh"]["lifted"] for d in r["diagnostics"]]
        names = [o["name"] for o in scene["objects"]]
        res = {"task": scene["task"], "answer": names[ans - 1], "load_N": load, "compression_m": squeeze,
               "lifted": lifted, "objects": names}
        truth = [o.get("real", {}) for o in scene["objects"]]
        if scene["task"] == "heavy" and all("mass_g" in t for t in truth):
            res["truth"] = names[int(np.argmax([t["mass_g"] for t in truth]))]
        if scene["task"] == "deformable" and all("soft" in t for t in truth):
            res["truth"] = names[int(np.argmax([t["soft"] for t in truth]))]
        results[sid] = res
        print(f"{sid}: answer {res['answer']}" + (f" (truth {res['truth']})" if "truth" in res else "") + "  " +
              "  ".join(f"{n}: load {l * 1000:.1f} mN, compression {c * 1000:.2f} mm, lifted {lf}"
                        for n, l, c, lf in zip(names, load, squeeze, lifted)))
    json.dump(results, open(os.path.join(args.sim, "answers.json"), "w"), indent=2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("step", choices=["recon", "materialize", "score"])
    ap.add_argument("--scenes", default="data/photo/scenes")
    ap.add_argument("--recon", default="data/photo/recon")
    ap.add_argument("--out", default=None)
    ap.add_argument("--sim", default="data/photo/sim")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--config", default="configs/real.yaml", help="robot frames, for the preview grasps.json")
    ap.add_argument("--table_z_base", type=float, default=-0.011,
                    help="table height in base for the preview grasps.json (the lab's plans: -11 mm)")
    args = ap.parse_args()
    if args.step == "recon":
        args.out = args.out or args.recon
        recon(args)
    elif args.step == "materialize":
        args.out = args.out or args.sim
        materialize(args)
    else:
        score(args)


if __name__ == "__main__":
    main()
