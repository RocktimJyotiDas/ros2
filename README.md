# SLAT-Phys real-robot validation: "which is heavier?" and "which is deformable?"

ROS 2 code for a UR3e + Robotiq Hand-E that answers two questions about two objects on a table,
plus a Gazebo test bench that runs the same code before it touches the real robot.

| | |
|---|---|
| Robot | UR3e, Robotiq Hand-E (50 mm stroke, 20–185 N), the UR's wrist force/torque sensor |
| Lab stack | [ur3e_experiments](https://github.com/alexhjkim/ur3e_experiments) (ROS 2 Humble in Docker, ur_robot_driver, ENME480 bridge) |
| Tasks | **heavy**: which object is heavier · **deformable**: which object is deformable |
| Simulation | Gazebo Harmonic + ur_simulation_gz (ROS 2 Jazzy) |

## What the robot does

One probing run answers both tasks. For each object, the robot does two passes:

1. **Squeeze pass (→ deformable).**
   - The open gripper moves to 10 cm above the object, then descends straight down to the grasp height.
   - It closes at **20 N**, then at **60 N**, recording the finger opening at each force.
   - It releases and retreats straight up.
   - Compression = opening at 20 N − opening at 60 N.
2. **Weigh pass (→ heavy).**
   - The closed, empty gripper goes to the hold point (25 cm in front of the base, 22 cm above the table) and the wrist force is averaged for 2 s.
   - The gripper goes back, descends, grips at **20 N**, lifts straight up and carries the object to the hold point.
   - The force is averaged again with the object held.
   - It puts the object back where it was, releases and retreats.
   - The empty gripper is weighed once more to cancel sensor drift.
   - Load = |F_with − F_without|.

Answers: **heavy** = largest load, **deformable** = largest compression. Grasp positions and gripper yaw come from a calibrated camera photo (`locate` → `grasps.json`); grasp widths from the measured object sizes.

## Safety layers (all on by default)

| Layer | What it does |
|---|---|
| Arming phrase | Nothing moves without `--execute MOVE_REAL_UR3E` (the lab's ur3e_plan_runner convention). |
| `--dry_run` | Prints every TCP target; no motion, no gripper. Run this first. |
| `--step` | Waits for Enter before every motion. |
| Preflight (`run_tasks`) | Before any motion, refuses grasps that are wider than the stroke − 4 mm, have no IK solution, lie beyond 0.45 m reach, or go below the table floor. Every problem is printed. |
| Lab PlanSender checks | Every joint goal goes through ur3e_plan_runner: robot RUNNING, safety mode NORMAL/REDUCED, External Control playing, **speed slider ≤ 30 %**, fresh state, one bridge, ≤ 0.35 rad per goal. |
| Motion limits (`configs/real.yaml`) | ≤ 0.8 rad/s joint speed. TCP never closer than 18 mm to the table. Straight descents and lifts stay within 1.5 mm of the line. A goal is reached when joints are within 0.003 rad and still. IK branch flips abort the motion. |
| Forces | Grip 20–60 N only (the Hand-E's range starts at 20 N). |

## Repository

```
src/slatphys_real/          the robot code (ament_python)
  run_tasks.py              ONE command for both tasks: preflight -> probe -> answers.json
  locate.py                 calibrated camera photo (front or top) -> grasps.json: box fit per object
  probe.py                  the scripted squeeze + weigh passes
  tasks.py                  the answer rule (heavy / deformable); `answer` re-scores saved readings
  arm_enme480.py            arm through the lab stack (PlanSender -> ENME480 bridge)  [real robot]
  arm.py                    arm through FollowJointTrajectory                        [Gazebo]
  hande.py                  Robotiq Hand-E over the URCap socket (port 63352)        [real robot]
  gripper_ros.py            force-controlled fingers over ros2_control               [Gazebo]
  ur_kinematics.py          UR3e FK / analytic IK (numpy)
  common.py                 config, frames, backend selection
  place.py capture.py calibrate_camera.py calibrate_gripper.py   scene setup / calibration
  launch/bringup.launch.py  lab stack: ur_robot_driver + ENME480 bridge + camera
src/slatphys_gazebo/        the Gazebo test bench (ament_cmake)
  urdf/ur3e_hande_gz.urdf.xacro   UR3e + Hand-E-sized gripper + wrist F/T sensor
  config/controllers.yaml   arm trajectory controller + gripper effort controller
  launch/sim.launch.py      ur_simulation_gz + gripper controller + F/T bridge
  scripts/make_world.py     Gazebo world from a SLAT-Phys twin (objects, masses, softness)
  worlds/                   generated scenes for this experiment (scripts/make_worlds.sh)
configs/real.yaml           the real robot: IP, topics, gripper geometry, protocol, safety limits
configs/gazebo.yaml         real.yaml with only the simulation backends changed (marked GAZEBO)
data/plans/img4827.json     the scene: object names, planned positions, ground truth to fill in
data/twin/img4827/          the photo, masks, and the SLAT-Phys twin (sim.json, hull meshes, grasps.json)
real2sim/                   optional offline (GPU) SLAT-Phys comparison:
  make_scene.py             photo + calibration -> object masks and real table positions
  materialize_real.py       reconstruction -> grasps.json (base frame) + the simulation twin
  plan_scenes.py            sample scene plans from an object inventory (configs/objects.yaml)
  run_sim_probes.py, sim_probe_launcher.py, sim_squeeze_model.py   the twin probed in Isaac Sim
  correlate.py              sim-vs-real metrics over many scenes
  photo_scene.py, photo_pipeline.py   the paper's pipeline on an uncalibrated phone photo (data/twin)
scripts/offline.sh          recon / materialize / simprobe / correlate wrappers (needs SLATPHYS_SCENE)
scripts/sim_camera_pipeline.sh   Gazebo: the real sequence (capture, capture, locate, run_tasks), simulated front camera
scripts/sim_demo.sh         Gazebo: launch, check the bench, run both tasks (RECORD=1 records video)
scripts/make_worlds.sh      regenerate the Gazebo worlds from the twin
scripts/record_world.py, record_frames.py, make_video.py   Gazebo video recording
tests/                      no-robot tests (mock, lab-stack mock, Gazebo checks, offline/)
results/gazebo/, results/videos/   reference Gazebo runs and their videos
docker/                     robot image (on the lab's) and simulation image
docs/CODE_WALKTHROUGH.md    how the code works, file by file
```

## Run it in Gazebo

```bash
docker build -t slatphys-sim:jazzy -f docker/Dockerfile.sim .
xhost +local:docker
docker/run_sim.sh
# inside the container (repo at /ws):
scripts/sim_camera_pipeline.sh                                          # the real experiment's sequence, camera included
GUI=1 scripts/sim_demo.sh                                              # twin scene, Gazebo window
GUI=1 scripts/sim_demo.sh src/slatphys_gazebo/worlds/img4827_fit40_rigiddisc
scripts/sim_demo.sh src/slatphys_gazebo/worlds/img4827_asis           # preflight refuses (disc too wide)
tests/mock/run.sh                                                      # no simulator: dry run, mock run, refusal
RECORD=1 scripts/sim_demo.sh && python3 scripts/make_video.py runs/img4827_fit40   # captioned MP4
```

Videos of both scenes: `results/videos/scene1_twin.mp4`, `results/videos/scene2_control_rigid_disc.mp4`.
The real experiment's camera sequence (front camera, capture → `locate` → `run_tasks`), seen from the
front camera and an overview: `results/videos/scene3_front_camera_pipeline.mp4`
(`RECORD=1 scripts/sim_camera_pipeline.sh`, then `scripts/make_video.py runs/img4827_cam --cams front overview`).

Each run takes ~7 min. It writes `runs/<world>/answers.json`; the log shows every probing step.

## Gazebo results (`results/gazebo/`)

The bench reproduces the robot's interfaces: the same trajectory action, the F/T topic of
ur_robot_driver, and a gripper with a constant closing force.

Checks:
- The bench's kinematics match Gazebo's TF: `ur_kinematics` FK = `base`→`tool0` (0.00 mm).
- The F/T sensor reads the gripper's weight (11.97 N).
- In every run the finger opening equals the object's actual compression to 0.01 mm, and the
  measured load equals mass × g (14.7 / 22.2 mN for 1.50 / 2.27 g).

| Scene | heavy | deformable |
|---|---|---|
| `img4827_fit40`: the SLAT-Phys twin, scaled so the grasps fit the Hand-E | disc ✔ | disc ✘ (expected toy) |
| `img4827_fit40_rigiddisc`: control, disc rigid | disc ✔ | toy ✔ |
| `img4827_asis`: the twin at the photo's scale | refused by preflight: disc 82 mm > 46 mm | — |
| `img4827_cam`: **the real sequence**, capture → `locate` → `run_tasks`, simulated front camera, soft toy + rigid disc | disc ✔ | toy ✔ |

Why the twin's "deformable" differs:
- SLAT-Phys predicts **both** objects soft (E = 8e4 and 2.2e5 Pa), so their predicted
  compressions are close: 9.7 vs 7.7 mm.
- In Gazebo's contact model a flat object (shorter than the finger pads) receives only about 2/3
  of the grip force, which flips that close call.
- The code's readings themselves are exact (see above). The control scene shows the full
  deformable path working when the answer is unambiguous.

## Run it on the robot

The robot PC uses the lab's Docker stack.

```bash
cd ur3e_experiments && docker build -t ur3e-real:humble .
cd <this repo> && docker build -t ur3e-slatphys:humble -f docker/Dockerfile.robot .
UR_IP=192.168.77.22 UR3E_EXPERIMENTS=~/ur3e_experiments docker/run_robot.sh
# inside (every new terminal: the three source lines)
source /opt/ros/humble/setup.bash && source /opt/enme480_mrc/install/setup.bash
cd /workspace && colcon build --symlink-install --packages-select ur3e_plan_runner slatphys_real
source /workspace/install/setup.bash && cd /workspace/src/slatphys
ros2 launch slatphys_real bringup.launch.py robot_ip:=$UR_IP        # then Play External Control, slider <= 30 %
```

Then, all on the robot PC (no GPU, no reconstruction, no simulation).

**The camera.** Use any RGB camera on a fixed mount in front of the table, looking down at the
object area at roughly 30–50°. A top-down mount also works. For example: 0.25–0.40 m above the
table, 0.4–0.6 m from the objects. The object area and the calibration markers must be in view.
Seen from the camera, the objects must stand **side by side, not one behind the other**. The arm
may appear in the photos as long as it doesn't hide the objects; `capture` sends it to `home_q`
for every photo.

1. **Calibrate (once).**
   ```bash
   ros2 run slatphys_real calibrate_camera --markers 6 --execute MOVE_REAL_UR3E   # >= 4 ArUco markers, 6 recommended
   ros2 run slatphys_real calibrate_gripper --widths_mm 10 20 30 40               # optional: finger width model
   ros2 run slatphys_real capture --out data/real/captures/_empty --execute MOVE_REAL_UR3E   # empty table
   ```
2. **Set up the scene.** Fill `size_mm` for each object in the plan: `[width, length, height]` in
   mm, measured with calipers. `locate` models each object as a box of that size, and the grasp
   width is the smaller of width and length. Place the objects (`place` points the fingertips at
   each planned position), then photograph them:
   ```bash
   ros2 run slatphys_real place   --plan data/plans/img4827.json --execute MOVE_REAL_UR3E
   ros2 run slatphys_real capture --out data/real/captures/img4827 --execute MOVE_REAL_UR3E
   ```
3. **Locate the objects** (`locate`, a few seconds):
   ```bash
   ros2 run slatphys_real locate --plan data/plans/img4827.json --capture data/real/captures/img4827 \
       --empty data/real/captures/_empty --out data/real/grasps/img4827.json
   ```
   - It masks each object by its colour change against the empty table. Shadows only darken
     the table, so they are ignored.
   - It models each object as a box of its measured size, standing on the table.
   - It searches the box's position and rotation until its outline, projected through the
     calibrated camera, best overlaps the mask (IoU). This works from the front or from above.
   - The grasp closes across the box's narrow side, at its centre.

   **Check `data/real/grasps/img4827_overlay.png`:** each white box outline should sit on its
   object, and the yellow line (the closing axis) should run across the narrow side. It warns if
   the overlap is below 0.70. **Do not move the objects from here on.**
4. **Run the two tasks** (squeeze pass → deformable, weigh pass → heavy):
   ```bash
   G=data/real/grasps/img4827.json
   ros2 run slatphys_real run_tasks --grasps $G --plan data/plans/img4827.json --out runs/real_img4827 --dry_run
   ros2 run slatphys_real run_tasks --grasps $G --plan data/plans/img4827.json --out runs/real_img4827 \
       --execute MOVE_REAL_UR3E --activate_gripper --step
   ```
   With the default 30 % slider a scene of two objects takes about 7 minutes. Keep a hand on the e-stop.

**Validated in Gazebo** (`scripts/sim_camera_pipeline.sh`, `results/gazebo/img4827_cam/`): the same
four commands, with a simulated **front camera** standing in for the lab camera (0.50 m in front
of the objects, 0.30 m above the table, 29° down, 1280×720):
- `locate` placed the boxes within 0.8 and 1.0 mm of the true centres, with the closing axis
  within about 1° and fit IoU 0.97 and 0.95.
- `run_tasks` then answered **heavier = green_disc ✔** and **deformable = yellow_toy ✔**
  (soft toy, rigid disc).
- The top-down variant (`img4827_cam_top`) is in `results/gazebo/img4827_cam_top/`.

### Optional: compare with SLAT-Phys's predictions (GPU, not needed to run the tasks)

`real2sim/` and `scripts/offline.sh` reconstruct the same captured scene with SAM-3D + SLAT-Phys
(`make_scene.py` → `offline.sh recon` → `offline.sh materialize`), probe the twin in Isaac Sim
(`offline.sh simprobe`) and correlate sim with real (`offline.sh correlate`). Their
`grasps.json` can replace `locate`'s.

## Offline dependencies (optional SLAT-Phys comparison)

The offline steps (`real2sim/`, `scripts/offline.sh`, `tests/offline/`) need, on a GPU machine:

- the **SLAT-Phys-Scene** checkout, `export SLATPHYS_SCENE=...`, which provides the SAM-3D +
  SLAT-Phys reconstruction (`slatscene.recon_task_scenes`), its helpers and the Isaac Sim probe
- its python env (SAM-3D-Objects + SLAT-Phys), activated by `$SLATPHYS_SCENE/setup/sam3d_env.sh`,
  or your own via `SLATPHYS_ENV_SH`
- Isaac Sim 6.2, only for `offline.sh simprobe` (runner: `$SLATPHYS_SCENE/setup/isaac62_run.sh`,
  or `SLATPHYS_ISAAC_RUNNER`)

Check them with `PYTHONPATH=$SLATPHYS_SCENE python tests/offline/test_offline.py`, which covers IK,
camera calibration, frames and reach, `make_scene` (including an unmeasured plan), `correlate`, and
the Isaac launcher.

## Open items before the real run

- **Object size.** The photo has no calibration, so the twin's scale assumes the camera was
  0.30 m above the table. At that scale the disc is 82 mm across, which the Hand-E (50 mm) cannot
  grasp. Measure both objects; if the disc really is ~8 cm, replace it with an object whose
  narrow side is 15–40 mm.
- **Ground truth.** Fill `mass_g`, `size_mm` and `soft` in `data/plans/img4827.json` before
  scoring. `make_scene` accepts them as `null`.
- **Use real grasps.** `data/twin/img4827/grasps.json` is a preview from the uncalibrated photo
  (`preview_from_uncalibrated_photo: true`). Never run the robot on it: real positions come from
  step 3.
- **Check on the robot.** `gripper.tcp_z`, `tip_z` and `pad_width`, and `world.workspace_yaw_deg`
  in `configs/real.yaml`.
