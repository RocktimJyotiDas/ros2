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

Answers: **heavy** = largest load, **deformable** = largest compression. Grasp positions, gripper
yaw and grasp width come from the SLAT-Phys reconstruction of a top-down photo of the scene (`grasps.json`).

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
scripts/sim_demo.sh         Gazebo: launch, check the bench, run both tasks
scripts/make_worlds.sh      regenerate the Gazebo worlds from the twin
tests/                      no-robot tests (mock, lab-stack mock, Gazebo checks)
results/gazebo/             reference Gazebo runs (readings, answers, logs)
docker/                     robot image (on the lab's) and simulation image
docs/CODE_WALKTHROUGH.md    how the code works, file by file
```

## Run it in Gazebo

```bash
docker build -t slatphys-sim:jazzy -f docker/Dockerfile.sim .
xhost +local:docker
docker/run_sim.sh
# inside the container (repo at /ws):
GUI=1 scripts/sim_demo.sh                                              # twin scene, Gazebo window
GUI=1 scripts/sim_demo.sh src/slatphys_gazebo/worlds/img4827_fit40_rigiddisc
scripts/sim_demo.sh src/slatphys_gazebo/worlds/img4827_asis           # preflight refuses (disc too wide)
tests/mock/run.sh                                                      # no simulator: dry run, mock run, refusal
```

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

Then:

1. **Calibrate (once).** Run `calibrate_camera` (touch 6 ArUco markers), `calibrate_gripper`, and
   `capture --out data/real/captures/_empty` for the empty-table photo.
2. **Set up the scene.** Run `place --plan data/plans/img4827.json --execute MOVE_REAL_UR3E` (the
   fingertips point at each planned position), then `capture`.
3. **Reconstruct offline.** Run the SLAT-Phys reconstruction on the captured photo (GPU, outside
   this repo). It writes `grasps.json` in the robot's `base` frame. **Do not move the objects.**
4. **Run the tasks:**
   ```bash
   G=data/real/grasps/img4827.json
   ros2 run slatphys_real run_tasks --grasps $G --plan data/plans/img4827.json --out runs/real_img4827 --dry_run
   ros2 run slatphys_real run_tasks --grasps $G --plan data/plans/img4827.json --out runs/real_img4827 \
       --execute MOVE_REAL_UR3E --activate_gripper --step
   ```
   With the default 30 % slider a scene of two objects takes about 7 minutes. Keep a hand on the e-stop.

## Open items before the real run

- **Object size.** The photo has no calibration, so the twin's scale assumes the camera was
  0.30 m above the table. At that scale the disc is 82 mm across, which the Hand-E (50 mm) cannot
  grasp. Measure both objects; if the disc really is ~8 cm, replace it with an object whose
  narrow side is 15–40 mm.
- **Ground truth.** Fill `mass_g`, `size_mm` and `soft` in `data/plans/img4827.json`.
- **Check on the robot.** `gripper.tcp_z`, `tip_z` and `pad_width`, and `world.workspace_yaw_deg`
  in `configs/real.yaml`.
