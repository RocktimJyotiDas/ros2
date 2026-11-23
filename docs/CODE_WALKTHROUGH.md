# Code walkthrough

## 1. Data flow

```
calibrated camera photo + empty-table photo + measured sizes ──(locate)──► grasps.json  (per object: centre, yaw, width, top, height; robot base frame)
                                                    │
configs/real.yaml ─────────────────────────────────►│
                                                    ▼
                               run_tasks ── preflight ── probe (squeeze pass, weigh pass)
                                                    │
                                                    ▼
                          readings.json ──► tasks.py ──► answers.json  (heavy, deformable)
```

`grasps.json` is the only scene input. For each object it gives:
- the grasp centre (x, y) in the UR `base` frame
- the yaw of the closing axis (the object's narrow horizontal axis)
- the grasp width across the fingers
- the object's top height and its height above the table

The same file drives the real robot and the Gazebo bench.

## 2. `src/slatphys_real` (the robot code)

### `run_tasks.py`: the entry point
1. Loads the config (`SLATPHYS_REAL_CONFIG` or `--config`) and `grasps.json`.
2. **Preflight** (`preflight()`), with no motion. For every object it checks:
   - the width is at most 46 mm (50 mm stroke − 4 mm margin)
   - the TCP above the object and at the grasp is inside the 0.45 m reach and above the table safety floor
   - an IK solution exists for the grasp yaw or yaw + 180°

   Any problem is printed and the program exits before moving.
3. Calls `probe.main(... --phase both)` with the user's flags (`--dry_run`, `--step`,
   `--execute MOVE_REAL_UR3E`, `--activate_gripper`).
4. Reads `readings.json` and prints and saves both answers (`tasks.report`). With `--plan` it
   scores them against the measured ground truth; with `--sim` it compares them with the twin's
   simulated readings.

### `locate.py`: where the objects are (camera → `grasps.json`)
1. Undistort the scene photo and the empty-table photo with the calibration's `K`, `D`.
2. **Masks (`diff_masks`).**
   - Compare chromaticity, r, g, b each divided by their sum, so colour independent of brightness.
     A shadow darkens the table without changing its colour, so it drops out.
   - A pixel that got brighter also counts.
   - Clean up with morphological open/close and hole filling, then keep the N largest blobs. N is
     the number of plan objects.
   - Hand-drawn masks (`--masks_dir`) skip this step.
3. **Matching.** Each blob's centroid is cast onto the plane at the objects' typical mid-height,
   and blobs are assigned to plan objects by nearest `xy_base` (Hungarian assignment). It warns if
   a blob is more than 6 cm away. That cast point is also the search's starting point.
4. **Box fit (`BoxFitter`).** Each object is a box: narrow side × long side × height, from the
   plan's `size_mm`.
   - For a candidate pose (x, y, yaw on the table), `project` maps the box's 8 corners through
     `K · cam2base⁻¹`.
   - `iou` fills the convex hull of those corners (the outline of a convex box) and measures its
     overlap with the mask.
   - `fit` runs a coarse grid: ±5 cm in 4 mm steps, every 5° over 180° (90° for a square
     footprint). Then a fine grid around the best: ±4 mm in 1 mm steps, ±5° in 1° steps.
   - From an angled front view the depth of an object isn't visible directly, but its height is
     known, so the box that matches the outline also fixes its position.
5. **Grasp.** Centre = the box centre, closing axis across the narrow side, width = the narrow
   side, top = table + height.
6. **Output.** Writes `grasps.json` with each fit's IoU, and an overlay PNG showing the masks, the
   fitted box outlines and the closing axes. It warns if IoU < 0.70: the size is wrong, or the
   object is hidden.

### `probe.py`: the scripted probing (`Prober`)
- `geometry(o)` computes three things:
  - Grasp height: pad centre at max(table + pad half length + clearance, object mid-height).
  - The point 10 cm above the object top.
  - The yaw. Yaw and yaw + 180° close the fingers along the same line, so it takes the one
    needing less wrist rotation.
- `squeeze(k, o)`:
  1. Open, move above the object (joint move), descend in a straight line.
  2. `close_on(20 N)`, then `close_on(60 N)`, recording the finger width after each.
  3. Open and lift straight.
- `weigh(k, o)`:
  1. Close the empty gripper, go to the hold point, average the wrench (`empty`).
  2. Open, go above the object, descend, grip at 20 N, lift, go to the hold point, average the
     wrench (`load`).
  3. Go back, descend to 5 mm above the grasp height, open, lift.
  4. Weigh the empty gripper again at the hold point (`after`).

  `lifted` = the fingers stopped on something (Hand-E OBJ = 2) and are still more than 2 mm apart.
- `main()` refuses real motion without the arming phrase, picks the backends
  (`arm.start`, `common.make_gripper`), runs both passes over all objects, homes the arm, and
  writes `readings_squeeze.json`, `readings_weigh.json` and the merged `readings.json`.

### `tasks.py`: the answer rule (no ROS)
- **load** = |F_with − mean(F_without, F_without_after)|, the force magnitude. It doesn't depend
  on the frame the driver publishes in.
- **compression** = width at the first squeeze force − width at the last.
- **heavy** = argmax load, **deformable** = argmax compression. This is the same rule as the
  paper's simulation scorer.

### Arm backends
- **`arm_enme480.py` (`EnmeArm`, real robot).** Subclasses the lab's `ur3e_plan_runner.PlanSender`,
  so every goal passes the lab's checks: robot and safety mode, External Control, speed slider
  ≤ 30 %, one bridge, ≤ 0.35 rad per goal. It adds:
  - a tighter settle: within 0.003 rad, and still
  - straight-line Cartesian moves, split into joint goals until the TCP stays within 1.5 mm of
    the line
  - wrench averaging from `/force_torque_sensor_broadcaster/wrench`
- **`arm.py` (`Arm`, Gazebo).** The same interface, sending `FollowJointTrajectory` goals straight
  to `scaled_joint_trajectory_controller`.
  - Cosine-profile moves, stretched so that no joint exceeds 0.8 rad/s.
  - Straight lines are sampled every 0.1 s through IK, and a jump of more than 0.35 rad between
    samples aborts the move.
  - `start()` chooses the backend from `motion_backend` in the config.

### Gripper backends (same methods: `open`, `close_on(force)`, `width()`, `shutdown`)
- **`hande.py` (`HandE`, real robot).** The lab's Robotiq socket client (port 63352) with a chosen
  force and slow close.
  - Force → register is linear over 20–185 N.
  - Width = w0 + slope · POS, with the slope from `calibrate_gripper`.
  - `close_on` returns the Hand-E's OBJ code: 2 = stopped on an object, 3 = fully closed.
- **`gripper_ros.py` (`RosGripper`, Gazebo).** Publishes `[−F, −F]` to the finger effort controller
  (a constant closing force) and reads the finger joints from `/joint_states`.
  - "Settled" means the opening moved by less than 0.2 mm over 0.5 s of message time, so it
    stays correct when Gazebo runs slower than real time.
  - Returns the same OBJ codes.

### `ur_kinematics.py`
Nominal UR3e DH parameters with analytic IK in the ur_robot_driver `base` frame.
`tool_pose(p, yaw, tcp_z)` gives a top-down tool orientation with the closing axis along `yaw`;
`solve_tcp` returns the IK solution nearest the current joints. On the real robot the camera
calibration uses the same FK, so commanded and measured positions are consistent.

### `common.py`
- `load_config`: relative paths resolve against the repo root.
- `table_z`: from the camera calibration, or fixed in the config (Gazebo).
- `make_gripper`: picks the backend from `gripper.backend`.
- Frame helpers between the simulation's world frame and the robot `base`.

### Scene setup and calibration (real robot)
- **`calibrate_camera.py`**: you free-drive the closed fingertips onto 6 ArUco markers; solvePnP
  gives the camera pose in `base`, and the touch points give the table height.
- **`calibrate_gripper.py`**: gauge blocks give the width model.
- **`place.py`**: points the fingertips at each planned position so the objects go where the plan
  says.
- **`capture.py`**: takes the scene photo (10 frames averaged) from the ROS image topic, after
  sending the arm to `home_q`, so it is in the same pose in every photo.
- **`launch/bringup.launch.py`**: starts ur_robot_driver, the ENME480 bridge (8 s later) and the
  RealSense camera.

## 3. `src/slatphys_gazebo` (the test bench)

### `urdf/ur3e_hande_gz.urdf.xacro`
- **Arm.** ur_simulation_gz's UR3e, starting at the lab's home pose.
- **Gripper.** A 0.86 kg body on `tool0` with two 50 g fingers on prismatic joints along tool x
  (0–25 mm each).
  - The geometry matches `configs/gazebo.yaml`: tips 0.145 m below the flange, pads 40 mm long
    and 22 mm wide.
  - The fingers take an **effort** command. A constant force is what the Hand-E's force setting
    does.
- **F/T.** A Gazebo `force_torque` sensor on `wrist_3_joint`, published on
  `/force_torque_sensor_broadcaster/wrench` (bridged as `WrenchStamped`). It is the same topic
  and type as ur_robot_driver's.

### `config/controllers.yaml`, `launch/sim.launch.py`
- `controllers.yaml` runs the arm's `scaled_joint_trajectory_controller` and the gripper's
  `JointGroupEffortController` at 5 kHz, the physics rate.
- `sim.launch.py` includes ur_simulation_gz's launch with this description, controllers and the
  chosen world, then spawns the gripper controller and the F/T bridge.

### `scripts/make_world.py`
Builds a Gazebo world from a SLAT-Phys twin (`sim.json` + `grasps.json`):
- Each object becomes a box with the reconstructed footprint and height, oriented along the grasp
  axis, carrying the twin's mass.
- A **soft** object (SLAT-Phys E < 5e7 Pa) is two outer plates on a prismatic spring:
  - stiffness k = E · pad_width · h / L (the linear-elastic contact model of the offline simulation)
  - travel limited to 0.8 L
- It also writes `grasps.json` in `base` and `plan.json` with the scene's ground truth.
- `--fit_width` scales the scene uniformly, and `--rigid` makes chosen objects rigid (control scenes).
- Worlds step at 0.2 ms. At 1 ms, Gazebo's DART engine passed the wrong force from the fingers to
  the gram-scale plates.

### Bench checks (`scripts/sim_demo.sh`, `tests/gazebo/`)
- `SMOKE=1` checks:
  - the controllers are active
  - the F/T sensor reads the gripper's weight
  - `ur_kinematics` FK equals Gazebo's `base`→`tool0` TF
- `SWEEP="10 20 30"` grips each object at several forces and prints the finger opening next to
  the object's own spring compression (read from Gazebo).
- `spring_check.sh` checks that Gazebo honours SDF spring stiffness: 1 kg on k = 1000 N/m sags
  9.80 mm.

## 4. Configs

`configs/real.yaml` holds everything about the robot:
- IP, topics, home pose
- gripper geometry (`tcp_z`, `tip_z`, `pad_half`, `pad_width`)
- protocol (squeeze forces, lift force, clearances, hold point, averaging windows)
- safety limits
- frames

`configs/gazebo.yaml` is a copy with only the lines marked `GAZEBO` changed: `motion_backend:
trajectory`, `gripper.backend: ros` (plus its joint and topic names), and a fixed `table_z_base`
instead of the camera calibration. The protocol and the safety limits are identical.
