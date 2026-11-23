#!/bin/bash
# The real experiment's sequence, end to end in Gazebo, with a simulated camera (front view by
# default) standing in for the lab camera. Every step is the command used on the robot:
#   1. capture   the empty table             (world_empty.sdf: no objects; the arm goes home first)
#   2. capture   the scene                   (world.sdf)
#   3. locate    camera photos -> grasps.json (no reconstruction, no simulation data)
#   4. run_tasks preflight -> squeeze pass + weigh pass -> answers (heavier, deformable)
# The camera calibration is the simulated camera's exact pose (what calibrate_camera measures in the lab)
# with K from the captured camera_info. The located grasps are also compared with the scene's true ones.
#
#   scripts/sim_camera_pipeline.sh [world_dir]       (default: src/slatphys_gazebo/worlds/img4827_cam)
#   RECORD=1   record steps 2-4 from the scene camera's own view ("front") and an overview, then
#              python3 scripts/make_video.py runs/<world> --cams front overview
# Needs ROS 2 Jazzy + Gazebo Harmonic (docker/Dockerfile.sim). Writes runs/<world>/.
set -e
R=$(cd "$(dirname "$0")/.." && pwd)
WORLD_DIR=$(cd "${1:-$R/src/slatphys_gazebo/worlds/img4827_cam}" && pwd)
NAME=$(basename "$WORLD_DIR")
W=${BUILD_DIR:-/tmp/$USER/slatphys_ws}; mkdir -p $W
OUT=$R/runs/$NAME; rm -rf $OUT; mkdir -p $OUT/captures $OUT/calib
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-$((RANDOM % 100 + 100))} ROS_LOCALHOST_ONLY=1 PYTHONNOUSERSITE=1 PYTHONUNBUFFERED=1
export GZ_PARTITION=slatphys_$$ SLATPHYS_REAL_CONFIG=$R/configs/gazebo.yaml GZ_SIM_RESOURCE_PATH=/opt/ros/jazzy/share
export LIBGL_ALWAYS_SOFTWARE=1
cd $R && colcon --log-base $W/log build --base-paths src --symlink-install --build-base $W/build \
    --install-base $W/install --packages-select slatphys_real slatphys_gazebo 2>&1 | tail -1
source $W/install/setup.bash
if [ -z "$DISPLAY" ]; then  # the camera renders through X
  XD=:$((50 + RANDOM % 40)); Xvfb $XD -screen 0 1280x1024x24 >/dev/null 2>&1 & XVFB=$!; export DISPLAY=$XD; sleep 2
fi

SIM=""; BR=""; RECB=""; RECF=""
TL=$OUT/run_tasks_timed.log
stamp() {  # pass lines through, and keep a wall-clock-stamped copy (the video captions are timed by it)
  python3 -u -c "import sys,time
f = open('$TL', 'a')
for l in sys.stdin:
    f.write(f'{time.time():.3f} {l}'); f.flush(); print(l, end='')"
}
start_sim() {  # $1 = world file
  setsid ros2 launch slatphys_gazebo sim.launch.py world:=$1 > $OUT/gazebo_$(basename $1 .sdf).log 2>&1 &
  SIM=$!
  echo -n "  waiting for the controllers "
  for i in $(seq 120); do
    C=$(ros2 control list_controllers 2>/dev/null || true)
    echo "$C" | grep -q "scaled_joint_trajectory_controller.*active" && echo "$C" | grep -q "gripper_effort_controller.*active" && break
    echo -n .; sleep 2
  done; echo
  ros2 run ros_gz_bridge parameter_bridge /scene_cam/image@sensor_msgs/msg/Image[gz.msgs.Image \
      /scene_cam/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo > $OUT/bridge.log 2>&1 & BR=$!
  if [ -n "$2" ]; then  # recording: bridge the recording cameras and save their frames
    ros2 run ros_gz_bridge parameter_bridge /rec/front/image@sensor_msgs/msg/Image[gz.msgs.Image \
        /rec/overview/image@sensor_msgs/msg/Image[gz.msgs.Image > $OUT/rec_bridge.log 2>&1 & RECB=$!
    python3 $R/scripts/record_frames.py --out $OUT/frames --cams front overview > $OUT/rec_frames.log 2>&1 & RECF=$!
  fi
  sleep 3
}
stop_sim() {  # every kill may find the process already gone: never let that stop the script (set -e)
  if [ -n "$BR" ]; then kill $BR 2>/dev/null || true; BR=""; fi
  if [ -n "$RECF" ]; then kill $RECF $RECB 2>/dev/null || true; RECF=""; RECB=""; fi
  [ -n "$SIM" ] || return 0
  kill -INT -- -$SIM 2>/dev/null || true
  for i in $(seq 20); do kill -0 $SIM 2>/dev/null || break; sleep 1; done
  kill -KILL -- -$SIM 2>/dev/null || true
  SIM=""; sleep 2
}
cleanup() { stop_sim; [ -n "$XVFB" ] && kill $XVFB 2>/dev/null; ros2 daemon stop >/dev/null 2>&1; true; }
trap cleanup EXIT

echo "=== 1. capture the empty table"
start_sim $WORLD_DIR/world_empty.sdf
ros2 run slatphys_real capture --out $OUT/captures/_empty --execute MOVE_REAL_UR3E 2>&1 | grep -v WARN
stop_sim

SCENE_WORLD=$WORLD_DIR/world.sdf
if [ -n "$RECORD" ]; then
  python3 $R/scripts/record_world.py --world $SCENE_WORLD --out $OUT/world_rec.sdf --frames $OUT/frames \
      --cams front overview --size 640 360
  SCENE_WORLD=$OUT/world_rec.sdf
fi
echo "=== 2. capture the scene"
start_sim $SCENE_WORLD $RECORD
echo "=== 2. capture the scene" | stamp >/dev/null
ros2 run slatphys_real capture --out $OUT/captures/scene --execute MOVE_REAL_UR3E 2>&1 | grep --line-buffered -v WARN | stamp

echo "=== 3. locate the objects" | stamp
python3 - $WORLD_DIR/camera_pose.json $OUT/captures/scene/camera_info.json $OUT/calib/camera.json <<'EOF'
import json, sys
pose, info = json.load(open(sys.argv[1])), json.load(open(sys.argv[2]))
json.dump({**pose, "K": info["K"], "D": info["D"] or [0.0] * 5, "resolution": info.get("resolution")}, open(sys.argv[3], "w"), indent=2)
EOF
ros2 run slatphys_real locate --plan $WORLD_DIR/plan.json --capture $OUT/captures/scene --empty $OUT/captures/_empty \
    --calib $OUT/calib/camera.json --out $OUT/grasps.json 2>&1 | grep --line-buffered -v WARN | stamp
python3 - $OUT/grasps.json $WORLD_DIR/grasps.json $WORLD_DIR/plan.json <<'EOF' | tee $OUT/locate_vs_truth.txt
import json, sys, numpy as np
got, true, plan = (json.load(open(a)) for a in sys.argv[1:4])
print("located vs the scene's true grasps:")
for g, t, o in zip(got["objects"], true["objects"], plan["objects"]):
    a, b = g["grasp"], t["grasp"]
    period = np.pi / 2 if abs(o["size_mm"][0] - o["size_mm"][1]) < 2 else np.pi  # a square repeats every 90 deg
    dyaw = abs((a["yaw"] - b["yaw"] + period / 2) % period - period / 2)  # the closing axis has no sign
    print(f"  {g['name']:12s} centre error {np.linalg.norm(np.subtract(a['centre_xy'], b['centre_xy'])) * 1000:5.1f} mm   "
          f"axis error {np.degrees(dyaw):4.1f} deg   width {a['width'] * 1000:5.1f} mm (true {b['width'] * 1000:5.1f})")
EOF

echo "=== 4. run_tasks on the located grasps" | stamp
ros2 run slatphys_real run_tasks --grasps $OUT/grasps.json --plan $WORLD_DIR/plan.json --out $OUT \
    --execute MOVE_REAL_UR3E 2>&1 | grep --line-buffered -v WARN | stamp | tee $OUT/run_tasks.log | grep -vE "^\s+dry run"
