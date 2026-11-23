#!/bin/bash
# Both validation tasks end to end in Gazebo: UR3e + Hand-E-sized gripper + wrist F/T
# (slatphys_gazebo) on a SLAT-Phys twin scene, driven by the robot code that will run in the lab
# (slatphys_real run_tasks -> probe), with configs/gazebo.yaml selecting the simulation backends.
#
# Needs ROS 2 Jazzy + Gazebo Harmonic + ur_simulation_gz (docker/Dockerfile.sim has all of it):
#   scripts/sim_demo.sh [world_dir] [extra run_tasks args...]
#     world_dir   a scripts/make_worlds.sh output (default: src/slatphys_gazebo/worlds/img4827_fit40)
#   GUI=1      show the Gazebo window (needs a display: docker/run_sim.sh forwards X11)
#   SMOKE=1    only launch and check the bench (controllers, F/T, FK == TF), no probing
#   SWEEP="10 20 30"   grip each object at these forces and print the finger opening vs the object's spring
#   RECORD=1   record two cameras (overview, close-up) to runs/<world>/frames and a time-stamped log;
#              then: python3 scripts/make_video.py runs/<world>
# Writes runs/<world>/: readings*.json, answers.json, run_tasks.log, gazebo.log.
# Success = runs/<world>/answers.json. (Under apptainer the container may still exit 1 with
# "Terminating squashfuse_ll after timeout" after everything finished: that is its unmount.)
set -e
R=$(cd "$(dirname "$0")/.." && pwd)
WORLD_DIR=${1:-$R/src/slatphys_gazebo/worlds/img4827_fit40}; shift || true
WORLD_DIR=$(cd "$WORLD_DIR" && pwd)
NAME=$(basename "$WORLD_DIR")
W=${BUILD_DIR:-/tmp/$USER/slatphys_ws}; mkdir -p $W
OUT=$R/runs/$NAME; mkdir -p $OUT; rm -f $OUT/run_tasks_timed.log
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-$((RANDOM % 100 + 100))} ROS_LOCALHOST_ONLY=1 PYTHONNOUSERSITE=1
export PYTHONUNBUFFERED=1   # log lines must arrive when they happen (RECORD captions are timed by them)
export GZ_PARTITION=slatphys_$$   # isolate this run's Gazebo topics from any other Gazebo on the machine
export SLATPHYS_REAL_CONFIG=$R/configs/gazebo.yaml
export GZ_SIM_RESOURCE_PATH=/opt/ros/jazzy/share
[ -z "$GUI" ] && export LIBGL_ALWAYS_SOFTWARE=1 QT_QPA_PLATFORM=offscreen
cd $R && colcon --log-base $W/log build --base-paths src --symlink-install --build-base $W/build \
    --install-base $W/install --packages-select slatphys_real slatphys_gazebo 2>&1 | tail -2
source $W/install/setup.bash

LAUNCH_WORLD=$WORLD_DIR/world.sdf
if [ -n "$RECORD" ]; then  # cameras render through X: a virtual display unless GUI already has one
  rm -rf $OUT/frames
  python3 $R/scripts/record_world.py --world $WORLD_DIR/world.sdf --out $OUT/world_rec.sdf --frames $OUT/frames
  LAUNCH_WORLD=$OUT/world_rec.sdf
  if [ -z "$GUI" ]; then
    XD=:$((50 + RANDOM % 40)); Xvfb $XD -screen 0 1280x1024x24 >/dev/null 2>&1 & XVFB=$!
    export DISPLAY=$XD; unset QT_QPA_PLATFORM; sleep 2
  fi
fi
setsid ros2 launch slatphys_gazebo sim.launch.py world:=$LAUNCH_WORLD gui:=$([ -n "$GUI" ] && echo true || echo false) \
    > $OUT/gazebo.log 2>&1 &
SIM=$!
cleanup() {  # SIGINT lets ros2 launch stop its children; then make sure the whole group is gone
  kill -INT -- -$SIM 2>/dev/null; for i in $(seq 20); do kill -0 $SIM 2>/dev/null || break; sleep 1; done
  kill -KILL -- -$SIM 2>/dev/null; pkill -KILL -f "$LAUNCH_WORLD" 2>/dev/null
  [ -n "$RECF" ] && kill $RECF $RECB 2>/dev/null
  [ -n "$XVFB" ] && kill $XVFB 2>/dev/null
  ros2 daemon stop >/dev/null 2>&1; true; }
trap cleanup EXIT
echo -n "waiting for the controllers "
for i in $(seq 120); do
  C=$(ros2 control list_controllers 2>/dev/null || true)
  echo "$C" | grep -q "scaled_joint_trajectory_controller.*active" && echo "$C" | grep -q "gripper_effort_controller.*active" && break
  echo -n .; sleep 2
done; echo
ros2 control list_controllers
if [ -n "$RECORD" ]; then  # bridge the cameras to ROS and save them (Gazebo renders only subscribed cameras)
  ros2 run ros_gz_bridge parameter_bridge /rec/overview/image@sensor_msgs/msg/Image[gz.msgs.Image \
      /rec/closeup/image@sensor_msgs/msg/Image[gz.msgs.Image > $OUT/rec_bridge.log 2>&1 & RECB=$!
  python3 $R/scripts/record_frames.py --out $OUT/frames > $OUT/rec_frames.log 2>&1 & RECF=$!
  sleep 5
fi
echo "--- wrist F/T (one message: the gripper's weight, ~12 N)"
timeout 20 ros2 topic echo --once /force_torque_sensor_broadcaster/wrench | sed -n '/force/,/torque/p' | head -5
echo "--- base -> tool0: Gazebo's TF vs slatphys_real.ur_kinematics FK at the same joints"
python3 - <<'EOF'
import sys, time, numpy as np, rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from slatphys_real import ur_kinematics as K
rclpy.init(); n = Node("fk_check"); buf = Buffer(); TransformListener(buf, n); q = {}
n.create_subscription(JointState, "/joint_states", lambda m: q.update(zip(m.name, m.position)), 10)
t0 = time.time()
while time.time() - t0 < 15:
    rclpy.spin_once(n, timeout_sec=0.1)
    if q and buf.can_transform("base", "tool0", rclpy.time.Time()):
        break
tf = buf.lookup_transform("base", "tool0", rclpy.time.Time()).transform.translation
fk = K.fk(np.array([q[j] for j in K.JOINT_NAMES]))[:3, 3]
err = np.linalg.norm(fk - [tf.x, tf.y, tf.z])
print(f"TF {np.round([tf.x, tf.y, tf.z], 4)}  FK {np.round(fk, 4)}  |diff| {err * 1000:.2f} mm")
sys.exit(0 if err < 0.002 else 1)
EOF
[ -n "$SMOKE" ] && exit 0
if [ -n "$SWEEP" ]; then
  python3 $R/tests/gazebo/force_sweep.py --grasps $WORLD_DIR/grasps.json --world slatphys --forces $SWEEP 2>&1 | grep -v WARN
  exit 0
fi

echo "=== run_tasks: both tasks, one probing run"
ros2 run slatphys_real run_tasks --grasps $WORLD_DIR/grasps.json --plan $WORLD_DIR/plan.json --out $OUT \
    --execute MOVE_REAL_UR3E "$@" 2>&1 \
  | python3 -u -c "import sys,time
for l in sys.stdin: print(f'{time.time():.3f} {l}', end='', file=open('$OUT/run_tasks_timed.log','a')); print(l, end='')" \
  | tee $OUT/run_tasks.log | grep -vE "^\s+dry run"
