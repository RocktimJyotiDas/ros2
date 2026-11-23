#!/bin/bash
# run_tasks (both validation tasks, one probing run) with no robot and no simulator: a dry run
# (every TCP target printed), a mock run, the arming-phrase refusal, and re-scoring saved readings.
# Needs ROS 2 (Jazzy; docker/Dockerfile.sim):   bash tests/mock/run.sh
set -e
T=$(cd "$(dirname "$0")" && pwd); R=$(cd "$T/../.." && pwd)
W=/tmp/$USER/slatphys_run_tasks; rm -rf $W; mkdir -p $W
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=$((RANDOM % 100 + 100)) ROS_LOCALHOST_ONLY=1 SLATPHYS_REAL_CONFIG=$T/repo/configs/real.yaml PYTHONNOUSERSITE=1
cd $R && colcon --log-base $W/log build --base-paths src --packages-select slatphys_real --symlink-install --build-base $W/build --install-base $W/install 2>&1 | tail -2
source $W/install/setup.bash
echo "=== dry run (prints every TCP target)"
ros2 run slatphys_real run_tasks --grasps $T/grasps.json --out $W/dry --dry_run 2>&1 | grep -E "preflight|dry run|squeeze -|weigh the|heavy|deformable" | head -12
echo "=== mock arm + gripper, then both answers"
ros2 run slatphys_real run_tasks --grasps $T/grasps.json --out $W/mock --mock 2>&1 | tail -8
echo "=== refused: real motion without the arming phrase"
ros2 run slatphys_real run_tasks --grasps $T/grasps.json --out $W/x 2>&1 | tail -1 || true
echo "=== answer from saved readings"
ros2 run slatphys_real answer --readings $W/mock/readings.json 2>&1 | tail -2
