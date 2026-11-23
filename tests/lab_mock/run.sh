#!/bin/bash
# The lab stack end to end without a robot (ROS 2 Jazzy here; the lab runs Humble in Docker): ENME480 bridge (pinned commit) + ur3e_plan_runner + slatphys_real are built
# together; ur_robot_driver runs with mock hardware; the probe drives the arm through PlanSender ->
# /ur3e/command -> bridge with a mock gripper. --mock_driver skips only the robot-mode / safety-mode /
# speed-slider checks, which mock hardware does not publish.
T=$(cd "$(dirname "$0")" && pwd); R=$(cd "$T/../.." && pwd)
# LAB_SRC must hold the lab's repos:
#   git clone https://github.com/MarylandRoboticsCenter/ENME480_mrc   (tested at 97a36d1)
#   git clone https://github.com/alexhjkim/ur3e_experiments           (tested at 70bf821)
EXT=${LAB_SRC:?set LAB_SRC to the directory with ENME480_mrc and ur3e_experiments}
W=$T/_work; rm -rf $W; mkdir -p $W/src
ln -s $EXT/ENME480_mrc/src/ur3e_mrc_msgs $W/src/ur3e_mrc_msgs
ln -s $EXT/ENME480_mrc/src/ur3e_mrc $W/src/ur3e_mrc
ln -s $EXT/ur3e_experiments $W/src/UR3e_Real
ln -s $R/src/slatphys_real $W/src/slatphys_real
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=$((RANDOM % 100 + 100)) ROS_LOCALHOST_ONLY=1 SLATPHYS_REAL_CONFIG=$T/repo/configs/real.yaml PYTHONNOUSERSITE=1
cd $W && colcon build --symlink-install --event-handlers console_direct- 2>&1 | grep -E "Finished|Failed|error|Summary" | head -20
source $W/install/setup.bash
ros2 launch slatphys_real bringup.launch.py robot_ip:=0.0.0.0 fake_hardware:=true camera:=false > $W/bringup.log 2>&1 &
BR=$!
for i in $(seq 60); do ros2 node list 2>/dev/null | grep -q ur3e_mrc_enme480_control && grep -q "Vacuum generator\|IO Start/Stop" $W/bringup.log && break; sleep 2; done
grep -E "enme480|Connected|Vacuum|IO|ERROR" $W/bringup.log | head -8
echo "=== audit (no --execute) must refuse"
ros2 run slatphys_real probe --grasps $T/grasps.json --out $W/out_refuse 2>&1 | tail -1
echo "=== probe through PlanSender + ENME480 bridge"
timeout 1500 ros2 run slatphys_real probe --mock_gripper --mock_driver --execute MOVE_REAL_UR3E \
    --grasps $T/grasps.json --out $W/out 2>&1 | grep -v "point .*source row" | tail -40
echo "=== bridge goals sent: $(grep -c 'Goal was reached' $W/bringup.log)"
kill $BR; sleep 2; pkill -f ros2_control_node; pkill -f ur3e_mrc; pkill -f robot_state_publisher
