#!/usr/bin/env bash
# Like ur3e_experiments/run_docker.sh, with this repo mounted next to the lab's:
#   /workspace/src/UR3e_Real       <- ur3e_experiments (ur3e_plan_runner)
#   /workspace/src/slatphys  <- this repo (slatphys_real)
#   UR_IP=192.168.77.22 UR3E_EXPERIMENTS=~/ur3e_experiments docker/run_robot.sh
set -euo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
lab=${UR3E_EXPERIMENTS:?Set UR3E_EXPERIMENTS to your ur3e_experiments checkout}
robot_ip=${UR_IP:?Set UR_IP to the physical robot address}
exec docker run --rm -it \
  --name ur3e-slatphys \
  --network host --ipc host \
  --cap-add SYS_NICE --ulimit rtprio=99 --ulimit rttime=-1 \
  --device /dev/bus/usb --privileged \
  -e UR_IP="${robot_ip}" -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" -e PYTHONDONTWRITEBYTECODE=1 \
  -e SLATPHYS_REAL_CONFIG=/workspace/src/slatphys/configs/real.yaml \
  -v "${lab}:/workspace/src/UR3e_Real" \
  -v "${repo}:/workspace/src/slatphys" \
  -w /workspace \
  ur3e-slatphys:humble bash
