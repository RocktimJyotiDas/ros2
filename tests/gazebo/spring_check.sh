#!/bin/bash
# Does Gazebo (DART) honour SDF <spring_stiffness>? A 1 kg plate on a vertical prismatic spring,
# k = 1000 and 3000 N/m: the static sag must be m g / k = 9.81 mm and 3.27 mm.
# Run inside ur3e-jazzy.sif.
source /opt/ros/jazzy/setup.bash
D=$(cd "$(dirname "$0")" && pwd)
(timeout 40 gz sim -s -r $D/spring_check.sdf > /tmp/spring_check_gz.log 2>&1 &)
sleep 12
for m in s0 s1; do echo -n "$m: "; timeout 8 gz topic -e -n 1 -t /world/w/model/$m/joint_state | grep -A12 'name: "spring"' | grep -A1 -m1 'position {' | tail -1; done
echo "expected: s0 position -0.00981, s1 -0.00327"
true
