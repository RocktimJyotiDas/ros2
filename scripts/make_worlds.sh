#!/bin/bash
# Gazebo worlds for the img4827 scene, from the SLAT-Phys twin in data/twin/img4827 (python3 + numpy only).
#   img4827_asis             the twin at the photo's assumed scale (disc 82 mm: preflight must refuse it)
#   img4827_fit40            the twin scaled so the widest grasp is 40 mm (fits the Hand-E)
#   img4827_fit40_rigiddisc  control: same, with the disc rigid (an unambiguous deformable answer)
set -e
R=$(cd "$(dirname "$0")/.." && pwd)
T=$R/data/twin/img4827
MW="python3 $R/src/slatphys_gazebo/scripts/make_world.py --sim $T/sim.json --grasps $T/grasps.json --scene_id img4827"
WD=$R/src/slatphys_gazebo/worlds
$MW --out $WD/img4827_asis
$MW --out $WD/img4827_fit40 --fit_width 0.040
$MW --out $WD/img4827_fit40_rigiddisc --fit_width 0.040 --rigid green_disc
