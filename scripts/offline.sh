#!/bin/bash
# Offline real2sim steps for one or all real scenes, on a GPU machine with SLAT-Phys-Scene
# (the SAM-3D + SLAT-Phys reconstruction and the Isaac Sim probe; a separate checkout):
#   export SLATPHYS_SCENE=/path/to/SLAT-Phys-Scene      # required
#   export SLATPHYS_ENV_SH=...                          # optional: activates the slatphys python env
#                                                       # (default: $SLATPHYS_SCENE/setup/sam3d_env.sh)
#
#   scripts/offline.sh recon        # SAM-3D + SLAT-Phys reconstruction of data/real/scenes/*
#   scripts/offline.sh materialize  # -> data/real/sim/slatphys/<scene>/{sim.json, grasps.json}
#   scripts/offline.sh simprobe     # Isaac Sim probing with the real protocol (cluster runner)
#   scripts/offline.sh correlate    # -> results/
# Optional 2nd argument: one scene directory to reconstruct (default: all of data/real/scenes).
set -e
REAL=$(cd "$(dirname "$0")/.." && pwd)
SCENE_REPO=${SLATPHYS_SCENE:?set SLATPHYS_SCENE to your SLAT-Phys-Scene checkout}
export SLATPHYS_SCENE=$SCENE_REPO
SLATPHYS_ENV_SH=${SLATPHYS_ENV_SH:-$SCENE_REPO/setup/sam3d_env.sh}
export PYTHONNOUSERSITE=1
source "$SLATPHYS_ENV_SH"
export PYTHONPATH=$SCENE_REPO:$PYTHONPATH ATTN_BACKEND=${ATTN_BACKEND:-xformers}
cd "$REAL"
case "$1" in
  recon)
    SC=${2:-$REAL/data/real/scenes}
    if [ -f "$SC/scene.json" ]; then  # a single scene: reconstruct it alone
      TMPS=$(mktemp -d); ln -s "$(cd "$SC" && pwd)" "$TMPS/$(basename "$SC")"; SC=$TMPS
    fi
    (cd "$SCENE_REPO" && python -m slatscene.recon_task_scenes --scenes "$SC" --out "$REAL/data/real/recon") ;;
  materialize) python real2sim/materialize_real.py --scenes data/real/scenes --recon data/real/recon --out data/real/sim ;;
  simprobe)    python real2sim/run_sim_probes.py --task_sim data/real/sim ;;
  correlate)   python real2sim/correlate.py --scenes data/real/scenes --sim data/real/sim/slatphys --real data/real/robot --out results ;;
  *) echo "usage: $0 {recon [scene_dir]|materialize|simprobe|correlate}"; exit 1 ;;
esac
