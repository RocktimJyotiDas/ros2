#!/usr/bin/env bash
# Open a shell in the simulation image with this repo at /ws and the host's X display (for GUI=1).
#   xhost +local:docker        # once per login, lets the container open windows
#   docker/run_sim.sh          # inside: scripts/sim_demo.sh, GUI=1 scripts/sim_demo.sh, tests/mock/run.sh
set -euo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
exec docker run --rm -it --name slatphys-sim \
  --network host --ipc host \
  -e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  $( [ -e /dev/dri ] && echo --device /dev/dri ) \
  -v "${repo}:/ws" -w /ws \
  slatphys-sim:jazzy bash
