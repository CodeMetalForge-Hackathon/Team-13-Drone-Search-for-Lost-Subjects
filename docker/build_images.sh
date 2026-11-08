#!/usr/bin/env bash
# Build the cf_vio Kalibr and OpenVINS images. Usage: ./build_images.sh [kalibr|openvins|all] (JOBS=4 by default)
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
src="${VIO_TOOLS:-$HOME/vio_tools}"
jobs="${JOBS:-4}"
what="${1:-all}"
[ -d "$src/kalibr" ]    || git clone https://github.com/ethz-asl/kalibr.git "$src/kalibr"
[ -d "$src/open_vins" ] || git clone https://github.com/rpng/open_vins.git "$src/open_vins"
if [[ $what == all || $what == kalibr ]]; then
  docker build -t cf_vio/kalibr -f "$here/Dockerfile.kalibr" --build-context src="$src/kalibr" \
    --build-arg JOBS="$jobs" --label kalibr.commit="$(git -C "$src/kalibr" rev-parse --short HEAD)" "$here"
fi
if [[ $what == all || $what == openvins ]]; then
  docker build -t cf_vio/openvins -f "$here/Dockerfile.openvins" --build-context src="$src/open_vins" \
    --build-arg JOBS="$jobs" --label open_vins.commit="$(git -C "$src/open_vins" rev-parse --short HEAD)" "$here"
fi
