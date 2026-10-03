#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
image=ghcr.io/xgc-team/xgc2-images/xgc2-build-focal-full-noetic:1.0.0@sha256:2d0ab240a669e59dc6e86e41806041a7f5d46751c787b5cbb225b10e1faed39b
work_dir=""
output_dir=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --image) image="$2"; shift 2 ;;
    --work-dir) work_dir="$2"; shift 2 ;;
    --output-dir) output_dir="$2"; shift 2 ;;
    *) echo "unknown argument: $1" >&2; exit 1 ;;
  esac
done
test -n "$work_dir" && test -n "$output_dir"
install -d "$work_dir" "$output_dir"
work_dir="$(cd "$work_dir" && pwd)"
output_dir="$(cd "$output_dir" && pwd)"
# Read-only source; every writable host bind remains owned by the caller.
docker run --rm --network none --cpus 1 --user "$(id -u):$(id -g)"   -e HOME=/tmp -e ROS_HOME=/tmp/ros-home -e ROS_LOG_DIR=/tmp/ros-log   -e XGC2_RUN_SOURCE_TESTS="${XGC2_RUN_SOURCE_TESTS:-1}"   -v "$repo_root:/source:ro" -v "$work_dir:/work" -v "$output_dir:/out" "$image" bash -c '
    set -eo pipefail
    source /opt/ros/noetic/setup.bash
    set -u
    cmake -S /source -B /work/build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic -DCATKIN_ENABLE_TESTING=ON
    cmake --build /work/build -j1
    if [[ "$XGC2_RUN_SOURCE_TESTS" == 1 ]]; then
      cmake --build /work/build --target tests -j1
      cmake --build /work/build --target run_tests -j1
      catkin_test_results /work/build/test_results
    fi
    python3 /source/test/node_arguments_test.py /work/build/devel/lib/xgc2_ros_display_relays/xgc2_display_relays
    DESTDIR=/work/install cmake --install /work/build
    /source/.xgc2/scripts/package_debs.sh --install-root /work/install --output-dir /out
  '
# Installation mutates only the ephemeral container, never a host prefix.
docker run --rm --network none --cpus 1   -e HOME=/tmp -e ROS_HOME=/tmp/ros-home -e ROS_LOG_DIR=/tmp/ros-log   -v "$repo_root:/source:ro" -v "$output_dir:/out:ro" "$image" bash -c '
    set -eo pipefail
    dpkg -i /out/*.deb
    source /opt/ros/noetic/setup.bash
    set -u
    /source/.xgc2/scripts/check_installed_packages.sh
  '
