#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
prefix=/opt/ros/noetic
package=ros-noetic-xgc2-ros-visualizer
expected="$(awk '/^version:/ {print $2; exit}' "$repo_root/.xgc2/product.yml")~focal"
test "$(dpkg-query -W -f='${db:Status-Status}' "$package")" = installed
test "$(dpkg-query -W -f='${Version}' "$package")" = "$expected"
test "$(rospack find xgc2_ros_visualizer)" = "$prefix/share/xgc2_ros_visualizer"
for path in lib/libxgc2_ros_visualizer_runtime.so lib/libxgc2_ros_visualizer_contract.so lib/pkgconfig/xgc2_ros_visualizer.pc include/xgc2_ros_visualizer/config.hpp include/xgc2_ros_visualizer/declared_wire.hpp include/xgc2_ros_visualizer/scene_contract.hpp share/xgc2_ros_visualizer/cmake/xgc2_ros_visualizerConfig.cmake; do
  test -s "$prefix/$path"
done
binary="$prefix/lib/xgc2_ros_visualizer/xgc2_ros_visualizer_node"
test -x "$binary"
for path in "$binary" "$prefix/lib/libxgc2_ros_visualizer_runtime.so" "$prefix/lib/libxgc2_ros_visualizer_contract.so"; do
  linkage="$(ldd "$path")"
  [[ "$linkage" != *'not found'* ]]
done
consumer="$(mktemp -d)"
trap 'rm -rf "$consumer"' EXIT
cat > "$consumer/CMakeLists.txt" <<'CMAKE'
cmake_minimum_required(VERSION 3.10)
project(visualizer_installed_consumer LANGUAGES CXX)
find_package(catkin REQUIRED COMPONENTS xgc2_ros_visualizer)
add_executable(consumer main.cpp)
target_compile_features(consumer PRIVATE cxx_std_14)
target_include_directories(consumer PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(consumer PRIVATE ${catkin_LIBRARIES})
CMAKE
cat > "$consumer/main.cpp" <<'CPP'
#include <xgc2_ros_visualizer/config.hpp>
int main() {
  const auto rates=xgc2_ros_visualizer::defaultRates();
  return xgc2_ros_visualizer::parseRates(rates.json(),true).maximum()>0?0:1;
}
CPP
cmake -S "$consumer" -B "$consumer/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$consumer/build" -j1
"$consumer/build/consumer"
python3 "$repo_root/test/installed_node_probe.py" "$binary"
python3 "$repo_root/test/check_description_resources.py"
echo 'Installed single-entry SDK, RPC, persistent relay, URDF and owned cleanup checks passed'
