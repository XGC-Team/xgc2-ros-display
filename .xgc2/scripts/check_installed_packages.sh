#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
prefix=/opt/ros/noetic
package=ros-noetic-xgc2-ros-visualizer
expected="$(awk '/^version:/ {print $2; exit}' "$repo_root/.xgc2/product.yml")~focal"
test "$(dpkg-query --admindir=/var/lib/dpkg -W -f='${db:Status-Status}' "$package")" = installed
test "$(dpkg-query --admindir=/var/lib/dpkg -W -f='${Version}' "$package")" = "$expected"
dependencies="$(dpkg-query --admindir=/var/lib/dpkg -W -f='${Depends}' "$package")"
for dependency in ros-noetic-roscpp ros-noetic-sensor-msgs ros-noetic-nav-msgs ros-noetic-geometry-msgs libjsoncpp1; do
  [[ "$dependencies" =~ (^|,)[[:space:]]*$dependency([[:space:]]|,|$) ]]
done
test "$(rospack find xgc2_ros_display_relays)" = "$prefix/share/xgc2_ros_display_relays"
for path in lib/libxgc2_ros_display_relays.so lib/pkgconfig/xgc2_ros_display_relays.pc include/xgc2_ros_display_relays/display_relays.hpp include/xgc2_ros_display_relays/declared_wire.hpp include/xgc2_ros_display_relays/rate_gate.hpp share/xgc2_ros_display_relays/cmake/xgc2_ros_display_relaysConfig.cmake; do
  test -s "$prefix/$path"
done
binary="$prefix/lib/xgc2_ros_display_relays/xgc2_display_relays"
test -x "$binary"
test -x "$prefix/lib/xgc2_ros_visualizer/xgc2_ros_visualizer_node"
test -s "$prefix/lib/libxgc2_ros_visualizer_contract.so"
ldd "$binary" | tee /tmp/xgc2-display-node-ldd.txt
! grep -q 'not found' /tmp/xgc2-display-node-ldd.txt
ldd "$prefix/lib/libxgc2_ros_display_relays.so" | tee /tmp/xgc2-display-library-ldd.txt
! grep -q 'not found' /tmp/xgc2-display-library-ldd.txt
python3 "$repo_root/test/node_arguments_test.py" "$binary"
consumer="$(mktemp -d)"
trap 'rm -rf "$consumer"' EXIT
cat > "$consumer/CMakeLists.txt" <<'EOF'
cmake_minimum_required(VERSION 3.10)
project(display_installed_consumer LANGUAGES CXX)
find_package(catkin REQUIRED COMPONENTS xgc2_ros_display_relays)
add_executable(consumer main.cpp)
target_compile_features(consumer PRIVATE cxx_std_14)
target_include_directories(consumer PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(consumer PRIVATE ${catkin_LIBRARIES})
EOF
cat > "$consumer/main.cpp" <<'EOF'
#include <xgc2_ros_display_relays/display_relays.hpp>
int main() {
  xgc2_ros_display_relays::DisplayRelays owner(xgc2_ros_display_relays::parseRelaySpecs("[]"));
  owner.stop(); owner.rethrowFailure();
}
EOF
cmake -S "$consumer" -B "$consumer/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$consumer/build" -j1
"$consumer/build/consumer"
python3 "$repo_root/test/installed_node_probe.py" "$binary"
echo "Installed Deb SDK, argv, four-type payload and Stop checks passed"
