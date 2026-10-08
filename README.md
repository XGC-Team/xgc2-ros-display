# ROS1 visualization

This ROS1 C++ product is `xgc2-ros-visualizer`. It owns subscriber-gated display
topic copies and the publisher that turns interface data into one visualization
interface. The catkin package stays `xgc2_ros_display_relays` so the relay
install paths stay. The publisher node installs as
`lib/xgc2_ros_visualizer/xgc2_ros_visualizer_node`. Source repository:
[XGC-Team/xgc2-ros-visualizer](https://github.com/XGC-Team/xgc2-ros-visualizer).
Gazebo shadow rendering is not in this package. The relay library does no
algorithm conversion. Core consumer migration is separate work; this repository
does not retain an alternate Python relay.

Authority: `xgc2/process-catalog/current/platform/lichtblick-display-relays.json`.
Reviewed authority SHA-256: `0f955708788b410fc3f3f0cd97df83ac07ff4daf32e434dff84abe31ddf69e16`.
`parseRelaySpecs` accepts its exact four fields and four declared message types:
PointCloud2, OccupancyGrid, Path and PoseArray. Source names must be absolute,
unique and outside `/xgc/display`; outputs are exactly `/xgc/display` + source.
Maximum 64 relays; each copy budget is finite 0.1–100 Hz. Both ROS queues are 1;
no latching, periodic republishing, retained samples or field conversion.
Embedding ROS remaps which alter a configured source or copy are rejected.

DeclaredWire<T> copies the upstream serialized payload once on receipt and
publishes that same immutable ROS shared_ptr. Its type/MD5/definition come from
the declared generated ROS type, so advertising never needs an upstream sample.
HasHeader is false to stop roscpp from rewriting header.seq; source seq, stamp,
frame, float bits and payload bytes remain unchanged. Source topics and their
recorders have no display budget applied. Steady time controls only the copy.

The library creates its own callback queue and one callback thread; it never
initializes or globally shuts down ROS. Publisher status and receipt callbacks
run serially. Connect subscribes only when actual display peers exist; the last
disconnect releases the upstream subscription. Subscription generations fence
old callbacks across reconnect. Stop is idempotent: prohibit publication, stop
the callback queue, join its worker, release subscriptions and advertisements.
After stop returns no callback can publish. Already handed-off ROS transport
bytes may arrive later at a display consumer. Embedding owners must call stop
from outside this private callback thread and keep their ROS context alive until
it returns. Callback failures are reported by rethrowFailure for owner teardown.

`xgc2_display_relays RELAYS_JSON` is the thin process entry. Missing or extra
arguments fail before ROS initialization. ROS environment variables continue to
select the master/IP. Empty lists wait for
SIGTERM/SIGINT without initializing ROS or contacting a master. Nonempty owners
stop and shut down their own process ROS context on those signals. No worker
child exists to orphan. Process supervision retains its existing grace timeout.

Build and install with a sourced ROS Noetic environment:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/path/to/prefix
cmake --build build -j1
cmake --build build --target tests -j1
cmake --build build --target run_tests -j1
catkin_test_results build/test_results
cmake --install build
```

The installed library is `lib/libxgc2_ros_display_relays.so`; public headers are
under `include/xgc2_ros_display_relays`; the node is
`lib/xgc2_ros_display_relays/xgc2_display_relays`. Source the installed
`setup.bash` when using a nonstandard prefix. Embedding consumers use
`find_package(catkin REQUIRED COMPONENTS xgc2_ros_display_relays)` and link the
exported catkin libraries. The embedding owner must keep its NodeHandle alive.

On 2026-10-03, Release build, install and tests passed on amd64 with GCC 9.4,
catkin 0.8.12 and jsoncpp 1.7.4 in the existing Noetic CI image pinned to
`sha256:2d0ab240a669e59dc6e86e41806041a7f5d46751c787b5cbb225b10e1faed39b`.
Validation ran as UID/GID 1000 with one CPU, no external network, no exposed
ports and no formal runtime ownership labels. Three contract tests and two ROS
lifecycle tests passed (catkin's combined wrapper/result count: 11, zero
failures). They cover schema rejection, wire/type/MD5 identity, steady-time
budget/no catchup/reconnect reset, and all four types' laziness, disconnect,
reconnect, original subscriber survival and concurrent Stop cleanup.

A separate consumer compiled, linked and ran using only the installed catkin
export. The actual installed standalone node was also exercised across process
boundaries against a private ROS master: all four types advertised before any
source sample; no viewer meant zero source connections; each original recorder
received 80 serialized packets while its display copy received 5. Every copy's
entire serialized payload matched an original packet. The last viewer disconnect
released the relay connection, reconnect restored it, and SIGTERM returned zero
with no relay connection left while original recorders still received data.
An empty-list installed process opened no network socket and stopped cleanly.
These are correctness and lifecycle checks; no performance improvement or
formal viewer/Experiment/APT acceptance is claimed. Arm64 remains unverified.

ROS API reference: [AdvertiseOptions](https://github.com/ros/ros_comm/blob/noetic-devel/clients/roscpp/include/ros/advertise_options.h)
documents serialized header sequence rewriting; [serialization](https://github.com/ros/roscpp_core/blob/noetic-devel/roscpp_serialization/include/ros/serialization.h)
provides the bounded streams used by the raw payload serializer.

The Noetic/Focal package is `ros-noetic-xgc2-ros-visualizer`.
One native package contains the relay library and node, the
visualization publisher, and their headers. Native system-library requirements
are derived with dpkg-shlibdeps. The package depends on
`ros-noetic-xgc2-robot-visualization`.

Push and PR CI on main use native amd64/arm64 GitHub-hosted runners and the
existing controlled Noetic 1.0.0 image, locked by multiarch digest above (amd64
`b3e2b84d857d69c98aa37f4509890cd3d4c23c084c7b28fe7de84a8532b8b667`, arm64
`2d28bb572abb63825dddbffa2b1ad9198e4b0b472aedd021783f906e99c72d78`). Source
binds are read-only; writable build/output binds use the calling UID/GID. Build,
source tests, new argv rejection controls and installation run within private
containers. Build and installation obtain the robot visualization dependency
from production APT. Installed-Deb gates check the native DSO
and executable, compile/link an independent SDK consumer, and exercise the
actual node across processes with all four serialized payloads and Stop. CI
retains only Debs and strict `xgc2.build-artifact.v1` manifests for 14 days.

```sh
.xgc2/scripts/build_debs_in_docker.sh --work-dir /tmp/ros-visualizer-build --output-dir "$PWD/debs"
```

`release.yml` accepts only the existing central prepare/compatibility contract;
it has no production publishing credentials or index writer. Ordinary releases
reuse exact-source push CI artifacts through the central xgc2-devops release
orchestrator. CI/package preparation does not assert production APT visibility,
Core consumer migration, or formal viewer/Experiment acceptance.
