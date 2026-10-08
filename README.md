# ROS1 visualizer

`xgc2-ros-visualizer` provides one C++ ROS server, one executable
`xgc2_ros_visualizer_node`, and the canonical catkin package
`xgc2_ros_visualizer`. A server owns a fixed input pool (default 2, maximum 32
workers), one publication scheduler, and a bounded Unix HTTP control loop.
Robots, descriptions and relays are instance data. Adding robots does not create
application threads, helper processes or `robot_state_publisher` children.
Core uses its ordinary supervised process definition and a frozen startup file;
normal Stop terminates that Run's server and removes its bootstrap file.

The separate `xgc2_robot_visualization` dependency supplies FS150, Scout and
Mecanum geometry, wheel/rotor animation calculations, frame names and path
styles. This server owns ROS subscriptions, publication, scheduling, URDF
parameters and required fixed/joint TF. Gazebo shadow rendering, algorithm
planning, map production and Viewer rendering remain their owning products.

## Startup and RPC

Source a ROS Noetic environment and select its master/IP normally. Required
private parameters are `socket_path` (an absent absolute Unix socket path) and
`server_instance_id`. Optional parameters are `callback_workers` (1–32),
`rates_json` (an object overlaying defaults), `world_clock` (`wall` or
`simulation`, default `wall`) and `initial_instance_file`. The clock is immutable
and selected before ROS initialization; an existing `/use_sim_time` remap is
preserved. Do not separately pass `_use_sim_time`.

```sh
rosrun xgc2_ros_visualizer xgc2_ros_visualizer_node \
  _socket_path:=/tmp/visualizer.sock _server_instance_id:=my-run \
  _world_clock:=simulation _initial_instance_file:=/private/input.json
curl --unix-socket /tmp/visualizer.sock http://localhost/v1/status
```

The socket is mode 0600. It is never adopted or replaced if already present.
Control requests have fixed client/header/body limits and finite deadlines.

| Request | Behavior |
| --- | --- |
| `GET /health` | Server identity, readiness and fixed callback-worker count |
| `GET /v1/status` | Membership, resource counts, rates and publication counters |
| `GET /v1/instances/<id>` | Observe an existing instance; 404 never creates one |
| `PUT /v1/instances/<id>` | Validate and create complete membership; identical retry is a no-op, different content is 409 |
| `DELETE /v1/instances/<id>` | Idempotently remove only that instance; wait for its publication fence |
| `GET /v1/rates` | Return the current complete table in `rates` |
| `PUT /v1/rates` | Atomically replace the complete validated table; invalid candidates leave it unchanged |

A native instance request contains exactly `robots`, `descriptions`,
`worldBoundary`, `settings` and `displayRelays`. The first two are the existing
robot visualization/description roster arrays. Settings use the existing
snake-case field names; unknown fields and legacy individual publish-rate
settings are rejected. Empty rosters are valid. Configured scene instances,
including zero-robot worlds, publish `/xgc/robot_scene/ready`; this acknowledges
configuration, not fresh robot poses or scientific progress. Descriptions also
publish `/xgc/robot_descriptions/ready`.

The optional startup file contains exactly:

```json
{"instanceId":"existing-run-id","robots":[],"context":{"runMode":"simulation","worldClock":"simulation","worldBoundary":null},"settings":{},"displayRelays":[]}
```

`robots` is the complete frozen public Robot array, `context` the complete
session context, and `settings` the entry's full camel-case panel settings.
The server projects these values and creates the instance through the same
validated registry path as RPC. Namespace, scene model and installed description
metadata come from each Robot's visualization configuration. FS150 AR uses only
`localizationSources[context.runMode].poseTopic` and its frozen offset. Missing
selected sources fail startup; no VRPN/profile/topic guessing or second offset
application occurs. Numeric slot palette ordering and the original Scout mocap
scene-model binding are preserved. Unrelated panel fields remain Viewer-owned.
A startup file must be a nonempty regular file of at most 1 MiB; symlinks are
rejected.

Instances cannot share the existing global scene/frame outputs on the same ROS
graph. Independent relay-only instances can disable scene/transforms and use
disjoint display topics. Conflicts, duplicate models and duplicate path outputs
fail before membership becomes visible. DELETE never publishes SceneDeletion
ALL or purges all ROS description parameters. A replaced foreign parameter value
is preserved during cleanup.

## Data and publication rates

All subscriptions remain active until their instance is deleted or the server
stops, regardless of Viewer connections. ROS input queues are size 1. Callbacks
update fixed latest state and fixed source-time history under short locks;
serialization, publication, URDF parsing and network I/O occur outside those
locks. Membership and complete rate tables are immutable publication snapshots.
Rate changes and Stop wake the single scheduler immediately, including when all
rates are 0.1 Hz.

| Kind/channel | Default ceiling | Output |
| --- | ---: | --- |
| FS150/Scout/Mecanum `pose_tf` | 120 Hz | `/xgc/tf` body/label/camera and AR anchor transforms, new source stamps only |
| Each kind `joint_tf`, `markers` | 30 Hz | `/xgc/tf` visual rotor/wheel TF; optional `/markers` geometry/path/labels |
| Each kind `scene`, `scene_path`, `path`, `ar_path`, `ar_identity` | 10 Hz | `/xgc/scene` labels/optional paths, namespace Path topics, `/xgc/scene_ar` |
| FS150 `height_projection`, `height_projection_ar` | 10 Hz | `/xgc/uav_height_projection` and `/xgc/uav_height_projection_ar` |
| Global `tf_root` | 30 Hz | Standard `/tf` world root |
| Global `tf_static`, `world_boundary`, `readiness` | 1 Hz | Latched fixed aliases/URDF TF, world boundary layers and ready events |
| Global `joint_tf` | 30 Hz | In-process URDF joint TF for descriptions without a matching concrete scene kind |
| All kinds `display_pointcloud`, `display_grid`, `display_path`, `display_pose_array` | 10 Hz | Original serialized data under `/xgc/display` |

All applicable ceilings accept finite 0.1–1000 Hz numbers. Static, readiness and
label outputs are event-driven with ceilings, rather than periodic replay.
Description joint TF uses the matching robot kind's `joint_tf` ceiling when
available; global `joint_tf` does not override the three concrete kinds.
World boundary topics remain `/xgc/world_boundary`, `/xgc/world_boundary/ar`,
`/xgc/world_boundary_walls` and `/xgc/world_boundary_walls/ar`.

3D FS150 reads its fused `/<namespace>/mavros/local_position/pose`; ground robots
read canonical `/<namespace>/pose`. AR reads the exact frozen source with its
world offset once. Source stamps, finite/frame/freshness checks, the Scout body
height, ground-path z=0, raw AR quaternion and the off/ground/walls semantics are
preserved. Rotor state reads MAVROS state/extended-state; ground wheel animation
reads canonical twist, with cmd_vel as the existing fallback. The server never
writes those scientific inputs or controller parameters.

Nav Path, Marker path and Scene path use the same source-stamp sampled 6-second,
10-Hz, maximum-61-point ring. Lowering an output ceiling does not lower history
sampling, including with accelerated simulation time. History expires using ROS
time and respects rollback/reset semantics. URDF XML is loaded from installed
package-relative files. Parameters and fixed/movable joint transforms are
produced in-process using the original description prefix; no RSP is forked.

A relay request is `{source,topic,messageType,robotKind}`. Source names are unique
canonical absolute names outside `/xgc/display`; output is exactly
`/xgc/display` + source. Maximum 64 per instance. Only PointCloud2,
OccupancyGrid, Path and PoseArray full-state messages are accepted. The relay
receives serialized bytes once and publishes the same immutable pointer using
the original type/MD5/definition. It does not decode, rewrite Header sequence,
convert fields, replay previously published samples or budget the source itself.
Legacy bootstrap `maxRateHz` is replaced by the kind/channel table.

Membership, queues, latest-message retention and history point counts are
bounded. Variable ROS payload bytes and ROS serialization/transport allocations
remain message-dependent; this is not a claim of zero allocations or a universal
byte-memory limit.

## Build, package and validation

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/path/to/prefix
cmake --build build -j2
cmake --build build --target tests -j2
cmake --build build --target run_tests -j2
catkin_test_results build/test_results
cmake --install build
```

The native package is `ros-noetic-xgc2-ros-visualizer`, version 0.3.0-1. It installs
one executable, the runtime/contract libraries and public canonical headers.
Consumers use `find_package(catkin REQUIRED COMPONENTS xgc2_ros_visualizer)`.
The installed Robot SDK ABI dependency is at least 0.2.0-16.

`.xgc2/scripts/build_debs_in_docker.sh` uses the pinned Noetic build image and
installs published SDK dependencies only inside independent containers. It runs
source tests, produces the Deb, then installs that Deb in a second container and
checks the catkin exports, linkage and actual executable. This build requires
network access to production APT for dependencies; it never uses an active Core
container as its builder.

`test/installed_node_probe.py` creates a finite private ROS graph. Its gate covers
zero-robot startup readiness and clock remapping, no-peer persistent subscriptions,
four-type exact-byte relays, RPC atomic validation/retries/deletion, foreign
instance/parameter preservation, 20/100 robot resource counts, source-time paths
at accelerated clock/low publication rates, in-process URDF joint TF and bounded
low-rate SIGTERM. Transport tests separately exercise real Unix HTTP framing,
client limits, deadlines and foreign-inode socket cleanup. Private probes are
correctness/resource checks, not formal station, browser or scientific-mission
acceptance. Source CI and Deb preparation do not imply production APT visibility
or live station adoption; the central release workflow owns publication.
