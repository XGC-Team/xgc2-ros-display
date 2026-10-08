# ROS1 visualizer

`xgc2-ros-visualizer` provides one C++ ROS server, one executable
`xgc2_ros_visualizer_node`, and the canonical catkin package
`xgc2_ros_visualizer`. A server owns a fixed input pool (default 2, maximum 32
workers), one publication scheduler, and a bounded XRPC Unix HTTP control loop plus one fixed domain worker.
Robots, descriptions and relays are instance data. Adding robots does not create
application threads, helper processes or `robot_state_publisher` children.
Core uses its ordinary supervised process definition, registers the actual
ServiceRef and explicitly activates that Run through RPC. Normal Stop terminates
that Run's native server.
The native application owns readiness, input/publication workers and the XRPC
endpoint for that same lifetime. Its domain control functions own instance
activation/removal, rate CAS and status data. The RPC adapter maps wire requests
to those functions; frozen Robot/context projection also stays in the native
product. Startup creates no domain membership.

The separate `xgc2_robot_visualization` dependency supplies FS150, Scout and
Mecanum geometry, wheel/rotor animation calculations, frame names and path
styles. This server owns ROS subscriptions, publication, scheduling, URDF
parameters and required fixed/joint TF. Gazebo shadow rendering, algorithm
planning, map production and Viewer rendering remain their owning products.

## Startup and RPC

Source a ROS Noetic environment and select its master/IP normally. The
supervisor supplies allocated `ROS_HOME` and `ROS_LOG_DIR` locations for native
cache/log ownership. The native process accepts one required
`--bootstrap-input` path plus standard ROS remaps. The shared XRPC loader reads
that owned mode0600 file under an existing owned mode0700 parent and validates
its bounded binding/grants. The binding selects the target and private Unix
endpoint; the provider creates its incarnation. Its `application` requires
`rosHomeGrant` and `rosLogGrant` naming the two declared storage grants. Optional
`callbackWorkers` (1–32, default2), `worldClock` (`wall` or `simulation`, default
`wall`) and `rates` (a partial object overlaying defaults) remain native settings.
Startup creates no Run membership. Old socket/target/worker/clock/rate flags and
`--initial-instance-file` are rejected; none is another startup authority.
The product reads no ROS master bootstrap
parameters; private product parameter aliases are rejected. The clock is immutable
and selected before ROS initialization; an existing `/use_sim_time` remap is
preserved. Do not separately pass `_use_sim_time`.
ROS name/namespace/master/IP remaps remain available; `__log` overrides are
rejected so native log writes retain their supervisor-allocated location.

```sh
rosrun xgc2_ros_visualizer xgc2_ros_visualizer_node \
  --bootstrap-input "$XGC_RUNTIME_DIR/visualizer-bootstrap.json"
curl --unix-socket "$XGC_RUNTIME_DIR/visualizer.sock" \
  -H 'X-Request-ID: discover-1' -H 'X-Xrpc-Timeout-Ms: 3000' \
  http://localhost/v1/describe
```

The socket is mode 0600. XRPC owns the endpoint lease, framing, admission and
finite deadlines. Every request carries `X-Request-ID` and `X-Xrpc-Timeout-Ms`;
all routes except explicit GET discovery also require the returned
`X-Xrpc-Instance-ID`. The complete [service contract](docs/visualization-v1.md)
defines resource policy, application receipts and write ownership. Runtime
policy uses startup-snapshotted `XGC2_XRPC_` settings and explicit ceilings.

| Request | Behavior |
| --- | --- |
| `GET /v1/health` | Server identity, readiness and fixed callback-worker count |
| `GET /v1/status` | Membership, resource counts, rates and publication counters |
| `GET /v1/instances/<id>` | Observe an existing instance; 404 never creates one |
| `PUT /v1/instances/<id>` | Validate and create complete membership; identical retry is a no-op, different content is 409 |
| `DELETE /v1/instances/<id>` | Idempotently remove only that instance; wait for its publication fence |
| `GET /v1/rates` | Complete table and desired/applied/persisted revisions |
| `PUT /v1/rates` | `{expectedRevision, rates}` atomically replaces the complete table; invalid/stale candidates leave it unchanged |

A wire instance request contains exactly `robots`, `context`, `settings` and
`displayRelays`: the full frozen public Robot/context/panel values. The native
product projects and validates its own visualization fields atomically; Core
does not construct a second projected roster. The Run ID is in the request path,
and the ServiceRef incarnation is provider-generated. Empty rosters are valid. Configured scene instances,
including zero-robot worlds, publish `/xgc/robot_scene/ready`; this acknowledges
configuration, not fresh robot poses or scientific progress. Descriptions also
publish `/xgc/robot_descriptions/ready`.

After native startup and ServiceRef discovery, the explicit bound PUT body is:

```json
{"robots":[],"context":{"runMode":"simulation","worldClock":"simulation","worldBoundary":null},"settings":{},"displayRelays":[]}
```

`robots` is the complete frozen public Robot array, `context` the complete
session context, and `settings` the entry's full camel-case panel settings.
The server projects these values and creates the instance through the same
validated registry path as RPC. Namespace, scene model and installed description
metadata come from each Robot's visualization configuration. FS150 AR uses only
`localizationSources[context.runMode].poseTopic` and its frozen offset. Missing
selected sources reject activation; no VRPN/profile/topic guessing or second offset
application occurs. Numeric slot palette ordering and the original Scout mocap
scene-model binding are preserved. Unrelated panel fields remain Viewer-owned.
Optional `settings.publication` has boolean `markers`, `transforms`, `scene`,
`scenePaths`, `paths`, `groundScene` for explicit native publication control.
Native internal projected configuration remains directly callable data, and is
not an alternate RPC wire format. The process root consumes the shared
`--bootstrap-input` loader before ROS starts; Run activation remains explicit.

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

A wire relay request is `{source,topic,messageType}`; its kind comes from the
frozen Robot roster. Source names are unique
canonical absolute names outside `/xgc/display`; output is exactly
`/xgc/display` + source. Maximum 64 per instance. Only PointCloud2,
OccupancyGrid, Path and PoseArray full-state messages are accepted. The relay
receives serialized bytes once and publishes the same immutable pointer using
the original type/MD5/definition. It does not decode, rewrite Header sequence,
convert fields, replay previously published samples or budget the source itself.
The native relay representation also carries the product-projected kind. A wire
`robotKind` or individual relay rate field is rejected.

Membership, queues, latest-message retention and history point counts are
bounded. Variable ROS payload bytes and ROS serialization/transport allocations
remain message-dependent; this is not a claim of zero allocations or a universal
byte-memory limit.

## Build, package and validation

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DXgcXrpc_DIR=/path/to/xrpc/lib/cmake/XgcXrpc -DCMAKE_INSTALL_PREFIX=/path/to/prefix
cmake --build build -j2
cmake --build build --target tests -j2
cmake --build build --target run_tests -j2
catkin_test_results build/test_results
cmake --install build
```

The XRPC transport translation unit requires C++20 and the installed
`XgcXrpc` 0.1 CMake package. Its private facade keeps the existing Noetic ROS
message translation units on C++14; the transport SDK is not copied into this
product.

The native package is `ros-noetic-xgc2-ros-visualizer`, version 0.4.0-1. It installs
one executable, the runtime/contract libraries and public canonical headers.
Consumers use `find_package(catkin REQUIRED COMPONENTS xgc2_ros_visualizer)`.
The installed Robot SDK ABI dependency is at least 0.2.0-16.

`.xgc2/scripts/build_debs_in_docker.sh` requires an immutable Noetic build image with an
explicit preprovisioned C++20 `--cxx` compiler, and installs published SDK dependencies only inside independent containers. It runs
source tests, produces the Deb, then installs that Deb in a second container and
checks the catkin exports, linkage and actual executable. CI selects these resources through `XGC2_FOCAL_CXX_BUILD_IMAGE` and
`XGC2_FOCAL_CXX`; images are selected by digest. This build requires
network access to production APT for dependencies; it never uses an active Core
container as its builder.

`test/installed_node_probe.py` creates a finite private ROS graph. Its gate covers
zero-robot explicit activation readiness and clock remapping, no-peer persistent subscriptions,
four-type exact-byte relays, RPC atomic validation/retries/deletion, foreign
instance/parameter preservation, 20/100 robot resource counts, source-time paths
at accelerated clock/low publication rates, in-process URDF joint TF and bounded
low-rate SIGTERM. Transport tests separately exercise real Unix HTTP framing,
client limits, deadlines and foreign-inode socket cleanup. Private probes are
correctness/resource checks, not formal station, browser or scientific-mission
acceptance. Source CI and Deb preparation do not imply production APT visibility
or live station adoption; the central release workflow owns publication.

Pass a fixed external process catalog to verify the consumer's actual startup
arguments/environment, ServiceRef declaration and Stop grace period:

```sh
python3 test/installed_node_probe.py \
  /opt/ros/noetic/lib/xgc2_ros_visualizer/xgc2_ros_visualizer_node \
  /fixture/xgc2-ros-visualizer.json
```

This mode runs in an independent container with the actual Deb installed at the
catalog's executable path. It renders the catalog unchanged using explicit
private ROS/socket/cache/log allocations, records its input hash and actual
argv/environment, and uses it for initial startup and every restart. The native
render/topic/relay/removal checks then run through the same application. A retired
flag or missing grant remains a failed integration gate; the fixture never patches
catalog arguments or supplies a legacy product alias. The direct fixture mode
without a catalog verifies native product behavior and does not prove catalog,
Core planner or authored workflow integration. Those consumers remain separately
owned and audited.
