# Visualization v1

The service is `xgc2.visualization`, API version `1`, profile `http.v1`.
The process supervisor supplies one `--bootstrap-input` file, ROS graph
remaps and managed ROS cache/log directories. Startup creates no domain instances.
The provider creates a fresh random incarnation on every process start. Product
bootstrap is explicit native CLI input, never an inherited ROS master parameter;
old private product parameter aliases are rejected.
`GET /v1/describe` returns its `service_ref`; discovery never creates an instance
or starts another provider. All other calls bind that incarnation.

Every request has one `X-Request-ID` and one `X-Xrpc-Timeout-Ms`. Bound calls also
have one `X-Xrpc-Instance-ID`. The XRPC SDK validates metadata, response identity,
HTTP framing, admission, cancellation and the exclusive Unix lease. A caller
retains its request ID and distinguishes a transport failure with unknown effects
from a confirmed domain rejection; writes are never automatically replayed.

| Method and route | Body | Result |
| --- | --- | --- |
| `GET /v1/describe` | Empty | ServiceRef, configuration capabilities and effective resource policy |
| `GET /v1/xrpc/policy` | Empty | One startup-resolved policy revision, values, sources and ceilings |
| `GET /v1/xrpc/storage` | Empty | Resolved runtime/cache/log write allocations and writer declarations |
| `GET /v1/xrpc/diagnostics` | Empty | Drain at most 16 fixed redacted SDK diagnostic records; capacity/drop counters |
| `GET /v1/xrpc/log-level` | Empty | Effective diagnostic policy revision |
| `PUT /v1/xrpc/log-level` | `{expectedRevision, level}` | SDK revision CAS for the in-memory log filter; format requires restart |
| `GET /v1/xrpc/status` | Empty | SDK transport counters and the domain handoff occupancy, with source time |
| `GET /v1/health` | Empty | Provider incarnation, domain readiness and fixed callback-worker count |
| `GET /v1/status` | Empty | Bounded membership, publication counts and the effective rate table |
| `GET /v1/rates` | Empty | Complete rate table and desired/applied/persisted revisions |
| `PUT /v1/rates` | `{expectedRevision, rates}` | Revision CAS and atomic replacement of the complete validated table |
| `GET /v1/instances/<id>` | Empty | Existing immutable configuration readiness and native counts |
| `PUT /v1/instances/<id>` | Complete frozen input `{robots, context, settings, displayRelays}` | Native projection and activation or an identical owned-configuration no-op; conflicting content is 409 |
| `DELETE /v1/instances/<id>` | Empty | Native callback/publication fence and deletion of only owned outputs |

IDs contain 1–128 ASCII letters, digits, `_`, `.` or `-`. The path owns the Run's
domain instance ID; the body has exactly `robots`, `context`, `settings`,
`displayRelays`. `robots` is the complete frozen public Robot array (at most 256);
`context` is the complete frozen session context, including `runMode`,
`worldClock`, `worldBoundary`. `settings` is the full panel settings object.
`displayRelays` contains at most 64 records with exactly `source`, `topic`,
`messageType`. The native product projects description/model metadata, selected
localization source/offset, palettes and relay kind from those values. Core
passes them through without reimplementing domain projection. The retired
`instanceId` body envelope and projected `descriptions`/`worldBoundary` wire
document are rejected; neither is another accepted activation format.

Optional `settings.publication` contains only boolean `markers`, `transforms`,
`scene`, `scenePaths`, `paths`, `groundScene`. Defaults are respectively false,
true, true, false, true, true. These explicit native publication controls retain
relay-only operation, independent Marker/Scene paths and description-only ground
rosters. A normal frozen panel needs no publication override. Other panel fields
remain owned by their existing Viewer/product consumers.
The complete rate table has the kind/channel fields described in the README.
Rates are finite 0.1–1000 Hz values; unknown/missing fields fail before mutation.
A rate update requires an integer positive `expectedRevision`; unknown fields,
persistence requests and stale revisions fail without changing the table.

The rate table's atomic immutable-snapshot publication is its application
boundary. A publication tick already using an older table finishes under that
table. Later ticks acquire the replacement; the provider wakes the scheduler on
change. The receipt has equal `desiredRevision` and `appliedRevision`, an
`appliedAtSteadyNs`, `state: applied`, and `persistedRevision: null`. The table is
in-memory configuration, not a durable preference store. Instance configuration
is likewise immutable and ephemeral: activation has desired/applied revision 1
and no persisted revision. Native activation does not assert fresh sensor data,
Viewer rendering, or scientific progress.

The process incarnation in ServiceRef is distinct from the Run-owned domain
instance ID. The provider generates it; no `serverInstanceId` is supplied by a
manifest, stored or copied between starts. Individual relay rate fields and a
caller-supplied relay `robotKind` are rejected.

## Consumer sequence and completion

The existing workflow explicitly starts this one native executable. Its generic
service discovery performs `GET /v1/describe` with finite request metadata and
registers the returned ServiceRef only after checking the selected `target_id`,
`service: xgc2.visualization`, `api_version: "1"`, `profile: http.v1` and allocated
endpoint. Discovery starts no process and creates no domain membership. No
separate RPC process, ROS ready-topic probe, `/health` probe or manufactured
incarnation is part of this sequence.

The workflow next sends one bound `PUT /v1/instances/<runId>` with its already
frozen values, for example:

```json
{"robots":[],"context":{"runMode":"simulation","worldClock":"simulation","worldBoundary":null},"settings":{},"displayRelays":[]}
```

Completion requires HTTP 200, `ok: true`, `id` equal to the requested Run ID,
`ready: true`, and `configuration.desiredRevision == appliedRevision == 1` with
`persistedRevision: null`. The native provider validates the whole candidate,
realizes installed URDFs, registers inputs/outputs and activates membership before
returning this receipt. It does not merely enqueue the operation. Matching owned
configuration is a no-op with `unchanged: true`; a different owned candidate is
409 and requires an explicit DELETE before replacement. Invalid input, clock
mismatch and output conflicts leave existing membership unchanged. Native ready
does not prove a fresh pose, Viewer frame, completed camera operation or scientific
progress; those existing consumer success conditions remain separate.

Run removal sends bound `DELETE /v1/instances/<runId>` with a genuinely empty
body. HTTP 200, `ok: true`, the same `id` and `removed: true` mean that native
publication in flight is fenced, relay/subscription work is stopped and only
owned outputs/unchanged owned parameters are removed. Repeating removal is safe.
ROS master discovery unregistration can settle asynchronously after this native
fence; the receipt does not assert that the remote discovery cache is empty.

Stopping the native process uses its existing SIGTERM/process-stop owner and
waits for actual process exit. The application joins domain/publication/input
work, removes owned outputs and shuts down ROS before SDK drain releases its
socket lease. A caller timeout or forced crash is an unknown-effects outcome,
not a successful stop or rollback. There is no separate RPC shutdown method or
Core sidecar drain. Restart changes ServiceRef incarnation, rejects stale bound
calls and restores no instance/rate state; reactivation is a new explicit action,
never an automatic replay of an uncertain mutation.

## Startup binding migration

`--initial-instance-file`, its reader/public Bootstrap helper, and implicit
startup activation are removed. Old ROS-master `initial_instance_file` and
`server_instance_id` values remain non-authoritative; private parameter aliases
and the retired CLI option fail startup. Consumers remove `bootstrapJson`,
`bootstrapFile` and product-specific file materialization. Freeze the domain input
once in the workflow, then pass it in the explicit PUT above.

The executable accepts one required `--bootstrap-input /explicit/path` plus
standard ROS remaps. It consumes the common XRPC
`contracts/bootstrap-input.schema.json` and `bootstrap.schema.json` through the
shared C++ loader. The binding fixes `target_id`, `service`, `api_version`,
`profile`, `endpoint`, `runtime_grant`, `authentication`, `secret_handles`,
`storage_grants`; it contains no incarnation or Run membership. This native
provider requires `xgc2.visualization`, API `"1"`, `http.v1`, Unix and
`local_private`. Remote profiles fail explicitly before ROS initialization;
there is no plaintext or local transport fallback. Local-private Unix uses
empty credential grants and secret handles.

The loader owns the 16 KiB bounded secure input read, shared identity/schema
validation and grant resolution. The process owner's runtime grant resolves to
the existing owned mode0700 endpoint parent, is validated against that parent
inode and is passed to the shared HTTP host as a retained descriptor. No
product/Core directory creation, chmod, lease or credential parser is added.
The domain request remains separately bounded by the 1 MiB HTTP policy.

The opaque `application` is a native settings object with only these fields:

| Field | Native meaning |
| --- | --- |
| `rosHomeGrant` | Required opaque name matching one declared storage grant; resolves to the existing owned `ROS_HOME` directory |
| `rosLogGrant` | Required distinct opaque name matching the other storage grant; resolves to the existing owned `ROS_LOG_DIR` directory |
| `callbackWorkers` | Optional immutable integer1–32, default2; existing shared ROS input pool |
| `worldClock` | Optional immutable `wall` or `simulation`, default`wall`; must match activation's `context.worldClock` |
| `rates` | Optional partial kind/channel object overlaying native defaults, default`{}`; runtime PUT still requires the complete table |

Both storage grants are resolved by the SDK before ROS starts and remain held
through the native lifecycle. `ROS_HOME` and `ROS_LOG_DIR` are explicit absolute
process-owner allocations; grant names are not paths. ROS libraries retain
their native path-based writes, and the process supervisor owns quota/rotation.
The provider creates no missing directory. Unknown application fields, missing
or unresolved grants and invalid settings fail before native startup. Robots,
Run IDs and display relays belong solely to the explicit domain PUT.
`--socket`, `--target-id`, `--callback-workers`, `--world-clock` and `--rates-json`
are retired rather than aliases or alternative authority.

```json
{"schema_version":1,"binding":{"schema_version":1,"target_id":"fixture:target","service":"xgc2.visualization","api_version":"1","profile":"http.v1","endpoint":{"kind":"unix","address":"/allocated/private/visualizer.sock"},"runtime_grant":"visualizer:runtime","authentication":"local_private","secret_handles":{},"storage_grants":["visualizer:ros-home","visualizer:ros-log"]},"grants":{},"application":{"rosHomeGrant":"visualizer:ros-home","rosLogGrant":"visualizer:ros-log","callbackWorkers":2,"worldClock":"simulation","rates":{}}}
```

The transport uses one XRPC HTTP owner and one fixed domain worker. Native ROS
input uses the existing fixed pool; publication uses one scheduler. Domain work
belongs to directly callable native instance, rate and status functions. HTTP
paths, methods, envelope fields and error status mapping live in the thin RPC
adapter. Frozen-input projection is also a native function; startup activates no
instances. The
native executable owns both domain and transport lifetimes; its readiness and
completion evidence come from actual native work.
The native runtime library links without XRPC. Only the native executable links
the adapter and shared XRPC transport; native data/function consumers remain
C++14 and do not require transport headers or an RPC endpoint.
Domain work
never runs in the HTTP IO handler. The preallocated handoff is bounded by
`HOST_MAX_IN_FLIGHT`, including active work. Expired or cancelled queued calls do
not begin native work. Cancellation after dispatch does not roll back activation,
rate application or deletion. The endpoint lease and admission remain retained
until actual work is quiescent, including during shutdown. The host finishes its
domain worker, stops the native publication scheduler and input pool, fences
owned output deletion and shuts down the native ROS graph before SDK drain can
release the endpoint lease.
A slow native operation
can therefore outlive a caller or drain deadline; lease release never asserts
that a cancelled operation has ended.
After an owner crash, restart uses the shared lease's unreachable-socket reclaim:
the exclusive lock, ownership, finite reachability proof and unchanged inode are
checked before deletion. A live endpoint or non-socket path is never reclaimed.

Runtime environment is snapshotted once by the process root and passed to XRPC.
The product declares 32 connections/in-flight calls, 16 KiB headers, 1 MiB
request/response bodies and 5 s call/header/idle/drain limits as deployment
choices and ceilings. Smaller supported settings use `XGC2_XRPC_` registry names;
unsupported explicit settings, empty values, unknown names and excess limits
fail startup. This host exposes policy/status only through its private endpoint;
there is no extra diagnostic listener. The private log-level method applies the
SDK's declared dynamic log filter with CAS; transport limits and log format remain
startup-only. SDK diagnostic records stay in a bounded 256-record memory ring;
only an explicit authorized drain returns them. Records contain stable codes and
identities, without headers, payloads, secrets or arbitrary metric labels.

| Write class | Writer and location | Trigger and recovery |
| --- | --- | --- |
| Instance/rate state | Provider memory | Explicit configuration; discarded at process exit, activated only by a subsequent explicit domain call |
| Unix socket and lease | XRPC; supervisor-granted private 0700 runtime directory | Process bind/stop; lock is retained, only the owned socket inode is removed |
| Robot descriptions/assets | Owning installed packages; read-only | Native URDF realization; the provider never rewrites packages |
| ROS parameters | Provider on the selected ROS graph | Native activation/deletion; only an unchanged owned value is removed |
| ROS caches and logs | ROS libraries; explicitly allocated `ROS_HOME`/`ROS_LOG_DIR` | Native lifecycle; deployment owns quota, log rotation and cache cleanup |
| Process diagnostics | Bounded individual failure messages on stderr | Supervisor owns log location, rotation and retention |

No database schema, import, durable rate-save, browser-store or filesystem
fallback is part of this service. Cache cleanup and transport stop never delete
user documents. ROS payload retention and serialization are message-dependent;
fixed application thread/history counts do not imply a universal ROS byte limit.
