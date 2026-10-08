# Visualization v1

The service is `xgc2.visualization`, API version `1`, profile `http.v1`.
The process supervisor supplies `--socket`, `--target-id`, ROS graph settings,
managed ROS cache/log directories and an optional frozen startup document.
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
| `PUT /v1/instances/<id>` | Complete instance document | Native activation or an identical-content no-op; conflicting content is 409 |
| `DELETE /v1/instances/<id>` | Empty | Native callback/publication fence and deletion of only owned outputs |

IDs contain 1–128 ASCII letters, digits, `_`, `.` or `-`. Instance documents have
exactly `robots`, `descriptions`, `worldBoundary`, `settings`, `displayRelays`.
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

The startup document has exactly `instanceId`, `robots`, `context`, `settings`,
`displayRelays`. It carries the public frozen Robot records and full panel/context
values; the product projects only its owned visualization fields. Bootstrap relay
records have exactly `source`, `topic`, `messageType`; robot kind comes from the
frozen Robot roster. RPC relay records instead carry `robotKind` explicitly.
Individual relay rate fields are rejected. The process incarnation is distinct
from the startup document's Run-owned instance ID.

The transport uses one XRPC HTTP owner and one fixed domain worker. Native ROS
input uses the existing fixed pool; publication uses one scheduler. Domain work
never runs in the HTTP IO handler. The preallocated handoff is bounded by
`HOST_MAX_IN_FLIGHT`, including active work. Expired or cancelled queued calls do
not begin native work. Cancellation after dispatch does not roll back activation,
rate application or deletion. The endpoint lease and admission remain retained
until actual work is quiescent, including during shutdown. The host finishes its
domain worker, stops the native publication scheduler and input pool, and fences
owned output deletion before SDK drain can release the endpoint lease.
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
| Instance/rate state | Provider memory | Explicit configuration; discarded at process exit, restored only from an explicit supervisor document |
| Unix socket and lease | XRPC; supervisor-granted private 0700 runtime directory | Process bind/stop; lock is retained, only the owned socket inode is removed |
| Bootstrap document | Supervisor; explicit read-only file grant | Provider reads at startup; supervisor owns retention/removal |
| Robot descriptions/assets | Owning installed packages; read-only | Native URDF realization; the provider never rewrites packages |
| ROS parameters | Provider on the selected ROS graph | Native activation/deletion; only an unchanged owned value is removed |
| ROS caches and logs | ROS libraries; explicitly allocated `ROS_HOME`/`ROS_LOG_DIR` | Native lifecycle; deployment owns quota, log rotation and cache cleanup |
| Process diagnostics | Bounded individual failure messages on stderr | Supervisor owns log location, rotation and retention |

No database schema, import, durable rate-save, browser-store or filesystem
fallback is part of this service. Cache cleanup and transport stop never delete
user documents. ROS payload retention and serialization are message-dependent;
fixed application thread/history counts do not imply a universal ROS byte limit.
