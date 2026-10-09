#!/usr/bin/env python3
"""Finite, private ROS graph acceptance for the actual single server executable.
Run only in an independent build/install container. Never uses a station master.
"""
import collections
import copy
import hashlib
import http.client
import json
import os
import re
import signal
import socket
import subprocess
import sys
import tempfile
import time
import urllib.parse
import xml.etree.ElementTree as ET
import xmlrpc.client
import uuid


class UnixHTTP(http.client.HTTPConnection):
    def __init__(self, path):
        super().__init__("localhost", timeout=3)
        self.path = path

    def connect(self):
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.settimeout(self.timeout)
        self.sock.connect(self.path)


def require(value, message):
    if not value:
        raise AssertionError(message)


def wait(predicate, message, seconds=6):
    until = time.monotonic() + seconds
    while time.monotonic() < until:
        if predicate():
            return
        time.sleep(.02)
    raise AssertionError(message)


def resources(pid):
    values = {}
    with open('/proc/%d/status' % pid) as stream:
        for line in stream:
            if line.startswith(('VmRSS:', 'Threads:')):
                key, value = line.split(':', 1)
                values[key] = int(value.split()[0])
    children = []
    for tid in os.listdir('/proc/%d/task' % pid):
        with open('/proc/%d/task/%s/children' % (pid, tid)) as stream:
            children.extend(stream.read().split())
    values['children'] = children
    require(not children, 'server forked a child process')
    return values


def robot(name, ar=False):
    return dict(name=name, namespace='/' + name, kind='px4_multirotor',
                visualization=dict(sceneClass='fs150', sceneModel=name,
                    descriptionPackage='fs150_description', descriptionFile='urdf/fs150_visual.urdf',
                    robotStatePublisher=False, jointStateTopic='joint_states', pathTopic='path'),
                hybridSource='physical' if ar else 'simulation', profileId='px4-multirotor.physical.vrpn',
                px4=dict(mocapRigidBodyName=name),
                simulationPoseTopic='/raw/' + name if ar else '/' + name + '/mavros/local_position/pose')


def instance(count=0, relays=None, scene=True):
    rows = [robot('uav%d' % (i + 1), i == 0) for i in range(count)]
    return dict(robots=rows, context=dict(runMode='hybrid', worldClock='simulation', worldBoundary=None, scene=dict(simulator='xsim'), localizationOffset=dict(x=1, y=2, z=3)),
                settings=dict(publication=dict(transforms=scene, scene=scene, markers=scene,
                                               scenePaths=scene, paths=scene)), displayRelays=relays or [])


class CatalogLaunch:
    """Read one external catalog snapshot; never fix its argv or invent aliases."""
    def __init__(self, path):
        self.path = os.path.abspath(path)
        with open(self.path, 'rb') as stream:
            contents = stream.read(1024 * 1024 + 1)
        require(len(contents) <= 1024 * 1024, 'catalog input exceeds fixture limit')
        self.sha256 = hashlib.sha256(contents).hexdigest()
        catalog = json.loads(contents)
        definitions = [value for value in catalog['definitions'] if value['id'] == 'ros-visualizer']
        require(len(definitions) == 1, 'catalog must have one ros-visualizer definition')
        self.definition = definitions[0]
        require(self.definition['command'].get('directExecutable') is True,
                'catalog must launch the native executable directly')
        self.stop_seconds = self.definition['stop']['gracePeriod'] / 1e9
        require(0 < self.stop_seconds <= 10, 'catalog Stop must have a finite fixture grace period')

    def render(self, binary, allocations):
        command = self.definition['command']
        require(command['executable'] == binary,
                'catalog executable differs; run against its actually installed native path')
        properties = self.definition['parameters']['properties']
        parameters = {name: spec['default'] for name, spec in properties.items() if 'default' in spec}
        parameters.update({name: value for name, value in allocations.items() if name in properties})
        require(all(name in parameters for name in self.definition['parameters']['required']),
                'catalog requires missing owner-supplied input')
        def value(text):
            require(isinstance(text, str), 'catalog argv/env must be strings')
            def replace(match):
                name = match.group(1)
                require(name in parameters, 'unresolved catalog parameter: ' + name)
                entry = parameters[name]
                require(isinstance(entry, (str, int)) and not isinstance(entry, bool),
                        'fixture expects scalar native startup parameters')
                return str(entry)
            rendered = re.sub(r'\$\{([^{}]+)\}', replace, text)
            require('${' not in rendered, 'unresolved catalog template')
            return rendered
        # Keep the catalog's argument order, flags and empty arguments verbatim.
        # Environment inheritance matches ordinary native process launch; only
        # declared command.env values are overlaid, without patching its library
        # paths, directory fields or protocol arguments in the test.
        argv = [binary] + [value(argument) for argument in command['args']]
        environment = dict(os.environ)
        environment.update({name: value(entry) for name, entry in command['env'].items()})
        return argv, environment

    def verify_reference(self, reference, allocations):
        services = self.definition.get('services', [])
        require(len(services) == 1, 'catalog must declare its native ServiceRef discovery')
        declaration = services[0]
        require(declaration['describePath'] == '/v1/describe', 'catalog discovery method differs')
        for name in ('service', 'api_version', 'profile'):
            require(reference[name] == declaration[name], 'ServiceRef differs from catalog: ' + name)
        require(reference['target_id'] == allocations['targetId'], 'ServiceRef target differs from owner')
        require(reference['endpoint']['kind'] == 'unix' and
                reference['endpoint']['address'] == allocations[declaration['endpointParameter']],
                'ServiceRef endpoint differs from catalog allocation')


def main(binary, catalog_path=None):
    catalog = CatalogLaunch(catalog_path) if catalog_path else None
    with tempfile.TemporaryDirectory(prefix='xgc2-single-server-probe-') as work:
        probe_socket = os.path.join(work, 'control.sock')
        reserve = socket.socket()
        reserve.bind(('127.0.0.1', 0))
        port = reserve.getsockname()[1]
        reserve.close()
        master_uri = 'http://127.0.0.1:%d' % port
        os.environ.update(ROS_MASTER_URI=master_uri, ROS_IP='127.0.0.1', ROS_HOME=work,
                          ROS_LOG_DIR=os.path.join(work, 'roslog'))
        os.environ.pop('ROS_HOSTNAME', None)
        allocations = dict(socketPath=probe_socket, targetId='private-probe',
            callbackWorkers=2, rates={'fs150': {'path': 5}}, worldClock='simulation',
            rosHomeGrant='probe:ros-home', rosLogGrant='probe:ros-log',
            rosMasterUri=master_uri, rosIp='127.0.0.1', rosHome=work,
            rosLogDir=os.path.join(work, 'roslog'), bootstrapInput=os.path.join(work, 'bootstrap.json'))
        os.makedirs(allocations['rosLogDir'], mode=0o700)
        bootstrap_document = dict(schema_version=1, binding=dict(schema_version=1,
            target_id=allocations['targetId'], service='xgc2.visualization', api_version='1',
            profile='http.v1', endpoint=dict(kind='unix', address=probe_socket),
            runtime_grant='probe:runtime', authentication='local_private', secret_handles={},
            storage_grants=[allocations['rosHomeGrant'], allocations['rosLogGrant']]), grants={},
            application=dict(rosHomeGrant=allocations['rosHomeGrant'], rosLogGrant=allocations['rosLogGrant'],
                callbackWorkers=allocations['callbackWorkers'], worldClock=allocations['worldClock'],
                rates=allocations['rates']))
        def write_bootstrap(path, document):
            descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC | os.O_CLOEXEC, 0o600)
            with os.fdopen(descriptor, 'w') as stream:
                json.dump(document, stream)
        write_bootstrap(allocations['bootstrapInput'], bootstrap_document)
        logfile = open(os.path.join(work, 'process.log'), 'w')
        processes = []
        def launch_provider():
            if catalog:
                argv, environment = catalog.render(binary, allocations)
                print('CATALOG LAUNCH: ' + json.dumps(dict(path=catalog.path,
                    sha256=catalog.sha256, argv=argv, declaredEnvironment={
                        name: environment[name] for name in catalog.definition['command']['env']}),
                    sort_keys=True), flush=True)
            else:
                argv = [binary, '--bootstrap-input', allocations['bootstrapInput'],
                    '/use_sim_time:=/xgc2_ros_visualizer/use_sim_time']
                environment = None
            provider = subprocess.Popen(argv, env=environment, stdout=logfile,
                                        stderr=logfile, start_new_session=True)
            processes.append(provider)
            return provider
        try:
            master = subprocess.Popen(['roscore', '-p', str(port)], stdout=logfile,
                                      stderr=logfile, start_new_session=True)
            processes.append(master)
            def master_ready():
                try:
                    return xmlrpc.client.ServerProxy(master_uri).getPid('/private_probe')[0] == 1
                except (OSError, xmlrpc.client.Error):
                    return False
            wait(master_ready, 'private ROS master failed')
            import rospy
            from rosgraph_msgs.msg import Clock
            from geometry_msgs.msg import Pose, PoseStamped, PoseArray
            from nav_msgs.msg import OccupancyGrid, Path
            from sensor_msgs.msg import PointCloud2, JointState
            from std_msgs.msg import Empty, Header
            from tf2_msgs.msg import TFMessage
            from visualization_msgs.msg import MarkerArray, Marker
            from foxglove_msgs.msg import SceneUpdate
            rospy.init_node('visualizer_private_probe', anonymous=True, disable_signals=True)
            clock_pub = rospy.Publisher('/clock', Clock, queue_size=1)
            # The private Python source exposes 100 topics through one TCPROS
            # listener (rospy defaults to backlog 5). Size this test fixture for
            # the simultaneous connection burst; production server is untouched.
            from rospy.impl import tcpros_base
            private_source = tcpros_base.init_tcpros_server()
            private_source.start_server()
            private_source.tcp_ros_server.server_sock.listen(256)
            instance_id = None
            rate_revision = 1
            def rpc(method, path, body=None, expected=200, rate_expected=None, wrap_rates=True):
                nonlocal rate_revision
                headers = {'Content-Type': 'application/json', 'X-Request-ID': uuid.uuid4().hex,
                           'X-Xrpc-Timeout-Ms': '3000'}
                if instance_id is not None:
                    headers['X-Xrpc-Instance-ID'] = instance_id
                if method == 'PUT' and path == '/v1/rates' and wrap_rates:
                    body = dict(expectedRevision=rate_revision if rate_expected is None else rate_expected, rates=body)
                client = UnixHTTP(probe_socket)
                try:
                    client.request(method, path, None if body is None else json.dumps(body),
                                   headers)
                    response = client.getresponse()
                    status = response.status
                    require(response.getheader('X-Request-ID') == headers['X-Request-ID'], 'response request identity lost')
                    if instance_id is not None and status != 409:
                        require(response.getheader('X-Xrpc-Instance-ID') == instance_id, 'response instance identity lost')
                    result = json.loads(response.read(1024 * 1024 + 1))
                finally:
                    client.close()
                require(status == expected, '%s %s: %s %s' % (method, path, status, result))
                if path == '/v1/rates' and status == 200:
                    rate_revision = result['appliedRevision']
                    require(result['desiredRevision'] == rate_revision and result['persistedRevision'] is None,
                            'rate receipt conflates desired/applied/persisted')
                return result
            retired_initial = os.path.join(work, 'retired-input.json')
            with open(retired_initial, 'w') as stream:
                json.dump(dict(instanceId='old-initial', robots=[], context=dict(
                    runMode='simulation', worldClock='simulation', worldBoundary=None, scene=dict(simulator='xsim'), localizationOffset=dict(x=1, y=2, z=3)),
                    settings={}, displayRelays=[]), stream)
            server = launch_provider()
            wait(lambda: os.path.exists(probe_socket) or server.poll() is not None,
                 'server socket absent')
            require(server.poll() is None, 'server exited during native startup')
            description = rpc('GET', '/v1/describe')
            if catalog:
                catalog.verify_reference(description['service_ref'], allocations)
            instance_id = description['service_ref']['instance_id']
            require(description['service_ref']['target_id'] == 'private-probe', 'wrong ServiceRef target')
            require(rpc('GET', '/v1/health')['callbackWorkers'] == 2, 'wrong input pool size')
            require(rpc('GET', '/v1/status')['instanceCount'] == 0, 'startup implicitly activated an instance')
            legacy = dict(instanceId='old', robots=[], context=dict(runMode='simulation',
                worldClock='simulation', worldBoundary=None), settings={}, displayRelays=[])
            rpc('PUT', '/v1/instances/retired-input', legacy, 400)
            projected = dict(robots=[], descriptions=[], worldBoundary=None, settings={}, displayRelays=[])
            rpc('PUT', '/v1/instances/retired-input', projected, 400)
            rpc('GET', '/v1/instances/retired-input', expected=404)
            receipt = rpc('PUT', '/v1/instances/explicit-zero', instance())
            require(receipt['ready'] and receipt['configuration']['desiredRevision'] == 1 and
                    receipt['configuration']['appliedRevision'] == 1 and
                    receipt['configuration']['persistedRevision'] is None, 'zero activation receipt invalid')
            ready = []
            ready_sub = rospy.Subscriber('/xgc/robot_scene/ready', Empty, lambda _: ready.append(True), queue_size=1)
            sim_time = 100.0
            def stamp():
                return rospy.Time.from_sec(sim_time)
            def pump(seconds, publishers=(), ar_publisher=None, joint_publisher=None, frame='map'):
                nonlocal sim_time
                until = time.monotonic() + seconds
                previous = time.monotonic()
                while time.monotonic() < until:
                    current = time.monotonic()
                    sim_time += 3 * (current - previous)
                    previous = current  # Actual RTF=3 even when publishing 100 slots costs CPU.
                    clock_pub.publish(Clock(clock=stamp()))
                    for index, publisher in enumerate(publishers):
                        msg = PoseStamped(); msg.header.stamp = stamp(); msg.header.frame_id = frame
                        msg.pose.position.x = sim_time / 10 + index; msg.pose.position.z = 1
                        msg.pose.orientation.w = 1; publisher.publish(msg)
                    if ar_publisher:
                        msg = PoseStamped(); msg.header.stamp = stamp(); msg.header.frame_id = 'frozen'
                        msg.pose.position.x = 1; msg.pose.position.y = 2; msg.pose.position.z = .5
                        msg.pose.orientation.w = 2; ar_publisher.publish(msg)
                    if joint_publisher:
                        joint_publisher[0].publish(JointState(header=Header(stamp=stamp()),
                            name=joint_publisher[1], position=[.5] * len(joint_publisher[1])))
                    time.sleep(.02)
            pump(.15)
            wait(lambda: bool(ready), 'zero-robot explicit activation readiness missing')
            require(rospy.get_param('/xgc2_ros_visualizer/use_sim_time'), 'catalog clock remap not honored')
            rpc('DELETE', '/v1/instances/explicit-zero')
            rpc('GET', '/v1/instances/explicit-zero', expected=404)
            baseline = resources(server.pid)
            default_rates = rpc('GET', '/v1/rates')['rates']
            require(default_rates['fs150']['path'] == 5, 'native startup partial rates overlay lost')
            rpc('PUT', '/v1/rates', {'fs150': {'path': 7}}, 400)
            require(rpc('GET', '/v1/rates')['rates'] == default_rates, 'partial runtime PUT changed rates')
            original_revision = rate_revision
            rpc('PUT', '/v1/rates', default_rates, 400, wrap_rates=False)
            require(rate_revision == original_revision, 'retired flat rate body mutated state')
            rpc('PUT', '/v1/rates', default_rates)
            rpc('PUT', '/v1/rates', default_rates, 409, rate_expected=original_revision)
            receipt = rpc('GET', '/v1/rates')
            require(receipt['appliedRevision'] == original_revision + 1, 'stale CAS changed rates')
            rpc('GET', '/health', expected=404)
            storage = rpc('GET', '/v1/xrpc/storage')
            require(storage['rosCache']['root'] == work and storage['rosLogs']['root'] == os.path.join(work, 'roslog'),
                    'native writes do not expose their explicit allocations')
            bad = copy.deepcopy(default_rates); bad['fs150']['path'] = 0
            rpc('PUT', '/v1/rates', bad, 400)
            require(rpc('GET', '/v1/rates')['rates'] == default_rates, 'bad rate changed configuration')

            # Four full-state types: subscribe without peers, forward original bytes,
            # no replay, and stay subscribed after the display peers leave.
            types = [PointCloud2, OccupancyGrid, Path, PoseArray]
            type_names = ['sensor_msgs/PointCloud2', 'nav_msgs/OccupancyGrid', 'nav_msgs/Path', 'geometry_msgs/PoseArray']
            sources = ['/probe/cloud', '/probe/grid', '/probe/path', '/probe/poses']
            relay_specs = [dict(source=s, topic='/xgc/display' + s, messageType=t) for s, t in zip(sources, type_names)]
            source_pubs = [rospy.Publisher(s, t, queue_size=1) for s, t in zip(sources, types)]
            original = [collections.deque(maxlen=256) for _ in types]
            displayed = [collections.deque(maxlen=256) for _ in types]
            original_subs = [rospy.Subscriber(s, rospy.AnyMsg, lambda m, i=i: original[i].append(bytes(m._buff)), queue_size=100) for i, s in enumerate(sources)]
            foreign = instance(relays=relay_specs, scene=False)
            rpc('PUT', '/v1/instances/foreign-relay', foreign)
            wait(lambda: all(p.get_num_connections() >= 2 for p in source_pubs), 'relay did not subscribe before a viewer')
            rospy.set_param('/foreign/visual_robot_description', 'foreign-owned')
            def packets(count):
                for n in range(count):
                    for i, publisher in enumerate(source_pubs):
                        msg = types[i](); msg.header.seq = n + 31; msg.header.stamp = stamp(); msg.header.frame_id = 'original'
                        if i == 0:
                            msg.width = 256; msg.height = 1; msg.point_step = 4; msg.row_step = 1024; msg.data = bytes([n % 256]) * 1024
                        elif i == 1:
                            msg.info.width = 2; msg.info.height = 1; msg.info.resolution = .125; msg.data = [-1, n % 100]
                        elif i == 2:
                            for x in (n * .01, n * .01 + 1):
                                sample = PoseStamped();sample.header = copy.deepcopy(msg.header)
                                sample.pose.position.x = x;sample.pose.orientation.w = 1;msg.poses.append(sample)
                        else:
                            for x in (n * .01, n * .01 + 1):
                                sample = Pose();sample.position.x = x;sample.orientation.w = 1;msg.poses.append(sample)
                        publisher.publish(msg)
                    time.sleep(.01)
            packets(20); time.sleep(.25)
            peers = [rospy.Subscriber('/xgc/display' + s, rospy.AnyMsg, lambda m, i=i: displayed[i].append(bytes(m._buff)), queue_size=100) for i, s in enumerate(sources)]
            wait(lambda: all(peer.get_num_connections() for peer in peers), 'display publishers missing')
            time.sleep(.15)
            require(not any(displayed), 'relay replayed an already-published source sample')
            rate_table = copy.deepcopy(default_rates)
            for key in ['display_pointcloud', 'display_grid', 'display_path', 'display_pose_array']:
                rate_table['global'][key] = 5
            rpc('PUT', '/v1/rates', rate_table)
            packets(80); time.sleep(.25)
            for i in range(4):
                require(len(original[i]) >= 40, 'original recorder was throttled')
                require(1 <= len(displayed[i]) <= 8, 'relay ceiling/delivery failed')
                require(all(sample in original[i] for sample in displayed[i]), 'relay changed raw serialized payload')
            relay_counts = [len(samples) for samples in displayed]
            for peer in peers:
                peer.unregister()
            wait(lambda: all(p.get_num_connections() >= 2 for p in source_pubs), 'disconnect removed source subscription')
            changed = copy.deepcopy(foreign); changed['settings']['publication']['paths'] = True
            rpc('PUT', '/v1/instances/foreign-relay', changed, 409)
            require(rpc('PUT', '/v1/instances/foreign-relay', foreign)['unchanged'], 'retry was not a no-op')
            invalid = instance(1);invalid['robots'][0]['namespace'] = '/bad_slot'
            rpc('PUT', '/v1/instances/invalid-label', invalid, 400)
            rpc('GET', '/v1/instances/invalid-label', expected=404)
            require(rpc('GET', '/v1/instances/foreign-relay')['ready'], 'invalid Scene request stopped foreign instance')
            rpc('PUT', '/v1/rates', default_rates)

            tf_messages = collections.deque(maxlen=20)
            paths = collections.deque(maxlen=10)
            ar_paths = collections.deque(maxlen=10)
            markers = collections.deque(maxlen=10)
            scenes = collections.deque(maxlen=10)
            standard_tf = collections.deque(maxlen=20)
            def observe(paths_enabled=True):
                subscriptions = [rospy.Subscriber('/xgc/tf', TFMessage, tf_messages.append, queue_size=1),
                    rospy.Subscriber('/markers', MarkerArray, markers.append, queue_size=1),
                    rospy.Subscriber('/xgc/scene', SceneUpdate, scenes.append, queue_size=1),
                    rospy.Subscriber('/tf', TFMessage, standard_tf.append, queue_size=1)]
                if paths_enabled:
                    subscriptions += [rospy.Subscriber('/uav1/path', Path, paths.append, queue_size=1),
                        rospy.Subscriber('/uav1/ar_path', Path, ar_paths.append, queue_size=1)]
                return subscriptions
            observers = observe()
            pose_pubs = [rospy.Publisher('/uav%d/mavros/local_position/pose' % (i + 1), PoseStamped, queue_size=1) for i in range(100)]
            ar_pub = rospy.Publisher('/vrpn_client_node_physical/uav1/pose', PoseStamped, queue_size=1)
            rpc('PUT', '/v1/instances/robots20', instance(20))
            wait(lambda: all(p.get_num_connections() for p in pose_pubs[:20]), '20 robot inputs missing')
            pump(.7, pose_pubs[:20], ar_pub)
            wait(lambda: bool(paths) and bool(tf_messages), '20 representative pose/path delivery missing')
            twenty = resources(server.pid)
            require(rpc('GET', '/v1/status')['publisherWorkers'] == 1, 'publisher count changed')
            started = time.monotonic();rpc('DELETE', '/v1/instances/robots20');delete20 = time.monotonic() - started
            require(delete20 < 2, 'DELETE20 exceeded bounded completion')
            require(rpc('GET', '/v1/instances/foreign-relay')['ready'], 'DELETE removed foreign instance')
            require(rospy.get_param('/foreign/visual_robot_description') == 'foreign-owned', 'global URDF purge occurred')
            for observer in observers:
                observer.unregister()
            retained_topics = set(sources + [spec['topic'] for spec in relay_specs] + ['/rosout', '/clock'])
            def discovery_released():
                _, _, state = xmlrpc.client.ServerProxy(master_uri).getSystemState('/private_probe')
                return not any('/xgc2_ros_visualizer' in nodes and topic not in retained_topics
                               for entries in state[:2] for topic, nodes in entries)
            wait(discovery_released, 'deleted robot publications/subscriptions remained in master registry')
            # Exercise the publication/DELETE fence repeatedly, using the real RPC.
            for i in range(25):
                rpc('PUT', '/v1/instances/delete-race', instance(1))
                clock_pub.publish(Clock(clock=stamp()))
                rpc('DELETE', '/v1/instances/delete-race')
                # ROS master unregistration is asynchronous inside roscpp.
                # Wait for discovery to settle before reusing the same namespace.
                wait(discovery_released, 'deleted resources remained in private master registry')
            hundred_request = instance(100)
            description = robot('described');description['kind'] = 'description_only'
            description['visualization'].update(sceneClass='', sceneModel='', robotStatePublisher=True,
                descriptionPackage='scout_description', descriptionFile='urdf/scout_visual.urdf')
            hundred_request['robots'].append(description)
            low = {kind: {channel: .1 for channel in channels} for kind, channels in default_rates.items()}
            rpc('PUT', '/v1/rates', low)
            rpc('PUT', '/v1/instances/robots100', hundred_request)
            wait(lambda: all(p.get_num_connections() for p in pose_pubs), '100 robot inputs missing')
            observers = observe()
            wait(lambda: all(observer.get_num_connections() for observer in observers), '100 robot outputs missing')
            paths.clear();markers.clear();scenes.clear();ar_paths.clear()
            pump(1.35, pose_pubs, ar_pub)
            high = copy.deepcopy(default_rates);high['fs150'].update(path=20, ar_path=20, markers=20, scene_path=20)
            rpc('PUT', '/v1/rates', high)
            pump(.2, pose_pubs, ar_pub)
            wait(lambda: any(20 <= len(p.poses) <= 61 for p in paths), 'source history was sampled by low publication cadence')
            path_size = max(len(p.poses) for p in paths)
            require(any(len(m.points) >= 20 for a in markers for m in a.markers if m.type == Marker.LINE_STRIP), 'Marker history lost low-rate source samples')
            require(any(len(line.points) >= 20 for update in scenes for entity in update.entities for line in entity.lines), 'Scene history lost low-rate source samples')
            require(any(p.poses and p.poses[-1].pose.orientation.w == 2 and abs(p.poses[-1].pose.position.z - 3.5) < 1e-9 for p in ar_paths), 'AR raw quaternion/offset-once changed')
            hundred = resources(server.pid)
            require(hundred['Threads'] <= twenty['Threads'] + 2, 'OS threads grew with robot count')
            require(hundred['Threads'] <= baseline['Threads'] + 3, 'unexpected application thread growth')
            require(hundred['VmRSS'] - twenty['VmRSS'] < 96 * 1024, 'representative membership memory growth excessive')
            require(rpc('GET', '/v1/status')['callbackWorkers'] == 2, 'input pool grew with robot count')
            xml = rospy.get_param('/described/robot_description')
            names = [j.attrib['name'] for j in ET.fromstring(xml).findall('joint') if j.attrib.get('type') in ('continuous', 'revolute', 'prismatic')]
            require(names, 'URDF fixture has no movable joints')
            joints = rospy.Publisher('/described/joint_states', JointState, queue_size=1)
            wait(lambda: joints.get_num_connections() > 0, 'in-process joint subscriber missing')
            pump(.2, pose_pubs[:1], ar_pub, (joints, names))
            require(any(t.child_frame_id.startswith('described/') for batch in standard_tf for t in batch.transforms), 'in-process URDF joint TF absent')
            rospy.set_param('/described/visual_robot_description', 'foreign-replacement')
            started = time.monotonic();rpc('DELETE', '/v1/instances/robots100');delete100 = time.monotonic() - started
            require(delete100 < 2, 'DELETE100 exceeded bounded completion')
            require(rospy.get_param('/described/visual_robot_description') == 'foreign-replacement', 'DELETE erased a replaced foreign parameter')
            require(not rospy.has_param('/described/robot_description'), 'owned URDF parameter leaked')
            rpc('DELETE', '/v1/instances/robots100')
            wait(discovery_released, '100 robot discovery did not settle after DELETE')
            for observer in observers:
                observer.unregister()
            path_only = instance(1);path_only['robots'] = [robot('uav101')]
            path_only['settings']['publication']['paths'] = False
            markers.clear();scenes.clear()
            rpc('PUT', '/v1/instances/path-only', path_only)
            observers = observe(paths_enabled=False)
            independent = rospy.Publisher('/uav101/mavros/local_position/pose', PoseStamped, queue_size=1)
            wait(lambda: independent.get_num_connections() > 0, 'independent Marker/Scene path input absent')
            pump(.6, [independent])
            wait(lambda: any(m.ns == 'uav101_actual_path' and len(m.points) >= 5
                             for update in markers for m in update.markers), 'Marker path depended on nav Path enable')
            require(any('uav101' in entity.id and len(line.points) >= 5
                        for update in scenes for entity in update.entities for line in entity.lines),
                    'Scene path depended on nav Path enable')
            _, _, state = xmlrpc.client.ServerProxy(master_uri).getSystemState('/private_probe')
            require(not any(topic == '/uav101/path' for topic, _ in state[0]), 'disabled nav Path was advertised')
            rpc('DELETE', '/v1/instances/path-only')
            wait(discovery_released, 'path-only discovery did not settle after DELETE')
            disabled_ground = instance()
            ground_row = robot('ugv102')
            ground_row['kind'] = 'scout_mini'
            ground_row['visualization'].update(sceneClass='scout', descriptionPackage='scout_description',
                descriptionFile='urdf/scout_visual.urdf')
            disabled_ground['robots'] = [ground_row]
            disabled_ground['settings']['publication']['groundScene'] = False
            ground_source = rospy.Publisher('/ugv102/pose', PoseStamped, queue_size=1)
            status = rpc('PUT', '/v1/instances/ground-disabled', disabled_ground)
            require(status['robotCount'] == 0 and status['descriptionCount'] == 1,
                    'track_ugv false did not separate ground scene from descriptions')
            pump(.2, [ground_source], frame='world')
            require(ground_source.get_num_connections() == 0, 'disabled ground scene subscribed to scientific pose')
            require(rospy.has_param('/ugv102/visual_robot_description'), 'ground flag disabled independent description')
            _, _, state = xmlrpc.client.ServerProxy(master_uri).getSystemState('/private_probe')
            require(not any(topic == '/ugv102/path' for topic, _ in state[0]), 'disabled ground scene advertised path')
            rpc('DELETE', '/v1/instances/ground-disabled')
            require(not rospy.has_param('/ugv102/visual_robot_description'), 'independent ground description leaked')
            rpc('PUT', '/v1/rates', low)
            # Lowest legal rates must not make SIGTERM wait ten seconds.
            started = time.monotonic();server.send_signal(signal.SIGTERM)
            server.wait(timeout=catalog.stop_seconds if catalog else 3)
            stop_seconds = time.monotonic() - started
            require(server.returncode == 0 and stop_seconds < 2, 'low-rate Stop failed')
            require(not os.path.exists(probe_socket), 'owned RPC socket leaked')
            wait(lambda: all(p.get_num_connections() == 1 for p in source_pubs), 'server source subscriptions leaked')
            require(rospy.get_param('/foreign/visual_robot_description') == 'foreign-owned', 'Stop erased foreign parameter')
            def reject_startup(arguments, environment=None, message=None):
                rejected = subprocess.run([binary] + arguments, env=environment,
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=3)
                require(rejected.returncode != 0 and not os.path.exists(probe_socket),
                        'invalid/retired startup input created a provider')
                if message:
                    require(message in rejected.stderr.decode(), 'unexpected startup rejection reason')
            base = ['--bootstrap-input', allocations['bootstrapInput']]
            for arguments in (["_server_instance_id:=old"], ["__log:=" + work + "/unallocated.log"],
                              ["--callback-workers", "2"], ["--world-clock", "wall"],
                              ["--rates-json", "{}"], ["--target-id", "duplicate"],
                              ["--socket", probe_socket], ["--bootstrap-input", allocations['bootstrapInput']],
                              ["--initial-instance-file", retired_initial]):
                reject_startup(base + arguments)
            reject_startup([], message='--bootstrap-input is required')
            missing_allocation = dict(os.environ);missing_allocation.pop('ROS_HOME', None)
            reject_startup(base, environment=missing_allocation, message='ROS_HOME requires')
            bad_input = os.path.join(work, 'invalid-bootstrap.json')
            for change in (lambda d: d['binding'].update(service='other-domain'),
                           lambda d: d['binding'].update(storage_grants=[]),
                           lambda d: d['application'].update(rosLogGrant='unresolved'),
                           lambda d: d['application'].update(callbackWorkers=0),
                           lambda d: d['application'].update(worldClock='guess'),
                           lambda d: d['application'].update(rates={'fs150': {'path': 0}}),
                           lambda d: d['application'].update(robots=[])):
                candidate = json.loads(json.dumps(bootstrap_document));change(candidate)
                write_bootstrap(bad_input, candidate)
                reject_startup(['--bootstrap-input', bad_input])
            write_bootstrap(bad_input, bootstrap_document);os.chmod(bad_input, 0o644)
            reject_startup(['--bootstrap-input', bad_input]);os.chmod(bad_input, 0o600)
            with open(bad_input, 'w') as stream: stream.write(' ' * (16 * 1024 + 1))
            reject_startup(['--bootstrap-input', bad_input])
            symlink_input=os.path.join(work, 'symlink-bootstrap.json')
            os.symlink(allocations['bootstrapInput'], symlink_input)
            reject_startup(['--bootstrap-input', symlink_input])
            fifo_input=os.path.join(work, 'fifo-bootstrap.json');os.mkfifo(fifo_input, 0o600)
            reject_startup(['--bootstrap-input', fifo_input])
            nonprivate_runtime=os.path.join(work, 'nonprivate-runtime');os.mkdir(nonprivate_runtime, 0o750)
            candidate=json.loads(json.dumps(bootstrap_document))
            candidate['binding']['endpoint']['address']=os.path.join(nonprivate_runtime, 'control.sock')
            write_bootstrap(bad_input, candidate)
            reject_startup(['--bootstrap-input', bad_input])
            require(not os.path.exists(candidate['binding']['endpoint']['address']),
                    'nonprivate runtime grant created an endpoint')
            # Stale master parameters and retired files cannot restore native membership.
            rospy.set_param('/xgc2_ros_visualizer/initial_instance_file', retired_initial)
            rospy.set_param('/xgc2_ros_visualizer/server_instance_id', 'old-provider')
            previous_incarnation = instance_id
            restarted = launch_provider()
            wait(lambda: os.path.exists(probe_socket) or restarted.poll() is not None, 'restart socket absent')
            require(restarted.poll() is None, 'restart failed')
            rpc('GET', '/v1/health', expected=409)
            instance_id = None
            fresh = rpc('GET', '/v1/describe')['service_ref']
            if catalog:
                catalog.verify_reference(fresh, allocations)
            instance_id = fresh['instance_id']
            require(instance_id != previous_incarnation, 'restart retained old provider incarnation')
            restored = rpc('GET', '/v1/rates')
            require(restored['appliedRevision'] == 1 and restored['rates'] == default_rates,
                    'ephemeral rate state was falsely persisted')
            require(rpc('GET', '/v1/status')['instanceCount'] == 0, 'restart resurrected old native instances')
            displayed[0].clear()
            crash_viewer = rospy.Subscriber('/xgc/display' + sources[0], rospy.AnyMsg,
                lambda message: displayed[0].append(bytes(message._buff)), queue_size=100)
            rpc('PUT', '/v1/instances/crash-owned', instance(relays=relay_specs[:1], scene=False))
            wait(lambda: source_pubs[0].get_num_connections() >= 2 and crash_viewer.get_num_connections(),
                 'crash fixture did not activate its native relay')
            packets(5)
            wait(lambda: bool(displayed[0]), 'native relay carried no bytes before crash')
            require(all(sample in original[0] for sample in displayed[0]), 'pre-crash relay changed source bytes')
            rpc('PUT', '/v1/rates', low)
            crash_incarnation = instance_id
            restarted.send_signal(signal.SIGKILL);restarted.wait(timeout=3)
            require(restarted.returncode == -signal.SIGKILL, 'crash fixture did not kill its owned provider')
            require(os.path.exists(probe_socket), 'crash fixture did not retain the stale socket')
            wait(lambda: source_pubs[0].get_num_connections() == 1 and not crash_viewer.get_num_connections(),
                 'killed native relay retained live data connections')
            recovered = launch_provider()
            instance_id = None
            recovery_reference = None
            def recovery_ready():
                nonlocal recovery_reference
                require(recovered.poll() is None, 'crash replacement exited')
                try:
                    recovery_reference = rpc('GET', '/v1/describe')['service_ref']
                    return True
                except (OSError, http.client.HTTPException):
                    return False
            wait(recovery_ready, 'crash replacement did not acquire the stale endpoint')
            instance_id = crash_incarnation
            rpc('GET', '/v1/health', expected=409)
            instance_id = recovery_reference['instance_id']
            if catalog:
                catalog.verify_reference(recovery_reference, allocations)
            require(instance_id != crash_incarnation, 'crash replacement reused provider incarnation')
            require(rpc('GET', '/v1/status')['instanceCount'] == 0, 'crash recovery resurrected native membership')
            restored = rpc('GET', '/v1/rates')
            require(restored['appliedRevision'] == 1 and restored['rates'] == default_rates,
                    'crash recovery imported ephemeral rate state')
            require(source_pubs[0].get_num_connections() == 1 and not crash_viewer.get_num_connections(),
                    'replacement silently reactivated the old native relay')
            displayed[0].clear()
            rpc('PUT', '/v1/instances/recovered-relay', instance(relays=relay_specs[:1], scene=False))
            wait(lambda: source_pubs[0].get_num_connections() >= 2 and crash_viewer.get_num_connections(),
                 'replacement failed explicit native relay activation')
            time.sleep(.1)
            require(not displayed[0], 'replacement replayed a source sample from the killed provider')
            packets(5)
            wait(lambda: bool(displayed[0]), 'replacement native relay did not deliver fresh bytes')
            require(all(sample in original[0] for sample in displayed[0]), 'replacement relay changed fresh bytes')
            rpc('DELETE', '/v1/instances/recovered-relay')
            wait(lambda: source_pubs[0].get_num_connections() == 1 and not crash_viewer.get_num_connections(),
                 'replacement DELETE did not fence its native relay')
            crash_viewer.unregister()
            recovered.send_signal(signal.SIGTERM)
            recovered.wait(timeout=catalog.stop_seconds if catalog else 3)
            require(recovered.returncode == 0 and not os.path.exists(probe_socket),
                    'crash replacement failed native operation or endpoint cleanup')
            print(json.dumps(dict(ok=True, scope='private ROS graph; not station/scientific acceptance',
                relay_received=relay_counts, robots20=twenty, robots100=hundred,
                source_history_points=path_size, simulated_rtf=3, delete20_seconds=delete20,
                delete100_seconds=delete100, low_rate_stop_seconds=stop_seconds,
                zero_robot_readiness=True, explicit_activation=True, retired_initial_file_rejected=True,
                retired_wire_envelopes_rejected=True, fixed_callback_workers=2, publisher_workers=1,
                restart_incarnation_fenced=True, rate_cas_conflict=True, ephemeral_restart=True,
                sigkill_stale_endpoint_recovered=True, crash_incarnation_fenced=True,
                sigkill_native_relay_fenced=True, crash_relay_no_replay=True,
                launch_source='catalog' if catalog else 'direct native fixture',
                shared_bootstrap_loader=True, single_startup_binding=True,
                startup_partial_rates=True, runtime_full_table_required=True,
                catalog_sha256=catalog.sha256 if catalog else None,
                no_child_processes=True), sort_keys=True))
            rospy.signal_shutdown('private probe complete')
        except Exception:
            logfile.flush()
            print('PRIVATE PROCESS STATES: ' + repr([(p.pid, p.poll()) for p in processes]), file=sys.stderr)
            if 'paths' in locals():
                print('PRIVATE PATH SIZES: ' + repr([len(p.poses) for p in paths]), file=sys.stderr)
            if 'pose_pubs' in locals():
                print('PRIVATE MISSING INPUTS: ' + repr([i + 1 for i, p in enumerate(pose_pubs) if not p.get_num_connections()]), file=sys.stderr)
            if 'observers' in locals():
                print('PRIVATE OBSERVER CONNECTIONS: ' + repr([(p.resolved_name, p.get_num_connections()) for p in observers]), file=sys.stderr)
            if 'rpc' in locals() and 'server' in locals() and server.poll() is None:
                try:
                    _, _, state = xmlrpc.client.ServerProxy(master_uri).getSystemState('/private_probe')
                    owned = {label: [(topic, nodes) for topic, nodes in entries if '/xgc2_ros_visualizer' in nodes]
                             for label, entries in zip(('publishers', 'subscribers'), state[:2])}
                    print('PRIVATE SERVER REGISTRY: ' + json.dumps(owned), file=sys.stderr)
                    print('PRIVATE SERVER STATUS: ' + json.dumps(rpc('GET', '/v1/status')), file=sys.stderr)
                except Exception as error:
                    print('PRIVATE FAILURE READBACK: ' + repr(error), file=sys.stderr)
            with open(logfile.name) as stream:
                print('PRIVATE PROCESS LOG (last 5000 chars):\n' + stream.read()[-5000:], file=sys.stderr)
            raise
        finally:
            for process in reversed(processes):
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGTERM)
                    try:
                        process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL);process.wait(timeout=2)
            logfile.close()


if __name__ == '__main__':
    require(len(sys.argv) in (2, 3), 'expected actual server executable and optional external catalog')
    main(os.path.abspath(sys.argv[1]), sys.argv[2] if len(sys.argv) == 3 else None)
