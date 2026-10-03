import io
import json
import math
import os
import pathlib
import socket
import subprocess
import sys
import threading
import time
import xmlrpc.client

binary = sys.argv[1]
socket.setdefaulttimeout(2)
result = {"installedExecutable": binary}
environment = dict(os.environ, ROS_MASTER_URI="http://127.0.0.1:11317", ROS_IP="127.0.0.1")

def require(condition, message):
    if not condition:
        raise AssertionError(message)

def until(predicate, timeout=6):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            if predicate():
                return True
        except (OSError, xmlrpc.client.Error):
            pass
        time.sleep(0.02)
    return False

empty = subprocess.Popen([binary, "[]"], env=dict(environment, ROS_MASTER_URI="http://127.0.0.1:1"),
                         stdout=subprocess.PIPE, stderr=subprocess.PIPE)
try:
    time.sleep(0.2)
    require(empty.poll() is None, "empty relay owner exited before Stop")
    sockets = [os.readlink(fd) for fd in pathlib.Path("/proc/{}/fd".format(empty.pid)).iterdir()]
    require(not any(target.startswith("socket:") for target in sockets), "empty owner opened a network socket")
    empty.terminate()
    require(empty.wait(timeout=3) == 0, "empty owner failed Stop")
    result["emptyNoROSNoSocketStop"] = True
finally:
    if empty.poll() is None:
        empty.kill()
        empty.wait()

master_log = open("/tmp/xgc-ros-display-validation-master.log", "w")
master = subprocess.Popen(["roscore", "-p", "11317"], env=environment, stdout=master_log, stderr=subprocess.STDOUT)
node = None
try:
    proxy = xmlrpc.client.ServerProxy(environment["ROS_MASTER_URI"])
    require(until(lambda: proxy.getSystemState("/display_install_probe")[0] == 1), "private ROS master unavailable")
    os.environ.update(environment)
    import rospy
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import OccupancyGrid, Path
    from geometry_msgs.msg import PoseArray
    rospy.init_node("display_install_probe", disable_signals=True)
    types = {"sensor_msgs/PointCloud2": PointCloud2, "nav_msgs/OccupancyGrid": OccupancyGrid,
             "nav_msgs/Path": Path, "geometry_msgs/PoseArray": PoseArray}
    specs = [{"source": "/installed_probe_" + str(index), "topic": "/xgc/display/installed_probe_" + str(index),
              "messageType": name, "maxRateHz": 5} for index, name in enumerate(types)]
    publishers = [rospy.Publisher(spec["source"], types[spec["messageType"]], queue_size=1) for spec in specs]
    node = subprocess.Popen([binary, json.dumps(specs)], env=environment, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    require(until(lambda: all(spec["topic"] in dict(proxy.getTopicTypes("/display_install_probe")[2]) for spec in specs)), "copies were not declared before source messages")
    require(all(pub.get_num_connections() == 0 for pub in publishers), "declaration subscribed an unwatched source")
    originals = [set() for _ in specs]
    copies = [[] for _ in specs]
    lock = threading.Lock()
    def original(index):
        def receive(message):
            with lock:
                originals[index].add(bytes(message._buff))
        return receive
    def copied(index):
        def receive(message):
            with lock:
                copies[index].append(bytes(message._buff))
        return receive
    watchers = [rospy.Subscriber(spec["topic"], rospy.AnyMsg, copied(index), queue_size=1) for index, spec in enumerate(specs)]
    require(until(lambda: all(pub.get_num_connections() == 1 for pub in publishers)), "display viewer did not subscribe sources")
    recorders = [rospy.Subscriber(spec["source"], rospy.AnyMsg, original(index), queue_size=1) for index, spec in enumerate(specs)]
    require(until(lambda: all(pub.get_num_connections() == 2 for pub in publishers)), "independent source recorders unavailable")
    messages = []
    for spec in specs:
        message = types[spec["messageType"]]()
        message.header.stamp = rospy.Time(123, 456)
        message.header.frame_id = "installed_original_frame"
        if spec["messageType"] == "sensor_msgs/PointCloud2":
            message.height, message.width, message.point_step, message.row_step = 1, 2, 4, 8
            message.data = bytes([0, 255, 127, 128, 1, 2, 3, 4])
        elif spec["messageType"] == "nav_msgs/OccupancyGrid":
            message.info.height, message.info.width, message.info.resolution = 1, 3, 0.05
            message.data = [-1, 0, 100]
        messages.append(message)
    started = time.monotonic()
    for _ in range(80):
        for pub, message in zip(publishers, messages):
            pub.publish(message)
        time.sleep(0.01)
    elapsed = time.monotonic() - started
    require(until(lambda: all(copies)), "one declared type forwarded no data")
    time.sleep(0.1)
    counts = []
    with lock:
        for spec, source_packets, forwarded in zip(specs, originals, copies):
            require(all(packet in source_packets for packet in forwarded), "serialized source bytes changed: " + spec["messageType"])
            require(len(forwarded) <= math.ceil(elapsed * 5) + 1, "copy budget exceeded")
            require(len(source_packets) > len(forwarded), "source recording was display limited")
            counts.append({"messageType": spec["messageType"], "sourcePackets": len(source_packets), "copyPackets": len(forwarded)})
    for watcher in watchers:
        watcher.unregister()
    require(until(lambda: all(pub.get_num_connections() == 1 for pub in publishers)), "last display disconnect retained upstream")
    watchers = [rospy.Subscriber(spec["topic"], rospy.AnyMsg, copied(index), queue_size=1) for index, spec in enumerate(specs)]
    require(until(lambda: all(pub.get_num_connections() == 2 for pub in publishers)), "reconnect failed to resubscribe sources")
    node.terminate()
    stdout, stderr = node.communicate(timeout=5)
    require(node.returncode == 0, "installed node Stop failed: " + stderr.decode())
    require(until(lambda: all(pub.get_num_connections() == 1 for pub in publishers)), "Stop left relay source connections")
    with lock:
        before = [len(items) for items in originals]
        copied_before = [len(items) for items in copies]
    for pub, message in zip(publishers, messages):
        pub.publish(message)
    require(until(lambda: all(len(items) > previous for items, previous in zip(originals, before))), "Stop disrupted original recorders")
    time.sleep(0.1)
    with lock:
        require([len(items) for items in copies] == copied_before, "copy continued after Stop")
    result.update({"declaredBeforeSource": True, "noViewerNoSource": True,
                   "fourTypesRawBytesAndCopiesOnlyRate": counts, "disconnectReconnectStop": True,
                   "sourceRecorderSurvivesStop": True})
    pathlib.Path("/tmp/xgc-ros-display-installed-receipt.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
finally:
    if node is not None and node.poll() is None:
        node.terminate()
        try:
            node.wait(timeout=5)
        except subprocess.TimeoutExpired:
            node.kill()
            node.wait()
    if "rospy" in globals():
        rospy.signal_shutdown("installed probe complete")
    master.terminate()
    try:
        master.wait(timeout=5)
    except subprocess.TimeoutExpired:
        master.kill()
        master.wait()
    master_log.close()
