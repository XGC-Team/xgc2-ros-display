#!/usr/bin/env python3
"""Exercise the real node entry point before any ROS master is available."""
import os
import pathlib
import subprocess
import sys
import time
binary = sys.argv[1]
environment = dict(os.environ, ROS_MASTER_URI="http://127.0.0.1:1", ROS_IP="127.0.0.1")
for arguments in ([], ["[]", "/unused/legacy"], ["[]", "extra", "extra"], ["not-json"]):
    result = subprocess.run([binary] + arguments, env=environment, capture_output=True, timeout=3)
    assert result.returncode == 1, (arguments, result.returncode, result.stderr)
    if len(arguments) != 1:
        assert b"usage: xgc2_display_relays RELAYS_JSON" in result.stderr, result.stderr
process = subprocess.Popen([binary, "[]"], env=environment, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
try:
    time.sleep(0.2)
    assert process.poll() is None, "empty layout did not retain its owner"
    descriptors = pathlib.Path("/proc/{}/fd".format(process.pid)).iterdir()
    assert not any(os.readlink(fd).startswith("socket:") for fd in descriptors), "empty layout opened a ROS socket"
    process.terminate()
    assert process.wait(timeout=3) == 0, "empty owner failed Stop"
finally:
    if process.poll() is None:
        process.kill()
        process.wait()
print("node argv rejection and empty no-ROS Stop passed")
