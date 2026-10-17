"""Existing Python Unitree bridge -> synchronized observations -> C++ ONNX -> DDS PD."""
import argparse
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
import numpy as np
import mujoco

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'simulate_python'))
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py_bridge import UnitreeSdk2Bridge
from parkour_observation import ParkourObservationPublisher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--stale', action='store_true', help='stop observations and check C++ watchdog')
parser.add_argument('--ground', action='store_true', help='release base and check zero-command standing')
parser.add_argument('--stand-seconds', type=float, default=8., help='ground standing duration in wall seconds')
args = parser.parse_args()
if args.stale and args.ground:
    parser.error('--stale and --ground are separate checks')
if not np.isfinite(args.stand_seconds) or args.stand_seconds <= 0:
    parser.error('--stand-seconds must be positive and finite')
# Let the OS allocate an isolated UDP port for the test.
with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
    probe.bind(('127.0.0.1', 0))
    port = probe.getsockname()[1]
model = mujoco.MjModel.from_xml_path(str(ROOT / 'unitree_robots/g1/scene_29dof.xml'))
model.opt.timestep = .002
data = mujoco.MjData(model)
ChannelFactoryInitialize(192, 'lo')
bridge = UnitreeSdk2Bridge(model, data)
observations = ParkourObservationPublisher(model, data, port)
log = tempfile.TemporaryFile(mode='w+t')
controller = subprocess.Popen([str(ROOT / 'example/cpp/build/g1_parkour'), '--domain', '192',
                               '--port', str(port), '--reset-seconds', '.5'],
                              stdin=subprocess.PIPE, stdout=log, stderr=subprocess.STDOUT)
events = [(1., b'p'), (1.1,b'r'), (1.2,b'9'), (1.3,b'p'), (2.6,b'r'),
          (2.8,b'w'), (2.9,b'q'), (3.1,b'x'), (3.3,b'p'), (4.5,b'r'), (5.,b' ')]
if args.ground:
    events = [(1.2,b'9'), (1.3,b'p'), (2.6,b'r'), (2.7+args.stand_seconds,b' ')]
started = time.monotonic()
policy_changed = False
grounded = False
min_height = float('inf')
max_drift = 0.
ground_start = None
try:
    while controller.poll() is None and time.monotonic()-started < (6+args.stand_seconds if args.ground else 8):
        before = time.monotonic()
        elapsed = before-started
        while events and elapsed >= events[0][0]:
            _, key = events.pop(0)
            controller.stdin.write(key);controller.stdin.flush()
        if not args.ground or elapsed < 2.7:
            data.qpos[:7] = [0,0,1.2,1,0,0,0]
            data.qvel[:6] = 0
        elif not grounded:
            data.qpos[:7] = [0,0,.78,1,0,0,0]
            data.qvel[:6] = 0
            grounded = True
            ground_start = data.time
        mujoco.mj_forward(model,data)
        if not args.stale or elapsed < 4.7:
            observations.update()
        bridge.UpdateMotorControl()
        adapter = observations.adapter
        data.ctrl[adapter.actuators] = np.clip(data.ctrl[adapter.actuators], -adapter.limits, adapter.limits)
        mujoco.mj_step(model,data)
        if args.ground and grounded:
            min_height = min(min_height, float(data.xpos[adapter.torso,2]))
            max_drift = max(max_drift, float(np.linalg.norm(data.qpos[:2])))
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.ctrl).all():
            raise RuntimeError('Non-finite simulation state')
        if elapsed < 1.3 and np.any(bridge.motor_command[2:]):
            raise RuntimeError('Commands started before ordered enable/reset')
        if 2.7 < elapsed < 3.2:
            neutral = np.empty(29)
            neutral[adapter.actuators] = adapter.policy.default
            policy_changed |= np.max(np.abs(bridge.motor_command[0]-neutral)) > .01
        time.sleep(max(0, .002-(time.monotonic()-before)))
    controller.wait(timeout=2)
    time.sleep(.05)
    log.seek(0);output=log.read();print(output)
    expected = 1 if args.stale else 0
    if controller.returncode != expected:
        raise RuntimeError(f'Unexpected exit code {controller.returncode}, expected {expected}')
    assert output.count('Parkour policy ON.') == (1 if args.ground else 2), 'Policy entry failed'
    assert policy_changed, 'C++ policy targets did not change'
    assert not np.any(bridge.motor_command[2:]), 'Exit did not clear control output'
    if args.stale:
        assert 'stale; stopping' in output, 'Watchdog was not triggered'
    if args.ground:
        assert min_height > .65 and max_drift < .5, (min_height, max_drift)
        print(f'Zero-command ground standing: {data.time-ground_start:.3f}s simulated, minimum torso height={min_height:.3f}m, maximum XY drift={max_drift:.3f}m')
    mode = 'zero-command standing' if args.ground else ('stale watchdog' if args.stale else 'reset and reentry')
    print('PASS: Python simulator / UDP / C++ double ONNX / DDS / PD / ' + mode)
finally:
    if controller.poll() is None:
        controller.terminate()
        try:controller.wait(timeout=3)
        except subprocess.TimeoutExpired:controller.kill();controller.wait()
    observations.close()
