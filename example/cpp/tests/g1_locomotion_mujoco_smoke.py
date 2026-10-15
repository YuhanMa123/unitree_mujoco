"""Uses MuJoCo and unitree_sdk2py only; no viewer or Isaac runtime.

The free base is held in place to isolate command transitions from balance.
With --ground, lower it onto the floor after policy entry and test walking.
Run without other participants on the selected DDS domain.
"""
import argparse
import sys
import time
import subprocess
import tempfile
from pathlib import Path
import numpy as np
import mujoco

root = Path(__file__).resolve().parents[3]
parser = argparse.ArgumentParser(description='Headless suspended-base G1 reset/locomotion DDS integration check')
parser.add_argument('--domain', type=int, default=191)
parser.add_argument('--ground', action='store_true')
args = parser.parse_args()
if args.domain <= 0:
    parser.error('Use an isolated positive DDS domain')
sys.path.insert(0, str(root / 'simulate_python'))
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py_bridge import UnitreeSdk2Bridge

model = mujoco.MjModel.from_xml_path(str(root / 'unitree_robots/g1/scene_29dof.xml'))
model.opt.timestep = 0.002
data = mujoco.MjData(model)
data.qpos[:7] = [0, 0, 1.2, 1, 0, 0, 0]
mujoco.mj_forward(model, data)
ChannelFactoryInitialize(args.domain, 'lo')
bridge = UnitreeSdk2Bridge(model, data)
log = tempfile.TemporaryFile(mode='w+t')
controller = subprocess.Popen(['stdbuf', '-oL', str(root/'example/cpp/build/g1_locomotion'), '0.5', 'lo', str(args.domain)], stdin=subprocess.PIPE, stdout=log, stderr=subprocess.STDOUT)
start = time.monotonic()
# Check ordered transitions, power-on lock, and that D/Q/E are motion keys.
events = [(1.0, b'p'), (1.1, b'r'), (1.2, b'9'), (1.3, b'r'),
          (1.9, b'p'), (2.0, b'r'), (2.1, b'9'), (3.6, b'r'), (3.9, b'9'),
          (4.0, b'a'), (4.1, b'd'), (4.2, b'w'), (4.3, b's'),
          (4.4, b'q'), (4.5, b'e'), (4.6, b'x'), (5.0, b'p'), (6.1, b'r'),
          (6.5, b'd'), (6.6, b'q'), (6.7, b'9'), (7.4, b' ')]
if args.ground:
    events = [(1.0, b'p'), (1.1, b'r'), (1.2, b'9'), (1.3, b'r'),
              (1.9, b'p'), (2.0, b'r'), (2.1, b'9'), (3.6, b'r'), (3.9, b'9'),
              (5.0, b'w'), (5.1, b'w'), (5.2, b'w'), (9.0, b'x'),
              (12.0, b'x'), (12.2, b'd'), (12.3, b'q'), (12.4, b'e'), (12.5, b'9'), (12.6, b' ')]
max_error = 0.0
grounded = False
minimum_height = 10.0
walk_start_x = None
walk_end_x = None
try:
    while controller.poll() is None and time.monotonic()-start < (15 if args.ground else 10):
        step = time.monotonic()
        elapsed = step-start
        while events and elapsed >= events[0][0]:
            _, key = events.pop(0)
            controller.stdin.write(key); controller.stdin.flush()
        # Suspend the free base to test all control transitions without a viewer.
        if not args.ground or elapsed < 3.7:
            data.qpos[:7] = [0, 0, 1.2, 1, 0, 0, 0]
            data.qvel[:6] = 0
        elif not grounded:
            data.qpos[:7] = [0, 0, 0.79, 1, 0, 0, 0]
            data.qvel[:6] = 0
            grounded = True
        if args.ground and 4.5 < elapsed < 12.0:
            minimum_height = min(minimum_height, float(data.qpos[2]))
            if data.qpos[2] < 0.35:
                raise RuntimeError(f'Ground policy fell: height={data.qpos[2]}')
        if args.ground and 5.0 <= elapsed <= 9.0:
            if walk_start_x is None:
                walk_start_x = float(data.qpos[0])
            walk_end_x = float(data.qpos[0])
        bridge.UpdateMotorControl()
        mujoco.mj_step(model, data)
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.ctrl).all():
            raise RuntimeError('Non-finite simulation state')
        if 3.3 < elapsed < 3.5:
            max_error = max(max_error, float(np.max(np.abs(data.qpos[7:] - bridge.motor_command[0]))))
        if elapsed < 1.85 and np.any(bridge.motor_command[2:]):
            raise RuntimeError('Command output started before ordered reset')
        time.sleep(max(0, 0.002-(time.monotonic()-step)))
    controller.wait(timeout=3)
    time.sleep(0.05)
    assert not np.any(bridge.motor_command[2:]), "Exit did not clear kp/kd/tau"
    log.flush()
    log.seek(0)
    output = log.read()
    print(output)
    assert controller.returncode == 0, controller.returncode
    assert output.count('Locomotion policy ON') == (1 if args.ground else 2)
    assert output.count('Policy requires power-on and completed reset') >= 3
    assert output.count('Reset requires power-on.') == 1
    assert output.count('Power-on locked after reset/policy entry') == 3
    assert output.count('Command output ENABLED.') == 1
    assert output.count('Reset target generated') == (1 if args.ground else 2)
    assert 'Velocity: 0.1, 0, 0' in output
    assert 'Velocity: 0, -0.1, 0' in output
    assert 'Velocity: 0, -0.1, 0.1' in output
    if not args.ground:
        assert 'Velocity: 0, 0.1, 0' in output
        assert 'Velocity: 0, 0, 0.1' in output
    assert max_error < 0.35, max_error
    if args.ground:
        distance = walk_end_x - walk_start_x
        print('Minimum enabled base height:', minimum_height, 'Forward distance:', distance)
        assert distance > 0.05, f'No forward progress at 0.3 m/s: {distance}'
    print('HEADLESS DDS/MUJOCO PASS; reset max error', max_error)
finally:
    if controller.poll() is None:
        controller.terminate(); controller.wait(timeout=3)
    for thread in (bridge.lowStateThread, bridge.HighStateThread, bridge.WirelessControllerThread):
        thread.Wait(1)
    log.close()
