"""Compare native UDP/depth/history/ONNX numerically against Python on flat ground."""
from pathlib import Path
import subprocess
import sys
import tempfile
import numpy as np
import mujoco

root = Path(sys.argv[2])
sys.path.insert(0, str(root / 'example/parkour'))
from policy import ParkourPolicy
from mujoco_adapter import MujocoAdapter

policy = ParkourPolicy()
model = mujoco.MjModel.from_xml_path(str(root / 'unitree_robots/g1/scene_29dof_flat.xml'))
data = mujoco.MjData(model)
adapter = MujocoAdapter(model, data, policy, configure_dynamics=False)
errors = dict(state=0., depth=0., proprio=0., images=0., targets=0.)

with tempfile.TemporaryDirectory(prefix='g1-parkour-reference-') as directory:
    path = Path(directory) / 'samples.bin'
    subprocess.run([sys.argv[1], str(root), str(path)], check=True)
    with path.open('rb') as stream:
        def read(dtype, count):
            result = np.frombuffer(stream.read(np.dtype(dtype).itemsize * count), dtype=dtype)
            assert result.size == count
            return result

        def compare(name, actual, expected, atol):
            assert np.array_equal(np.isinf(actual), np.isinf(expected)), name
            finite = np.isfinite(expected)
            difference = np.max(np.abs(actual[finite] - expected[finite]))
            errors[name] = max(errors[name], float(difference))
            np.testing.assert_allclose(actual, expected, atol=atol, rtol=1e-5, err_msg=name)

        for t in range(40):
            data.qpos[:] = read('<f8', model.nq)
            data.qvel[:] = read('<f8', model.nv)
            header = stream.read(32)
            assert header[:8] == b'G1PKR001'
            assert int.from_bytes(header[8:16], 'little') == t
            values = read('<f4', 2369)
            cpp_proprio = read('<f4', 768).reshape(1, 768)
            cpp_images = read('<f4', 4608).reshape(1, 8, 18, 32)
            cpp_target = read('<f4', 29)
            mujoco.mj_forward(model, data)
            q, dq, omega, gravity, command, depth = adapter.observation(np.array([.3, 0, .1], np.float32))
            compare('state', values[:65], np.concatenate([q, dq, omega, gravity, [data.xpos[adapter.torso, 2]]]), 1e-6)
            compare('depth', values[65:].reshape(36, 64), depth, 2e-5)
            proprio, images = policy.observe(q, dq, omega, gravity, command, depth)
            compare('proprio', cpp_proprio, proprio, 3e-5)
            compare('images', cpp_images, images, 2e-5)
            target = policy.infer(proprio, images)
            # Native output is DDS order and MuJoCo sign.
            dds_joint_ids = model.actuator_trnid[:, 0]
            joint_ids = [model.joint(name).id for name in policy.joint_names]
            ids = [int(np.flatnonzero(dds_joint_ids == joint)[0]) for joint in joint_ids]
            compare('targets', cpp_target[ids] * adapter.sign, target, 2e-4)
        assert not stream.read(1)
print('PASS 40 flat-scene frames: native UDP + ray depth + history + encoder/actor; max errors:', errors)
