"""Physical coordinate checks independent of ONNX network composition."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
import mujoco
import numpy as np

PARKOUR = Path(__file__).resolve().parents[1]
ROOT = PARKOUR.parents[1]
sys.path.insert(0, str(PARKOUR))
from mujoco_adapter import MujocoAdapter


class JointMappingTest(unittest.TestCase):
    def setUp(self):
        cfg = json.loads((PARKOUR / 'g1.json').read_text())
        self.params = SimpleNamespace(config=cfg, joint_names=cfg['joint_names'],
            default=np.array(cfg['default_joint_pos']), effort=np.array(cfg['effort_limit']),
            kp=np.array(cfg['kp']), kd=np.array(cfg['kd']))
        self.model = mujoco.MjModel.from_xml_path(str(ROOT / 'unitree_robots/g1/scene_29dof.xml'))
        self.data = mujoco.MjData(self.model)
        self.adapter = MujocoAdapter(self.model, self.data, self.params, configure_dynamics=False)

    def test_reversed_waist_chain_kinematics(self):
        # Training URDF: torso -> pitch(+Y) -> roll(+X) -> yaw(+Z) -> pelvis.
        # MuJoCo: pelvis -> yaw(+Z) -> roll(+X) -> pitch(+Y) -> torso.
        qy, qr, qp = .17, -.11, .09
        for name, q in [('waist_yaw_joint', qy), ('waist_roll_joint', qr), ('waist_pitch_joint', qp)]:
            self.data.qpos[self.model.joint(name).qposadr] = q
        mujoco.mj_forward(self.model, self.data)
        policy_q, *_ = self.adapter.observation([0,0,0])
        expected = {'waist_yaw_joint': -qy, 'waist_roll_joint': -qr, 'waist_pitch_joint': -qp}
        for name, q in expected.items():
            self.assertAlmostEqual(policy_q[self.params.joint_names.index(name)], q)
        def rotation(axis, angle):
            quat = np.zeros(4);mat = np.zeros(9)
            mujoco.mju_axisAngle2Quat(quat, np.array(axis, dtype=float), angle)
            mujoco.mju_quat2Mat(mat, quat)
            return mat.reshape(3,3)
        torso_to_pelvis = rotation([0,1,0],-qp) @ rotation([1,0,0],-qr) @ rotation([0,0,1],-qy)
        r_torso = self.data.xmat[self.adapter.torso].reshape(3,3)
        r_pelvis = self.data.xmat[self.model.body('pelvis').id].reshape(3,3)
        np.testing.assert_allclose(r_torso.T @ r_pelvis, torso_to_pelvis, atol=1e-12)

    def test_velocity_and_target_signs(self):
        a = self.adapter
        for i, name in enumerate(self.params.joint_names):
            self.data.qvel[a.vadr[i]] = .1
        mujoco.mj_forward(self.model, self.data)
        _, velocity, *_ = a.observation([0,0,0])
        a.target = np.full(29, .1)
        self.data.qvel[:] = 0
        a.control()
        for i, name in enumerate(self.params.joint_names):
            sign = -1 if name.startswith('waist_') else 1
            self.assertAlmostEqual(velocity[i], sign * .1)
            self.assertEqual(np.sign(self.data.ctrl[a.actuators[i]]), sign)


if __name__ == '__main__':
    unittest.main()
