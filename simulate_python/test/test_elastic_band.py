"""Suspension behavior without DDS, pygame or a viewer."""
from pathlib import Path
import sys
import unittest
import json
from types import SimpleNamespace

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from elastic_band import ElasticBand
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'example/parkour'))
from mujoco_adapter import MujocoAdapter


class ElasticBandTest(unittest.TestCase):
    def setUp(self):
        root = Path(__file__).resolve().parents[2]
        self.model = mujoco.MjModel.from_xml_path(str(root / 'unitree_robots/g1/scene_29dof_flat.xml'))
        self.data = mujoco.MjData(self.model)
        self.body = self.model.body('torso_link').id
        self.band = ElasticBand()
        mujoco.mj_forward(self.model, self.data)

    def test_waist_motion_is_damped_at_torso_com(self):
        waist = self.model.joint('waist_pitch_joint').dofadr[0]
        self.data.qvel[waist] = 1.
        mujoco.mj_forward(self.model, self.data)
        self.band.point = self.data.xipos[self.body] + [1., 0., 0.]
        self.band.length = 1.
        self.band.Apply(self.model, self.data, self.body)
        velocity = np.zeros(6)
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY,
                                self.body, velocity, 0)
        self.assertGreater(abs(velocity[3]), .1)
        force = self.data.xfrc_applied[self.body, :3]
        np.testing.assert_allclose(force, [-self.band.damping * velocity[3], 0., 0.], atol=1e-10)
        self.assertLess(np.dot(force, velocity[3:]), 0.)

    def test_coincident_anchor_is_finite(self):
        self.band.point = self.data.xipos[self.body].copy()
        self.band.Apply(self.model, self.data, self.body)
        np.testing.assert_array_equal(self.data.xfrc_applied[self.body, :3], 0.)

    def test_release_and_reattach_after_translation(self):
        self.band.Apply(self.model, self.data, self.body)
        self.band.MujuocoKeyCallback(ord('9'))
        self.band.Apply(self.model, self.data, self.body)
        self.assertFalse(self.band.enable)
        np.testing.assert_array_equal(self.data.xfrc_applied[self.body, :3], 0.)
        self.data.qpos[:2] = [5., -3.]
        mujoco.mj_forward(self.model, self.data)
        self.band.MujuocoKeyCallback(ord('9'))
        self.band.Apply(self.model, self.data, self.body)
        self.assertTrue(self.band.enable)
        np.testing.assert_allclose(self.band.point[:2], self.data.xipos[self.body, :2])
        force = self.data.xfrc_applied[self.body, :3]
        np.testing.assert_allclose(force[:2], 0., atol=1e-10)
        self.assertAlmostEqual(force[2], mujoco.mj_getTotalmass(self.model) * 9.81)

    def test_keys_raise_and_lower(self):
        self.band.length = 1.
        self.band.MujuocoKeyCallback(ord('7'))
        self.band.Apply(self.model, self.data, self.body)
        self.assertAlmostEqual(self.band.length, .9)
        self.band.MujuocoKeyCallback(ord('8'))
        self.band.Apply(self.model, self.data, self.body)
        self.assertAlmostEqual(self.band.length, 1.)
        for _ in range(30):
            self.band.MujuocoKeyCallback(ord('7'))
        self.band.Apply(self.model, self.data, self.body)
        self.assertAlmostEqual(self.band.length, -2.)

    def test_unpowered_visual_dynamics(self):
        root = Path(__file__).resolve().parents[2]
        for task in ('g1_parkour/g1_torsobase_cfg.json', 'g1_test/g1_ca_pe_amp_vae_cfg.json'):
            with self.subTest(task=task):
                cfg = json.loads((root / 'example/exported' / task).read_text())
                model = mujoco.MjModel.from_xml_path(str(root / 'unitree_robots/g1/scene_29dof_flat.xml'))
                model.opt.timestep = cfg['physics_dt']
                data = mujoco.MjData(model)
                params = SimpleNamespace(config=cfg, joint_names=cfg['joint_names'],
                                         default=np.array(cfg['default_joint_pos']),
                                         effort=np.array(cfg['effort_limit']))
                damping, friction = model.dof_damping.copy(), model.dof_frictionloss.copy()
                adapter = MujocoAdapter(model, data, params)
                adapter.set_motor_control_active(False)
                np.testing.assert_array_equal(model.dof_damping, damping)
                np.testing.assert_array_equal(model.dof_frictionloss, friction)
                band = ElasticBand()
                peak = 0.
                for _ in range(round(20. / model.opt.timestep)):
                    mujoco.mj_forward(model, data)
                    band.Apply(model, data, adapter.torso)
                    mujoco.mj_step(model, data)
                    if data.time > 10.:
                        peak = max(peak, np.max(np.abs(data.qvel[6:])))
                np.testing.assert_array_equal(data.ctrl, 0.)
                self.assertLess(peak, .05)
                adapter.set_motor_control_active(True)
                np.testing.assert_array_equal(model.dof_damping[adapter.vadr], 0.)
                np.testing.assert_array_equal(model.dof_frictionloss[adapter.vadr], 0.)
                adapter.set_motor_control_active(False)
                np.testing.assert_array_equal(model.dof_damping, damping)
                np.testing.assert_array_equal(model.dof_frictionloss, friction)


if __name__ == '__main__':
    unittest.main()
