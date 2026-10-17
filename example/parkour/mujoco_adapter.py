"""State and ray depth access for the running MuJoCo scene; no ONNX dependency."""
import numpy as np
import mujoco

class MujocoAdapter:
    def __init__(self, model, data, policy, configure_dynamics=True):
        self.model, self.data, self.policy = model, data, policy
        self.sign = np.asarray(policy.config['joint_sign'], dtype=float)
        if self.sign.shape != (29,) or not np.isin(self.sign, [-1, 1]).all():
            raise ValueError('joint_sign must contain 29 values in {-1, 1}')
        self.torso = model.body('torso_link').id
        joints = np.array([model.joint(n).id for n in policy.joint_names])
        self.qadr = model.jnt_qposadr[joints]
        self.vadr = model.jnt_dofadr[joints]
        self.actuators = []
        for joint in joints:
            ids = np.flatnonzero((model.actuator_trnid[:, 0] == joint) &
                                (model.actuator_trntype == mujoco.mjtTrn.mjTRN_JOINT))
            if len(ids) != 1:
                raise ValueError(f'expected one motor for {model.joint(joint).name}')
            actuator = ids[0]
            if not np.allclose(model.actuator_gear[actuator], [1, 0, 0, 0, 0, 0]):
                raise ValueError('only direct unit-gear torque motors are supported')
            self.actuators.append(actuator)
        self.actuators = np.array(self.actuators)
        if configure_dynamics:
            model.dof_armature[self.vadr] = policy.config['armature']
            # Training PD already supplies damping; remove extra passive motor damping/friction.
            model.dof_damping[self.vadr] = 0
            model.dof_frictionloss[self.vadr] = 0
        self.limits = np.minimum(policy.effort,
                                 np.max(np.abs(model.actuator_ctrlrange[self.actuators]), axis=1))
        self.target = policy.default.copy()
        self._make_rays()

    def _make_rays(self):
        u, v = np.meshgrid(np.arange(64) + .5, np.arange(36) + .5)
        # Camera convention: x forward, y left, z up. Full torso rotation is used.
        rays = np.stack([np.ones_like(u), -(u-32)/32*np.tan(np.deg2rad(89.51)/2),
                         -(v-18)/18*np.tan(np.deg2rad(58.29)/2)], axis=-1).reshape(-1, 3)
        rays /= np.linalg.norm(rays, axis=1, keepdims=True)
        self.cosines = rays[:, 0].copy()
        mat = np.zeros(9)
        quat = np.array([.9135367613482678, .004363309284746571, .4067366430758002, 0.])
        quat /= np.linalg.norm(quat)
        mujoco.mju_quat2Mat(mat, quat)
        self.rays = rays @ mat.reshape(3, 3).T
        self.offset = np.array([.0487988662332928, .01, .4378029937970051])

    def depth(self):
        rotation = self.data.xmat[self.torso].reshape(3, 3)
        origin = self.data.xpos[self.torso] + rotation @ self.offset
        directions = np.ascontiguousarray(self.rays @ rotation.T)
        distances = np.empty(len(directions))
        ids = np.empty(len(directions), dtype=np.int32)
        # Include collision groups only; visual mesh copies must not occlude the rays.
        groups = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)
        options = {'nray': len(directions), 'cutoff': 100.}
        if 'normal:' in (mujoco.mj_multiRay.__doc__ or ''):
            options['normal'] = None
        mujoco.mj_multiRay(self.model, self.data, origin, directions.ravel(), groups,
                          1, self.torso, ids, distances, **options)
        distances[distances < 0] = np.inf
        return (distances * self.cosines).reshape(36, 64).astype(np.float32)

    def observation(self, command):
        rotation = self.data.xmat[self.torso].reshape(3, 3)
        velocity = np.zeros(6)
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY,
                                self.torso, velocity, 1)
        return (self.sign * self.data.qpos[self.qadr], self.sign * self.data.qvel[self.vadr], velocity[:3],
                rotation.T @ np.array([0., 0., -1.]), command, self.depth())

    def control(self):
        torque = self.policy.kp * (self.sign * self.target - self.data.qpos[self.qadr])
        torque -= self.policy.kd * self.data.qvel[self.vadr]
        if not np.isfinite(torque).all():
            raise RuntimeError('non-finite torque')
        self.data.ctrl[self.actuators] = np.clip(torque, -self.limits, self.limits)
