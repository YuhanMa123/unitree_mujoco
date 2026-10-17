"""Standalone MuJoCo rollout: no DDS publishers and no Isaac imports."""
import argparse
from contextlib import nullcontext
from pathlib import Path
import time
import numpy as np
import mujoco
from policy import ParkourPolicy, DEFAULT_MODELS

ROOT = Path(__file__).resolve().parents[2]


from mujoco_adapter import MujocoAdapter


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model-dir', type=Path, default=DEFAULT_MODELS)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--scene', type=Path, default=ROOT / 'unitree_robots/g1/scene_29dof.xml')
    parser.add_argument('--seconds', type=float, default=10.)
    parser.add_argument('--command', nargs=3, type=float, default=[0., 0., 0.], metavar=('VX','VY','WZ'))
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--check-policy', action='store_true', help='synthetic inference only')
    parser.add_argument('--fixed-base', action='store_true', help='pin pelvis for interface smoke test')
    args = parser.parse_args()
    policy = ParkourPolicy(args.model_dir, args.config)
    command = np.asarray(args.command, dtype=np.float32)
    if not np.isfinite(command).all() or not np.isfinite(args.seconds) or args.seconds <= 0:
        parser.error('command must be finite and seconds must be positive and finite')
    if args.check_policy:
        target = policy.step(policy.default, np.zeros(29), np.zeros(3), [0,0,-1], command,
                             np.full((36,64), 2.5, dtype=np.float32))
        print(f'CPU inference OK: encoder [1,8,18,32] -> [1,128]; actor [1,896] -> [1,29]; target range {target.min():.4f}..{target.max():.4f}')
        return
    model = mujoco.MjModel.from_xml_path(str(args.scene.resolve()))
    model.opt.timestep = policy.config['physics_dt']
    ratio = policy.config['step_dt'] / model.opt.timestep
    decimation = round(ratio)
    if decimation < 1 or not np.isclose(ratio, decimation):
        raise ValueError('step_dt must be an integer multiple of physics_dt')
    data = mujoco.MjData(model)
    adapter = MujocoAdapter(model, data, policy)
    data.qpos[adapter.qadr] = adapter.sign * policy.default
    data.qpos[:3] = [0, 0, .78]
    data.qpos[3:7] = [1, 0, 0, 0]
    mujoco.mj_forward(model, data)
    initial_root = data.qpos[:7].copy()
    min_height = data.xpos[adapter.torso,2]
    if args.headless:
        context = nullcontext(None)
    else:
        from mujoco import viewer as mj_viewer
        context = mj_viewer.launch_passive(model, data)
    steps = 0
    try:
        with context as viewer:
            while data.time < args.seconds and (viewer is None or viewer.is_running()):
                start = time.perf_counter()
                if steps % decimation == 0:
                    adapter.target = policy.step(*adapter.observation(command))
                adapter.control()
                mujoco.mj_step(model, data)
                if args.fixed_base:
                    data.qpos[:7] = initial_root
                    data.qvel[:6] = 0
                mujoco.mj_forward(model, data)
                min_height = min(min_height, data.xpos[adapter.torso,2])
                gravity_z = -data.xmat[adapter.torso].reshape(3,3)[2,2]
                if data.xpos[adapter.torso,2] < .5 or gravity_z > -np.cos(1.):
                    raise RuntimeError(f'rollout stopped: torso height={data.xpos[adapter.torso,2]:.3f}, gravity_z={gravity_z:.3f}')
                if viewer:
                    if steps % decimation == 0:
                        viewer.sync()
                    time.sleep(max(0, model.opt.timestep - (time.perf_counter()-start)))
                steps += 1
    finally:
        data.ctrl[:] = 0
    print(f'MuJoCo OK: {data.time:.3f}s, {steps} steps, min torso height={min_height:.3f}m, pelvis xy={data.qpos[:2]}')


if __name__ == '__main__':
    main()
