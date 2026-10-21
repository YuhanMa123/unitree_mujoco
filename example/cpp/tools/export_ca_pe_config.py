#!/usr/bin/env python3
"""Extract the saved CA PE AMP VAE training parameters into the deployment JSON."""
import argparse
import json
from pathlib import Path
import re
import yaml

TASK = Path(__file__).resolve().parents[2] / 'exported/g1_CA_PE_AMP_VAE'
# IsaacLab pelvis-root breadth-first articulation order; also declared by the
# training g1_amp/mdp/symmetry.py. This differs from torso-root InstinctLab parkour.
JOINTS = '''left_hip_pitch right_hip_pitch waist_yaw left_hip_roll right_hip_roll
waist_roll left_hip_yaw right_hip_yaw waist_pitch left_knee right_knee
left_shoulder_pitch right_shoulder_pitch left_ankle_pitch right_ankle_pitch
left_shoulder_roll right_shoulder_roll left_ankle_roll right_ankle_roll
left_shoulder_yaw right_shoulder_yaw left_elbow right_elbow left_wrist_roll
right_wrist_roll left_wrist_pitch right_wrist_pitch left_wrist_yaw right_wrist_yaw'''.split()


class ParamsLoader(yaml.SafeLoader):
    """Read serialized tuples/slices as data; never instantiate Python objects."""


def python_data(loader, suffix, node):
    if suffix not in ('tuple', 'object/apply:builtins.slice'):
        raise ValueError(f'Unsupported parameter tag: {suffix}')
    return loader.construct_sequence(node)


ParamsLoader.add_multi_constructor('tag:yaml.org,2002:python/', python_data)


def resolve(value, name, default=None):
    if not isinstance(value, dict):
        return value
    matches = [v for pattern, v in value.items() if re.fullmatch(pattern, name)]
    if len(matches) > 1:
        raise ValueError(f'Ambiguous parameter for {name}')
    if matches:
        return matches[0]
    if default is None:
        raise ValueError(f'Missing parameter for {name}')
    return default


def generate(task, history_action_delay=None):
    env = yaml.load((task / 'params/env.yaml').read_text(), Loader=ParamsLoader)
    agent = yaml.load((task / 'params/agent.yaml').read_text(), Loader=ParamsLoader)
    robot = env['scene']['robot']
    names = [name + '_joint' for name in JOINTS]
    cfg = {
        'source': 'params/env.yaml + params/agent.yaml; pelvis-root IsaacLab breadth-first joint order (training mdp/symmetry.py)',
        'policy_type': 'ca_pe_amp_vae', 'joint_names': names, 'joint_sign': [1] * 29,
        'step_dt': env['sim']['dt'] * env['decimation'], 'physics_dt': env['sim']['dt'],
        'state_body': 'pelvis', 'depth_delay_frames': 0,
        'default_joint_pos': [resolve(robot['init_state']['joint_pos'], name, 0) for name in names],
    }
    for field, source in [('kp', 'stiffness'), ('kd', 'damping'),
                          ('effort_limit', 'effort_limit_sim'), ('armature', 'armature')]:
        cfg[field] = []
        for name in names:
            actuators = [v for v in robot['actuators'].values()
                         if any(re.fullmatch(p, name) for p in v['joint_names_expr'])]
            if len(actuators) != 1:
                raise ValueError(f'Expected one actuator for {name}')
            cfg[field].append(resolve(actuators[0][source], name))
    cfg['action_scale'] = [resolve(env['actions']['joint_pos']['scale'], name) for name in names]
    camera = env['scene']['camera']
    depth = env['observations']['depth_image']['depth_image']['params']
    noise = camera['noise_pipeline']
    history = env['observations']['obs_history']['history']['params']
    ranges = env['commands']['base_velocity']['ranges']
    keys = ['lin_vel_x', 'lin_vel_y', 'ang_vel_z']
    cfg.update(
        action_clip=agent['clip_actions'], angular_velocity_scale=history['base_ang_vel_scale'],
        joint_velocity_scale=history['joint_vel_scale'], proprio_history_frames=history['history_length'],
        depth_history_frames=depth['num_output_frames'], depth_history_stride=depth['history_skip_frames'],
        depth_crop=noise['crop_and_resize']['crop_region'],
        depth_range=noise['depth_normalization']['depth_range'], depth_blur_sigma=noise['gaussian_blur']['sigma'],
        command_min=[ranges[key][0] for key in keys], command_max=[ranges[key][1] for key in keys],
        actor_model='exported/actor.onnx', depth_model='exported/depth_token_encoder.onnx',
        camera=dict(body=camera['prim_path'].rsplit('/', 1)[1], position=camera['offset']['pos'],
                    quaternion=camera['offset']['rot'],
                    horizontal_aperture=camera['pattern_cfg']['horizontal_aperture'],
                    vertical_aperture=camera['pattern_cfg']['vertical_aperture'],
                    focal_length=camera['pattern_cfg']['focal_length'], min_distance=camera['min_distance'], include_robot=True),
    )
    if history_action_delay is not None:
        if history_action_delay not in (0, 1):
            raise ValueError('history_action_delay must be 0 or 1')
        cfg['history_action_delay'] = history_action_delay
    return cfg


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task-dir', type=Path, default=TASK)
    parser.add_argument('--check', action='store_true', help='Check saved JSON without modifying it')
    parser.add_argument('--history-action-delay', type=int, choices=[0, 1],
                        help='0: history uses action; 1: history uses prev_action; verify training source')
    args = parser.parse_args()
    path = args.task_dir / 'g1_ca_pe_amp_vae_cfg.json'
    delay = args.history_action_delay
    if delay is None and path.exists():
        delay = json.loads(path.read_text()).get('history_action_delay')
    config = generate(args.task_dir, delay)
    if args.check:
        if json.loads(path.read_text()) != config:
            raise SystemExit(f'{path}: deployment config differs from saved training params')
        print('Deployment JSON matches saved training parameters.')
    else:
        path.write_text(json.dumps(config, indent=2) + '\n')
        print(path)


if __name__ == '__main__':
    main()
