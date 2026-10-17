"""InstinctLab G1 parkour CPU inference; all joint vectors use config joint_names order."""
from collections import deque
from pathlib import Path
import json
import numpy as np
import onnxruntime as ort

DEFAULT_MODELS = Path(__file__).resolve().parents[2] / 'exported' / 'g1_parkour'


def vector(value, size, name):
    value = np.asarray(value, dtype=np.float32)
    if value.shape != (size,) or not np.isfinite(value).all():
        raise ValueError(f'{name}: expected finite ({size},), got {value.shape}')
    return value


from depth_image import preprocess_depth


class ParkourPolicy:
    def __init__(self, model_dir=DEFAULT_MODELS, config=None):
        self.config = json.loads(Path(config or Path(__file__).with_name('g1.json')).read_text())
        c = self.config
        if type(c['depth_delay_frames']) is not int or c['depth_delay_frames'] not in (0, 1):
            raise ValueError('depth_delay_frames must be 0 or 1')
        self.joint_names = c['joint_names']
        if len(self.joint_names) != 29 or len(set(self.joint_names)) != 29:
            raise ValueError('joint_names must contain 29 unique joints')
        self.default = vector(c['default_joint_pos'], 29, 'default_joint_pos')
        self.kp = vector(c['kp'], 29, 'kp')
        self.kd = vector(c['kd'], 29, 'kd')
        self.scale = vector(c['action_scale'], 29, 'action_scale')
        self.effort = vector(c['effort_limit'], 29, 'effort_limit')
        if (self.kp <= 0).any() or (self.kd < 0).any() or (self.effort <= 0).any():
            raise ValueError('invalid PD gains or effort limits')
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        self.encoder = ort.InferenceSession(str(Path(model_dir) / '0-depth_encoder.onnx'), options,
                                            providers=['CPUExecutionProvider'])
        self.actor = ort.InferenceSession(str(Path(model_dir) / 'actor.onnx'), options,
                                          providers=['CPUExecutionProvider'])
        for session, ins, outs in [(self.encoder, [1, 8, 18, 32], [1, 128]),
                                   (self.actor, [1, 896], [1, 29])]:
            if len(session.get_inputs()) != 1 or len(session.get_outputs()) != 1:
                raise ValueError('expected exactly one input and output per network')
            for node, shape in [(session.get_inputs()[0], ins), (session.get_outputs()[0], outs)]:
                if node.shape != shape or node.type != 'tensor(float)':
                    raise ValueError(f'incompatible ONNX interface: {node.name} {node.shape} {node.type}')
        self.reset()

    def reset(self):
        self.histories = [deque(maxlen=8) for _ in range(6)]
        self.depth_history = deque(maxlen=37)
        self.last_action = np.zeros(29, dtype=np.float32)

    def observe(self, joint_pos, joint_vel, angular_velocity, gravity, command, depth):
        terms = [vector(angular_velocity, 3, 'angular_velocity') * .25,
                 vector(gravity, 3, 'gravity'), vector(command, 3, 'command'),
                 vector(joint_pos, 29, 'joint_pos') - self.default,
                 vector(joint_vel, 29, 'joint_vel') * .05, self.last_action.copy()]
        image = preprocess_depth(depth)
        for history, term in zip(self.histories, terms):
            if not history:
                history.extend(term.copy() for _ in range(8))
            else:
                history.append(term.copy())
        if not self.depth_history:
            self.depth_history.extend(image.copy() for _ in range(37))
        else:
            self.depth_history.append(image.copy())
        proprio = np.concatenate([np.asarray(h).ravel() for h in self.histories])[None]
        # Oldest -> newest, 5 sensor updates between sampled frames; deterministic delay.
        delay = self.config['depth_delay_frames']
        frames = np.stack([self.depth_history[-1-delay-offset] for offset in range(35, -1, -5)])[None]
        return np.ascontiguousarray(proprio), np.ascontiguousarray(frames)

    def infer(self, proprio, depth_frames):
        """Use already prepared inputs, without updating observation history."""
        feeds = []
        for value, shape in [(proprio, (1, 768)), (depth_frames, (1, 8, 18, 32))]:
            value = np.asarray(value, dtype=np.float32)
            if value.shape != shape or not np.isfinite(value).all():
                raise ValueError(f'expected finite {shape}')
            feeds.append(np.ascontiguousarray(value))
        latent = self.encoder.run(None, {self.encoder.get_inputs()[0].name: feeds[1]})[0]
        actor_input = np.concatenate([feeds[0], latent], axis=1)
        action = self.actor.run(None, {self.actor.get_inputs()[0].name: actor_input})[0][0]
        action = vector(action, 29, 'network action')
        target = self.default + self.scale * action
        vector(target, 29, 'target')
        self.last_action = action.copy()  # raw training action, no added clipping
        return target

    def step(self, joint_pos, joint_vel, angular_velocity, gravity, command, depth):
        return self.infer(*self.observe(joint_pos, joint_vel, angular_velocity, gravity, command, depth))
