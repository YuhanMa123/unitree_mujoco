"""Send synchronized state/depth from this simulator to the C++ parkour controller."""
from pathlib import Path
import json
import socket
import struct
import sys
import threading
from types import SimpleNamespace
import numpy as np

PARKOUR = Path(__file__).resolve().parents[1] / 'example' / 'parkour'
DEFAULT_CONFIG = PARKOUR.parent / 'exported' / 'g1_parkour' / 'g1_torsobase_cfg.json'
sys.path.insert(0, str(PARKOUR))
from mujoco_adapter import MujocoAdapter


def joint_signature(names, signs):
    value = 2166136261
    for byte in ''.join(f'{name}:{int(sign)}\n' for name, sign in zip(names, signs)).encode():
        value = ((value ^ byte) * 16777619) & 0xffffffff
    return value


class ParkourObservationPublisher:
    def __init__(self, model, data, port=17200, config_path=None):
        if not 1024 <= port <= 65535:
            raise ValueError('parkour observation port must be 1024..65535')
        cfg = json.loads(Path(config_path or DEFAULT_CONFIG).read_text())
        params = SimpleNamespace(config=cfg, joint_names=cfg['joint_names'],
                                 default=np.array(cfg['default_joint_pos']),
                                 effort=np.array(cfg['effort_limit']))
        self.adapter = MujocoAdapter(model, data, params, configure_dynamics=True)
        self.signature = joint_signature(params.joint_names, cfg['joint_sign'])
        self.data = data
        self.period = cfg['step_dt']
        self.next_time = -1.
        self.last_time = -1.
        self.sequence = 0
        self.preview_lock = threading.Lock()
        self.latest_depth = None
        self.latest_depth_time = 0.
        self.address = ('127.0.0.1', port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setblocking(False)

    def update(self):
        now = self.data.time
        if now < self.last_time:
            self.next_time = now
        self.last_time = now
        if now + 1e-9 < self.next_time:
            return
        self.next_time = now + self.period
        q, dq, omega, gravity, _, depth = self.adapter.observation(np.zeros(3))
        with self.preview_lock:
            self.latest_depth = depth.copy()
            self.latest_depth_time = now
        height = self.data.xpos[self.adapter.torso, 2]
        payload = np.concatenate([q, dq, omega, gravity, [height], depth.ravel()]).astype('<f4')
        header = struct.pack('<8sQdII', b'G1PKR001', self.sequence, now, self.signature, len(payload))
        self.sequence += 1
        try:
            self.socket.sendto(header + payload.tobytes(), self.address)
        except BlockingIOError:
            pass  # Never block the physics thread waiting for the controller.

    def close(self):
        self.socket.close()

    def preview_frame(self):
        with self.preview_lock:
            return (None if self.latest_depth is None else self.latest_depth.copy(),
                    self.latest_depth_time)
