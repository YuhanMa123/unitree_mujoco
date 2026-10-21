"""Virtual suspension force at the attached body's center of mass."""
import mujoco
import numpy as np
import threading


class ElasticBand:
    def __init__(self):
        self.stiffness = 200
        self.damping = 100
        self.point = np.array([0., 0., 3.])
        self.length = 0.
        self.enable = True
        self.pending_toggles = 0
        self.pending_length_steps = 0
        self.key_lock = threading.Lock()

    def Advance(self, x, dx):
        delta = self.point - x
        distance = np.linalg.norm(delta)
        if distance < 1e-9:
            return np.zeros(3)
        direction = delta / distance
        velocity = np.dot(dx, direction)
        return (self.stiffness * (distance - self.length) - self.damping * velocity) * direction

    def Apply(self, model, data, body):
        """Call under the simulation lock after mj_forward, before every step."""
        # Consume queued keys on the physics thread, away from viewer callbacks.
        with self.key_lock:
            toggles, self.pending_toggles = self.pending_toggles, 0
            steps, self.pending_length_steps = self.pending_length_steps, 0
        if toggles % 2:
            if not self.enable:
                self.point = data.xipos[body].copy() + [0., 0., 2.]
                weight = mujoco.mj_getTotalmass(model) * np.linalg.norm(model.opt.gravity)
                self.length = max(0., 2. - weight / self.stiffness)
            self.enable = not self.enable
        self.length += .1 * steps
        if not self.enable:
            data.xfrc_applied[body, :3] = 0.
            return
        velocity = np.zeros(6)
        mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, body, velocity, 0)
        data.xfrc_applied[body, :3] = self.Advance(data.xipos[body], velocity[3:])

    def MujuocoKeyCallback(self, key):
        # GLFW 7/8/9 are the corresponding ASCII codes.
        with self.key_lock:
            if key == ord('7'):
                self.pending_length_steps -= 1
            elif key == ord('8'):
                self.pending_length_steps += 1
            elif key == ord('9'):
                self.pending_toggles += 1
