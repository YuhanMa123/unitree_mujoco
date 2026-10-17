import sys
from pathlib import Path
import unittest
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from policy import ParkourPolicy, preprocess_depth


class PolicyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.policy = ParkourPolicy()

    def setUp(self):
        self.policy.reset()

    def inputs(self, value=0):
        return (self.policy.default.copy(), np.zeros(29), np.full(3, value),
                [0,0,-1], [0,0,0], np.full((36,64), value, dtype=np.float32))

    def test_term_major_history_and_depth_stride(self):
        for tick in range(40):
            proprio, frames = self.policy.observe(*self.inputs(tick / 100))
        np.testing.assert_allclose(proprio[0,:24].reshape(8,3)[:,0],
                                   np.arange(32,40) / 100 * .25)
        np.testing.assert_allclose(frames[0,:,0,0], np.arange(4,40,5) / 100 / 2.5, atol=1e-7)
        np.testing.assert_allclose(proprio[0,24:48].reshape(8,3), np.tile([0,0,-1], (8,1)))

    def test_depth_crop_and_reflect_blur(self):
        image = np.arange(36*64, dtype=np.float32).reshape(36,64) / 1000
        result = preprocess_depth(image)
        crop = image[18:,16:48]
        weights = np.exp(-np.arange(-1,2,dtype=np.float64)**2 / 2)
        weights /= weights.sum()
        padded = np.pad(crop,1,mode='reflect')
        reference = np.empty((18,32))
        for y in range(18):
            for x in range(32):
                reference[y,x] = np.sum(padded[y:y+3,x:x+3] * np.outer(weights, weights))
        np.testing.assert_allclose(result, np.clip(reference,0,2.5)/2.5, atol=2e-7)
        np.testing.assert_allclose(preprocess_depth(np.full((36,64),np.inf)),1.)

    def test_network_composition_and_action_feedback(self):
        p = self.policy
        proprio, depth = p.observe(*self.inputs(1))
        latent = p.encoder.run(None, {'input': depth})[0]
        action = p.actor.run(None, {'input': np.concatenate([proprio,latent],axis=1)})[0][0]
        np.testing.assert_allclose(p.infer(proprio,depth), p.default+p.scale*action)
        next_obs, _ = p.observe(*self.inputs(1))
        np.testing.assert_allclose(next_obs[0,-29:], action)
        p.reset()
        reset_obs, _ = p.observe(*self.inputs(1))
        np.testing.assert_array_equal(reset_obs[0,-232:],0)

    def test_invalid_input_does_not_enter_histories(self):
        args = list(self.inputs())
        args[0][0] = np.nan
        with self.assertRaises(ValueError):
            self.policy.observe(*args)
        self.assertFalse(self.policy.histories[0])
        with self.assertRaises(ValueError):
            preprocess_depth(np.zeros((18,32)))


if __name__ == '__main__':
    unittest.main()
