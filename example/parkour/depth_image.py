"""Depth preprocessing shared by the reference policy and simulator preview."""
import numpy as np


def preprocess_depth(depth):
    """36x64 image-plane depth in metres -> crop, reflect Gaussian, normalize 18x32."""
    depth = np.asarray(depth, dtype=np.float32)
    if depth.shape != (36, 64) or np.isnan(depth).any() or (depth < 0).any():
        raise ValueError('depth must be nonnegative 36x64 metres, no NaN')
    # Positive infinity is the explicit no-hit value. Filter finite far distances.
    image = np.minimum(depth, np.finfo(np.float32).max)[18:, 16:-16]
    weights = np.exp(-np.arange(-1, 2, dtype=np.float32)**2 / 2)
    weights /= weights.sum()
    padded = np.pad(image, 1, mode='reflect')
    image = sum(weights[y] * weights[x] * padded[y:y+18, x:x+32]
                for y in range(3) for x in range(3))
    return np.clip(image, 0, 2.5) / 2.5
