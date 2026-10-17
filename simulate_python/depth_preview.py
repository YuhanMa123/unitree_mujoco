"""GUI preview of the exact ray depth sent to C++, without additional ray casting."""
import os
import sys
import numpy as np


class DepthPreview:
    title = 'G1 depth | raw 36x64 + network frame 18x32 | Esc: close preview'

    def __init__(self):
        import cv2
        self.cv2 = cv2
        self.open = False
        if sys.platform.startswith('linux') and not (os.getenv('DISPLAY') or os.getenv('WAYLAND_DISPLAY')):
            print('Depth preview disabled: no desktop display available.')
            return
        if 'GUI:                           NONE' in cv2.getBuildInformation():
            print('Depth preview disabled: OpenCV has no GUI backend.')
            return
        try:
            cv2.namedWindow(self.title, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.title, 1280, 450)
        except cv2.error as error:
            print(f'Depth preview disabled: {error}')
            return
        self.open = True

    def render(self, depth, sim_time):
        """Create BGR preview; usable offline without opening a GUI window."""
        from depth_image import preprocess_depth
        cv = self.cv2
        depth = np.asarray(depth, dtype=np.float32)
        processed = preprocess_depth(depth)

        def color(values, no_hit=None):
            level = np.rint((1 - np.clip(values, 0, 1)) * 255).astype(np.uint8)
            rgb = cv.applyColorMap(level, cv.COLORMAP_TURBO)
            if no_hit is not None:
                rgb[no_hit] = (0, 0, 0)
            return cv.resize(rgb, (640, 360), interpolation=cv.INTER_NEAREST)

        raw = color(np.nan_to_num(depth / 2.5, posinf=1.), ~np.isfinite(depth))
        # Raw crop retained for the encoder: rows 18..35, columns 16..47.
        cv.rectangle(raw, (160, 180), (479, 359), (255, 255, 255), 2)
        network = color(processed)
        canvas = np.full((450, 1280, 3), 24, dtype=np.uint8)
        canvas[48:408, :640] = raw
        canvas[48:408, 640:] = network
        for label, position in [
            (f'Raw image-plane depth 36x64 | t={sim_time:.2f}s', (12, 30)),
            ('Encoder frame 18x32 | crop + blur + normalize', (652, 30)),
            ('Red: near 0m | Blue: 2.5m+ | Black: no hit | White box: crop', (12, 436))]:
            cv.putText(canvas, label, position, cv.FONT_HERSHEY_SIMPLEX, .6, (240, 240, 240), 1, cv.LINE_AA)
        return canvas

    def update(self, depth, sim_time):
        if not self.open:
            return
        cv = self.cv2
        if cv.getWindowProperty(self.title, cv.WND_PROP_VISIBLE) < 1:
            self.close()
            return
        if depth is not None:
            cv.imshow(self.title, self.render(depth, sim_time))
        if cv.waitKey(1) & 0xff == 27:
            self.close()

    def close(self):
        if self.open:
            self.cv2.destroyWindow(self.title)
            self.open = False
