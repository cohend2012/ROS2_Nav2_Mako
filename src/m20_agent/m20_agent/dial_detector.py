"""Dial detectors behind one interface: detector(rgb) -> [(cx, cy, r_px, score)], best first.

  classical : gauge_finder.detect_dials (round bright face + red needle). No model file.
  yolo      : a YOLO model exported to ONNX (tools/perception/train_dial_yolo.py), run
              with onnxruntime: no PyTorch on the robot, same runtime as the driving policy.
              It finds dials the classical test misses (far away, off-axis, no red
              needle); reading still checks the needle up close.

    det = make_detector("yolo", "src/m20_agent/models/dial_yolo.onnx")
    for cx, cy, r, score in det(rgb): ...
"""
import os

import numpy as np

from m20_agent.gauge_finder import detect_dials

def _default_yolo():
    """src/m20_agent/models/dial_yolo.onnx in a checkout; share/m20_agent/models when
    installed with colcon (the Docker image)."""
    here = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "models", "dial_yolo.onnx")
    if os.path.exists(here):
        return here
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("m20_agent"), "models", "dial_yolo.onnx")
    except Exception:
        return here


DEFAULT_YOLO = _default_yolo()


class ClassicalDetector:
    name = "classical"
    # where it reliably finds a dial (measured, tools/perception/eval_dial_detector.py):
    # ~55 % even head-on within 3 m, falling to ~0 past 3.5 m or 50 deg off the face
    envelope = {"rng_m": (1.0, 3.0), "half_fov_deg": 18.0, "max_incidence_deg": 35.0}

    def __call__(self, rgb):
        return detect_dials(rgb)


class YoloDetector:
    """Single-class YOLO (ultralytics export, output [1, 5, N]: cx, cy, w, h, score in
    letterboxed input pixels). Letterbox and NMS done here in numpy."""
    name = "yolo"
    # measured 100 % (1010/1010 sim poses) to 6 m, 65 deg off the face, 24 deg off-centre;
    # the coverage rule uses a margin inside that
    envelope = {"rng_m": (0.8, 4.5), "half_fov_deg": 24.0, "max_incidence_deg": 60.0}

    def __init__(self, path=DEFAULT_YOLO, conf=0.35, iou=0.5, threads=2):
        import onnxruntime as ort
        so = ort.SessionOptions()
        so.intra_op_num_threads = threads
        self.sess = ort.InferenceSession(path, so, providers=["CPUExecutionProvider"])
        inp = self.sess.get_inputs()[0]
        self.input = inp.name
        self.size = int(inp.shape[2]) if isinstance(inp.shape[2], int) else 320
        self.conf, self.iou = conf, iou

    def _letterbox(self, rgb):
        h, w = rgb.shape[:2]
        s = self.size / max(h, w)
        nh, nw = int(round(h * s)), int(round(w * s))
        from PIL import Image
        img = np.asarray(Image.fromarray(rgb).resize((nw, nh), Image.BILINEAR))
        canvas = np.full((self.size, self.size, 3), 114, np.uint8)
        top, left = (self.size - nh) // 2, (self.size - nw) // 2
        canvas[top:top + nh, left:left + nw] = img
        x = canvas.astype(np.float32).transpose(2, 0, 1)[None] / 255.0
        return x, s, left, top

    def __call__(self, rgb):
        x, s, left, top = self._letterbox(np.asarray(rgb)[..., :3])
        out = self.sess.run(None, {self.input: x})[0][0]          # (5, N)
        cx, cy, bw, bh, sc = out
        keep = sc >= self.conf
        if not keep.any():
            return []
        boxes = np.stack([cx[keep] - bw[keep] / 2, cy[keep] - bh[keep] / 2,
                          cx[keep] + bw[keep] / 2, cy[keep] + bh[keep] / 2], 1)
        sc = sc[keep]
        order, picked = np.argsort(-sc), []
        while order.size:
            i = order[0]
            picked.append(i)
            xx0 = np.maximum(boxes[i, 0], boxes[order[1:], 0]); yy0 = np.maximum(boxes[i, 1], boxes[order[1:], 1])
            xx1 = np.minimum(boxes[i, 2], boxes[order[1:], 2]); yy1 = np.minimum(boxes[i, 3], boxes[order[1:], 3])
            inter = np.clip(xx1 - xx0, 0, None) * np.clip(yy1 - yy0, 0, None)
            area = lambda b: (b[..., 2] - b[..., 0]) * (b[..., 3] - b[..., 1])  # noqa: E731
            iou = inter / (area(boxes[i]) + area(boxes[order[1:]]) - inter + 1e-9)
            order = order[1:][iou < self.iou]
        dets = []
        for i in picked:
            x0, y0, x1, y1 = boxes[i]
            x0, x1 = (x0 - left) / s, (x1 - left) / s
            y0, y1 = (y0 - top) / s, (y1 - top) / s
            # the box is the whole plate; the round face is ~92 % of it
            dets.append(((x0 + x1) / 2, (y0 + y1) / 2, 0.46 * max(x1 - x0, y1 - y0), float(sc[i])))
        return dets


def make_detector(name="classical", path=None):
    if name == "yolo":
        return YoloDetector(path or DEFAULT_YOLO)
    if name == "auto":
        return YoloDetector(path or DEFAULT_YOLO) if os.path.exists(path or DEFAULT_YOLO) else ClassicalDetector()
    return ClassicalDetector()
