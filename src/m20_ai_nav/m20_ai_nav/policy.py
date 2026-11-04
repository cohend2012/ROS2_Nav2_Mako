"""ONNX runtime for the trained navigator: no PyTorch on the robot.

The exported graph maps obs[1, OBS_DIM] -> action[1, 2] (deterministic mean action,
tanh-free; clipped here). Export: tools/ai_nav/export_onnx.py.
"""
import json
import os

import numpy as np

from m20_ai_nav import spec


class OnnxPolicy:
    def __init__(self, path):
        if not os.path.exists(path):
            raise FileNotFoundError(f"policy model not found: {path}")
        import onnxruntime as ort          # lazy: tests can use a fake policy
        so = ort.SessionOptions()
        so.intra_op_num_threads = 1
        self.sess = ort.InferenceSession(path, so, providers=["CPUExecutionProvider"])
        self.inp = self.sess.get_inputs()[0].name
        shape = self.sess.get_inputs()[0].shape
        if shape[-1] != spec.OBS_DIM:
            raise ValueError(f"model expects obs dim {shape[-1]}, spec says {spec.OBS_DIM}"
                             " — model and code are from different OBS_VERSIONs")
        meta = path + ".json"
        self.meta = json.load(open(meta)) if os.path.exists(meta) else {}
        if self.meta.get("obs_version", spec.OBS_VERSION) != spec.OBS_VERSION:
            raise ValueError("policy OBS_VERSION mismatch: retrain or check out matching code")

    def __call__(self, obs):
        x = np.asarray(obs, np.float32).reshape(1, spec.OBS_DIM)
        a = self.sess.run(None, {self.inp: x})[0].reshape(2)
        return np.clip(a, -1.0, 1.0)
