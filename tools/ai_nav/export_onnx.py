#!/usr/bin/env python3
"""Export a trained SB3 PPO checkpoint to ONNX (deterministic mean action).

    python3 tools/ai_nav/export_onnx.py ~/m20_ai_runs/v1/final.zip \
        src/m20_ai_nav/models/m20_nav_policy.onnx

Writes <out>.json metadata (obs_version, obs_dim, source checkpoint, sha256) and
VERIFIES the ONNX graph against PyTorch on 256 random observations (fails loudly if
max |diff| > 1e-4 — Engineering Rule 1: no silent failure paths).
"""
import hashlib
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "src", "m20_ai_nav"))

import numpy as np  # noqa: E402
import torch  # noqa: E402
from stable_baselines3 import PPO  # noqa: E402

from m20_ai_nav import spec  # noqa: E402


class Actor(torch.nn.Module):
    def __init__(self, policy):
        super().__init__()
        self.ext = policy.mlp_extractor.policy_net
        self.head = policy.action_net

    def forward(self, obs):
        return self.head(self.ext(obs))


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    src, out = sys.argv[1], sys.argv[2]
    model = PPO.load(src, device="cpu")
    actor = Actor(model.policy).eval()
    dummy = torch.zeros(1, spec.OBS_DIM)
    torch.onnx.export(actor, dummy, out, input_names=["obs"], output_names=["action"],
                      dynamic_axes={"obs": {0: "batch"}, "action": {0: "batch"}},
                      opset_version=17, dynamo=False)
    import onnxruntime as ort
    sess = ort.InferenceSession(out, providers=["CPUExecutionProvider"])
    x = np.random.default_rng(0).uniform(0, 1, (256, spec.OBS_DIM)).astype(np.float32)
    with torch.no_grad():
        ref = actor(torch.from_numpy(x)).numpy()
        sb3 = model.policy.get_distribution(torch.from_numpy(x)).distribution.mean.numpy()
    got = sess.run(None, {"obs": x})[0]
    err = float(np.abs(ref - got).max()); err2 = float(np.abs(sb3 - got).max())
    if err > 1e-4 or err2 > 1e-4:
        sys.exit(f"ONNX parity FAILED: |torch-onnx|={err:.2e} |sb3-onnx|={err2:.2e}")
    sha = hashlib.sha256(open(out, "rb").read()).hexdigest()
    meta = {"obs_version": spec.OBS_VERSION, "obs_dim": spec.OBS_DIM, "source": os.path.basename(src),
            "timesteps": int(model.num_timesteps), "sha256": sha, "parity_max_abs_err": err}
    json.dump(meta, open(out + ".json", "w"), indent=2)
    print(f"exported {out} ({os.path.getsize(out)/1024:.0f} KB) parity {err:.1e}  sha {sha[:12]}")


if __name__ == "__main__":
    main()
