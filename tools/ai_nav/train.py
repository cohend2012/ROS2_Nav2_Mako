#!/usr/bin/env python3
"""Train the L1 navigator with PPO (Schulman 2017) via Stable-Baselines3.

Offline only, never in the runtime image. Chunked and resumable (Engineering Rule 4):
a checkpoint is written every --ckpt steps to --out, and --resume continues from the
newest one.

    python3 tools/ai_nav/train.py --out ~/m20_ai_runs/v1 --steps 3000000 --envs 4
    python3 tools/ai_nav/train.py --out ~/m20_ai_runs/v1 --steps 3000000 --resume

Curriculum: obstacle density ramps from 0.3 to 1.0 over the first 60% of training.
"""
import argparse
import glob
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "src", "m20_ai_nav"))

import numpy as np  # noqa: E402
import torch  # noqa: E402
from stable_baselines3 import PPO  # noqa: E402
from stable_baselines3.common.callbacks import BaseCallback  # noqa: E402
from stable_baselines3.common.vec_env import SubprocVecEnv, VecMonitor  # noqa: E402


def make_env(rank, seed):
    def _f():
        from m20_ai_nav.sim.env import M20NavEnv
        return M20NavEnv(seed=seed + rank, density=0.3)
    return _f


class Progress(BaseCallback):
    """Curriculum + checkpoints + a JSON-lines log of rolling episode stats."""

    def __init__(self, out, total, ckpt):
        super().__init__()
        self.out, self.total, self.ckpt = out, total, ckpt
        self.next_ckpt = None
        self.ep = []
        self.log = open(os.path.join(out, "train_log.jsonl"), "a")
        self.t0 = time.time()

    def _on_training_start(self):
        self.next_ckpt = (self.num_timesteps // self.ckpt + 1) * self.ckpt

    def _on_step(self):
        for info in self.locals["infos"]:
            if "episode" in info:
                self.ep.append((info.get("success", False), info.get("collision", False),
                                info.get("tipped", False), info["episode"]["r"]))
        frac = self.num_timesteps / self.total
        dens = 0.3 + 0.7 * min(1.0, frac / 0.6)
        if self.n_calls % 200 == 0:
            self.training_env.set_attr("density", dens)
        if self.num_timesteps >= self.next_ckpt:
            path = os.path.join(self.out, f"ckpt_{self.num_timesteps:09d}.zip")
            self.model.save(path)
            last = self.ep[-200:]
            rec = {"t": self.num_timesteps, "wall_s": round(time.time() - self.t0),
                   "density": round(dens, 2), "episodes": len(self.ep)}
            if last:
                a = np.array(last, float)
                rec.update(success=round(a[:, 0].mean(), 3), collision=round(a[:, 1].mean(), 3),
                           tipped=round(a[:, 2].mean(), 3), ret=round(a[:, 3].mean(), 2))
            self.log.write(json.dumps(rec) + "\n"); self.log.flush()
            print(json.dumps(rec), flush=True)
            self.next_ckpt += self.ckpt
        return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--steps", type=int, default=3_000_000)
    ap.add_argument("--envs", type=int, default=4)
    ap.add_argument("--ckpt", type=int, default=100_000)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--resume", action="store_true")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    torch.set_num_threads(1)
    venv = VecMonitor(SubprocVecEnv([make_env(i, a.seed * 100) for i in range(a.envs)]),
                      info_keywords=("success", "collision", "tipped"))
    ckpts = sorted(glob.glob(os.path.join(a.out, "ckpt_*.zip")))
    if a.resume and ckpts:
        print(f"resuming from {ckpts[-1]}")
        model = PPO.load(ckpts[-1], env=venv, device="cpu")
        reset = False
    else:
        if a.resume:
            print("no checkpoint found, starting fresh")
        model = PPO("MlpPolicy", venv, device="cpu", seed=a.seed, verbose=0,
                    n_steps=1024, batch_size=512, n_epochs=8, gamma=0.99, gae_lambda=0.95,
                    learning_rate=3e-4, clip_range=0.2, ent_coef=0.003,
                    policy_kwargs=dict(net_arch=dict(pi=[256, 256], vf=[256, 256]),
                                       log_std_init=-0.5))
        reset = True
    remaining = a.steps - model.num_timesteps
    if remaining <= 0:
        print("already trained to target"); return
    model.learn(total_timesteps=remaining, reset_num_timesteps=reset,
                callback=Progress(a.out, a.steps, a.ckpt))
    model.save(os.path.join(a.out, "final.zip"))
    print("done:", os.path.join(a.out, "final.zip"))


if __name__ == "__main__":
    main()
