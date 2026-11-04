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


import gymnasium as _gym


class MixEnv(_gym.Env):
    """Per-episode mix of scenes: office (path mode, people) with prob p_office, else
    the outdoor procedural slots. Both share the policy contract (spec.py), so one
    policy serves both sites; the outdoor share stops v2 forgetting v1's skills."""

    def __init__(self, seed, p_office, density, shield=False):
        import gymnasium as gym
        import numpy as np
        from m20_ai_nav.sim.env import M20NavEnv
        self.outdoor = M20NavEnv(seed=seed, density=density, shield=shield)
        self.office = M20NavEnv(scene="office", seed=seed + 7919, shield=shield)
        self.p_office = p_office
        self.rng = np.random.default_rng(seed + 1)
        self.cur = self.outdoor
        self.observation_space = self.outdoor.observation_space
        self.action_space = self.outdoor.action_space
        self.metadata = {"render_modes": []}

    @property
    def density(self):
        return self.outdoor.density

    @density.setter
    def density(self, v):
        self.outdoor.density = v

    def reset(self, seed=None, options=None):
        self.cur = self.office if self.rng.random() < self.p_office else self.outdoor
        obs, info = self.cur.reset(seed=seed, options=options)
        info["office"] = self.cur is self.office
        return obs, info

    def step(self, a):
        o, r, te, tr, info = self.cur.step(a)
        info["office"] = self.cur is self.office
        return o, r, te, tr, info

    def close(self):
        pass


def make_env(rank, seed, p_office=0.0, density=0.3, shield=False):
    def _f():
        import gymnasium as gym
        from m20_ai_nav.sim.env import M20NavEnv
        if p_office <= 0:
            return M20NavEnv(seed=seed + rank, density=density, shield=shield)
        e = MixEnv(seed + rank, p_office, density, shield)
        return e
    return _f


class Progress(BaseCallback):
    """Curriculum + checkpoints + a JSON-lines log of rolling episode stats."""

    def __init__(self, out, total, ckpt, d0=0.3):
        super().__init__()
        self.d0 = d0
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
                                info.get("tipped", False), info["episode"]["r"],
                                info.get("office", False)))
        frac = self.num_timesteps / self.total
        dens = self.d0 + (1.0 - self.d0) * min(1.0, frac / 0.6)
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
                off = a[a[:, 4] > 0]
                if len(off):
                    rec.update(office_success=round(off[:, 0].mean(), 3),
                               office_collision=round(off[:, 1].mean(), 3), office_n=int(len(off)))
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
    ap.add_argument("--init", default=None, help="warm-start weights from this checkpoint (fine-tune)")
    ap.add_argument("--p-office", type=float, default=0.0, help="share of office episodes (0 = v1 recipe)")
    ap.add_argument("--density0", type=float, default=0.3, help="initial outdoor obstacle density")
    ap.add_argument("--shield", action="store_true", help="shielded RL: train inside core.shield()")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    torch.set_num_threads(1)
    venv = VecMonitor(SubprocVecEnv([make_env(i, a.seed * 100, a.p_office, a.density0, a.shield)
                                     for i in range(a.envs)]),
                      info_keywords=("success", "collision", "tipped", "office"))
    ckpts = sorted(glob.glob(os.path.join(a.out, "ckpt_*.zip")))
    if a.resume and ckpts:
        print(f"resuming from {ckpts[-1]}")
        model = PPO.load(ckpts[-1], env=venv, device="cpu")
        reset = False
    elif a.init:
        print(f"fine-tuning from {a.init} (p_office={a.p_office})")
        model = PPO.load(a.init, env=venv, device="cpu", learning_rate=1.5e-4, ent_coef=0.002)
        model.num_timesteps = 0
        reset = True
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
                callback=Progress(a.out, a.steps, a.ckpt, a.density0))
    model.save(os.path.join(a.out, "final.zip"))
    print("done:", os.path.join(a.out, "final.zip"))


if __name__ == "__main__":
    main()
