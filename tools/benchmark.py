#!/usr/bin/env python3
"""M20 movement benchmark (offline MuJoCo, ground-truth pose).

Regression test for locomotion/control quality. Stands the robot up, then runs three
courses and reports metrics + a trajectory plot:
  1. waypoint course  — visit 5 marked waypoints (turn-to-face then drive); count reached
  2. full circle      — constant (v, w); report loop-closure gap
  3. figure-8         — alternating w; S-turns / crossover

Uses the same cmd_vel->wheel skid-steer mapping/gains as m20_locomotion_bridge, so a
regression here reflects the real control path. Needs the model from tools/setup_sim.sh.

Usage (WSL):  python3 tools/benchmark.py [output.png]
"""
import sys, math, numpy as np, mujoco
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from pathlib import Path

MJCF = Path.home()/"m20_sim/sdk_deploy/src/M20_sdk_deploy/M20_description/m20_mjcf/mjcf/M20.xml"
OUT = sys.argv[1] if len(sys.argv) > 1 else str(Path.home()/"m20_benchmark.png")
model = mujoco.MjModel.from_xml_path(str(MJCF)); model.opt.timestep = 1/500
LO = model.actuator_ctrlrange[:, 0].copy(); HI = model.actuator_ctrlrange[:, 1].copy()
WHEELS = (3, 7, 11, 15); LEFT = (3, 11); RIGHT = (7, 15)
STAND = np.array([0,-0.7,1.4,0, 0,-0.7,1.4,0, 0,0.7,-1.4,0, 0,0.7,-1.4,0], float)
CROUCH = np.array([0,-1.0,2.3,0, 0,-1.0,2.3,0, 0,1.0,-2.3,0, 0,1.0,-2.3,0], float)
R, B, WHEEL_KD = 0.10, 0.40, 5.0     # must match bridge SimVendorSDK

def smooth(t): t = max(0, min(1, t)); return t*t*(3-2*t)
def yaw(d):
    w, x, y, z = d.qpos[3:7]; return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
def wrap(a): return math.atan2(math.sin(a), math.cos(a))
def stepc(d, legcmd, wl, wr):
    q = d.qpos[7:23]; qd = d.qvel[6:22]; tau = np.zeros(16)
    for i in range(16):
        tau[i] = (WHEEL_KD*((wl if i in LEFT else wr)-qd[i])) if i in WHEELS \
            else (200*(legcmd[i]-q[i]) + 4*(0-qd[i]))
    d.ctrl[:] = np.clip(tau, LO, HI); mujoco.mj_step(model, d)
def cmd_vel(d, v, w):                 # +v = forward, +w = CCW (matches bridge)
    stepc(d, STAND, (-v + w*B/2)/R, (-v - w*B/2)/R)
def standing():
    d = mujoco.MjData(model); d.qpos[:] = 0; d.qpos[3] = 1; d.qpos[7:23] = CROUCH
    d.qpos[2] = 0.6; mujoco.mj_forward(model, d)
    d.qpos[2] = 0.6 - float(d.geom_xpos[:, 2].min()) + 0.02; mujoco.mj_forward(model, d)
    for _ in range(600): stepc(d, CROUCH, 0, 0)
    for k in range(1500): stepc(d, CROUCH + smooth(k/1500)*(STAND-CROUCH), 0, 0)
    return d

fig, ax = plt.subplots(1, 3, figsize=(16, 5.5))

WPs = [(2.0,0.0),(2.0,2.5),(-1.5,2.5),(-1.5,-1.0),(0.5,-1.0)]
d = standing(); traj = []; reached = []
for tx, ty in WPs:
    for k in range(int(18*500)):
        dx, dy = tx-d.qpos[0], ty-d.qpos[1]; dist = math.hypot(dx, dy)
        if dist < 0.25: reached.append(True); break
        herr = wrap(math.atan2(dy, dx) - yaw(d))
        # Blended (no mode switch = no wiggle): turn continuously; forward speed scaled
        # by how well we face the target (cos), so it pivots when off, drives when aligned.
        w = max(-2.0, min(2.0, 1.8*herr))
        v = min(0.5, 0.7*dist) * max(0.0, math.cos(herr))
        cmd_vel(d, v, w)
        if k % 10 == 0: traj.append((d.qpos[0], d.qpos[1]))
    else: reached.append(False)
traj = np.array(traj)
ax[0].plot(traj[:,0], traj[:,1], '-b', lw=1.5, label='actual')
for i,(x,y) in enumerate(WPs):
    ax[0].plot(x, y, 'r*', ms=16); ax[0].annotate(f'WP{i+1}', (x,y), textcoords="offset points", xytext=(6,6))
ax[0].plot(0, 0, 'go', label='start')
ax[0].set_title(f"waypoint course ({sum(reached)}/{len(WPs)} reached)"); ax[0].axis('equal'); ax[0].grid(True); ax[0].legend()

d = standing(); c = []
for k in range(int(30*500)):
    cmd_vel(d, 0.5, 1.2)
    if k % 10 == 0: c.append((d.qpos[0], d.qpos[1]))
c = np.array(c); closure = math.hypot(c[-1,0]-c[1,0], c[-1,1]-c[1,1])
ax[1].plot(c[:,0], c[:,1], '-g'); ax[1].plot(c[1,0], c[1,1], 'go')
ax[1].set_title(f"full circle (closure gap={closure:.2f} m)"); ax[1].axis('equal'); ax[1].grid(True)

d = standing(); f = []
for k in range(int(44*500)):
    t = k/500.0
    cmd_vel(d, 0.5, 1.2 if (t % 22) < 11 else -1.2)
    if k % 10 == 0: f.append((d.qpos[0], d.qpos[1]))
f = np.array(f)
ax[2].plot(f[:,0], f[:,1], '-m'); ax[2].plot(f[0,0], f[0,1], 'go')
ax[2].set_title("figure-8"); ax[2].axis('equal'); ax[2].grid(True)

plt.tight_layout(); plt.savefig(OUT, dpi=90)
print(f"waypoints reached: {sum(reached)}/{len(WPs)}")
print(f"circle closure gap: {closure:.2f} m")
print(f"saved {OUT}")
