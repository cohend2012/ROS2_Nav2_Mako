"""Trajectory generation + tracking (NOT point-to-point):
  1. Catmull-Rom spline through a LONG waypoint loop -> smooth continuous reference path
  2. Pure-pursuit controller tracks the path (lookahead) -> smooth, no bang-bang
Reports cross-track error (how tightly it follows the planned path)."""
import math, numpy as np, mujoco
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from pathlib import Path
M = Path.home()/"m20_sim/sdk_deploy/src/M20_sdk_deploy/M20_description/m20_mjcf/mjcf/M20.xml"
model = mujoco.MjModel.from_xml_path(str(M)); model.opt.timestep = 1/500
LO=model.actuator_ctrlrange[:,0].copy(); HI=model.actuator_ctrlrange[:,1].copy()
WHEELS=(3,7,11,15); LEFT=(3,11); RIGHT=(7,15)
STAND=np.array([0,-0.7,1.4,0,0,-0.7,1.4,0,0,0.7,-1.4,0,0,0.7,-1.4,0],float)
CROUCH=np.array([0,-1.0,2.3,0,0,-1.0,2.3,0,0,1.0,-2.3,0,0,1.0,-2.3,0],float)
R,B=0.10,0.40
# LONGER course: 8-waypoint loop spread over ~8 m
WPs=[(3,0),(4,3),(1,4),(-3,3.5),(-4,0),(-3,-3),(0,-3.5),(3,-2)]

def smooth(t): t=max(0,min(1,t)); return t*t*(3-2*t)
def yaw(d):
    w,x,y,z=d.qpos[3:7]; return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
def wrap(a): return math.atan2(math.sin(a),math.cos(a))
def stepc(d,legcmd,wl,wr):
    q=d.qpos[7:23]; qd=d.qvel[6:22]; tau=np.zeros(16)
    for i in range(16):
        tau[i]=(5.0*((wl if i in LEFT else wr)-qd[i])) if i in WHEELS else (200*(legcmd[i]-q[i])+4*(0-qd[i]))
    d.ctrl[:]=np.clip(tau,LO,HI); mujoco.mj_step(model,d)
def cmd_vel(d,v,w): stepc(d,STAND,(-v+w*B/2)/R,(-v-w*B/2)/R)
def standing():
    d=mujoco.MjData(model); d.qpos[:]=0; d.qpos[3]=1; d.qpos[7:23]=CROUCH
    d.qpos[2]=0.6; mujoco.mj_forward(model,d); d.qpos[2]=0.6-float(d.geom_xpos[:,2].min())+0.02; mujoco.mj_forward(model,d)
    for _ in range(600): stepc(d,CROUCH,0,0)
    for k in range(1500): stepc(d,CROUCH+smooth(k/1500)*(STAND-CROUCH),0,0)
    return d

def catmull_rom(pts, n=40):
    P=[pts[-1]]+list(pts)+[pts[0],pts[1]]; path=[]
    for i in range(1,len(P)-2):
        p0,p1,p2,p3=(np.array(P[j],float) for j in (i-1,i,i+1,i+2))
        for t in np.linspace(0,1,n,endpoint=False):
            path.append(0.5*((2*p1)+(-p0+p2)*t+(2*p0-5*p1+4*p2-p3)*t*t+(-p0+3*p1-3*p2+p3)*t*t*t))
    return np.array(path)

PATH=catmull_rom(WPs)                       # smooth reference trajectory
Ld=0.6                                      # pure-pursuit lookahead (tighter tracking)
d=standing(); i0=0; act=[]; cte=[]; onpath=[]
for k in range(int(90*500)):
    x,y,th=d.qpos[0],d.qpos[1],yaw(d)
    win=range(i0,i0+80)
    dists=[np.hypot(PATH[j%len(PATH)][0]-x,PATH[j%len(PATH)][1]-y) for j in win]
    i0=(i0+int(np.argmin(dists)))%len(PATH)
    e=min(dists); cte.append(e); onpath.append(e<1.0)   # "on path" once within 1 m
    j=i0; s=0.0
    while s<Ld:
        s+=np.hypot(PATH[(j+1)%len(PATH)][0]-PATH[j%len(PATH)][0],PATH[(j+1)%len(PATH)][1]-PATH[j%len(PATH)][1])
        j+=1
    tx,ty=PATH[j%len(PATH)]
    alpha=wrap(math.atan2(ty-y,tx-x)-th)
    v=0.5                                   # constant; turn radius is set by Ld, not speed
    w=2*v*math.sin(alpha)/Ld
    cmd_vel(d,v,max(-2.5,min(2.5,w)))
    if k%10==0: act.append((x,y))
    if k>500 and i0>=len(PATH)-3: break
act=np.array(act); cte=np.array(cte)
# steady-state error: after the robot first gets onto the path
try: start=onpath.index(True)
except ValueError: start=0
cte_ss=cte[start:]
plt.figure(figsize=(8,8))
plt.plot(PATH[:,0],PATH[:,1],'--',color='gray',label='planned trajectory (spline)')
plt.plot(act[:,0],act[:,1],'-b',lw=1.6,label='actual (pure-pursuit)')
for i,(wx,wy) in enumerate(WPs): plt.plot(wx,wy,'r*',ms=15); plt.annotate(f'WP{i+1}',(wx,wy))
plt.axis('equal'); plt.grid(True); plt.legend()
plt.title(f"Trajectory generation + pure-pursuit tracking (8-WP loop)\nsteady cross-track: mean={cte_ss.mean():.2f} m, max={cte_ss.max():.2f} m")
out="/mnt/c/Users/16024/AppData/Local/Temp/claude/C--GIT-StatefullXOne-m20-autonomy-ws-m20-autonomy-ws/06227379-f151-4c17-a580-0b0e4555e6cc/scratchpad/traj_benchmark.png"
plt.savefig(out,dpi=90)
print(f"path points: {len(PATH)}, waypoints: {len(WPs)}")
print(f"steady cross-track error: mean {cte_ss.mean():.2f} m, max {cte_ss.max():.2f} m")
print("saved", out)
