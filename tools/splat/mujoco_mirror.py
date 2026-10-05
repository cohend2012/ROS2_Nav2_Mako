#!/usr/bin/env python3
"""Native MuJoCo viewer that MIRRORS the running sim (Windows side of the splat demo).

The physics runs headless in WSL (m20_sim_run); this window loads the SAME scene file
and poses the M20 from the live state that tools/splat/mujoco_mirror_relay.py streams
(base pose + 16 joints). It is a kinematic mirror: no physics is stepped here.

Run on Windows (needs `pip install mujoco`):
  python tools\\splat\\mujoco_mirror.py                 # localhost:8770, scene from the relay
  python tools\\splat\\mujoco_mirror.py --follow        # camera tracks the robot
Env/args: --host/--port, --mjcf-dir (defaults to the WSL model dir over \\\\wsl.localhost).
"""
import argparse, os, socket, time
import mujoco
import mujoco.viewer

DEFAULT_DIR = r"\\wsl.localhost\Ubuntu-24.04\home\{user}\m20_sim\sdk_deploy\src\M20_sdk_deploy\M20_description\m20_mjcf\mjcf"


def lines(sock):
    buf = b""
    while True:
        chunk = sock.recv(65536)
        if not chunk:
            return
        buf += chunk
        *done, buf = buf.split(b"\n")
        for l in done:
            yield l.decode()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=8770)
    ap.add_argument("--wsl-user", default=os.environ.get("M20_WSL_USER", "svedula24"))
    ap.add_argument("--mjcf-dir", default=None)
    ap.add_argument("--follow", action="store_true", help="camera follows the robot")
    a = ap.parse_args()
    mjcf_dir = a.mjcf_dir or DEFAULT_DIR.format(user=a.wsl_user)

    while True:   # (re)connect: the demo restarts the sim between runs
        try:
            sock = socket.create_connection((a.host, a.port), timeout=5)
            sock.settimeout(None)
        except OSError:
            print(f"waiting for the relay on {a.host}:{a.port} (is the demo running?) ...")
            time.sleep(3)
            continue
        stream = lines(sock)
        first = next(stream, "")
        scene = first.split(" ", 1)[1] if first.startswith("scene ") else "indoor_splat.xml"
        path = os.path.join(mjcf_dir, scene)
        print(f"loading {path}")
        m = mujoco.MjModel.from_xml_path(path)
        d = mujoco.MjData(m)
        with mujoco.viewer.launch_passive(m, d) as v:
            if a.follow:
                v.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
                v.cam.trackbodyid = 1          # robot base body
                v.cam.distance, v.cam.elevation = 3.5, -25
            try:
                for l in stream:
                    if not v.is_running():
                        return
                    vals = [float(x) for x in l.split(",")]
                    if len(vals) != 23:
                        continue
                    d.qpos[:23] = vals
                    mujoco.mj_forward(m, d)
                    v.sync()
            except (OSError, ValueError):
                pass
        print("relay disconnected (demo restarting?) — reconnecting")
        time.sleep(2)


if __name__ == "__main__":
    main()
