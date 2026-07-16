# M20 Dev Guide — run sims, poke the stack, add your own software

Everything runs in two Docker containers inside WSL (`Ubuntu`), talking over one
DDS bus (`ROS_DOMAIN_ID=42`):

| Container | Image | Role |
|---|---|---|
| `m20_sim_run` | `m20_sim` | MuJoCo physics + sensor sim (`tools/mujoco_sim.py`, mounted from the repo — edit & restart, no rebuild) |
| `docker-commander-1` | `m20_autonomy` | Everything else: commander, bridge, slam_toolbox, Nav2, your scripts. ROS 2 ws baked at `/ws`, runtime configs in `/cfg` |

## 1. Start / stop the whole stack

Two ways, depending on what you want:

### Two-terminal interactive workflow (recommended for driving/testing)
Backgrounded processes do NOT survive a closed session on this box, so Terminal 1
must stay open holding the stack:

```powershell
# TERMINAL 1 (PowerShell) — brings up sim+bridge+slam+Nav2, then STAYS OPEN.
#   Leave it running. Ctrl-C here tears the whole stack down.
wsl -d Ubuntu -e bash /mnt/c/GIT/StatefullXOne/m20_autonomy_ws/m20_autonomy_ws/tools/dev/sim_up.sh
```
```powershell
# TERMINAL 2 (PowerShell) — a ROS 2 shell to drive/inspect. Open as many as you like.
wsl -d Ubuntu -e bash /mnt/c/GIT/StatefullXOne/m20_autonomy_ws/m20_autonomy_ws/tools/dev/m20sh
```
(Note: run these from PowerShell/cmd, NOT Git Bash — Git Bash mangles the /mnt path.)

### One-shot self-test (bringup + one goal + teardown-safe)
```bash
bash tools/nav/bringup_plan_a.sh     # sim + bridge + slam + Nav2 + one goal + logging
bash tools/nav/kill_stack.sh         # tear it all down (keeps the commander container)
```
This backgrounds everything and exits, so it's for scripted runs (like the CI gate),
NOT for holding the stack open for a second terminal — use sim_up.sh for that.

IMPORTANT (learned the hard way, see TEST_PLAN "gotchas"): never restart the sim while
keeping Nav2 up — always fresh-stack. sim_up.sh and bringup do this for you.

## 2. Get a ROS 2 shell (your main tool)

```bash
bash tools/dev/m20sh
```

Drops you into the container with ROS sourced. Cookbook:

```bash
ros2 topic list                              # what's on the bus
ros2 topic echo /scan --once                 # one LiDAR scan
ros2 topic hz /LIDAR/POINTS                  # cloud rate (after interface switch)
ros2 run tf2_ros tf2_echo map base_link      # where SLAM thinks the robot is
ros2 topic echo /odom_true --once            # ground truth (eval only!)
ros2 service call /m20/set_mode m20_msgs/srv/SetMode "{mode: 1, requester: me}"   # arm
ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}}"            # drive (armed)
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: -6.0, y: -0.5}, orientation: {z: 1.0, w: 0.0}}}}"
```

## 3. Run your own software — three patterns

**A. Quick experiment (a single Python node)** — write `my_node.py` anywhere in the
repo, then:
```bash
docker cp my_node.py docker-commander-1:/cfg/my_node.py
bash tools/dev/m20sh          # then inside:
python3 /cfg/my_node.py
```
`rclpy`, `nav_msgs`, `sensor_msgs`, our `m20_msgs`/`drdds` are all importable.
This is how `drive_check.py`, `nav_logger.py`, the estimator etc. run today.

**B. Change an existing package (e.g. the bridge)** — edit under `src/…` in the
repo, then copy + rebuild just that package inside the container:
```bash
docker cp src/m20_locomotion_bridge docker-commander-1:/ws/src/
docker exec docker-commander-1 bash -lc \
  "source /opt/ros/humble/setup.bash; cd /ws; colcon build --packages-select m20_locomotion_bridge"
```
(Bridge specifically also runs fine as pattern A: `python3 /cfg/bridge_node.py
--ros-args -p sdk_backend:=sim` — that's what bringup does.)

**C. New permanent package** — create `src/my_pkg/` (ament_python or ament_cmake),
build as in B. When it's stable, add it to `docker/Dockerfile` so the image stays
reproducible, and rebuild the image when disk allows.

**Editing the sim** needs no build at all: `tools/mujoco_sim.py` is volume-mounted;
edit, then restart the stack.

## 4. Test your ideas (the gates)

```bash
docker exec docker-commander-1 python3 /cfg/drive_check.py        # L1.2 signs
docker exec docker-commander-1 python3 /cfg/odom_drift_check.py   # L1.3 drift model
bash tools/ci/precommit.sh                                        # CI tier 1 (<5 s, also a git hook)
bash tools/ci/batch_nav_test.sh 10                                # L3.2 reliability gate (~20 min)
```

Rule from TEST_PLAN.md: score against `/odom_true`, but never feed it to the stack.

## 5. Moving the robot around — teleop & RViz

**Right now, no install needed** (from a Terminal-2 `m20sh` shell):
```bash
# manual driving (raw teleop — the robot is armed by sim_up):
ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}}"     # forward
ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist "{angular: {z: 0.5}}"    # spin
# autonomous — let Nav2 plan+drive to a point:
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: -3.0, y: 1.0}, orientation: {w: 1.0}}}}"
```

**RViz (click-to-navigate + live visualization).** Possible on this machine — WSLg is
present (GPU-accelerated GUI) — but NOT installed yet, and it's a chunky add on a
disk-tight box, so it's opt-in:
- Needs: `apt install ros-humble-rviz2 ros-humble-nav2-rviz-plugins` in the container
  (add to `docker/Dockerfile` to persist), plus WSLg display wiring on the container
  (`-e DISPLAY -e WAYLAND_DISPLAY -v /mnt/wslg:/mnt/wslg -v /tmp/.X11-unix:/tmp/.X11-unix`).
- Then `rviz2` shows the map, robot, LiDAR scan, and costmaps live, and the **"Nav2 Goal"**
  toolbar button lets you click a point and watch the robot navigate there — exactly the
  "ask the robot to move around" experience.
- Lighter middle option: `teleop_twist_keyboard` (tiny) for WASD-style keyboard driving.
- Recommendation: use the CLI above for now; stand up RViz as its own task when you want
  the visual (it's a real image change + GUI plumbing, worth doing deliberately).

## 6. The LiDAR interface switch (current D task) — what changes

Today the sim publishes a 360-ray 2D `/scan` directly — a simplified slice of the
real sensor, and the WRONG interface (the real robot never publishes /scan).
Target (matches the real M20 exactly; see LIDAR_RESEARCH.md):

| | Today (sim shortcut) | After the switch (= real robot) |
|---|---|---|
| Sensor topic | `/scan` (LaserScan) from sim | `/LIDAR/POINTS` (PointCloud2, merged, body frame) |
| 2D scan | native | derived by the real `pointcloud_to_laserscan` node |
| 3D data | none | full cloud available for Plan B (STVL/elevation) |

Checklist:
1. `apt install ros-humble-pointcloud-to-laserscan` in the container + add to
   `docker/Dockerfile` (it is NOT in the image today — verified).
2. `tools/mujoco_sim.py`: publish reduced-beam hemispherical PointCloud2 on
   `/LIDAR/POINTS` (0.5 m blind radius per vendor config), keep `/odom_true`.
3. `bringup_plan_a.sh`: launch pointcloud_to_laserscan (cloud → `/scan`).
4. slam/Nav2 configs: UNCHANGED — they already consume `/scan`.
5. Re-run the L3.2 gate; D merges only on a clean pass.

On the REAL robot nothing is "switched": it already publishes `/LIDAR/POINTS`; we
subscribe, and the same projection node + configs run as-is. Day-1 hardware checks:
`ros2 run tf2_ros tf2_echo base_link lidar_link`, confirm 0.5 m blind zone, wall test
for the vertical FOV band.
