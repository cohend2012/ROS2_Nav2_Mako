# M20 Dev Guide — run sims, poke the stack, add your own software

Everything runs in two Docker containers inside WSL (`Ubuntu`), talking over one
DDS bus (`ROS_DOMAIN_ID=42`):

| Container | Image | Role |
|---|---|---|
| `m20_sim_run` | `m20_sim` | MuJoCo physics + sensor sim (`tools/mujoco_sim.py`, mounted from the repo — edit & restart, no rebuild) |
| `docker-commander-1` | `m20_autonomy` | Everything else: commander, bridge, slam_toolbox, Nav2, your scripts. ROS 2 ws baked at `/ws`, runtime configs in `/cfg` |

## 1. Start / stop the whole stack

```bash
# from WSL, repo root (from Windows prefix with: wsl -d Ubuntu -e bash ...)
bash tools/nav/bringup_plan_a.sh     # sim + bridge + slam + Nav2 + goal + logging
bash tools/nav/kill_stack.sh         # tear it all down (keeps the container)
```

GOAL_X / GOAL_Y env vars override the goal. IMPORTANT (learned the hard way, see
TEST_PLAN "gotchas"): never restart the sim while keeping Nav2 up — always fresh-stack.

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

## 5. The LiDAR interface switch (current D task) — what changes

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
