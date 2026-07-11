# Common entry points. `make help` lists targets.
.PHONY: help unit build test sim up down image gui-deps sim-setup sim-view benchmark

help:
	@echo "unit      - fast ROS-free logic tests (pytest + shims, runs anywhere)"
	@echo "image     - build the stack container"
	@echo "build     - colcon build inside the container"
	@echo "test      - colcon test inside the container (integration)"
	@echo "sim       - launch sim bringup inside the container"
	@echo "up        - docker compose up the full stack"
	@echo "down      - docker compose down"
	@echo "gui-deps  - one-time: install mesa/OpenGL for MuJoCo (sudo)"
	@echo "sim-setup - install mujoco + fetch pinned M20 model (no sudo)"
	@echo "sim-view  - open the MuJoCo viewer on the M20 (see the robot)"
	@echo "benchmark - movement regression: waypoint course + circle + figure-8"

unit:
	python3 -m pytest tests -q

image:
	docker build -f docker/Dockerfile -t m20_autonomy:latest .

build: image

test: image
	docker run --rm m20_autonomy:latest bash -lc \
	  "source /ws/install/setup.bash && cd /ws && colcon test && colcon test-result --verbose"

sim: image
	docker run --rm -it --network host --ipc host m20_autonomy:latest \
	  ros2 launch m20_bringup sim_bringup.launch.py

up:
	docker compose -f docker/compose.yaml up -d

down:
	docker compose -f docker/compose.yaml down

# --- MuJoCo sim preview (see the robot). Standalone until the ROS-integrated sim. ---
gui-deps:
	bash tools/install_gui_deps.sh

sim-setup:
	bash tools/setup_sim.sh

sim-view:
	bash tools/view_sim.sh

benchmark:
	python3 tools/benchmark.py
