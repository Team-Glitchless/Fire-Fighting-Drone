# Fire-Fighting-Drone

An autonomous drone stack that explores an unknown indoor environment,
incrementally maps it, and coordinates search-and-rescue by detecting humans
and reporting their world-frame coordinates.

This repository integrates what used to be two separate, disconnected
branches (`controller`: flight control + human detection, and `master`:
OctoMap-based exploration planning) into a single ROS 2 (Jazzy) workspace,
replaces the previous 10-ray greedy heuristic with a proper **RRT-based
Next-Best-View Planner (NBVP)**, and wires exploration, navigation and
detection together end to end.

## Architecture

| Package                 | Type          | Contains |
|--------------------------|---------------|----------|
| `fire_fighting_drone_cpp` | `ament_cmake` | `nbvp_planner_node` (RRT-based exploration planner over OctoMap) and `pointcloud_transform_node` (ORB-SLAM2 point cloud → world frame) |
| `fire_fighting_drone`     | `ament_python`| `flight_controller` (MAVROS/PX4 position + velocity control, consumes `/next_goal`), `trajectory_follower` (minimum-snap trajectory + PID velocity tracking), `object_detection` (YOLOv3 human detection, publishes `/detected_humans`) |
| `worlds/`                 | Gazebo assets | Simulation worlds used for SITL testing |

### Data flow

```
ORB-SLAM2 point cloud ──▶ pointcloud_transform_node ──▶ octomap_server (3rd-party) ──▶ /octomap_binary
                                                                                             │
                                                                                             ▼
                              /mavros/local_position/pose ──▶ nbvp_planner_node (RRT-NBVP)
                                                                                             │
                                                                                     /next_goal
                                                                                             │
                                                                                             ▼
                                                            flight_controller ──▶ MAVROS ──▶ PX4 SITL
                                                                                             │
                              /r200/rgb, /r200/depth ──▶ object_detection ──▶ /detected_humans
```

`octomap_server` itself is a well-maintained third-party ROS 2 package (not
vendored here) that subscribes to the transformed point cloud (`/pcl_out`)
and publishes the `/octomap_binary` message the planner consumes.

### RRT-based Next-Best-View Planner

`nbvp_planner_node` grows an RRT rooted at the drone's current pose over the
live OctoMap (following Bircher et al., 2016, *"Receding Horizon 'Next-Best-
View' Planner for 3D Exploration"*):

1. Sample a random point inside the configured exploration bounds.
2. Steer from the nearest tree node towards the sample, capped at
   `extension_range`.
3. Reject the edge if it passes through a **known-occupied** voxel (extending
   into *unknown* space is allowed and is in fact the point of exploration).
4. Estimate the information gain of the new viewpoint by casting a grid of
   rays through the sensor FOV at several candidate headings and counting
   unknown voxels observed before a known obstacle is hit; keep the best
   heading.
5. Accumulate a distance-discounted utility (`gain * exp(-lambda * cost)`)
   along the tree.
6. After `max_iterations`, pick the tree node with the highest accumulated
   utility, publish the root→best branch (`/nbvp_best_branch`), and publish
   the first edge as the next receding-horizon waypoint on `/next_goal`.
7. If no viewpoint's gain exceeds `gain_threshold`, publish
   `/exploration_complete = true` and stop replanning.

All parameters are configurable, see `fire_fighting_drone/config/params.yaml`.

## Prerequisites (WSL2, Ubuntu, ROS 2 Jazzy)

ROS 2 Jazzy targets Ubuntu 24.04 (Noble). These steps assume a Windows host
running **WSL2** with an **Ubuntu-24.04** distro.

1. **Install WSL2 + Ubuntu 24.04** (on Windows PowerShell, as Administrator):
   ```powershell
   wsl --install -d Ubuntu-24.04
   ```
   Reboot if prompted, then open the Ubuntu-24.04 shell and finish the
   first-run user setup.

2. **Update the system** (inside the WSL2 Ubuntu shell):
   ```bash
   sudo apt update && sudo apt upgrade -y
   ```

3. **Install ROS 2 Jazzy** (binary packages, desktop install):
   ```bash
   sudo apt install -y software-properties-common curl gnupg lsb-release
   sudo add-apt-repository universe
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
     http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
     | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   sudo apt update
   sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions \
     python3-rosdep python3-pip ros-jazzy-rmw-cyclonedds-cpp
   sudo rosdep init
   rosdep update
   echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

4. **Install ROS 2 dependencies used by this project**:
   ```bash
   sudo apt install -y \
     ros-jazzy-mavros ros-jazzy-mavros-extras \
     ros-jazzy-octomap ros-jazzy-octomap-msgs ros-jazzy-octomap-ros \
     ros-jazzy-cv-bridge ros-jazzy-tf-transformations \
     ros-jazzy-tf2-sensor-msgs ros-jazzy-tf2-geometry-msgs

   # GeographicLib datasets required by MAVROS:
   sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
   ```
   Install `octomap_server` for ROS 2 (used to build `/octomap_binary` from
   the transformed point cloud). If a binary `ros-jazzy-octomap-server`
   package is not available for your Ubuntu release, build the
   [`octomap_mapping`](https://github.com/OctoMap/octomap_mapping) ROS 2
   port from source into the same workspace (see step 6).

5. **Install PX4-Autopilot SITL** (Gazebo simulation target):
   ```bash
   cd ~
   git clone https://github.com/PX4/PX4-Autopilot.git --recursive
   bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
   ```
   Follow the PX4 docs for `make px4_sitl gazebo-classic` (or the Gazebo
   Harmonic target, depending on your PX4 version) to confirm SITL runs
   before integrating with this project.

6. **Install the remaining Python dependencies** (YOLOv3/trajectory
   generation are plain Python, not ROS packages):
   ```bash
   python3 -m pip install -r requirements.txt
   ```

## Building the workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <this-repo-url> Fire-Fighting-Drone
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Download YOLOv3 weights and convert them to the TensorFlow checkpoint format
expected by `ml_helper.py`/`object_detection.py` (`checkpoints/yolov3.tf`),
then point the `weights_path` parameter in
`fire_fighting_drone/config/params.yaml` at that checkpoint.

## Running the full mission

1. **Start PX4 SITL + Gazebo** with a world from `worlds/` (e.g. `world3.world`).
2. **Start MAVROS** connected to the PX4 SITL instance:
   ```bash
   ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557
   ```
3. **Start ORB-SLAM2** (or your chosen visual SLAM front-end) publishing
   `/orb_slam2_rgbd/map_points`.
4. **Start `octomap_server`**, subscribed to `/pcl_out` (published by
   `pointcloud_transform_node`), publishing `/octomap_binary`.
5. **Launch the Fire-Fighting-Drone mission** (point-cloud transform, RRT
   NBVP planner, flight controller, human detector):
   ```bash
   ros2 launch fire_fighting_drone mission.launch.py
   ```

The flight controller arms, takes off, switches to `OFFBOARD` mode, and then
autonomously flies to each viewpoint published by the NBVP planner on
`/next_goal` until `/exploration_complete` is published. Detected humans are
published on `/detected_humans` (`geometry_msgs/PointStamped`, world frame)
for downstream search-and-rescue coordination.

To fly a pre-defined minimum-snap path instead of following exploration
goals (useful for testing the controller/trajectory stack in isolation):
```bash
ros2 run fire_fighting_drone trajectory_follower
```

## Notable fixes made during the ROS 2 migration

* `pid.py` previously mixed scalar initial state with vector usage and never
  actually updated `lastError` (`self.lastError = self.lastError` was a
  no-op). Both bugs are fixed in the ported `fire_fighting_drone/pid.py`.
* The ROS 1 NBVP heuristic (`treestuff.cpp`) cast 10 fixed rays from the
  current pose only and had an unreachable/inverted coverage-termination
  check (`percent < 80` compared a 0–1 fraction against `80`). It is replaced
  by the RRT-based planner described above.
* The ROS 1 planner published `current_position - candidate_position` (a
  vector *away* from the goal) on `next_goal`, and nothing subscribed to it.
  The new planner publishes the absolute world-frame target position, and
  `flight_controller` subscribes to it and flies there automatically.
* `object_detection.py`'s detection loop previously only `print()`ed
  detected-human coordinates and had a center-finding bug that always read
  box index 0. It now publishes every detection on `/detected_humans` and
  fixes the indexing bug.

## Known limitations / follow-up work

* `octomap_server` is not vendored in this repository; you must install or
  build it separately (see step 4 above).
* The RRT-NBVP parameters (bounds, sensor FOV, iteration count) are tuned
  for the included `worlds/world3.world` scale and will need adjusting for
  other environments.
* This workspace has not been build-tested against a live ROS 2 Jazzy/PX4
  installation in this sandbox (no ROS 2 toolchain is available here); please
  run `colcon build` and the mission launch in your WSL2 environment and
  file an issue with any build errors.
