# GPS-Denied Drone Navigation with PX4, ROS 2, and Gazebo

A robotics project scaffold for developing autonomous quadrotor navigation when GNSS/GPS is unavailable or unreliable. The repository currently contains a reusable **2D grid-based A\* path planner**, a ROS 2 Python node that consumes an occupancy grid and start/goal poses, a launch file, and unit tests. The longer-term goal is to connect mapping and GPS-independent localization to a path follower and PX4 Offboard control, then validate the system in PX4 SITL and Gazebo.

> **Project status — in progress:** The standalone A\* algorithm and its six unit tests are implemented. The ROS 2 node, PX4 transport, sensor-to-map pipeline, GPS-independent localization, path following, and autonomous flight have **not yet been verified end to end**. Do not present the project as a completed autonomous drone system until those milestones have been demonstrated.

## Table of contents

- [Motivation](#motivation)
- [Objectives and scope](#objectives-and-scope)
- [Current features](#current-features)
- [System architecture](#system-architecture)
- [Repository structure](#repository-structure)
- [Technology stack](#technology-stack)
- [Prerequisites](#prerequisites)
- [Build and run the ROS 2 package](#build-and-run-the-ros-2-package)
- [ROS 2 interfaces](#ros-2-interfaces)
- [How the A* planner works](#how-the-a-planner-works)
- [Run the unit tests](#run-the-unit-tests)
- [PX4 SITL and Gazebo integration](#px4-sitl-and-gazebo-integration)
- [Validation plan and metrics](#validation-plan-and-metrics)
- [Limitations and safety](#limitations-and-safety)
- [Roadmap](#roadmap)
- [Contributing](#contributing)
- [License](#license)

## Motivation

A conventional drone often relies on GNSS/GPS for global position. Indoor spaces, covered areas, urban canyons, and deliberately denied environments can make that signal unavailable or unreliable. A drone operating in such places needs another way to estimate its motion, represent nearby obstacles, and choose a collision-avoiding route.

This project explores a modular approach: localization estimates the vehicle pose, perception/mapping creates an occupancy grid, a planner finds a route through free cells, and a controller follows that route. PX4 remains responsible for low-level flight control and safety-related flight modes.

## Objectives and scope

### Current milestone

1. Implement a reusable A\* search over a 2D occupancy grid.
2. Expose the planner through a ROS 2 Python node.
3. Define a small topic interface between map, poses, and planned path.
4. Test path-planning edge cases independently of a running ROS 2 installation.

### Intended final system

- Receive simulated depth-camera or LiDAR observations.
- Build/update a local obstacle map.
- Estimate position and orientation without relying on GPS, using a suitable visual, visual-inertial, or other odometry/localization system.
- Plan collision-avoiding paths to a target.
- Convert the route into dynamically feasible setpoints and execute them through PX4 Offboard control.
- Measure success rate, path length, clearance, and planning/control latency in repeatable simulation scenarios.

The final-system items are goals, not claims of existing functionality.

## Current features

- A reusable `astar_grid(...)` function independent of ROS 2.
- Four-connected or eight-connected grid movement; diagonal movement is enabled by default.
- Octile-distance heuristic for diagonal motion and Manhattan distance for cardinal-only motion.
- Diagonal corner-cutting prevention: a diagonal step is rejected if either adjacent orthogonal cell is blocked.
- Bounds checking, blocked start/goal handling, and a clear no-path result (`None`).
- ROS 2 node that subscribes to an occupancy grid, current start pose, and goal pose, then publishes a `nav_msgs/msg/Path`.
- Parameters to configure the occupancy threshold and treatment of unknown cells.
- Unit tests for normal routing and important edge cases.

## System architecture

```text
Simulated camera / LiDAR (planned)
              |
              v
     Perception and mapping (planned) ----> nav_msgs/OccupancyGrid on /map
                                                   |
Current pose from GPS-independent localization   |
  geometry_msgs/PoseStamped on /start_pose        |
                                                   v
Goal pose: geometry_msgs/PoseStamped on /goal_pose
                         |                         |
                         +-----------+-------------+
                                     v
                              ROS 2 A* planner
                                     |
                                     v
                       nav_msgs/Path on /planned_path
                                     |
                                     v
                  Path follower + safety checks (planned)
                                     |
                                     v
                      PX4 Offboard interface (planned)
                                     |
                                     v
                         PX4 SITL + Gazebo X500
```

The planner does **not** implement SLAM, visual-inertial odometry, sensor drivers, coordinate-frame transforms, or a flight controller. Those are separate components that must be selected, integrated, and tested.

## Repository structure

```text
gps_denied_drone_navigation/
├── README.md
├── LICENSE
├── .gitignore
├── requirements-dev.txt
├── docs/
│   └── DEVELOPMENT_PLAN.md
└── gps_denied_navigation/                  # ROS 2 ament_python package
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    ├── resource/gps_denied_navigation
    ├── gps_denied_navigation/
    │   ├── __init__.py
    │   └── astar_planner.py
    ├── launch/
    │   └── planner.launch.py
    └── test/
        └── test_astar_planner.py
```

## Technology stack

| Component | Role | Status |
|---|---|---|
| Python 3 | A\* algorithm and ROS 2 node | Implemented |
| ROS 2 Jazzy | Node runtime and message transport | Package integration requires local verification |
| `rclpy` | Python ROS 2 API | Used by the node |
| `nav_msgs` | Occupancy grid and path messages | Used by the node |
| `geometry_msgs` | Start and goal poses | Used by the node |
| PX4 Autopilot SITL | Flight-control simulation | Separate dependency; DDS connection still needs verification |
| Gazebo Harmonic / X500 | Simulated environment and vehicle | Simulation startup has been exercised; autonomous navigation not verified |
| Micro XRCE-DDS Agent | PX4–ROS 2 transport for supported PX4 topics | Agent ran locally; ROS 2 did not yet show PX4 `/fmu` topics |
| Camera/LiDAR and odometry | Obstacle mapping and GPS-independent localization | Planned; not integrated |

## Prerequisites

For the ROS 2 package:

- Ubuntu 24.04 or a compatible ROS 2 Jazzy environment (the development environment used for this project is Pop!_OS 24.04).
- ROS 2 Jazzy installed and sourced.
- `colcon`, `rosdep`, and the standard ROS 2 Python/message dependencies.

For the future simulator integration, additionally install PX4 SITL, Gazebo Harmonic, and the PX4-compatible ROS 2/DDS tooling. Keep the PX4 and Micro XRCE-DDS Agent source trees as separate dependencies rather than copying their complete repositories into this project.

Source ROS 2 in each terminal used for ROS commands:

```bash
source /opt/ros/jazzy/setup.bash
```

## Build and run the ROS 2 package

Run the following from the extracted repository root. If you extracted the archive somewhere other than `~/Downloads`, adjust the source path accordingly.

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/ros2_ws/src
cp -r ~/Downloads/gps_denied_drone_navigation/gps_denied_navigation \
  ~/ros2_ws/src/
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select gps_denied_navigation
source install/setup.bash
```

Run the node directly:

```bash
ros2 run gps_denied_navigation astar_planner
```

Or use the included launch file:

```bash
ros2 launch gps_denied_navigation planner.launch.py
```

The node will wait until it receives a map and then a goal. It also requires a start pose. Warnings about missing inputs are expected until the corresponding topics are published. Running the node alone does not move a drone.

### Avoid copying duplicate package versions

If `~/ros2_ws/src/gps_denied_navigation` already exists, do not blindly copy over it. Back up or update that workspace package deliberately so you know which source version is being built.

## ROS 2 interfaces

| Topic | Message type | Direction | Purpose |
|---|---|---|---|
| `/map` | `nav_msgs/msg/OccupancyGrid` | Subscribe | 2D grid containing free, occupied, and unknown cells |
| `/start_pose` | `geometry_msgs/msg/PoseStamped` | Subscribe | Current vehicle position in the map frame |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | Subscribe | Desired destination in the same frame as the map |
| `/planned_path` | `nav_msgs/msg/Path` | Publish | Sequence of grid-cell-centre poses from start to goal |

### Coordinate and map requirements

- The start and goal poses must use the same coordinate frame as `/map`.
- This version does not perform TF transforms. Transform incoming poses before publishing them if their frames differ.
- The map's origin, resolution, width, height, and data array must be valid.
- Occupancy values greater than or equal to `occupied_threshold` are treated as blocked. Unknown cells (`-1`) are blocked by default.
- A path pose is placed at the centre of each selected grid cell. The current implementation preserves the map frame and origin height; it does not plan altitude or vehicle dynamics.

### Parameters

| Parameter | Default | Meaning |
|---|---:|---|
| `occupied_threshold` | `50` | Cells with occupancy values at or above this value are obstacles |
| `allow_unknown` | `false` | If `true`, unknown cells may be traversed; use this cautiously |

## How the A* planner works

A\* searches a graph whose nodes are grid cells. Each free cell is connected to its valid neighbouring cells. The algorithm maintains the cost already paid to reach a cell, `g(n)`, and estimates the remaining cost to the goal, `h(n)`. It expands the cell with the smallest `f(n) = g(n) + h(n)` until the goal is reached or no route remains.

- Horizontal and vertical steps cost `1`.
- Diagonal steps cost `sqrt(2)` when diagonal movement is enabled.
- The heuristic is Manhattan distance for cardinal movement and octile distance for diagonal movement.
- A cell is not entered if it is outside the grid or marked blocked.
- Diagonal movement cannot pass between two blocked orthogonal neighbours.
- The result includes both start and goal cells. If no route exists, the function returns `None`.

This is a grid-level geometric planner. It does not account for drone radius, uncertainty, velocity, acceleration, turning limits, moving obstacles, map age, or three-dimensional clearance. Those constraints must be handled by map inflation, trajectory generation, and a safety layer before flight testing.

## Run the unit tests

The pure A\* tests can run with standard Python and do not require a running ROS 2 graph. From the repository root:

```bash
cd gps_denied_navigation
PYTHONPATH=. python3 -m unittest discover -s test -v
```

From a built ROS 2 workspace, run the package tests with:

```bash
cd ~/ros2_ws
colcon test --packages-select gps_denied_navigation
colcon test-result --verbose
```

The test suite covers an open-grid route, blocked goal, obstacle avoidance, diagonal corner-cutting prevention, identical start and goal, and out-of-bounds input. A passing unit test suite verifies the tested algorithm cases only; it does not verify ROS 2 integration or flight behaviour.

## PX4 SITL and Gazebo integration

PX4 should remain a separate repository. In the development setup it is located at `~/Documents/drone_ps/PX4-Autopilot`. Start components in separate terminals and keep each foreground process running while testing.

### 1. Start PX4 SITL and Gazebo

```bash
source /opt/ros/jazzy/setup.bash
cd ~/Documents/drone_ps/PX4-Autopilot
QT_QPA_PLATFORM=xcb make px4_sitl gz_x500
```

### 2. Start the Micro XRCE-DDS Agent

Use this only if the Agent has already been built at the shown path:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/src/Micro-XRCE-DDS-Agent-v2.4.3
./build-system/MicroXRCEAgent udp4 -p 8888
```

PX4's SITL configuration is expected to use localhost (`127.0.0.1`) and UDP port `8888` by default in this environment. Confirm that the PX4 client configuration and Agent settings match before troubleshooting further.

### 3. Verify the ROS 2 graph

In a third terminal:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list -t
```

A working PX4–ROS 2 bridge should expose PX4 topics, often under `/fmu/`. The exact topic set depends on PX4 configuration and version. In the environment used to prepare this repository, the Agent was listening on port `8888` and PX4 was running, but ROS 2 still showed only `/parameter_events` and `/rosout`. The DDS connection is therefore a known outstanding issue, not a completed integration.

Do not proceed to Offboard flight commands until the DDS topics are visible, the required message streams are understood, and a safe simulation test procedure is in place.

## Validation plan and metrics

Use repeatable Gazebo worlds and record each trial. Recommended measures:

- **Goal success rate:** completed routes divided by total attempts.
- **Path length:** sum of the distances between consecutive waypoints.
- **Minimum obstacle clearance:** smallest distance between the vehicle footprint and mapped obstacles.
- **Planning latency:** time from a valid planning request to path publication.
- **Replanning behaviour:** recovery after a new obstacle appears or the map changes.
- **Control tracking error:** distance between the planned trajectory and estimated vehicle position once a controller is integrated.
- **Localization drift:** estimated-position error against simulator ground truth, used for evaluation only.

Do not report numerical results until they have been collected from documented experiments. A useful comparison would evaluate a straight-line baseline against A\* in several obstacle layouts and repeat each scenario multiple times.

## Limitations and safety

- This repository is **not flight-ready software**.
- GPS-denied navigation requires a reliable localization method; avoiding GPS alone does not provide position estimates.
- The planner operates in 2D and does not create a dynamically feasible trajectory.
- No SLAM, camera/LiDAR driver, occupancy-grid builder, TF transform logic, path follower, PX4 Offboard setpoint publisher, or failsafe supervisor is included in this milestone.
- A map may be stale or incomplete. A route through free cells is not a guarantee of collision-free flight.
- Before any real flight, add obstacle inflation for vehicle dimensions, sensor/map freshness checks, velocity and acceleration limits, geofencing where applicable, manual override, failsafe behaviour, and extensive simulation/hardware-in-the-loop validation.
- Test in simulation first. Never test autonomous navigation near people or in an uncontrolled environment.

## Roadmap

See [`docs/DEVELOPMENT_PLAN.md`](docs/DEVELOPMENT_PLAN.md) for the detailed development plan. The high-level sequence is:

1. Build and verify the ROS 2 package in Jazzy.
2. Resolve PX4–ROS 2 DDS topic visibility.
3. Create a repeatable occupancy-grid input from a simulated sensor or test map.
4. Verify start/goal-to-path planning through ROS 2 topics.
5. Integrate GPS-independent localization and coordinate-frame handling.
6. Add path smoothing, vehicle-clearance margins, and a safety monitor.
7. Add a path follower and PX4 Offboard integration in SITL.
8. Record repeatable experiments, results, screenshots, and a demo video.

## Contributing

Contributions and issue reports are welcome. For a useful report, include your OS, ROS 2 distribution, PX4 version/commit, Gazebo version, exact commands, relevant terminal output, and steps to reproduce. Add tests for algorithm changes and document any changes to topic names or message contracts. Keep generated build directories, logs, bag files, credentials, and machine-specific absolute paths out of commits.

## License

This project is released under the MIT License. See [`LICENSE`](LICENSE).
