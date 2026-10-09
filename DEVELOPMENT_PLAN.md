# Development plan

## Milestone 1 — Project baseline
- [x] Create a ROS 2 `ament_python` package.
- [x] Implement grid-based A* with diagonal corner-cutting prevention.
- [x] Add unit tests for path existence, blocked goals, obstacles, and boundaries.
- [ ] Build and run package in the target ROS 2 Jazzy environment.

## Milestone 2 — ROS 2 and PX4 data path
- [ ] Diagnose the current PX4 uXRCE-DDS connection (Agent listens on UDP 8888, but PX4 topics were not visible in ROS 2 during initial setup).
- [ ] Verify expected `/fmu/out/*` topics with `ros2 topic list -t`.
- [ ] Add a tested PX4 vehicle-state adapter and publish a normalized `/start_pose`.

## Milestone 3 — Localization and mapping
- [ ] Select a simulated sensor (depth camera or LiDAR).
- [ ] Integrate a mapping or SLAM package appropriate for the sensor and environment.
- [ ] Verify that `/map`, `/start_pose`, and `/goal_pose` share a coordinate frame and timestamp convention.
- [ ] Add map freshness and localization-validity checks.

## Milestone 4 — Path following and safety
- [ ] Convert planned paths into time- and dynamically-feasible trajectories.
- [ ] Implement PX4 Offboard setpoint publishing and required mode/arming handshake.
- [ ] Add obstacle inflation, replanning, timeout handling, and failsafe behavior.
- [ ] Validate in simulation before any physical flight.

## Evaluation metrics
- Goal success rate across repeatable obstacle layouts.
- Path length and planning latency.
- Minimum obstacle clearance.
- Recovery/replanning behavior when obstacles or goals change.
- Localization error when a suitable ground-truth reference is available.

## Current known gaps
The repository does not yet contain SLAM/VIO, sensor processing, path following, PX4 Offboard control, or a validated end-to-end demo. The first priority is to get the ROS 2–PX4 data path working, then validate the planner with generated occupancy grids before integrating flight control.
