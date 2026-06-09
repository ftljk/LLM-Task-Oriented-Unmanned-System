# AGENTS.md — Session Context

## Project
`~/rmf_ws/llm_robot_agent/` — Go multi-robot NL→task system with Go sim and ROS2/Gazebo.

## Architecture
- `pkg/memory/` — in-memory session/message/robot state
- `pkg/task/` — Task/TaskPlan/TaskConfig types
- `pkg/robot/` — RobotAdapter interface + GOSimRobotAdapter + ROS2RobotAdapter
- `pkg/scheduler/` — DAG scheduler, ExecutePlan, FormatResultSummary
- `pkg/agent/` — Eino ADK agents: TaskPlanner (current) + RequestProcessor (legacy)
- `pkg/normalizer/` — 3-layer pipeline: preprocess → LLM → validate (Move+duration auto-expand → Wait→Stop)
- `pkg/ros/` — WebSocket ROS bridge client (gorilla/websocket)

## Phase 4 — Topological Path Planning (2026-06-07)
- **NavGraph package** (`pkg/navgraph/`): Parses `nav_graphs/0.yaml` (29 vertices, 46 bidirectional lanes) from RMF demos. Dijkstra shortest path. `PlanPath()` decomposes path into angle+distance segments.
- **`PlanRoute` tool**: LLM calls `PlanRoute(robot_name, waypoint_name)` → returns JSON segments each with `{angle, distance, speed}`. LLM decomposes each segment into rotate→forward→stop tasks.
- **Laser guard**: `execWait` now polls LIDAR every 100ms. If front < 0.3m or front-left/front-right < 0.25m, emergency stop (velocity=0) + return error.
- **Available waypoints**: presupplies, supplies, pantry, coe, lounge, hardware_2, patrol_A1/2, patrol_B/C, patrol_D1/2, tinyRobot1_charger, tinyRobot2_charger.
- **`--nav-graph` flag**: specify custom nav graph path; defaults to install path under `/home/mofus/rmf_ws`.

## Phase 3 — Robot Types & Role-Based Assignment
- **Two robot types**: robot1 = `StandardRobot` (blue, max_speed=0.5m/s, no cargo), robot2 = `DeliveryRobot` (orange, cargo box on top, max_speed=0.3m/s, cargo=1kg)
- `delivery_robot.urdf` has orange color, cargo box link, slower acceleration/velocity limits, heavier mass
- `launch.py` maps robot name → URDF file via `ROBOT_URDFS` dict
- `memory.RobotState` has `Type RobotType` field (`RobotTypeStandard` / `RobotTypeDelivery`)
- `RobotState.Capabilities()` returns Chinese capability description
- State injection includes `[RobotInfo]` line per robot with type+caps
- Prompt instructs LLM on role-based assignment: delivery→robot2, patrol/speed→robot1, general→either

## Phase 2 Key Features (Completed)
- **GPU LIDAR** embedded in URDF `<gazebo>` extension (`type="gpu_lidar"`, topic `/{name}/scan` absolute). CPU `type="ray"` deprecated.
- **Stable physics**: SDF with missing inertial caused NaN; SDF continuous joints crashed dartsim → use URDF + revolute joints.
- **Plan options**: `SubmitPlanOptions` tool — LLM submits 1 option (direct) or 2-3 (obstacle detected). User selects by number.
- **Relative motion**: `ComputeRelativeMotion` tool — server computes rotation angle + forward vector from odometry.
- **Sign convention**: `z > 0` = CCW = 左转; `z < 0` = CW = 右转. Explicit in prompt.
- **Plan expansion**: `ToTaskPlan()` auto-expands Move+duration → Move→Wait→Stop with dependency forwarding.
- **Concurrency fixes**:
  - `client.go`: `serviceMu` serializes `call_service` ops + 10s timeout
  - `ros_adapter.go`: hard-coded topic names (`/model/{name}/cmd_vel`, `/model/{name}/odometry`, `/{name}/scan`)
  - `ros_adapter.go`: `toolMu` serializes `GetOdometry`/`GetLaserScan` to prevent handler overwrite on concurrent Subscribe calls
- **Timeout**: bumped from 45s→120s to accommodate LLM latency.

## ROS2 + Gazebo Architecture
```
Go Agent (--simulator=ros2)
  │ WebSocket :9090
  ▼
bridge_node.py  (WebSocket ↔ ROS2 bridge)
  │ ROS2 topics
  ▼
ros_gz_bridge parameter_bridge  (ROS2 ↔ Gazebo transport)
  │ Gazebo topics
  ▼
Gazebo DiffDrive plugin
```

Topics: `/model/{name}/cmd_vel` (`]`), `/model/{name}/odometry` (`[`), `/{name}/scan` (`[`).

## Run Commands
```bash
cd ~/rmf_ws/llm_robot_agent && bash run.sh --simulator=go        # Go sim
cd ~/rmf_ws/llm_robot_agent && bash run_gazebo.sh                # ROS2+Gazebo
cd ~/rmf_ws/llm_robot_agent && bash run_gazebo.sh --headless     # Headless
cd ~/rmf_ws && colcon build --packages-select robot_sim_bridge robot_sim_gz  # Build ROS2
cd ~/rmf_ws/llm_robot_agent && source ~/.bashrc && GOTOOLCHAIN=local GOPROXY=https://goproxy.cn,direct ~/go1.23/bin/go build ./...
```

## Prompt Highlights
- Relative movement: use `ComputeRelativeMotion` first, then decompose returned `angle` into rotate tasks.
- Obstacle checking: check **intended movement direction**, not just `front`.
- `z>0` = left turn, `z<0` = right turn.
- Plan options: 1 (clear path) / 2-3 (obstacle within 3m) / 3 (obstacle within 1m: cautious + alternative + cancel).
- **Must always call `SubmitPlanOptions`** — no text-only responses.

## Spawn Positions (building_L1 office map)
- robot1: (10.0, -5.0) — center aisle, test wall at (13, -5)
- robot2: (17.0, -5.5) — pantry corridor (wall at x≈18.1, moved from 18.0)
- test_wall: (13.0, -5.0) — 3m in front of robot1

## Known Issues
- WebSocket bridge: one-shot subscribe (not streaming).
- `SetPosition` via ROS2: stub only.
- Occasional post-execution timeout (LLM produces output after timeout fires). Root cause unclear.

## ROS2 Packages
- `robot_sim_gz` — URDF models, Gazebo launch (spawn + ros_gz_bridge)
- `robot_sim_bridge` — WebSocket ↔ ROS2 bridge (websockets + rclpy)
