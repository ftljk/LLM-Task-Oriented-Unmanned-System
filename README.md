# LLM Task-Oriented Unmanned System

基于大语言模型结合任务依赖的无人系统协作技术。

## Architecture

```
User (Natural Language)
  ↓
┌─────────────────────────────────────────┐
│  Eino ADK TaskPlanner Agent             │
│  NL → Structured TaskPlan (JSON)        │
└─────────────────────────────────────────┘
  ↓
┌─────────────────────────────────────────┐
│  Scheduler (DAG-based)                  │
│  - Task dependency resolution           │
│  - Parallel execution                   │
│  - Timeout / Retry / Skip & Continue    │
│  - Execution summary reporting          │
└─────────────────────────────────────────┘
  ↓
┌─────────────────────────────────────────┐
│  RobotAdapter Interface                 │
│  ├─ GOSimRobotAdapter (Go built-in)     │
│  └─ ROS2RobotAdapter (WebSocket → ROS2) │
└─────────────────────────────────────────┘
  ↓
┌─────────────────────────────────────────┐
│  Robot Simulation                       │
│  ├─ GoSim (goroutine physics, test)     │
│  └─ Gazebo + ROS2 (real sim, optional)  │
└─────────────────────────────────────────┘
```

## Project Structure (Go)

```
llm_robot_agent/
├── main.go                     # Interactive CLI with multi-turn session
├── go.mod / go.sum
├── pkg/
│   ├── memory/
│   │   ├── memory.go           # Memory interface (Message, Session, RobotState)
│   │   └── in_memory.go        # InMemorySessionManager (thread-safe)
│   ├── task/
│   │   └── model.go            # Task/TaskPlan with Config, ExecutionResult
│   ├── robot/
│   │   ├── interface.go        # RobotAdapter interface
│   │   ├── sim_adapter.go      # GOSimRobotAdapter (pure Go)
│   │   └── ros_adapter.go      # ROS2RobotAdapter (WebSocket)
│   ├── scheduler/
│   │   └── scheduler.go        # DAG scheduler with retry/timeout/skip
│   ├── agent/
│   │   └── agents.go           # Eino Agent definitions
│   ├── tools/
│   │   └── robot_tools.go      # Eino Tool implementations
│   └── ros/
│       └── client.go           # WebSocket ROS bridge client
└── cmd/
    └── test_scheduler/
        └── main.go             # Standalone scheduler + GoSim test
```

## Project Structure (ROS2)

```
src/robot_sim/
├── robot_sim_bridge/            # Python WebSocket ↔ ROS2 bridge
│   └── robot_sim_bridge/
│       └── bridge_node.py
└── robot_sim_gz/                # Gazebo simulation package
    ├── urdf/diff_drive_robot.urdf
    ├── worlds/
    └── launch/two_robot_sim.launch.py
```

## Quick Start

### GoSim mode (no ROS2 needed):

```bash
cd ~/rmf_ws/llm_robot_agent
export ARK_API_KEY=your_api_key
go run main.go --simulator=go
```

### Scheduler standalone test:

```bash
go run ./cmd/test_scheduler/
```

### Gazebo + ROS2 mode:

```bash
# Terminal 1: Launch simulation
source ~/rmf_ws/install/setup.bash
ros2 launch robot_sim_gz two_robot_sim.launch.py

# Terminal 2: Launch bridge
ros2 run robot_sim_bridge bridge_node

# Terminal 3: Run LLM agent
cd ~/rmf_ws/llm_robot_agent
export ARK_API_KEY=your_api_key
go run main.go --simulator=ros2 --ws=ws://localhost:9090
```

## Key Features

- **Memory**: Multi-turn conversation with session management
- **DAG Scheduler**: Task dependency, parallel execution, deadlock detection
- **Fault Tolerance**: Per-task timeout, retry with backoff, skip-and-continue
- **Dual Simulator**: Pure-Go simulator for dev, ROS2/Gazebo for real sim
- **Framewok-Agnostic Core**: Memory, Scheduler, RobotAdapter interfaces have zero Eino dependency
