package agent

import (
	"context"
	"encoding/json"
	"fmt"
	"os"
	"regexp"
	"strings"

	"robot/pkg/memory"
	"robot/pkg/task"

	"github.com/cloudwego/eino-ext/components/model/ark"
	"github.com/cloudwego/eino/adk"
	"github.com/cloudwego/eino/components/tool"
	"github.com/cloudwego/eino/components/tool/utils"
	"github.com/cloudwego/eino/compose"
)



func NewChatModel(ctx context.Context) (*ark.ChatModel, error) {
	apiKey := os.Getenv("ARK_API_KEY")
	if apiKey == "" {
		return nil, fmt.Errorf("environment variable ARK_API_KEY is not set")
	}

	modelID := os.Getenv("ARK_MODEL_ID")
	if modelID == "" {
		modelID = "deepseek-v3-1-250821"
	}

	return ark.NewChatModel(ctx, &ark.ChatModelConfig{
		APIKey: apiKey,
		Model:  modelID,
	})
}

func NewRequestProcessor(ctx context.Context, model *ark.ChatModel) (adk.Agent, error) {
	setVelTool, _ := utils.InferTool("Robot speed control", "Send ros2 topic name and velocity data to control robot movement", SetVelToolFunc)
	getPosTool, _ := utils.InferTool("Robot position collection", "Send ros2 topic name to get robot position", GetPosToolFunc)
	getTopicsTool, _ := utils.InferTool("ROS2 topic query", "Send topic type to get all matching topic names", GetTopicsToolFunc)

	retriever, _ := adk.NewChatModelAgent(ctx, &adk.ChatModelAgentConfig{
		Name:        "TopicRetriever",
		Description: "Get ros2 topic name",
		Instruction: "Analyze the request target: " +
			"1. For robot motion control, topic type is geometry_msgs/Twist. " +
			"2. For robot data collection, topic type is nav_msgs/Odometry. " +
			"Pass topic type to the topic query tool, then filter results by robot name to get the correct topic. " +
			"Finally pass the complete request with the topic name to the next node.",
		Model: model,
		ToolsConfig: adk.ToolsConfig{
			ToolsNodeConfig: compose.ToolsNodeConfig{
				Tools: []tool.BaseTool{getTopicsTool},
			},
		},
	})

	operator, _ := adk.NewChatModelAgent(ctx, &adk.ChatModelAgentConfig{
		Name:        "Operator",
		Description: "Execute operations based on request",
		Instruction: "You are an operator. Determine and execute the appropriate operation based on the request. " +
			"Operations: 1. Robot motion control. Extract topic name and velocity data from request, pass to robot speed control tool. " +
			"2. Robot data collection. Extract topic name from request, pass to position collection tool, output the result.",
		Model: model,
		ToolsConfig: adk.ToolsConfig{
			ToolsNodeConfig: compose.ToolsNodeConfig{
				Tools: []tool.BaseTool{setVelTool, getPosTool},
			},
		},
	})

	agent, err := adk.NewSequentialAgent(ctx, &adk.SequentialAgentConfig{
		Name:        "RequestProcessor",
		Description: "Request processing: topic analysis -> operation execution",
		SubAgents:   []adk.Agent{retriever, operator},
	})
	return agent, err
}

func SetVelToolFunc(ctx context.Context, input struct {
	Name string  `json:"name" jsonschema:"required,description=robot topic name, e.g. /robot1/cmd_vel"`
	X    float64 `json:"x" jsonschema:"description=linear velocity x"`
	Y    float64 `json:"y" jsonschema:"description=linear velocity y"`
	Z    float64 `json:"z" jsonschema:"description=angular velocity z"`
}) (string, error) {
	return toolsSetVel(ctx, input.Name, input.X, input.Y, input.Z)
}

func GetPosToolFunc(ctx context.Context, input struct {
	Name string `json:"name" jsonschema:"required,description=robot topic name, e.g. /robot1/odom"`
}) (string, error) {
	return toolsGetPos(ctx, input.Name)
}

func GetTopicsToolFunc(ctx context.Context, input struct {
	Type string `json:"type" jsonschema:"required,description=ros2 topic type, e.g. geometry_msgs/Twist or nav_msgs/Odometry"`
}) ([]string, error) {
	return toolsGetTopics(ctx, input.Type)
}

func SetRobotPositionToolFunc(ctx context.Context, input struct {
	Name  string  `json:"name" jsonschema:"required,description=robot name, e.g. robot1 or robot2"`
	X     float64 `json:"x" jsonschema:"required,description=x coordinate"`
	Y     float64 `json:"y" jsonschema:"required,description=y coordinate"`
	Theta float64 `json:"theta" jsonschema:"description=orientation in radians (default 0)"`
}) (string, error) {
	if setPosCallback != nil {
		setPosCallback(input.Name, input.X, input.Y, input.Theta)
	}
	return toolsSetPos(ctx, input.Name, input.X, input.Y, input.Theta)
}

func CheckObstaclesToolFunc(ctx context.Context, input struct {
	RobotName string `json:"robot_name" jsonschema:"required,description=robot name, e.g. robot1 or robot2"`
	Direction string `json:"direction" jsonschema:"description=direction to check: front, front_left, front_right, left, right, back (default: front)"`
}) (string, error) {
	return toolsCheckObstacles(ctx, input.RobotName, input.Direction)
}

func ComputeMotionToolFunc(ctx context.Context, input struct {
	RobotName string  `json:"robot_name" jsonschema:"required,description=robot name, e.g. robot1 or robot2"`
	TargetX   float64 `json:"target_x" jsonschema:"required,description=target x coordinate"`
	TargetY   float64 `json:"target_y" jsonschema:"required,description=target y coordinate"`
	Speed     float64 `json:"speed" jsonschema:"description=desired linear speed (default 0.5)"`
}) (string, error) {
	return toolsComputeMotion(ctx, input.RobotName, input.TargetX, input.TargetY, input.Speed)
}

func ComputeRelativeMotionToolFunc(ctx context.Context, input struct {
	RobotName string  `json:"robot_name" jsonschema:"required,description=robot name, e.g. robot1 or robot2"`
	Direction string  `json:"direction" jsonschema:"required,description=relative direction: forward, backward, left, right"`
	Distance  float64 `json:"distance" jsonschema:"required,description=distance in meters"`
	Speed     float64 `json:"speed" jsonschema:"description=desired linear speed (default 0.5)"`
}) (string, error) {
	return toolsComputeRelativeMotion(ctx, input.RobotName, input.Direction, input.Distance, input.Speed)
}

func PlanRouteToolFunc(ctx context.Context, input struct {
	RobotName    string  `json:"robot_name" jsonschema:"required,description=robot name, e.g. robot1 or robot2"`
	WaypointName string  `json:"waypoint_name,omitempty" jsonschema:"description=destination waypoint name. Use 'patrol' for the full patrol circuit. Others: pantry, supplies, hardware_2, coe, lounge, patrol_A1, patrol_A2, patrol_B, patrol_C, patrol_D1, patrol_D2, tinyRobot1_charger, tinyRobot2_charger. If provided, target_x/target_y are ignored."`
	TargetX      float64 `json:"target_x,omitempty" jsonschema:"description=target x coordinate (used when waypoint_name is empty)"`
	TargetY      float64 `json:"target_y,omitempty" jsonschema:"description=target y coordinate (used when waypoint_name is empty)"`
	Speed        float64 `json:"speed" jsonschema:"description=desired linear speed (default 0.5)"`
}) (string, error) {
	return toolsPlanRoute(ctx, input.RobotName, input.WaypointName, input.TargetX, input.TargetY, input.Speed)
}

var toolsSetVel = func(ctx context.Context, name string, x, y, z float64) (string, error) {
	return fmt.Sprintf("unimplemented: %s %f %f %f", name, x, y, z), nil
}
var toolsGetPos = func(ctx context.Context, name string) (string, error) {
	return fmt.Sprintf("unimplemented: %s", name), nil
}
var toolsGetTopics = func(ctx context.Context, topicType string) ([]string, error) {
	return nil, fmt.Errorf("unimplemented")
}
var toolsSetPos = func(ctx context.Context, name string, x, y, theta float64) (string, error) {
	return fmt.Sprintf("unimplemented: %s %f %f %f", name, x, y, theta), nil
}
var toolsComputeMotion = func(ctx context.Context, name string, targetX, targetY, speed float64) (string, error) {
	return fmt.Sprintf("unimplemented: %s target=(%f,%f) speed=%f", name, targetX, targetY, speed), nil
}
var toolsCheckObstacles = func(ctx context.Context, robotName string, direction string) (string, error) {
	return fmt.Sprintf("unimplemented: %s %s", robotName, direction), nil
}
var toolsComputeRelativeMotion = func(ctx context.Context, robotName string, direction string, distance float64, speed float64) (string, error) {
	return fmt.Sprintf("unimplemented: %s %s %.1f %.1f", robotName, direction, distance, speed), nil
}
var toolsPlanRoute = func(ctx context.Context, robotName string, waypointName string, targetX, targetY, speed float64) (string, error) {
	return fmt.Sprintf("unimplemented: %s wp=%s (%.1f,%.1f)", robotName, waypointName, targetX, targetY), nil
}

var setPosCallback func(string, float64, float64, float64)

func SetRobotPositionCallback(cb func(string, float64, float64, float64)) {
	setPosCallback = cb
}

func InjectToolImplementations(
	setVel func(ctx context.Context, name string, x, y, z float64) (string, error),
	getPos func(ctx context.Context, name string) (string, error),
	getTopics func(ctx context.Context, topicType string) ([]string, error),
	setPos func(ctx context.Context, name string, x, y, theta float64) (string, error),
	computeMotion func(ctx context.Context, name string, targetX, targetY, speed float64) (string, error),
	checkObstacles func(ctx context.Context, robotName string, direction string) (string, error),
	computeRelativeMotion func(ctx context.Context, robotName string, direction string, distance float64, speed float64) (string, error),
	planRoute func(ctx context.Context, robotName string, waypointName string, targetX, targetY, speed float64) (string, error),
) {
	toolsSetVel = setVel
	toolsGetPos = getPos
	toolsGetTopics = getTopics
	toolsSetPos = setPos
	toolsComputeMotion = computeMotion
	toolsCheckObstacles = checkObstacles
	toolsComputeRelativeMotion = computeRelativeMotion
	toolsPlanRoute = planRoute
}

var tabBeforeKey = regexp.MustCompile(`\t([a-zA-Z_][a-zA-Z0-9_]*)\s*:`)

func repairToolArgs(s string) string {
	var dummy any
	if json.Unmarshal([]byte(s), &dummy) == nil {
		return s
	}
	cleaned := tabBeforeKey.ReplaceAllString(s, `"$1":`)
	if json.Unmarshal([]byte(cleaned), &dummy) == nil {
		return cleaned
	}
	return s
}

func NewTaskPlanner(ctx context.Context, model *ark.ChatModel, historyContext string) (adk.Agent, error) {
	submitPlanTool, _ := utils.InferTool("SubmitPlanOptions", "Submit multiple plan options (2-3 alternatives) for user to choose from. Each option must have a description explaining its strategy.", SubmitPlanOptionsFunc)
	setPosTool, err := utils.InferTool("SetRobotPosition", "Teleport a robot to an absolute (x, y, theta) coordinate. Use for: setting initial position, resetting to origin, moving to coordinate.", SetRobotPositionToolFunc)
	if err != nil {
		return nil, fmt.Errorf("SetRobotPosition tool: %w", err)
	}
	computeMotionTool, _ := utils.InferTool("ComputeMotionParams", "Given robot name, target position, and speed, compute the required velocity vector and duration. Use this to get correct velocity direction and timing for movement tasks.", ComputeMotionToolFunc)
	computeRelativeMotionTool, _ := utils.InferTool("ComputeRelativeMotion", "Given robot name, relative direction (forward/backward/left/right), distance in meters, and speed, compute the required rotation angle (positive=left, negative=right) and forward motion. Use this for relative movement instructions like '向右走1米' or '向左走2米'.", ComputeRelativeMotionToolFunc)
	checkObstaclesTool, _ := utils.InferTool("CheckObstacles", "Check LiDAR scan data to find obstacles around a robot. Returns distance in meters to nearest obstacle in the requested direction. Use this before planning movement to avoid collisions.", CheckObstaclesToolFunc)
	planRouteTool, _ := utils.InferTool("PlanRoute", "Plan a route. Returns pre-computed segments (rotate_z, rotate_duration, target_theta, forward_x, forward_duration, target_x, target_y). Copy all values into Move tasks. Call ONCE then submit. For patrol use waypoint_name='patrol'.", PlanRouteToolFunc)

	instruction := buildPlannerInstruction(historyContext)

	planner, err := adk.NewChatModelAgent(ctx, &adk.ChatModelAgentConfig{
		Name:        "TaskPlanner",
		Description: "Convert natural language to structured task plan. Can teleport robots to absolute coordinates. Can compute motion parameters.",
		Instruction: instruction,
		Model:       model,
		ToolsConfig: adk.ToolsConfig{
			ToolsNodeConfig: compose.ToolsNodeConfig{
				Tools: []tool.BaseTool{submitPlanTool, setPosTool, computeMotionTool, computeRelativeMotionTool, checkObstaclesTool, planRouteTool},
				ToolArgumentsHandler: func(ctx context.Context, name, arguments string) (string, error) {
					return repairToolArgs(arguments), nil
				},
			},
		},
	})
	return planner, err
}

func buildPlannerInstruction(historyContext string) string {
	base := `You are a task planning expert. Be concise. Call tools once, submit immediately.

ROBOT CAPABILITIES (check [RobotInfo] for each robot in session context):
- robot1 (standard): 标准巡逻机器人, max_speed=0.5m/s, 不可载货, 适合巡逻/运输/侦查
- robot2 (delivery): 配送机器人, max_speed=0.3m/s, 可载货1kg, 适合配送/货物运输
TASK ASSIGNMENT RULES:
- **配送/载货任务** → 必须分配给 robot2 (delivery)
- **巡逻/侦查/速度优先的任务** → 优先分配给 robot1 (standard)
- **非载货的普通移动任务** → 可分配给任一机器人; 如无特殊要求默认选 robot1
- **双机协作** − 如果任务包含配送和巡逻两部分, 配送部分给 robot2, 巡逻/移动部分给 robot1

Available tools:
1. SubmitPlanOptions - Submit 2-3 alternative plan options for user to choose from.
2. SetRobotPosition - Teleport a robot to an absolute (x, y, theta) coordinate.
3. ComputeMotionParams - Compute velocity and duration to reach a target (x, y) coordinate.
4. ComputeRelativeMotion - Compute rotation + forward motion for relative direction (forward/backward/left/right) + distance. Use this for "向右走1米" / "向左移动2米" type commands.
5. CheckObstacles - Check LiDAR scan data for obstacles.
6. PlanRoute - **CRITICAL: Call ONCE and submit immediately.** Returns pre-computed segments (rotate_z, rotate_duration, target_theta, forward_x, forward_duration, target_x, target_y). Copy values into Move tasks. For "巡逻" / "patrol": call PlanRoute with waypoint_name="patrol" for a full patrol circuit. The system auto-stops at each target. Available destinations: "patrol" (巡逻路线), presupplies, supplies, pantry, coe, lounge, hardware_2, patrol_A1, patrol_A2, patrol_B, patrol_C, patrol_D1, patrol_D2, tinyRobot1_charger, tinyRobot2_charger.

When to use each tool:
- For position setting: use SetRobotPosition directly.
- For absolute movement (e.g., "移动到(0,0)"): call ComputeMotionParams first.
- For relative movement (e.g., "向右走1米", "向前走2米"): call ComputeRelativeMotion first, which returns the rotation angle and forward duration. Then decompose into rotate + forward plan options.
- **For long-distance navigation across rooms/corridors** (e.g., "去pantry", "到coe", "去hardware_2", "巡逻"): call **PlanRoute** first. It returns a list of segments, each with angle and distance. Then decompose each segment into rotate + forward Move tasks.
- **For navigation** (e.g., "去pantry", "巡逻"): Call **PlanRoute ONCE**, then submit immediately. Do NOT call PlanRoute multiple times. Copy rotate_z/forward_x/target_x/target_y directly. For "巡逻" use waypoint_name="patrol". Be concise — no text summaries.
- Always call CheckObstacles before moving. Check the INTENDED movement direction, not just front. For example, if the plan is to move right, call CheckObstacles with direction="right"; if moving backward, check direction="back". Only check "front" when planning forward movement.

SubmitPlanOptions rules:
- If path is clear → submit exactly 1 option.
- If obstacle within 3.0m but first action is a rotation (e.g., PlanRoute starts with a turn) → submit exactly 1 option (the turn avoids the obstacle).
- If obstacle directly blocks forward movement with no alternative → submit 2 options: normal + cautious.
- If obstacle within 1.0m AND first action is forward → submit 3 options: cautious, alternative, cancel.
- **CRITICAL: Call SubmitPlanOptions ONCE and stop.** Never leave the user with text-only response. Be concise.
- Each option must have a description explaining its strategy.
- In each option, list tasks with: action (Move/Wait), target (robot name), x (linear vel), z (angular vel), duration (for Wait tasks), dependencies.

Task types:
1. Move - Set robot velocity (x=forward, z=rotate, NO y)
2. Wait - Pause for N seconds (duration=N)
3. CollectData - Get robot position

IMPORTANT - Differential Drive Robots:
Robots are differential drive and can ONLY move:
- Forward/backward (x parameter, e.g. x=0.5 to go forward, x=-0.5 to go backward)
- Rotate in place (z parameter)
They CANNOT strafe sideways (y parameter is unsupported and will be ignored).

ANGULAR VELOCITY SIGN CONVENTION (CRITICAL):
- z > 0 (positive) = counterclockwise rotation = **左转**
- z < 0 (negative) = clockwise rotation = **右转**
Example: To turn right, use z=-1.0. To turn left, use z=1.0.
**DO NOT get this wrong** — using the wrong sign will make the robot turn the opposite direction.

For rotation tasks, use the 'angle' value from ComputeRelativeMotion to determine sign and duration:
- angle > 0 → need left turn (z > 0)
- angle < 0 → need right turn (z < 0)
- duration = abs(angle) / abs(z)  (e.g., angle=-1.57 with z=-1.0 → Wait duration=1.57s)

For each option's tasks, use simple fields:
- Move+duration (recommended): action="Move", target="robot1", x=0.5, duration=2.0 (move forward at 0.5m/s for 2.0s)
- Rotate+duration: action="Move", target="robot1", z=-1.0, duration=1.57 (rotate right for 1.57s)
- Stop: action="Move", target="robot1" (no velocity params = stop)
- Wait: action="Wait", duration=5.0 (seconds) — only if you need a pure pause
- Dependencies: use task numbering task-1, task-2, etc.
PREFER putting duration on the Move task rather than using separate Wait tasks.
IMPORTANT: Always include target_x and target_y in forward Move+duration tasks. This enables closed-loop position checking.
For PlanRoute: copy rotate_z/rotate_duration/target_theta into rotate Move task, copy forward_x/forward_duration/target_x/target_y into forward Move task. Add a stop task after each forward. Chain segments with dependencies. The system auto-stops rotation when target_theta is reached.

PlanRoute example (copy all values directly):
PlanRoute returns: [{"rotate_z":-1,"rotate_duration":0.93,"target_theta":-0.93,"forward_x":0.5,"forward_duration":1.44,"target_x":10.43,"target_y":-5.58}, ...]
CRITICAL: Each segment becomes 3 tasks: rotate → forward → stop. Chain across segments with dependencies.
→ {"description":"沿路线去pantry","tasks":[
  {"action":"Move","target":"robot1","z":-1,"duration":0.93,"target_theta":-0.93},
  {"action":"Move","target":"robot1","x":0.5,"duration":1.44,"target_x":10.43,"target_y":-5.58,"dependencies":["task-1"]},
  {"action":"Move","target":"robot1","dependencies":["task-2"]},
  {"action":"Move","target":"robot1","z":1,"duration":1.57,"target_theta":0.63,"dependencies":["task-3"]},
  {"action":"Move","target":"robot1","x":0.5,"duration":2.00,"target_x":12.00,"target_y":-5.00,"dependencies":["task-4"]},
  {"action":"Move","target":"robot1","dependencies":["task-5"]}
]}

Multiple option example for obstacle scenario:
Option 1: {"description":"正常速度前进 x=0.5","tasks":[{"action":"Move","target":"robot1","x":0.5}]}
Option 2: {"description":"减速谨慎通过 x=0.2","tasks":[{"action":"Move","target":"robot1","x":0.2}]}
Option 3: {"description":"不移动","tasks":[{"action":"Wait","duration":0.1}]}

Example for "向右走2米" (go right 2m, no reorientation) — using duration on Move:
Option: {"description":"右转→前进2米","tasks":[
  {"action":"Move","target":"robot1","z":-1.0,"duration":1.57},
  {"action":"Move","target":"robot1","x":0.5,"duration":4.0,"dependencies":["task-1"]},
  {"action":"Move","target":"robot1","dependencies":["task-2"]}
]}

Example for "向右走2米并回正" (go right 2m and reorient):
Option: {"description":"右转→前进2米→左转回正","tasks":[
  {"action":"Move","target":"robot1","z":-1.0,"duration":1.57},
  {"action":"Move","target":"robot1","x":0.5,"duration":4.0,"dependencies":["task-1"]},
  {"action":"Move","target":"robot1","z":1.0,"duration":1.57,"dependencies":["task-2"]},
  {"action":"Move","target":"robot1","dependencies":["task-3"]}
]}`

	if historyContext == "" {
		return base
	}

	return fmt.Sprintf(`%s

## Conversation History
%s

Consider the conversation history and the new user instruction for planning.`, base, historyContext)
}

type SimpleTask struct {
	Action       string   `json:"action" jsonschema:"description=task action: Move/Wait/CollectData"`
	Target       string   `json:"target" jsonschema:"description=robot name, e.g. robot1"`
	X            float64  `json:"x,omitempty" jsonschema:"description=linear velocity x"`
	Z            float64  `json:"z,omitempty" jsonschema:"description=angular velocity z"`
	Duration     float64  `json:"duration,omitempty" jsonschema:"description=wait duration in seconds"`
	TargetX      float64  `json:"target_x,omitempty" jsonschema:"description=target x coordinate for closed-loop position check"`
	TargetY      float64  `json:"target_y,omitempty" jsonschema:"description=target y coordinate for closed-loop position check"`
	TargetTheta  float64  `json:"target_theta,omitempty" jsonschema:"description=target orientation for closed-loop rotation check"`
	Dependencies []string `json:"dependencies,omitempty" jsonschema:"description=task IDs this task depends on"`
}

type PlanOption struct {
	Description string       `json:"description" jsonschema:"description=human-readable description of this plan option"`
	Tasks       []SimpleTask `json:"tasks" jsonschema:"description=list of tasks in this plan"`
}

func (o PlanOption) ToTaskPlan() *task.TaskPlan {
	// Pass 1: find consecutive Move→Wait pairs
	movePair := map[int]int{} // Move index → Wait index
	consumed := map[int]bool{}
	for i := 0; i < len(o.Tasks); i++ {
		st := o.Tasks[i]
		if st.Action != "Move" || st.Duration > 0 {
			continue
		}
		if i+1 < len(o.Tasks) && o.Tasks[i+1].Action == "Wait" {
			movePair[i] = i + 1
			consumed[i+1] = true
		}
	}

	// Pre-compute expanded map: original task ID → stop task ID
	expanded := map[string]string{}
	for i := range o.Tasks {
		if consumed[i] {
			continue
		}
		id := fmt.Sprintf("task-%d", i+1)
		if _, isPaired := movePair[i]; isPaired {
			expanded[id] = fmt.Sprintf("task-%d-s", i+1)
		} else if o.Tasks[i].Action == "Move" && o.Tasks[i].Duration > 0 {
			expanded[id] = fmt.Sprintf("task-%d-s", i+1)
		}
	}

	// Pre-compute full forwarding: consumed ID + expanded ID → stop ID
	forwarding := map[string]string{}
	for id, stopID := range expanded {
		forwarding[id] = stopID
	}
	for moveIdx, waitIdx := range movePair {
		waitID := fmt.Sprintf("task-%d", waitIdx+1)
		moveID := fmt.Sprintf("task-%d", moveIdx+1)
		if stopID, ok := expanded[moveID]; ok {
			forwarding[waitID] = stopID
		}
	}

	// Resolve all dependencies through the forwarding table
	resolveDeps := func(deps []string) []string {
		res := make([]string, len(deps))
		for i, d := range deps {
			if s, ok := forwarding[d]; ok {
				res[i] = s
			} else {
				res[i] = d
			}
		}
		return res
	}

	// Pass 2: build task list
	var tasks []*task.Task
	for i, st := range o.Tasks {
		if consumed[i] {
			continue
		}
		id := fmt.Sprintf("task-%d", i+1)
		params := map[string]interface{}{}

		if waitIdx, isPaired := movePair[i]; isPaired {
			if st.X != 0 {
				params["x"] = st.X
			}
			if st.Z != 0 {
				params["z"] = st.Z
			}
			if st.TargetX != 0 || st.TargetY != 0 {
				params["target_x"] = st.TargetX
				params["target_y"] = st.TargetY
			}
			waitSt := o.Tasks[waitIdx]
			waitID := fmt.Sprintf("task-%d-w", i+1)
			stopID := fmt.Sprintf("task-%d-s", i+1)

			waitDeps := resolveDeps(waitSt.Dependencies)
			waitDeps = append(waitDeps, id)

			tasks = append(tasks, &task.Task{
				ID: id, Action: task.ActionMove,
				Target: st.Target, Params: params,
				Dependencies: resolveDeps(st.Dependencies),
			})
			waitParams := map[string]interface{}{"duration": waitSt.Duration}
			if st.X != 0 {
				waitParams["x"] = st.X
			}
			if st.Z != 0 {
				waitParams["z"] = st.Z
			}
			if st.TargetX != 0 || st.TargetY != 0 {
				waitParams["target_x"] = st.TargetX
				waitParams["target_y"] = st.TargetY
			}
			if st.Z != 0 && st.TargetTheta != 0 {
				waitParams["target_theta"] = st.TargetTheta
			}
			tasks = append(tasks, &task.Task{
				ID: waitID, Action: task.ActionWait,
				Target: waitSt.Target,
				Params: waitParams,
				Dependencies: waitDeps,
			})
			tasks = append(tasks, &task.Task{
				ID: stopID, Action: task.ActionMove,
				Target: st.Target, Params: map[string]interface{}{},
				Dependencies: []string{waitID},
			})
		} else if st.Action == "Move" && st.Duration > 0 {
			if st.X != 0 {
				params["x"] = st.X
			}
			if st.Z != 0 {
				params["z"] = st.Z
			}
			if st.TargetX != 0 || st.TargetY != 0 {
				params["target_x"] = st.TargetX
				params["target_y"] = st.TargetY
			}
			if st.Z != 0 && st.TargetTheta != 0 {
				params["target_theta"] = st.TargetTheta
			}
			waitID := fmt.Sprintf("task-%d-w", i+1)
			stopID := fmt.Sprintf("task-%d-s", i+1)
			tasks = append(tasks, &task.Task{
				ID: id, Action: task.ActionMove,
				Target: st.Target, Params: params,
				Dependencies: resolveDeps(st.Dependencies),
			})
			waitParams := map[string]interface{}{"duration": st.Duration}
			if st.X != 0 {
				waitParams["x"] = st.X
			}
			if st.Z != 0 {
				waitParams["z"] = st.Z
			}
			if st.TargetX != 0 || st.TargetY != 0 {
				waitParams["target_x"] = st.TargetX
				waitParams["target_y"] = st.TargetY
			}
			if st.Z != 0 && st.TargetTheta != 0 {
				waitParams["target_theta"] = st.TargetTheta
			}
			tasks = append(tasks, &task.Task{
				ID: waitID, Action: task.ActionWait,
				Target: st.Target,
				Params: waitParams,
				Dependencies: []string{id},
			})
			tasks = append(tasks, &task.Task{
				ID: stopID, Action: task.ActionMove,
				Target: st.Target, Params: map[string]interface{}{},
				Dependencies: []string{waitID},
			})
		} else if st.Action == "Wait" {
			if st.Duration > 0 {
				params["duration"] = st.Duration
			}
			tasks = append(tasks, &task.Task{
				ID: id, Action: task.ActionWait,
				Params: params, Dependencies: resolveDeps(st.Dependencies),
			})
		} else {
			if st.X != 0 {
				params["x"] = st.X
			}
			if st.Z != 0 {
				params["z"] = st.Z
			}
			tasks = append(tasks, &task.Task{
				ID: id, Action: task.TaskAction(st.Action),
				Target: st.Target, Params: params,
				Dependencies: resolveDeps(st.Dependencies),
			})
		}
	}
	return &task.TaskPlan{Tasks: tasks}
}

var SubmitPlanOptionsFunc func(ctx context.Context, input struct {
	Options []PlanOption `json:"options" jsonschema:"description=plan options for user to choose from"`
}) ([]PlanOption, error)

var planOptionsCallback func([]PlanOption)

func SetPlanOptionsCallback(cb func([]PlanOption)) {
	planOptionsCallback = cb
}

func init() {
	SubmitPlanOptionsFunc = func(ctx context.Context, input struct {
		Options []PlanOption `json:"options" jsonschema:"description=plan options for user to choose from"`
	}) ([]PlanOption, error) {
		if planOptionsCallback != nil {
			planOptionsCallback(input.Options)
		}
		return input.Options, nil
	}
}

func BuildHistoryContext(messages []memory.Message) string {
	if len(messages) == 0 {
		return ""
	}

	var b strings.Builder
	for _, msg := range messages {
		role := ""
		switch msg.Role {
		case memory.RoleUser:
			role = "User"
		case memory.RoleAssistant:
			role = "Agent"
		case memory.RoleTool:
			role = "System"
		default:
			role = string(msg.Role)
		}
		b.WriteString(fmt.Sprintf("[%s]: %s\n", role, msg.Content))
	}
	return b.String()
}
