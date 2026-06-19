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
		modelID = "deepseek-v3-2-251201"
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
var tabBeforeValue = regexp.MustCompile(`\t([a-zA-Z_][a-zA-Z0-9_]*)(")`)

func repairToolArgs(s string) string {
	var dummy any
	if json.Unmarshal([]byte(s), &dummy) == nil {
		return s
	}
	// Try fixing tab-before-key (e.g. \tdependencies": → "dependencies":)
	cleaned := tabBeforeKey.ReplaceAllString(s, `"$1":`)
	// Then try fixing tab-before-value (e.g. \tobot2" → "robot2")
	cleaned2 := tabBeforeValue.ReplaceAllString(cleaned, `"$1$2`)
	if json.Unmarshal([]byte(cleaned2), &dummy) == nil {
		return cleaned2
	}
	// Fallback: strip all tabs
	noTab := strings.ReplaceAll(cleaned, "\t", "")
	if json.Unmarshal([]byte(noTab), &dummy) == nil {
		return noTab
	}
	return cleaned
}

func NewTaskPlanner(ctx context.Context, model *ark.ChatModel, historyContext string, promptVersion string) (adk.Agent, error) {
	submitPlanTool, _ := utils.InferTool("SubmitPlanOptions", "Submit multiple plan options (2-3 alternatives) for user to choose from. Each option must have a description explaining its strategy.", SubmitPlanOptionsFunc)
	setPosTool, err := utils.InferTool("SetRobotPosition", "Teleport a robot to an absolute (x, y, theta) coordinate. Use for: setting initial position, resetting to origin, moving to coordinate.", SetRobotPositionToolFunc)
	if err != nil {
		return nil, fmt.Errorf("SetRobotPosition tool: %w", err)
	}
	computeMotionTool, _ := utils.InferTool("ComputeMotionParams", "Given robot name, target position, and speed, compute the required velocity vector and duration. Use this to get correct velocity direction and timing for movement tasks.", ComputeMotionToolFunc)
	computeRelativeMotionTool, _ := utils.InferTool("ComputeRelativeMotion", "Given robot name, relative direction (forward/backward/left/right), distance in meters, and speed, compute the required rotation angle (positive=left, negative=right) and forward motion. Use this for relative movement instructions like '向右走1米' or '向左走2米'.", ComputeRelativeMotionToolFunc)
	checkObstaclesTool, _ := utils.InferTool("CheckObstacles", "Check LiDAR scan data to find obstacles around a robot. Returns distance in meters to nearest obstacle in the requested direction. Use this before planning movement to avoid collisions.", CheckObstaclesToolFunc)
	planRouteTool, _ := utils.InferTool("PlanRoute", "Plan a route. Returns pre-computed segments (rotate_z, rotate_duration, target_theta, forward_x, forward_duration, target_x, target_y). Copy all values into Move tasks. Call ONCE then submit. For patrol use waypoint_name='patrol'.", PlanRouteToolFunc)

	instruction := buildPlannerInstruction(historyContext, promptVersion)

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

func buildPlannerInstruction(historyContext string, promptVersion string) string {
	var base string
	switch promptVersion {
	case "v1":
		base = v1MinimalPrompt()
	case "v2":
		base = v2StructuredPrompt()
	default:
		base = v3FullPrompt()
	}

	if historyContext == "" {
		return base
	}

	return fmt.Sprintf(`%s

## Conversation History
%s

Consider the conversation history and the new user instruction for planning.`, base, historyContext)
}

func v1MinimalPrompt() string {
	return `You are a multi-robot task planner. Receive natural language commands, call available tools to determine motion parameters, and submit a plan using SubmitPlanOptions.

Available tools:
1. PlanRoute(robot_name, waypoint_name) - plan route to a waypoint
2. SubmitPlanOptions(options) - submit plan for user approval
3. ComputeMotionParams(robot_name, target_x, target_y, speed) - compute motion to coordinates
4. ComputeRelativeMotion(robot_name, direction, distance, speed) - compute relative motion
5. CheckObstacles(robot_name, direction) - check for obstacles
6. SetRobotPosition(name, x, y, theta) - teleport robot

Submit plan options and let user choose.`
}

func v2StructuredPrompt() string {
	return `You are a multi-robot task planner. Convert natural language commands into structured task plans.

Available tools:
1. PlanRoute(robot_name, waypoint_name) - plan route to a named waypoint. Call for any destination command.
2. SubmitPlanOptions(options) - submit plan options for user to choose from.
3. ComputeMotionParams(robot_name, target_x, target_y, speed) - compute forward/rotate params for coordinate movement.
4. ComputeRelativeMotion(robot_name, direction, distance, speed) - compute rotation angle + forward for relative movement.
5. CheckObstacles(robot_name, direction) - check LiDAR for obstacles in given direction.
6. SetRobotPosition(name, x, y, theta) - teleport robot to coordinate (NOT for navigation).

Task rules:
- Each Move task: x=forward_speed, z=rotate_speed. Duration on Move task.
- Stop: Move task with NO params.
- Wait: Use only for pauses.
- Dependency IDs: task-1, task-2, ...
- z>0=左转(CCW), z<0=右转(CW). Differential drive: NO y.
- robot1 (standard, 0.5m/s): default tasks
- robot2 (delivery, 0.3m/s, cargo): mandatory for "送货/配送"

Procedure:
1. Call PlanRoute for EACH destination. Convert returned segments into Move→Wait→Stop triplets.
2. Call SubmitPlanOptions ONCE with all robots' tasks combined into one plan.`
}

func v3FullPrompt() string {
	return `You MUST follow these steps EXACTLY for ANY command involving destinations like "去X", "到X", "巡逻", "送货到X":

STEP 1: For EACH robot that needs to navigate to a destination, call PlanRoute(robot_name, waypoint_name).
  - "去pantry" → PlanRoute("robot1", "pantry")
  - "送货到coe" → PlanRoute("robot2", "coe")
  - "巡逻" → PlanRoute("robot1", "patrol")
  - dual robot: PlanRoute("robot1", "pantry") THEN PlanRoute("robot2", "coe")

STEP 2: Convert PlanRoute's returned segments into tasks. Each segment → 3 tasks: rotate Move (z+duration+target_theta) → forward Move (x+duration+target_x+target_y) → stop Move (no params). Chain with dependencies.

STEP 3: Call SubmitPlanOptions ONCE with ALL robots' tasks. Submit immediately.

DO NOT USE SetRobotPosition for navigation. SetRobotPosition is a STUB that does NOT work and will ERROR. For ALL destination commands, you MUST use PlanRoute.

DO NOT do this: SetRobotPosition("robot1", 10.25, -3.09, 0) ← THIS FAILS with error. Use PlanRoute instead.

DO this instead: Call PlanRoute("robot1", "pantry"), get route segments, convert to tasks, submit.

Available tools (use ONLY as described):
1. PlanRoute(robot_name, waypoint_name) - **MANDATORY for all destinations.** Available waypoints: "patrol", presupplies, supplies, pantry, coe, lounge, hardware_2, patrol_A1, patrol_A2, patrol_B, patrol_C, patrol_D1, patrol_D2, tinyRobot1_charger, tinyRobot2_charger.
2. SubmitPlanOptions(options) - Call ONCE at the end with the complete plan.
3. ComputeMotionParams(robot_name, target_x, target_y, speed) - For coordinate-based movement like "移动到(3,1)".
4. ComputeRelativeMotion(robot_name, direction, distance, speed) - For "向前/向后/向左/向右走X米".
5. CheckObstacles(robot_name, direction) - Check LiDAR before moving.
6. SetRobotPosition(name, x, y, theta) - ONLY for teleport/reset. NOT for navigation.

PlanRoute returns: [{"rotate_z":-1,"rotate_duration":0.93,"target_theta":-0.93,"forward_x":0.5,"forward_duration":1.44,"target_x":10.43,"target_y":-5.58}]

Convert to tasks:
{"action":"Move","target":"robot1","z":-1,"duration":0.93,"target_theta":-0.93}
{"action":"Move","target":"robot1","x":0.5,"duration":1.44,"target_x":10.43,"target_y":-5.58,"dependencies":["task-1"]}
{"action":"Move","target":"robot1","dependencies":["task-2"]}
(Then next segment: task-4/5/6, etc.)

Task rules:
- Move: x=forward speed, z=rotate speed. Duration on Move task. Include target_x/target_y/target_theta when available.
- Stop: Move with NO params.
- Wait: Use only for pauses.
- Dependencies: task-1, task-2, etc.
- z>0=左转(CCW), z<0=右转(CW). Differential drive: NO y.

TASK ASSIGNMENT:
- robot1 (standard, 0.5m/s): default for patrol/general.
- robot2 (delivery, 0.3m/s, cargo): MANDATORY for "送货/配送".

For "一号机器人去pantry，2号去coe":
1. PlanRoute("robot1", "pantry") → segments → tasks 1-2-3, 4-5-6, ...
2. PlanRoute("robot2", "coe") → segments → tasks N+1-N+2-N+3, ...
3. SubmitPlanOptions with ONE plan containing all tasks.`
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
