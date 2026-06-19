package main

import (
	"bufio"
	"context"
	"encoding/json"
	"flag"
	"fmt"
	"log"
	"math"
	"os"
	"strings"
	"sync"
	"time"

	"robot/pkg/agent"
	"robot/pkg/memory"
	"robot/pkg/navgraph"
	"robot/pkg/normalizer"
	"robot/pkg/robot"
	"robot/pkg/scheduler"
	"robot/pkg/task"
	"robot/pkg/tools"

	"github.com/cloudwego/eino-ext/components/model/ark"
	"github.com/cloudwego/eino/adk"
)

func init() {
	// Load .env file if it exists — minimal .env parser (no external dependency)
	data, err := os.ReadFile(".env")
	if err != nil {
		return
	}
	for _, line := range strings.Split(string(data), "\n") {
		line = strings.TrimSpace(line)
		if line == "" || strings.HasPrefix(line, "#") {
			continue
		}
		if k, v, ok := strings.Cut(line, "="); ok {
			k = strings.TrimSpace(k)
			v = strings.Trim(strings.TrimSpace(v), "\"'")
			if os.Getenv(k) == "" {
				os.Setenv(k, v)
			}
		}
	}
}

func main() {
	simMode := flag.String("simulator", "go", "simulator mode: go (built-in) or ros2 (WebSocket)")
	wsURL := flag.String("ws", "ws://localhost:9090", "ROS bridge WebSocket URL")
	navGraphPath := flag.String("nav-graph", "", "path to nav_graphs/0.yaml (topological map)")
	flag.Parse()

	mem := memory.NewInMemorySessionManager()
	sessionID := fmt.Sprintf("session-%d", time.Now().Unix())
	_, err := mem.CreateSession(nil, sessionID)
	if err != nil {
		log.Printf("Session may already exist: %v", err)
	}

	var adapter robot.RobotAdapter
	switch *simMode {
	case "go":
		adapter = robot.NewGOSimRobotAdapter()
	case "ros2":
		rosAdapter, rosErr := robot.NewROS2RobotAdapter(*wsURL)
		if rosErr != nil {
			log.Fatalf("Failed to connect to ROS2: %v", rosErr)
		}
		adapter = rosAdapter
	default:
		log.Fatalf("Unknown simulator mode: %s", *simMode)
	}
	defer adapter.Close()

	tools.DefaultRobotAdapter = adapter

	// Load topological navigation graph
	var nav *navgraph.Graph
	if *navGraphPath == "" {
		// Try default install path
		defaultPath := "/home/mofus/rmf_ws/install/rmf_demos_maps/share/rmf_demos_maps/maps/office/nav_graphs/0.yaml"
		if g, err := navgraph.Load(defaultPath); err == nil {
			nav = g
			log.Printf("[NavGraph] Loaded %d vertices, %d edges from %s", len(nav.Vertices), len(nav.Edges), defaultPath)
		} else {
			log.Printf("[NavGraph] Not available (optional): %v", err)
		}
	} else {
		var err error
		nav, err = navgraph.Load(*navGraphPath)
		if err != nil {
			log.Fatalf("Failed to load nav graph: %v", err)
		}
		log.Printf("[NavGraph] Loaded %d vertices, %d edges", len(nav.Vertices), len(nav.Edges))
	}

	sched := scheduler.NewScheduler(adapter, mem)

	// Multi-robot path conflict detection: track planned paths per robot
	var (
		robotPlanPathsMu sync.Mutex
		robotPlanPaths   = map[string][]int{}
	)

	agent.InjectToolImplementations(
		func(ctx context.Context, name string, x, y, z float64) (string, error) {
			robotName := extractRobotName(name)
			err := adapter.SetVelocity(ctx, robotName, x, y, z)
			if err != nil {
				return fmt.Sprintf("failed: %v", err), err
			}
			return fmt.Sprintf("set %s velocity to (%.2f, %.2f, %.2f)", name, x, y, z), nil
		},
		func(ctx context.Context, name string) (string, error) {
			robotName := extractRobotName(name)
			odo, err := adapter.GetOdometry(ctx, robotName)
			if err != nil {
				return fmt.Sprintf("failed: %v", err), err
			}
			return fmt.Sprintf("%s position: (%.3f, %.3f, %.3f)", name, odo.X, odo.Y, odo.Theta), nil
		},
		func(ctx context.Context, topicType string) ([]string, error) {
			return adapter.ListTopics(ctx, topicType)
		},
		func(ctx context.Context, name string, x, y, theta float64) (string, error) {
			_ = adapter.SetPosition(ctx, name, x, y, theta)
			return fmt.Sprintf("SetPosition is a STUB - this did NOT move the robot. You MUST use PlanRoute(robot_name=%q, waypoint_name=...) to navigate to destinations.", name), nil
		},
		func(ctx context.Context, robotName string, targetX, targetY, speed float64) (string, error) {
			if speed <= 0 {
				speed = 0.5
			}
			odo, err := adapter.GetOdometry(ctx, robotName)
			if err != nil {
				return fmt.Sprintf("failed to get odometry: %v", err), err
			}
			dx := targetX - odo.X
			dy := targetY - odo.Y
			distance := math.Sqrt(dx*dx + dy*dy)
			if distance < 0.001 {
				return `{"vx":0,"vy":0,"duration":0,"distance":0,"angle":0,"target_x":0,"target_y":0}`, nil
			}
			targetAngle := math.Atan2(dy, dx)
			angleDelta := targetAngle - odo.Theta
			for angleDelta > math.Pi {
				angleDelta -= 2 * math.Pi
			}
			for angleDelta < -math.Pi {
				angleDelta += 2 * math.Pi
			}
			duration := distance / speed
			return fmt.Sprintf(`{"vx":%.3f,"vy":0,"duration":%.1f,"distance":%.1f,"angle":%.3f,"target_x":%.3f,"target_y":%.3f}`,
				speed, duration, distance, angleDelta, targetX, targetY), nil
		},
		func(ctx context.Context, robotName string, direction string) (string, error) {
			scan, err := adapter.GetLaserScan(ctx, robotName)
			if err != nil {
				return fmt.Sprintf("failed to get laser scan: %v", err), err
			}
			if direction == "" {
				direction = "front"
			}
			var dist float64
			switch direction {
			case "front":
				dist = scan.Front
			case "front_left":
				dist = scan.FrontLeft
			case "front_right":
				dist = scan.FrontRight
			case "left":
				dist = scan.Left
			case "right":
				dist = scan.Right
			case "back_left":
				dist = scan.BackLeft
			case "back_right":
				dist = scan.BackRight
			case "back":
				dist = scan.Back
			default:
				return fmt.Sprintf("unknown direction: %s", direction), nil
			}
			status := "clear"
			if dist < 1.0 {
				status = "OBSTACLE WARNING"
			} else if dist < 3.0 {
				status = "caution"
			}
			return fmt.Sprintf("%s %s: %.1fm (%s)", robotName, direction, dist, status), nil
		},
		func(ctx context.Context, robotName string, direction string, distance float64, speed float64) (string, error) {
			if speed <= 0 {
				speed = 0.5
			}
			odo, err := adapter.GetOdometry(ctx, robotName)
			if err != nil {
				return fmt.Sprintf("failed to get odometry: %v", err), err
			}
			angle := 0.0
			switch direction {
			case "forward":
				angle = 0
			case "backward":
				angle = math.Pi
			case "left":
				angle = math.Pi / 2
			case "right":
				angle = -math.Pi / 2
			default:
				return fmt.Sprintf("unknown direction: %s", direction), nil
			}
			targetX := odo.X + distance*math.Cos(odo.Theta+angle)
			targetY := odo.Y + distance*math.Sin(odo.Theta+angle)
			dx := targetX - odo.X
			dy := targetY - odo.Y
			forwardDist := math.Sqrt(dx*dx + dy*dy)
			targetAngle := math.Atan2(dy, dx)
			angleDelta := targetAngle - odo.Theta
			for angleDelta > math.Pi {
				angleDelta -= 2 * math.Pi
			}
			for angleDelta < -math.Pi {
				angleDelta += 2 * math.Pi
			}
			forwardDuration := forwardDist / speed
			return fmt.Sprintf(`{"angle":%.3f,"vx":%.3f,"duration":%.1f,"distance":%.1f,"target_x":%.3f,"target_y":%.3f}`,
				angleDelta, speed, forwardDuration, forwardDist, targetX, targetY), nil
		},
		func(ctx context.Context, robotName string, waypointName string, targetX, targetY, speed float64) (string, error) {
			if nav == nil {
				return `{"error":"NavGraph not available. Use --nav-graph flag to specify path."}`, nil
			}
			if speed <= 0 {
				speed = 0.5
			}
			odo, err := adapter.GetOdometry(ctx, robotName)
			if err != nil {
				return fmt.Sprintf(`{"error":"failed to get odometry: %v"}`, err), err
			}

			var path []int
			var totalDist float64

			if waypointName == "patrol" || waypointName == "巡逻" {
				path, totalDist, err = nav.PatrolRoute(odo.X, odo.Y)
				if err != nil {
					return fmt.Sprintf(`{"error":"patrol route failed: %v"}`, err), nil
				}
			} else {
				toIdx := -1
				if waypointName != "" {
					idx, found := nav.FindByName(waypointName)
					if !found {
						return fmt.Sprintf(`{"error":"waypoint '%s' not found"}`, waypointName), nil
					}
					toIdx = idx
				} else {
					toIdx, _ = nav.Nearest(targetX, targetY)
				}

				fromIdx, fromDist := nav.Nearest(odo.X, odo.Y)
				if fromDist > 3.0 {
					return fmt.Sprintf(`{"error":"robot is %.1fm from nearest waypoint; too far for route planning"}`, fromDist), nil
				}

				path, totalDist, err = nav.ShortestPath(fromIdx, toIdx)
				if err != nil {
					return fmt.Sprintf(`{"error":"no route: %v"}`, err), nil
				}

				// Multi-robot conflict detection: check if this path conflicts with other robots' planned paths
				robotPlanPathsMu.Lock()
				var avoidVertices []int
				for otherRobot, otherPath := range robotPlanPaths {
					if otherRobot == robotName {
						continue
					}
					otherSet := make(map[int]bool, len(otherPath))
					for _, v := range otherPath {
						otherSet[v] = true
					}
					for _, v := range path {
						if v != fromIdx && v != toIdx && otherSet[v] {
							// Avoid ALL vertices of the other robot's path
							for _, ov := range otherPath {
								if ov != fromIdx && ov != toIdx {
									avoidVertices = append(avoidVertices, ov)
								}
							}
							break
						}
					}
				}
				if len(avoidVertices) > 0 {
					if altPath, altDist, altErr := nav.ShortestPathAvoiding(fromIdx, toIdx, avoidVertices); altErr == nil && !math.IsInf(altDist, 1) {
						log.Printf("[PlanRoute] %s conflict detected, rerouting (%.1fm → %.1fm)", robotName, totalDist, altDist)
						path = altPath
						totalDist = altDist
					} else if penPath, penDist, penErr := nav.ShortestPathMinimizeOverlap(fromIdx, toIdx, avoidVertices); penErr == nil && !math.IsInf(penDist, 1) {
						log.Printf("[PlanRoute] %s conflict detected, minimize-overlap path (%.1fm → %.1fm)", robotName, totalDist, penDist)
						path = penPath
						totalDist = penDist
					} else {
						log.Printf("[PlanRoute] %s conflict detected, keeping original path (avoid+minimize failed: %v)", robotName, penErr)
					}
				}
				pathCopy := make([]int, len(path))
				copy(pathCopy, path)
				robotPlanPaths[robotName] = pathCopy
				robotPlanPathsMu.Unlock()
			}

			segments := nav.PlanPath(odo.X, odo.Y, odo.Theta, path, speed)

		type segOut struct {
			Angle           float64 `json:"angle"`
			TargetTheta     float64 `json:"target_theta"`
			RotateZ         float64 `json:"rotate_z"`
			RotateDuration  float64 `json:"rotate_duration"`
			ForwardX        float64 `json:"forward_x"`
			ForwardDuration float64 `json:"forward_duration"`
			Distance        float64 `json:"distance"`
			TargetX         float64 `json:"target_x"`
			TargetY         float64 `json:"target_y"`
			FromName        string  `json:"from_name,omitempty"`
			ToName          string  `json:"to_name,omitempty"`
		}
		type routeOut struct {
			Segments      []segOut `json:"segments"`
			TotalDistance float64  `json:"total_distance"`
			WaypointNames []string `json:"waypoint_names"`
		}

		out := routeOut{TotalDistance: totalDist}
		for _, s := range segments {
			out.Segments = append(out.Segments, segOut{
				Angle: s.Angle, TargetTheta: s.TargetTheta,
				RotateZ: s.RotateZ, RotateDuration: s.RotateDuration,
				ForwardX: s.ForwardX, ForwardDuration: s.ForwardDuration,
				Distance: s.Distance,
				TargetX: s.ToX, TargetY: s.ToY,
				FromName: s.FromName, ToName: s.ToName,
			})
		}
		seen := map[string]bool{}
		for _, idx := range path {
			name := nav.Vertices[idx].Name
			if name != "" && !seen[name] {
				out.WaypointNames = append(out.WaypointNames, name)
				seen[name] = true
			}
		}
		b, _ := json.Marshal(out)
		return string(b), nil
		},
	)

	ctx := context.Background()
	chatModel, err := agent.NewChatModel(ctx)
	if err != nil {
		log.Fatalf("Failed to create chat model: %v\nMake sure ARK_API_KEY is set.", err)
	}

	fmt.Println(strings.Repeat("=", 55))
	fmt.Println("  LLM Task-Oriented Unmanned System")
	fmt.Println("  Natural Language → Structured Tasks → Multi-Robot Execution")
	fmt.Printf("  Simulator: %s | Session: %s\n", *simMode, sessionID)
	fmt.Println(strings.Repeat("=", 55))
	fmt.Println("  Commands:")
	fmt.Println("    /status  - show robot and session status")
	fmt.Println("    /history - show conversation history")
	fmt.Println("    /clear   - clear current session")
	fmt.Println("    /quit    - exit")
	fmt.Println(strings.Repeat("=", 55))

	stdinReader := bufio.NewReader(os.Stdin)

	for {
		fmt.Print("\n>>> ")
		input, err := stdinReader.ReadString('\n')
		if err != nil {
			break
		}
		input = strings.TrimSpace(input)
		if input == "" {
			continue
		}

		switch {
		case input == "/quit":
			fmt.Println("Bye!")
			return
		case input == "/status":
			printStatus(mem, sessionID, adapter)
			continue
		case input == "/history":
			printHistory(mem, sessionID)
			continue
		case input == "/clear":
			mem.Clear(nil, sessionID)
			fmt.Println("Session cleared.")
			continue
		}

		processInput(ctx, chatModel, mem, sessionID, sched, adapter, input, stdinReader)
	}
}

func processInput(ctx context.Context, chatModel *ark.ChatModel, mem *memory.InMemorySessionManager, sessionID string, sched *scheduler.Scheduler, adapter robot.RobotAdapter, input string, stdinReader *bufio.Reader) {
	mem.AddMessage(nil, sessionID, memory.Message{Role: memory.RoleUser, Content: input})

	// Step 1: Preprocess input (normalize robot names, extract action hints)
	norm := normalizer.NewNormalizer()
	pp := norm.Preprocess(input)
	if pp.NormalizedInput != input {
		fmt.Printf("[Normalizer] Normalized: %q → %q\n", input, pp.NormalizedInput)
	}
	if len(pp.Hints) > 0 {
		fmt.Printf("[Normalizer] Detected %d action(s):\n", len(pp.Hints))
		for _, h := range pp.Hints {
			switch h.Type {
			case normalizer.ActionRotate:
				fmt.Printf("  • %s: rotate %s %.0f°\n", h.Target, h.Direction, h.Value)
			case normalizer.ActionMoveForward, normalizer.ActionMoveBackward:
				fmt.Printf("  • %s: move %s %.1fm\n", h.Target, h.Direction, h.Value)
			}
		}
	}

	robotTypes := map[string]memory.RobotType{
		"robot1": memory.RobotTypeStandard,
		"robot2": memory.RobotTypeDelivery,
	}

	var stateLines []string
	for _, name := range []string{"robot1", "robot2"} {
		rt := robotTypes[name]
		capStr := memory.RobotCapabilitiesString(name, rt)
		stateLines = append(stateLines, fmt.Sprintf("[RobotInfo] %s: %s", name, capStr))

		if odo, err := adapter.GetOdometry(ctx, name); err == nil {
			line := fmt.Sprintf("[RobotState] %s at (%.2f, %.2f, %.2f)", name, odo.X, odo.Y, odo.Theta)
			if scan, err := adapter.GetLaserScan(ctx, name); err == nil {
				line += fmt.Sprintf(" | scan front=%.1fm left=%.1fm right=%.1fm",
					scan.Front, scan.Left, scan.Right)
			}
			stateLines = append(stateLines, line)
		}
	}
	robotStateStr := strings.Join(stateLines, "\n")

	recentMsgs, err := mem.GetRecentMessages(nil, sessionID, 10)
	if err != nil {
		recentMsgs = []memory.Message{}
	}
	historyContext := agent.BuildHistoryContext(recentMsgs)
	if robotStateStr != "" {
		historyContext = robotStateStr + "\n" + historyContext
	}

	plannerAgent, err := agent.NewTaskPlanner(ctx, chatModel, historyContext)
	if err != nil {
		log.Printf("Failed to create planner: %v", err)
		return
	}

	planChan := make(chan []agent.PlanOption, 1)
	agent.SetPlanOptionsCallback(func(opts []agent.PlanOption) {
		select {
		case planChan <- opts:
		default:
		}
	})

	posChan := make(chan struct{}, 1)
	agent.SetRobotPositionCallback(func(name string, x, y, theta float64) {
		select {
		case posChan <- struct{}{}:
		default:
		}
	})

	runner := adk.NewRunner(ctx, adk.RunnerConfig{
		Agent: plannerAgent,
	})

	// Send normalized input to LLM
	iter := runner.Query(ctx, pp.NormalizedInput)
		go func() {
			for {
				event, ok := iter.Next()
				if !ok {
					break
				}
				if event.Err != nil {
					log.Printf("Agent error: %v", event.Err)
					return
				}
			}
		}()

	var opts []agent.PlanOption
	gotPos := false
	select {
	case opts = <-planChan:
	case <-posChan:
		gotPos = true
	case <-time.After(180 * time.Second):
		fmt.Println("[Timeout] No plan options submitted by agent.")
		return
	}
	if gotPos {
		fmt.Println("[SetRobotPosition] Position set by agent.")
		select {
		case opts = <-planChan:
		case <-time.After(120 * time.Second):
			fmt.Println("[SetRobotPosition] No plan options submitted.")
			return
		}
	}

	if len(opts) == 0 {
		fmt.Println("No plan options generated.")
		return
	}

	var selected *task.TaskPlan
	if len(opts) == 1 {
		selected = opts[0].ToTaskPlan()
		fmt.Printf("\n[Plan] %s\n", opts[0].Description)
	} else {
		fmt.Println("\n--- Plan Options ---")
		for i, opt := range opts {
			fmt.Printf("  [%d] %s\n", i+1, opt.Description)
			for _, t := range opt.Tasks {
				var parts []string
				if t.X != 0 {
					parts = append(parts, fmt.Sprintf("x=%.2f", t.X))
				}
				if t.Z != 0 {
					parts = append(parts, fmt.Sprintf("z=%.2f", t.Z))
				}
				if t.Duration > 0 {
					parts = append(parts, fmt.Sprintf("duration=%.1fs", t.Duration))
				}
				if t.Target != "" {
					parts = append(parts, fmt.Sprintf("target=%s", t.Target))
				}
				pstr := strings.Join(parts, " ")
				if pstr != "" {
					pstr = " {" + pstr + "}"
				}
				deps := ""
				if len(t.Dependencies) > 0 {
					deps = fmt.Sprintf(" deps=%v", t.Dependencies)
				}
				fmt.Printf("       [%s]%s%s\n", t.Action, pstr, deps)
			}
		}
		fmt.Printf("Select option (1-%d) or 0 to cancel: ", len(opts))
		confirm, _ := stdinReader.ReadString('\n')
		confirm = strings.TrimSpace(confirm)
		idx := -1
		fmt.Sscanf(confirm, "%d", &idx)
		if idx < 1 || idx > len(opts) {
			fmt.Println("Cancelled.")
			mem.AddMessage(nil, sessionID, memory.Message{
				Role:    memory.RoleTool,
				Content: "Plan cancelled by user",
			})
			return
		}
		selected = opts[idx-1].ToTaskPlan()
	}

	// Validate and fix plan
	vr := norm.ValidateAndFix(selected, pp)
	if vr.WasCorrected {
		fmt.Println("\n[Corrections] Plan corrected:")
		for _, c := range vr.Corrections {
			fmt.Printf("  • %s\n", c)
		}
	}

	for _, t := range selected.Tasks {
		if t.Config == (task.TaskConfig{}) {
			cfg := task.DefaultTaskConfig()
			cfg.OnFailure = task.SkipAndContinue
			t.Config = cfg
		}
	}

	fmt.Println("\n[Executing] ...")
	execErr := sched.ExecutePlan(ctx, selected, sessionID)
	fmt.Print(scheduler.FormatResultSummary(selected))

	if execErr != nil {
		mem.AddMessage(nil, sessionID, memory.Message{
			Role:    memory.RoleTool,
			Content: fmt.Sprintf("Execution error: %v", execErr),
		})
	} else {
		success := 0
		for _, r := range selected.Results {
			if r.Status == task.StatusCompleted {
				success++
			}
		}
		mem.AddMessage(nil, sessionID, memory.Message{
			Role:    memory.RoleTool,
			Content: fmt.Sprintf("Execution complete: %d/%d tasks succeeded", success, len(selected.Results)),
		})
	}

	updateSessionRobotStates(mem, sessionID, adapter)
}

func updateSessionRobotStates(mem *memory.InMemorySessionManager, sessionID string, adapter robot.RobotAdapter) {
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()
	for _, name := range []string{"robot1", "robot2"} {
		odo, odoErr := adapter.GetOdometry(ctx, name)
		if odoErr != nil {
			continue
		}
		state := &memory.RobotState{
			Name:   name,
			X:      odo.X,
			Y:      odo.Y,
			Theta:  odo.Theta,
			Status: string(memory.RobotIdle),
		}
		if scan, err := adapter.GetLaserScan(ctx, name); err == nil {
			state.Laser = map[string]float64{
				"front":       scan.Front,
				"front_left":  scan.FrontLeft,
				"front_right": scan.FrontRight,
				"left":        scan.Left,
				"right":       scan.Right,
				"back_left":   scan.BackLeft,
				"back_right":  scan.BackRight,
				"back":        scan.Back,
			}
		}
		mem.UpdateRobotState(nil, sessionID, state)
	}
}

func printStatus(mem *memory.InMemorySessionManager, sessionID string, adapter robot.RobotAdapter) {
	// Always refresh states from live data (bridge returns current data < 1.5s)
	updateSessionRobotStates(mem, sessionID, adapter)

	session, ok := mem.GetSession(nil, sessionID)
	if !ok {
		fmt.Println("No session found.")
		return
	}

	fmt.Println("\n--- Robot Status ---")
	if len(session.RobotStates) == 0 {
		fmt.Println("  (no data yet — bridge may still be connecting)")
	} else {
		for _, state := range session.RobotStates {
			scan := ""
			if state.Laser != nil {
				scan = fmt.Sprintf(" scan front=%.1f", state.Laser["front"])
			}
			fmt.Printf("  %s: (%.2f, %.2f, %.2f) [%s]%s\n",
				state.Name, state.X, state.Y, state.Theta, state.Status, scan)
		}
	}
	fmt.Printf("\nMessages in session: %d\n", len(session.Messages))
}

func printHistory(mem *memory.InMemorySessionManager, sessionID string) {
	msgs, err := mem.GetMessages(nil, sessionID)
	if err != nil {
		fmt.Printf("Error: %v\n", err)
		return
	}
	if len(msgs) == 0 {
		fmt.Println("No messages yet.")
		return
	}

	fmt.Println("\n--- Conversation History ---")
	for _, msg := range msgs {
		prefix := ""
		switch msg.Role {
		case memory.RoleUser:
			prefix = "You"
		case memory.RoleAssistant:
			prefix = "Agent"
		case memory.RoleTool:
			prefix = "System"
		default:
			prefix = string(msg.Role)
		}
		content := msg.Content
		if len(content) > 120 {
			content = content[:120] + "..."
		}
		fmt.Printf("  [%s] %s\n", prefix, content)
	}
}

func extractRobotName(topicName string) string {
	if len(topicName) > 0 && topicName[0] == '/' {
		topicName = topicName[1:]
	}
	for i := 0; i < len(topicName); i++ {
		if topicName[i] == '/' {
			return topicName[:i]
		}
	}
	return topicName
}
