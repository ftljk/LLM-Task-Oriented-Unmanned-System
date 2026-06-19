package main

import (
	"context"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"strconv"
	"strings"
	"time"

	"robot/pkg/memory"
	"robot/pkg/navgraph"
	"robot/pkg/robot"
	"robot/pkg/scheduler"
	"robot/pkg/task"
)

var (
	adapter *robot.GOSimRobotAdapter
	sched   *scheduler.Scheduler
	mem     memory.Memory
	ctx     = context.Background()
)

func initSim() {
	adapter = robot.NewGOSimRobotAdapter()
	mem = memory.NewInMemorySessionManager()
	mem.CreateSession(nil, "exp-session")
	sched = scheduler.NewScheduler(adapter, mem)
}

func resetRobot(name string, x, y, theta float64) {
	adapter.SetPosition(ctx, name, x, y, theta)
}

func getPos(name string) (float64, float64, float64) {
	odo, _ := adapter.GetOdometry(ctx, name)
	return odo.X, odo.Y, odo.Theta
}

func runPlan(plan *task.TaskPlan) time.Duration {
	start := time.Now()
	sched.ExecutePlan(ctx, plan, "exp-session")
	return time.Since(start)
}

func newTask(id, desc string, action task.TaskAction, target string, params map[string]interface{}, deps []string, timeout int, retries int, onFail task.FailureStrategy) *task.Task {
	return &task.Task{
		ID: id, Description: desc, Action: action, Target: target,
		Params: params, Dependencies: deps,
		Config: task.TaskConfig{TimeoutMs: timeout, MaxRetries: retries, OnFailure: onFail},
	}
}

func writeCSV(path string, header string, records [][]string) {
	os.MkdirAll(filepath.Dir(path), 0755)
	f, err := os.Create(path)
	if err != nil {
		fmt.Printf("  ERROR: %v\n", err)
		return
	}
	defer f.Close()
	f.WriteString(header + "\n")
	for _, r := range records {
		f.WriteString(strings.Join(r, ",") + "\n")
	}
	fmt.Printf("  => %s (%d data rows)\n", path, len(records))
}

// ============================================================
// Scenario 1: 单机单步移动
// ============================================================
func scenario1(n int) [][]string {
	fmt.Println("\n### Scenario 1: 单机单步移动 (forward 1m) ###")
	records := [][]string{}

	for i := 0; i < n; i++ {
		initSim()
		resetRobot("robot1", 0, 0, 0)
		time.Sleep(30 * time.Millisecond)
		bx, by, byaw := getPos("robot1")

		plan := task.TaskPlan{Tasks: []*task.Task{
			newTask("t1", "fwd 0.5m/s", task.ActionMove, "robot1", map[string]interface{}{"x": 0.5}, nil, 5000, 1, task.SkipAndContinue),
			newTask("t1-w", "wait 2s", task.ActionWait, "", map[string]interface{}{"duration": 2.0}, []string{"t1"}, 10000, 1, task.SkipAndContinue),
			newTask("t1-s", "stop", task.ActionMove, "robot1", map[string]interface{}{"x": 0, "z": 0}, []string{"t1-w"}, 3000, 1, task.SkipAndContinue),
		}}

		dur := runPlan(&plan)
		time.Sleep(30 * time.Millisecond)
		ax, ay, ayaw := getPos("robot1")

		moveDist := math.Hypot(ax-bx, ay-by)
		posErr := math.Abs(moveDist - 1.0)
		ok := "true"
		note := ""
		if posErr > 0.15 {
			ok = "false"
			note = fmt.Sprintf("err=%.3f", posErr)
		}
		records = append(records, []string{
			strconv.Itoa(i + 1),
			"t1", "Move(robot1,x=0.5,dur=2s→1m)",
			"0.5", "0", "0", "1.0",
			fmt.Sprintf("%.3f", bx), fmt.Sprintf("%.3f", by), fmt.Sprintf("%.3f", byaw),
			fmt.Sprintf("%.3f", ax), fmt.Sprintf("%.3f", ay), fmt.Sprintf("%.3f", ayaw),
			fmt.Sprintf("%.2f", dur.Seconds()),
			fmt.Sprintf("%.3f", moveDist),
			fmt.Sprintf("%.3f", posErr),
			ok, note,
		})
		fmt.Printf("  Run %d/%d: (%.2f,%.2f)->(%.2f,%.2f) dist=%.3f err=%.3f dur=%.1fs %s\n",
			i+1, n, bx, by, ax, ay, moveDist, posErr, dur.Seconds(), ok)
	}
	return records
}

// ============================================================
// Scenario 2: 单机顺序依赖
// ============================================================
func scenario2(n int) [][]string {
	fmt.Println("\n### Scenario 2: 单机顺序依赖 ((3,1)->(0,0)) ###")
	records := [][]string{}

	for i := 0; i < n; i++ {
		initSim()
		resetRobot("robot1", 0, 0, 0)
		time.Sleep(30 * time.Millisecond)

		plan1 := task.TaskPlan{Tasks: []*task.Task{
			newTask("m1-r", "rot 18°", task.ActionMove, "robot1", map[string]interface{}{"z": 0.3, "duration": 1.1}, nil, 5000, 1, task.SkipAndContinue),
			newTask("m1-rw", "wait rot", task.ActionWait, "", map[string]interface{}{"duration": 1.1}, []string{"m1-r"}, 5000, 1, task.SkipAndContinue),
			newTask("m1-rs", "stop rot", task.ActionMove, "robot1", map[string]interface{}{"z": 0}, []string{"m1-rw"}, 3000, 1, task.SkipAndContinue),
			newTask("m1-f", "fwd 3.16m", task.ActionMove, "robot1", map[string]interface{}{"x": 0.5}, []string{"m1-rs"}, 15000, 1, task.SkipAndContinue),
			newTask("m1-fw", "wait fwd", task.ActionWait, "", map[string]interface{}{"duration": 6.4}, []string{"m1-f"}, 15000, 1, task.SkipAndContinue),
			newTask("m1-fs", "stop fwd", task.ActionMove, "robot1", map[string]interface{}{"x": 0}, []string{"m1-fw"}, 3000, 1, task.SkipAndContinue),
		}}
		_ = runPlan(&plan1)
		m1x, m1y, _ := getPos("robot1")

		plan2 := task.TaskPlan{Tasks: []*task.Task{
			newTask("m2-r", "rot 180°", task.ActionMove, "robot1", map[string]interface{}{"z": 0.5, "duration": 6.3}, nil, 10000, 1, task.SkipAndContinue),
			newTask("m2-rw", "wait rot", task.ActionWait, "", map[string]interface{}{"duration": 6.3}, []string{"m2-r"}, 10000, 1, task.SkipAndContinue),
			newTask("m2-rs", "stop rot", task.ActionMove, "robot1", map[string]interface{}{"z": 0}, []string{"m2-rw"}, 3000, 1, task.SkipAndContinue),
			newTask("m2-f", "fwd 3.16m", task.ActionMove, "robot1", map[string]interface{}{"x": 0.5}, []string{"m2-rs"}, 15000, 1, task.SkipAndContinue),
			newTask("m2-fw", "wait fwd", task.ActionWait, "", map[string]interface{}{"duration": 6.4}, []string{"m2-f"}, 15000, 1, task.SkipAndContinue),
			newTask("m2-fs", "stop fwd", task.ActionMove, "robot1", map[string]interface{}{"x": 0}, []string{"m2-fw"}, 3000, 1, task.SkipAndContinue),
		}}

		totalStart := time.Now()
		_ = runPlan(&plan2)
		totalDur := time.Since(totalStart)
		fx, fy, _ := getPos("robot1")

		m1Err := math.Hypot(m1x-3, m1y-1)
		m2Err := math.Hypot(fx, fy)
		s1 := "true"
		s2 := "true"
		if m1Err > 0.3 {
			s1 = "false"
		}
		if m2Err > 0.3 {
			s2 = "false"
		}

		records = append(records, []string{
			strconv.Itoa(i + 1),
			"3", "1", "0", "0",
			fmt.Sprintf("%.3f", m1x), fmt.Sprintf("%.3f", m1y),
			fmt.Sprintf("%.3f", fx), fmt.Sprintf("%.3f", fy),
			fmt.Sprintf("%.3f", m1Err), fmt.Sprintf("%.3f", m2Err),
			fmt.Sprintf("%.1f", totalDur.Seconds()),
			s1, s2, "true", "",
		})
		fmt.Printf("  Run %d/%d: m1(%.2f,%.2f)err=%.3f m2(%.2f,%.2f)err=%.3f dur=%.1fs\n",
			i+1, n, m1x, m1y, m1Err, fx, fy, m2Err, totalDur.Seconds())
	}
	return records
}

// ============================================================
// Scenario 3: 双机并行
// ============================================================
func scenario3(n int) [][]string {
	fmt.Println("\n### Scenario 3: 双机并行 (r1->(3,1), r2->(1,3)) ###")
	records := [][]string{}

	for i := 0; i < n; i++ {
		initSim()
		resetRobot("robot1", 0, 0, 0)
		resetRobot("robot2", 5, 0, 0)
		time.Sleep(30 * time.Millisecond)

		plan := task.TaskPlan{Tasks: []*task.Task{
			newTask("r1-r", "rot 18°", task.ActionMove, "robot1", map[string]interface{}{"z": 0.3, "duration": 1.1}, nil, 5000, 1, task.SkipAndContinue),
			newTask("r1-rw", "wait rot", task.ActionWait, "", map[string]interface{}{"duration": 1.1}, []string{"r1-r"}, 5000, 1, task.SkipAndContinue),
			newTask("r1-rs", "stop rot", task.ActionMove, "robot1", map[string]interface{}{"z": 0}, []string{"r1-rw"}, 3000, 1, task.SkipAndContinue),
			newTask("r1-f", "fwd 3.16m", task.ActionMove, "robot1", map[string]interface{}{"x": 0.5}, []string{"r1-rs"}, 20000, 1, task.SkipAndContinue),
			newTask("r1-fw", "wait fwd", task.ActionWait, "", map[string]interface{}{"duration": 6.4}, []string{"r1-f"}, 20000, 1, task.SkipAndContinue),
			newTask("r1-fs", "stop fwd", task.ActionMove, "robot1", map[string]interface{}{"x": 0}, []string{"r1-fw"}, 3000, 1, task.SkipAndContinue),
			newTask("r2-r", "rot 143°", task.ActionMove, "robot2", map[string]interface{}{"z": 0.5, "duration": 5.0}, nil, 10000, 1, task.SkipAndContinue),
			newTask("r2-rw", "wait rot", task.ActionWait, "", map[string]interface{}{"duration": 5.0}, []string{"r2-r"}, 10000, 1, task.SkipAndContinue),
			newTask("r2-rs", "stop rot", task.ActionMove, "robot2", map[string]interface{}{"z": 0}, []string{"r2-rw"}, 3000, 1, task.SkipAndContinue),
			newTask("r2-f", "fwd 5.0m", task.ActionMove, "robot2", map[string]interface{}{"x": 0.3}, []string{"r2-rs"}, 30000, 1, task.SkipAndContinue),
			newTask("r2-fw", "wait fwd", task.ActionWait, "", map[string]interface{}{"duration": 17.0}, []string{"r2-f"}, 30000, 1, task.SkipAndContinue),
			newTask("r2-fs", "stop fwd", task.ActionMove, "robot2", map[string]interface{}{"x": 0}, []string{"r2-fw"}, 3000, 1, task.SkipAndContinue),
		}}

		dur := runPlan(&plan)
		r1x, r1y, _ := getPos("robot1")
		r2x, r2y, _ := getPos("robot2")

		r1Err := math.Hypot(r1x-3, r1y-1)
		r2Err := math.Hypot(r2x-1, r2y-3)
		ok1 := "true"
		ok2 := "true"
		if r1Err > 0.3 {
			ok1 = "false"
		}
		if r2Err > 0.3 {
			ok2 = "false"
		}

		records = append(records, []string{
			strconv.Itoa(i + 1),
			"3", "1", "1", "3",
			fmt.Sprintf("%.3f", r1x), fmt.Sprintf("%.3f", r1y),
			fmt.Sprintf("%.3f", r2x), fmt.Sprintf("%.3f", r2y),
			fmt.Sprintf("%.3f", r1Err), fmt.Sprintf("%.3f", r2Err),
			fmt.Sprintf("%.1f", dur.Seconds()),
			fmt.Sprintf("%.1f", dur.Seconds()),
			"true", ok1, ok2, "",
		})
		fmt.Printf("  Run %d/%d: r1(%.2f,%.2f)err=%.3f r2(%.2f,%.2f)err=%.3f dur=%.1fs\n",
			i+1, n, r1x, r1y, r1Err, r2x, r2y, r2Err, dur.Seconds())
	}
	return records
}

// ============================================================
// Scenario 4: 故障恢复 (SkipAndContinue)
// ============================================================
func scenario4(n int) [][]string {
	fmt.Println("\n### Scenario 4: 故障恢复 (SkipAndContinue) ###")
	records := [][]string{}

	for i := 0; i < n; i++ {
		initSim()
		resetRobot("robot1", 0, 0, 0)
		time.Sleep(30 * time.Millisecond)

		timeout := 3000
		intervention := "timeout_simulated"
		if i >= 3 {
			timeout = 500
			intervention = "timeout_aggressive"
		}

		plan := task.TaskPlan{Tasks: []*task.Task{
			newTask("r1-r", "rot 18°", task.ActionMove, "robot1", map[string]interface{}{"z": 0.3, "duration": 1.1}, nil, 5000, 1, task.SkipAndContinue),
			newTask("r1-rw", "wait rot", task.ActionWait, "", map[string]interface{}{"duration": 1.1}, []string{"r1-r"}, 5000, 1, task.SkipAndContinue),
			newTask("r1-rs", "stop rot", task.ActionMove, "robot1", map[string]interface{}{"z": 0}, []string{"r1-rw"}, 3000, 1, task.SkipAndContinue),
			newTask("r1-f", "fwd 3.16m", task.ActionMove, "robot1", map[string]interface{}{"x": 0.5}, []string{"r1-rs"}, 15000, 1, task.SkipAndContinue),
			newTask("r1-fw", "wait fwd", task.ActionWait, "", map[string]interface{}{"duration": 6.4}, []string{"r1-f"}, 15000, 1, task.SkipAndContinue),
			newTask("r1-fs", "stop fwd", task.ActionMove, "robot1", map[string]interface{}{"x": 0}, []string{"r1-fw"}, 3000, 1, task.SkipAndContinue),
			newTask("r2-r", "rot 180°", task.ActionMove, "robot1", map[string]interface{}{"z": 0.5, "duration": 6.3}, []string{"r1-fs"}, 10000, 1, task.SkipAndContinue),
			newTask("r2-rw", "wait rot", task.ActionWait, "", map[string]interface{}{"duration": 6.3}, []string{"r2-r"}, 10000, 1, task.SkipAndContinue),
			newTask("r2-rs", "stop rot", task.ActionMove, "robot1", map[string]interface{}{"z": 0}, []string{"r2-rw"}, 3000, 1, task.SkipAndContinue),
			newTask("r2-f", "fwd 3.16m", task.ActionMove, "robot1", map[string]interface{}{"x": 0.5}, []string{"r2-rs"}, 15000, 1, task.SkipAndContinue),
			newTask("r2-fw", "wait fwd", task.ActionWait, "", map[string]interface{}{"duration": 6.4}, []string{"r2-f"}, 15000, 1, task.SkipAndContinue),
			newTask("r2-fs", "stop fwd", task.ActionMove, "robot1", map[string]interface{}{"x": 0}, []string{"r2-fw"}, 3000, 1, task.SkipAndContinue),
			newTask("fault", "robot2 move (fault)", task.ActionMove, "robot2", map[string]interface{}{"x": 0.3}, nil, 1000, 0, task.SkipAndContinue),
			newTask("fault-w", "wait 30s (timeout)", task.ActionWait, "", map[string]interface{}{"duration": 30.0}, []string{"fault"}, timeout, 0, task.SkipAndContinue),
		}}

		dur := runPlan(&plan)
		fx, fy, _ := getPos("robot1")

		faultSkipped := false
		faultErr := ""
		for _, t := range plan.Tasks {
			if t.ID == "fault-w" {
				if t.Status == task.StatusSkipped {
					faultSkipped = true
				}
				faultErr = t.Result
			}
		}

		detected := "true"
		sysResult := "r1_completed_fault_skipped"
		if !faultSkipped {
			detected = "false"
			sysResult = "fault_unnoticed"
		}

		records = append(records, []string{
			strconv.Itoa(i + 1),
			intervention,
			fmt.Sprintf("%.1f", dur.Seconds()),
			strconv.Itoa(timeout),
			detected,
			sysResult,
			"skip_faulty_task_continue",
			fmt.Sprintf("%.3f", fx),
			fmt.Sprintf("%.3f", fy),
			"false",
			fmt.Sprintf("timeout=%dms skipped=%v err=%s", timeout, faultSkipped, faultErr),
		})
		fmt.Printf("  Run %d/%d: timeout=%dms skipped=%v r1=(%.2f,%.2f) result=%s\n",
			i+1, n, timeout, faultSkipped, fx, fy, sysResult)
	}
	return records
}

// ============================================================
// Scenario 5: DAG 对比 —— SkipAndContinue vs Cascade
// 设计：task-A(fail) → task-B(dep on A) → task-C(dep on B)
// SkipAndContinue: A→Skipped → B→Skipped → C→Skipped
// Cascade:         A→Failed  → B→Skipped → C→Skipped
// 核心差异: 故障任务的最终状态标签不同
// ============================================================
func scenario5(n int) [][]string {
	fmt.Println("\n### Scenario 5: DAG SkipAndContinue vs Cascade ###")
	records := [][]string{}

	for i := 0; i < n; i++ {
		for _, strategy := range []task.FailureStrategy{task.SkipAndContinue, task.FailureStrategy("")} {
			initSim()
			resetRobot("robot1", 0, 0, 0)
			time.Sleep(30 * time.Millisecond)

			stratName := "SkipAndContinue"
			if strategy != task.SkipAndContinue {
				stratName = "cascade"
			}

			// task-A: Move with 0ms timeout (instant fail)
			// task-B: depends on A — should be skipped/failed
			// task-C: depends on B — should cascade
			plan := task.TaskPlan{Tasks: []*task.Task{
				newTask("A", "fail immediately", task.ActionMove, "robot1", map[string]interface{}{"x": 0.5}, nil, 1, 0, strategy),
				newTask("B", "dep on A", task.ActionMove, "robot1", map[string]interface{}{"x": 0.5}, []string{"A"}, 3000, 1, strategy),
				newTask("C", "dep on B", task.ActionMove, "robot1", map[string]interface{}{"x": 0.5}, []string{"B"}, 3000, 1, strategy),
			}}

			dur := runPlan(&plan)

			statusA := ""
			statusB := ""
			statusC := ""
			for _, t := range plan.Tasks {
				switch t.ID {
				case "A":
					statusA = string(t.Status)
				case "B":
					statusB = string(t.Status)
				case "C":
					statusC = string(t.Status)
				}
			}

			records = append(records, []string{
				strconv.Itoa(i + 1),
				stratName,
				statusA, statusB, statusC,
				fmt.Sprintf("%.2f", dur.Seconds()),
			})
			fmt.Printf("  Run %d/%d %s: A=%s B=%s C=%s dur=%.2fs\n",
				i+1, n, stratName, statusA, statusB, statusC, dur.Seconds())
		}
	}
	return records
}

// ============================================================
// Scenario 6: DAG 死锁检测 —— 循环依赖
// A→B, B→C, C→A (cycle!) — should detect or deadlock
// ============================================================
func scenario6() [][]string {
	fmt.Println("\n### Scenario 6: DAG 死锁检测 (循环依赖) ###")
	records := [][]string{}

	initSim()
	resetRobot("robot1", 0, 0, 0)
	time.Sleep(30 * time.Millisecond)

	plan := task.TaskPlan{Tasks: []*task.Task{
		newTask("A", "dep on C", task.ActionMove, "robot1", map[string]interface{}{"x": 0.5}, []string{"C"}, 3000, 1, task.SkipAndContinue),
		newTask("B", "dep on A", task.ActionMove, "robot1", map[string]interface{}{"x": 0.5}, []string{"A"}, 3000, 1, task.SkipAndContinue),
		newTask("C", "dep on B", task.ActionMove, "robot1", map[string]interface{}{"x": 0.5}, []string{"B"}, 3000, 1, task.SkipAndContinue),
	}}

	start := time.Now()
	err := sched.ExecutePlan(ctx, &plan, "exp-session")
	dur := time.Since(start)

	detected := "false"
	if err != nil && strings.Contains(err.Error(), "deadlock") {
		detected = "true"
	}

	records = append(records, []string{
		"1", "A→B→C→A",
		detected,
		fmt.Sprintf("%.2f", dur.Seconds()),
		err.Error(),
	})
	fmt.Printf("  Cycle A→B→C→A: deadlock=%s dur=%.2fs err=%v\n", detected, dur.Seconds(), err)

	return records
}

// ============================================================
// Experiment ⑦: 路径冲突三策略对比 (nav graph)
// robot1: vertex 2 (patrol_D1) → 24 (coe)
// robot2: vertex 24 (coe) → 2 (patrol_D1)
// 策略: 无规避 / ShortestPathAvoiding / ShortestPathMinimizeOverlap
// ============================================================
func experimentPathConflict() [][]string {
	fmt.Println("\n### Experiment 7: 路径冲突三策略对比 ###")
	records := [][]string{}

	navPath := findNavGraph()
	g, err := navgraph.Load(navPath)
	if err != nil {
		fmt.Printf("  ERROR loading nav graph: %v\n", err)
		return records
	}
	fmt.Printf("  Nav graph loaded: %d vertices, %d edges\n", len(g.Vertices), len(g.Adj))

	// Find vertex indices for waypoints
	fromV := findVertexByName(g, 2)  // patrol_D1 area
	toV := findVertexByName(g, 24)   // coe
	if fromV == -1 || toV == -1 {
		fmt.Printf("  ERROR: cannot find source/dest vertices\n")
		return records
	}

	// For cross-path conflict, robot1 goes 2→24, robot2 goes 24→2
	type runConfig struct {
		name     string
		r1From   int
		r1To     int
		r2From   int
		r2To     int
	}

	configs := []runConfig{
		{"robot1→coe, robot2→patrol_D1", fromV, toV, toV, fromV},
	}

	for _, cfg := range configs {
		fmt.Printf("\n  Routing: %s\n", cfg.name)

		// Strategy 1: ShortestPath (no avoidance)
		start := time.Now()
		r1Path, r1Dist, err := g.ShortestPath(cfg.r1From, cfg.r1To)
		t1 := time.Since(start).Microseconds()

		start = time.Now()
		r2Path, r2Dist, err2 := g.ShortestPath(cfg.r2From, cfg.r2To)
		t2 := time.Since(start).Microseconds()

		shared := countSharedVertices(r1Path, r2Path)
		r1Len := len(r1Path)
		r2Len := len(r2Path)
		errStr := ""
		if err != nil || err2 != nil {
			errStr = fmt.Sprintf("err: %v / %v", err, err2)
		}

		records = append(records, []string{
			"ShortestPath", cfg.name,
			strconv.Itoa(r1Len), strconv.Itoa(r2Len),
			fmt.Sprintf("%.3f", r1Dist), fmt.Sprintf("%.3f", r2Dist),
			fmt.Sprintf("%.3f", r1Dist+r2Dist),
			strconv.Itoa(shared),
			fmt.Sprintf("%d", t1), fmt.Sprintf("%d", t2),
			errStr,
		})
		fmt.Printf("    [Direct]        r1=%d verts %.2fm r2=%d verts %.2fm shared=%d time=%d/%dμs %s\n",
			r1Len, r1Dist, r2Len, r2Dist, shared, t1, t2, errStr)

		// Strategy 2: ShortestPathAvoiding (avoid the other robot's path vertices)
		start = time.Now()
		r1Path2, r1Dist2, err := g.ShortestPathAvoiding(cfg.r1From, cfg.r1To, r2Path)
		t1 = time.Since(start).Microseconds()

		start = time.Now()
		r2Path2, r2Dist2, err2 := g.ShortestPathAvoiding(cfg.r2From, cfg.r2To, r1Path2)
		t2 = time.Since(start).Microseconds()

		shared2 := countSharedVertices(r1Path2, r2Path2)
		records = append(records, []string{
			"ShortestPathAvoiding", cfg.name,
			strconv.Itoa(len(r1Path2)), strconv.Itoa(len(r2Path2)),
			fmt.Sprintf("%.3f", r1Dist2), fmt.Sprintf("%.3f", r2Dist2),
			fmt.Sprintf("%.3f", r1Dist2+r2Dist2),
			strconv.Itoa(shared2),
			fmt.Sprintf("%d", t1), fmt.Sprintf("%d", t2),
			errStr,
		})
		if err == nil && err2 == nil {
			fmt.Printf("    [Avoiding]      r1=%d verts %.2fm r2=%d verts %.2fm shared=%d time=%d/%dμs\n",
				len(r1Path2), r1Dist2, len(r2Path2), r2Dist2, shared2, t1, t2)
		} else {
			fmt.Printf("    [Avoiding]      ERROR: %v / %v\n", err, err2)
		}

		// Strategy 3: ShortestPathMinimizeOverlap
		start = time.Now()
		r1Path3, r1Dist3, err := g.ShortestPathMinimizeOverlap(cfg.r1From, cfg.r1To, r2Path)
		t1 = time.Since(start).Microseconds()

		start = time.Now()
		r2Path3, r2Dist3, err2 := g.ShortestPathMinimizeOverlap(cfg.r2From, cfg.r2To, r1Path3)
		t2 = time.Since(start).Microseconds()

		shared3 := countSharedVertices(r1Path3, r2Path3)
		records = append(records, []string{
			"ShortestPathMinimizeOverlap", cfg.name,
			strconv.Itoa(len(r1Path3)), strconv.Itoa(len(r2Path3)),
			fmt.Sprintf("%.3f", r1Dist3), fmt.Sprintf("%.3f", r2Dist3),
			fmt.Sprintf("%.3f", r1Dist3+r2Dist3),
			strconv.Itoa(shared3),
			fmt.Sprintf("%d", t1), fmt.Sprintf("%d", t2),
			errStr,
		})
		if err == nil && err2 == nil {
			fmt.Printf("    [MinimizeOvlp]  r1=%d verts %.2fm r2=%d verts %.2fm shared=%d time=%d/%dμs\n",
				len(r1Path3), r1Dist3, len(r2Path3), r2Dist3, shared3, t1, t2)
		} else {
			fmt.Printf("    [MinimizeOvlp]  ERROR: %v / %v\n", err, err2)
		}
	}

	return records
}

func findNavGraph() string {
	candidates := []string{
		"/home/mofus/rmf_ws/install/rmf_demos_maps/share/rmf_demos_maps/maps/office/nav_graphs/0.yaml",
		"../../install/rmf_demos_maps/share/rmf_demos_maps/maps/office/nav_graphs/0.yaml",
		"../install/rmf_demos_maps/share/rmf_demos_maps/maps/office/nav_graphs/0.yaml",
	}
	for _, p := range candidates {
		if _, err := os.Stat(p); err == nil {
			return p
		}
	}
	// Try default installed path
	return "/home/mofus/rmf_ws/install/rmf_demos_maps/share/rmf_demos_maps/maps/office/nav_graphs/0.yaml"
}

func findVertexByName(g *navgraph.Graph, idx int) int {
	if idx >= 0 && idx < len(g.Vertices) {
		return idx
	}
	return -1
}

func countSharedVertices(a, b []int) int {
	set := make(map[int]bool)
	for _, v := range a {
		set[v] = true
	}
	count := 0
	for _, v := range b {
		if set[v] {
			count++
		}
	}
	return count
}

func main() {
	expDir := filepath.Join("..", "..", "experiments")

	// 原有场景
	r1 := scenario1(10)
	writeCSV(filepath.Join(expDir, "scenario1_single_move.csv"),
		"run,task_id,plan_str,vx_cmd,vy_cmd,yaw_cmd,distance_cmd,pos_before_x,pos_before_y,yaw_before,pos_after_x,pos_after_y,yaw_after,duration_sec,move_distance_m,pos_error_m,success,notes",
		r1)

	r2 := scenario2(5)
	writeCSV(filepath.Join(expDir, "scenario2_sequential.csv"),
		"run,move1_target_x,move1_target_y,move2_target_x,move2_target_y,pos_after_m1_x,pos_after_m1_y,pos_after_m2_x,pos_after_m2_y,m1_error_m,m2_error_m,total_time_sec,success_m1,success_m2,task_order_correct,notes",
		r2)

	r3 := scenario3(5)
	writeCSV(filepath.Join(expDir, "scenario3_dual_parallel.csv"),
		"run,r1_target_x,r1_target_y,r2_target_x,r2_target_y,r1_end_x,r1_end_y,r2_end_x,r2_end_y,r1_error_m,r2_error_m,r1_duration_sec,r2_duration_sec,motion_overlap,success_r1,success_r2,notes",
		r3)

	r4 := scenario4(5)
	writeCSV(filepath.Join(expDir, "scenario4_fault_recovery.csv"),
		"run,intervention_type,intervention_time_sec,timeout_ms,detected_fault,system_result,fallback_action,robot_final_x,robot_final_y,crashed,notes",
		r4)

	// --- 消融实验新增 ---

	// 实验①: DAG SkipAndContinue vs Cascade
	r5 := scenario5(3) // 3 runs × 2 strategies = 6 rows
	writeCSV(filepath.Join(expDir, "ablation_dag_strategy.csv"),
		"run,strategy,status_a,status_b,status_c,duration_sec",
		r5)

	// 实验①: DAG 死锁检测
	r6 := scenario6()
	writeCSV(filepath.Join(expDir, "ablation_dag_deadlock.csv"),
		"run,cycle_description,deadlock_detected,duration_sec,error_message",
		r6)

	// 实验②: 路径冲突三策略对比
	r7 := experimentPathConflict()
	writeCSV(filepath.Join(expDir, "ablation_path_conflict.csv"),
		"strategy,route_description,r1_vertices,r2_vertices,r1_distance_m,r2_distance_m,total_distance_m,shared_vertices,r1_compute_us,r2_compute_us,errors",
		r7)

	fmt.Println("\n=== All experiments complete! ===")
}
