package main

import (
	"context"
	"fmt"
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

// ─── 实验①：DAG SkipAndContinue vs Cascade ───
func dagStrategy(n int) [][]string {
	fmt.Println("\n### 实验①-A: DAG SkipAndContinue vs Cascade ###")
	records := [][]string{}
	for i := 0; i < n; i++ {
		for _, strat := range []struct {
			val  task.FailureStrategy
			name string
		}{{task.SkipAndContinue, "SkipAndContinue"}, {task.FailImmediate, "cascade"}} {
			initSim()
			resetRobot("robot1", 0, 0, 0)
			time.Sleep(10 * time.Millisecond)

			// task-A: Wait for 60s, but timeout=1ms → instantly fails
			// task-B: depends on A → skipped/failed per OnFailure strategy
			// task-C: depends on B → cascade
			plan := task.TaskPlan{Tasks: []*task.Task{
				newTask("A", "fail by timeout", task.ActionWait, "", map[string]interface{}{"duration": 60.0}, nil, 1, 0, strat.val),
				newTask("B", "dep on A", task.ActionWait, "", map[string]interface{}{"duration": 1.0}, []string{"A"}, 3000, 0, strat.val),
				newTask("C", "dep on B", task.ActionWait, "", map[string]interface{}{"duration": 1.0}, []string{"B"}, 3000, 0, strat.val),
			}}
			dur := runPlan(&plan)

			sa, sb, sc := "", "", ""
			for _, t := range plan.Tasks {
				switch t.ID {
				case "A": sa = string(t.Status)
				case "B": sb = string(t.Status)
				case "C": sc = string(t.Status)
				}
			}

			records = append(records, []string{
				strconv.Itoa(i + 1), strat.name, sa, sb, sc,
				fmt.Sprintf("%.3f", dur.Seconds()),
			})
			fmt.Printf("  Run %d %s: A=%s B=%s C=%s (%.2fs)\n", i+1, strat.name, sa, sb, sc, dur.Seconds())
		}
	}
	return records
}

// ─── 实验①-B：DAG 循环依赖检测 ───
func dagDeadlock() [][]string {
	fmt.Println("\n### 实验①-B: DAG 循环依赖检测 ###")
	records := [][]string{}

	initSim()
	resetRobot("robot1", 0, 0, 0)
	time.Sleep(10 * time.Millisecond)

	plan := task.TaskPlan{Tasks: []*task.Task{
		newTask("A", "dep on C", task.ActionMove, "robot1", map[string]interface{}{"x": 0.5}, []string{"C"}, 3000, 0, task.SkipAndContinue),
		newTask("B", "dep on A", task.ActionMove, "robot1", map[string]interface{}{"x": 0.5}, []string{"A"}, 3000, 0, task.SkipAndContinue),
		newTask("C", "dep on B", task.ActionMove, "robot1", map[string]interface{}{"x": 0.5}, []string{"B"}, 3000, 0, task.SkipAndContinue),
	}}

	start := time.Now()
	err := sched.ExecutePlan(ctx, &plan, "exp-session")
	dur := time.Since(start)

	detected := "false"
	errMsg := ""
	if err != nil {
		errMsg = err.Error()
		if strings.Contains(errMsg, "deadlock") {
			detected = "true"
		}
	}

	records = append(records, []string{
		"1", "A→B→C→A", detected,
		fmt.Sprintf("%.3f", dur.Seconds()), errMsg,
	})
	fmt.Printf("  Cycle A→B→C→A: deadlock=%s (%.2fs) err=%s\n", detected, dur.Seconds(), errMsg)

	// Also test: normal DAG (no cycle) — should work
	initSim()
	resetRobot("robot1", 0, 0, 0)
	time.Sleep(10 * time.Millisecond)

	plan2 := task.TaskPlan{Tasks: []*task.Task{
		newTask("X", "no deps", task.ActionMove, "robot1", map[string]interface{}{"x": 0.5}, nil, 1, 0, task.SkipAndContinue),
		newTask("Y", "dep on X", task.ActionMove, "robot1", map[string]interface{}{"x": 0.5}, []string{"X"}, 1, 0, task.SkipAndContinue),
		newTask("Z", "dep on X", task.ActionMove, "robot1", map[string]interface{}{"x": 0.5}, []string{"X"}, 1, 0, task.SkipAndContinue),
	}}
	start = time.Now()
	err2 := sched.ExecutePlan(ctx, &plan2, "exp-session")
	dur2 := time.Since(start)

	detected2 := "false"
	errMsg2 := ""
	if err2 != nil {
		errMsg2 = err2.Error()
		if strings.Contains(errMsg2, "deadlock") {
			detected2 = "true"
		} else if strings.Contains(errMsg2, "no path") {
			detected2 = "no_path"
		}
	}

	records = append(records, []string{
		"2", "X→Y, X→Z (正常DAG)", detected2,
		fmt.Sprintf("%.3f", dur2.Seconds()), errMsg2,
	})
	fmt.Printf("  Normal DAG X→Y, X→Z: deadlock=%s (%.2fs) err=%v\n", detected2, dur2.Seconds(), err2)

	return records
}

// ─── 实验②：路径冲突三策略对比 ───
func pathConflict() [][]string {
	fmt.Println("\n### 实验②: 路径冲突三策略对比 ###")
	records := [][]string{}

	navPath := findNavGraph()
	g, err := navgraph.Load(navPath)
	if err != nil {
		fmt.Printf("  ERROR: %v\n", err)
		return records
	}
	fmt.Printf("  Graph: %d vertices, %d edges\n", len(g.Vertices), len(g.Adj))

	// Test routes: (patrol_D1 → coe) vs (coe → patrol_D1) — cross paths
	type route struct{ name string; from, to int }
	routes := []route{
		{"robot1: patrol_D1→coe", 2, 24},
		{"robot2: coe→patrol_D1", 24, 2},
		{"robot1: supplies→lounge", 8, 15},
		{"robot2: lounge→supplies", 15, 8},
		{"robot1: pantry→hardware_2", 4, 22},
		{"robot2: hardware_2→pantry", 22, 4},
	}

	// Verify vertices
	for _, rt := range routes {
		if rt.from >= len(g.Vertices) || rt.to >= len(g.Vertices) {
			fmt.Printf("  SKIP %s: invalid vertex (from=%d to=%d max=%d)\n", rt.name, rt.from, rt.to, len(g.Vertices))
		}
	}

	// Pair routes into conflict scenarios
	type scenario struct{ name string; r1, r2 route }
	scenarios := []scenario{
		{"对角交叉 (patrol_D1↔coe)", routes[0], routes[1]},
		{"对角交叉 (supplies↔lounge)", routes[2], routes[3]},
		{"对角交叉 (pantry↔hardware_2)", routes[4], routes[5]},
	}

	for _, sc := range scenarios {
		// Validate vertices
		maxV := len(g.Vertices)
		for _, v := range []int{sc.r1.from, sc.r1.to, sc.r2.from, sc.r2.to} {
			if v < 0 || v >= maxV {
				fmt.Printf("  SKIP %s: vertex %d out of range [0,%d)\n", sc.name, v, maxV)
				continue
			}
		}

		fmt.Printf("\n  Route: %s\n", sc.name)

		// ── Strategy 1: ShortestPath (direct) ──
		start := time.Now()
		p1, d1, err1 := g.ShortestPath(sc.r1.from, sc.r1.to)
		t1 := time.Since(start).Microseconds()
		start = time.Now()
		p2, d2, err2 := g.ShortestPath(sc.r2.from, sc.r2.to)
		t2 := time.Since(start).Microseconds()
		shared := countSharedVertices(p1, p2)
		totalDist := d1 + d2

		records = append(records, []string{
			"ShortestPath", sc.name,
			strconv.Itoa(len(p1)), strconv.Itoa(len(p2)),
			fmt.Sprintf("%.3f", d1), fmt.Sprintf("%.3f", d2), fmt.Sprintf("%.3f", totalDist),
			strconv.Itoa(shared),
			fmt.Sprintf("%d", t1), fmt.Sprintf("%d", t2),
			errStr(err1, err2),
		})
		fmt.Printf("    [Direct]    r1=%dv %.2fm  r2=%dv %.2fm  shared=%d  time=%d/%dμs\n",
			len(p1), d1, len(p2), d2, shared, t1, t2)

		// ── Strategy 2: ShortestPathAvoiding (exclude first/last, matching main.go avoidVertices) ──
		p2Middle := middleVertices(p2)
		start = time.Now()
		p1a, d1a, err1a := g.ShortestPathAvoiding(sc.r1.from, sc.r1.to, p2Middle)
		t1 = time.Since(start).Microseconds()
		p1aMiddle := middleVertices(p1a)
		start = time.Now()
		p2a, d2a, err2a := g.ShortestPathAvoiding(sc.r2.from, sc.r2.to, p1aMiddle)
		t2 = time.Since(start).Microseconds()
		sharedA := countSharedVertices(p1a, p2a)
		totalDistA := d1a + d2a

		records = append(records, []string{
			"ShortestPathAvoiding", sc.name,
			strconv.Itoa(len(p1a)), strconv.Itoa(len(p2a)),
			fmt.Sprintf("%.3f", d1a), fmt.Sprintf("%.3f", d2a), fmt.Sprintf("%.3f", totalDistA),
			strconv.Itoa(sharedA),
			fmt.Sprintf("%d", t1), fmt.Sprintf("%d", t2),
			errStr(err1a, err2a),
		})
		if err1a == nil && err2a == nil {
			fmt.Printf("    [Avoiding]  r1=%dv %.2fm  r2=%dv %.2fm  shared=%d  time=%d/%dμs\n",
				len(p1a), d1a, len(p2a), d2a, sharedA, t1, t2)
		} else {
			fmt.Printf("    [Avoiding]  ERROR: %v / %v\n", err1a, err2a)
		}

		// ── Strategy 3: ShortestPathMinimizeOverlap (exclude first/last) ──
		start = time.Now()
		p1m, d1m, err1m := g.ShortestPathMinimizeOverlap(sc.r1.from, sc.r1.to, p2Middle)
		t1 = time.Since(start).Microseconds()
		p1mMiddle := middleVertices(p1m)
		start = time.Now()
		p2m, d2m, err2m := g.ShortestPathMinimizeOverlap(sc.r2.from, sc.r2.to, p1mMiddle)
		t2 = time.Since(start).Microseconds()
		sharedM := countSharedVertices(p1m, p2m)
		totalDistM := d1m + d2m

		records = append(records, []string{
			"ShortestPathMinimizeOverlap", sc.name,
			strconv.Itoa(len(p1m)), strconv.Itoa(len(p2m)),
			fmt.Sprintf("%.3f", d1m), fmt.Sprintf("%.3f", d2m), fmt.Sprintf("%.3f", totalDistM),
			strconv.Itoa(sharedM),
			fmt.Sprintf("%d", t1), fmt.Sprintf("%d", t2),
			errStr(err1m, err2m),
		})
		if err1m == nil && err2m == nil {
			fmt.Printf("    [Minimize]  r1=%dv %.2fm  r2=%dv %.2fm  shared=%d  time=%d/%dμs\n",
				len(p1m), d1m, len(p2m), d2m, sharedM, t1, t2)
		} else {
			fmt.Printf("    [Minimize]  ERROR: %v / %v\n", err1m, err2m)
		}
	}
	return records
}

func findNavGraph() string {
	p := "/home/mofus/rmf_ws/install/rmf_demos_maps/share/rmf_demos_maps/maps/office/nav_graphs/0.yaml"
	if _, err := os.Stat(p); err == nil {
		return p
	}
	return ""
}

func countSharedVertices(a, b []int) int {
	s := make(map[int]bool)
	for _, v := range a {
		s[v] = true
	}
	c := 0
	for _, v := range b {
		if s[v] {
			c++
		}
	}
	return c
}

func middleVertices(path []int) []int {
	if len(path) <= 2 {
		return nil
	}
	r := make([]int, len(path)-2)
	copy(r, path[1:len(path)-1])
	return r
}

func errStr(errs ...error) string {
	for _, e := range errs {
		if e != nil {
			return e.Error()
		}
	}
	return ""
}

func main() {
	expDir := filepath.Join("experiments")

	// 实验①-A: DAG 策略对比
	r5 := dagStrategy(3)
	writeCSV(filepath.Join(expDir, "ablation_dag_strategy.csv"),
		"run,strategy,status_a,status_b,status_c,duration_sec", r5)

	// 实验①-B: DAG 死锁检测
	r6 := dagDeadlock()
	writeCSV(filepath.Join(expDir, "ablation_dag_deadlock.csv"),
		"run,cycle_description,deadlock_detected,duration_sec,error_message", r6)

	// 实验②: 路径冲突三策略对比
	r7 := pathConflict()
	writeCSV(filepath.Join(expDir, "ablation_path_conflict.csv"),
		"strategy,route_description,r1_vertices,r2_vertices,r1_distance_m,r2_distance_m,total_distance_m,shared_vertices,r1_compute_us,r2_compute_us,errors", r7)

	fmt.Println("\n=== All ablation experiments complete! ===")
}
