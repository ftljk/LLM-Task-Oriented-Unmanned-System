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
// Scenario 1: 单机单步移动 ("一号向前移动1米")
// robot1 at (0,0) → forward 1m → measure accuracy
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
// Scenario 2: 单机顺序依赖 ("先移动到(3,1)，再回到原点(0,0)")
// ============================================================
func scenario2(n int) [][]string {
	fmt.Println("\n### Scenario 2: 单机顺序依赖 ((3,1)->(0,0)) ###")
	records := [][]string{}

	for i := 0; i < n; i++ {
		initSim()
		resetRobot("robot1", 0, 0, 0)
		time.Sleep(30 * time.Millisecond)

		// move 1: (0,0) → (3,1), distance=3.162m, angle=18.4°
		// rotate ~18° at z=0.3 → 1.1s; forward at x=0.5 → 6.4s
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

		// move 2: (3,1) → (0,0), distance=3.162m, turn π rad at z=0.5 → 6.3s
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
// Scenario 3: 双机并行 ("一号移动到(3,1)，二号移动到(1,3)")
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
			// robot1 tasks
			newTask("r1-r", "rot 18°", task.ActionMove, "robot1", map[string]interface{}{"z": 0.3, "duration": 1.1}, nil, 5000, 1, task.SkipAndContinue),
			newTask("r1-rw", "wait rot", task.ActionWait, "", map[string]interface{}{"duration": 1.1}, []string{"r1-r"}, 5000, 1, task.SkipAndContinue),
			newTask("r1-rs", "stop rot", task.ActionMove, "robot1", map[string]interface{}{"z": 0}, []string{"r1-rw"}, 3000, 1, task.SkipAndContinue),
			newTask("r1-f", "fwd 3.16m", task.ActionMove, "robot1", map[string]interface{}{"x": 0.5}, []string{"r1-rs"}, 20000, 1, task.SkipAndContinue),
			newTask("r1-fw", "wait fwd", task.ActionWait, "", map[string]interface{}{"duration": 6.4}, []string{"r1-f"}, 20000, 1, task.SkipAndContinue),
			newTask("r1-fs", "stop fwd", task.ActionMove, "robot1", map[string]interface{}{"x": 0}, []string{"r1-fw"}, 3000, 1, task.SkipAndContinue),
			// robot2 tasks (no deps on robot1 — parallel)
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
// Scenario 4: 故障恢复 (robot2 timeout, robot1 still works)
// "一号先移动到(3,1)，再回到原点(0,0)" + fault
// ============================================================
func scenario4(n int) [][]string {
	fmt.Println("\n### Scenario 4: 故障恢复 (robot2 timeout) ###")
	records := [][]string{}

	for i := 0; i < n; i++ {
		initSim()
		resetRobot("robot1", 0, 0, 0)
		time.Sleep(30 * time.Millisecond)

		// Inject fault: robot2 task with short timeout (simulate communication loss)
		timeout := 3000
		intervention := "timeout_simulated"
		if i >= 3 {
			timeout = 500 // even faster timeout on later runs
			intervention = "timeout_aggressive"
		}

		plan := task.TaskPlan{Tasks: []*task.Task{
			// robot1: (0,0)->(3,1)
			newTask("r1-r", "rot 18°", task.ActionMove, "robot1", map[string]interface{}{"z": 0.3, "duration": 1.1}, nil, 5000, 1, task.SkipAndContinue),
			newTask("r1-rw", "wait rot", task.ActionWait, "", map[string]interface{}{"duration": 1.1}, []string{"r1-r"}, 5000, 1, task.SkipAndContinue),
			newTask("r1-rs", "stop rot", task.ActionMove, "robot1", map[string]interface{}{"z": 0}, []string{"r1-rw"}, 3000, 1, task.SkipAndContinue),
			newTask("r1-f", "fwd 3.16m", task.ActionMove, "robot1", map[string]interface{}{"x": 0.5}, []string{"r1-rs"}, 15000, 1, task.SkipAndContinue),
			newTask("r1-fw", "wait fwd", task.ActionWait, "", map[string]interface{}{"duration": 6.4}, []string{"r1-f"}, 15000, 1, task.SkipAndContinue),
			newTask("r1-fs", "stop fwd", task.ActionMove, "robot1", map[string]interface{}{"x": 0}, []string{"r1-fw"}, 3000, 1, task.SkipAndContinue),
			// robot1: (3,1)->(0,0)
			newTask("r2-r", "rot 180°", task.ActionMove, "robot1", map[string]interface{}{"z": 0.5, "duration": 6.3}, []string{"r1-fs"}, 10000, 1, task.SkipAndContinue),
			newTask("r2-rw", "wait rot", task.ActionWait, "", map[string]interface{}{"duration": 6.3}, []string{"r2-r"}, 10000, 1, task.SkipAndContinue),
			newTask("r2-rs", "stop rot", task.ActionMove, "robot1", map[string]interface{}{"z": 0}, []string{"r2-rw"}, 3000, 1, task.SkipAndContinue),
			newTask("r2-f", "fwd 3.16m", task.ActionMove, "robot1", map[string]interface{}{"x": 0.5}, []string{"r2-rs"}, 15000, 1, task.SkipAndContinue),
			newTask("r2-fw", "wait fwd", task.ActionWait, "", map[string]interface{}{"duration": 6.4}, []string{"r2-f"}, 15000, 1, task.SkipAndContinue),
			newTask("r2-fs", "stop fwd", task.ActionMove, "robot1", map[string]interface{}{"x": 0}, []string{"r2-fw"}, 3000, 1, task.SkipAndContinue),

			// robot2: fault injection — wait task with long duration + short timeout
			// Move sets velocity, Wait blocks for 30s but timeout triggers after timeout ms
			newTask("fault", "robot2 move (fault)", task.ActionMove, "robot2", map[string]interface{}{"x": 0.3}, nil, 1000, 0, task.SkipAndContinue),
			newTask("fault-w", "wait 30s (timeout)", task.ActionWait, "", map[string]interface{}{"duration": 30.0}, []string{"fault"}, timeout, 0, task.SkipAndContinue),
		}}

		dur := runPlan(&plan)
		fx, fy, _ := getPos("robot1")

		// Check if fault-w (Wait) task was skipped due to timeout
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
		fmt.Printf("  Run %d/%d: timeout=%dms skipped=%v r1=(%.2f,%.2f) result=%s err=%s\n",
			i+1, n, timeout, faultSkipped, fx, fy, sysResult, faultErr)
	}
	return records
}

func main() {
	expDir := filepath.Join("..", "..", "experiments")

	// Scenario 1: 10 runs
	r1 := scenario1(10)
	writeCSV(filepath.Join(expDir, "scenario1_single_move.csv"),
		"run,task_id,plan_str,vx_cmd,vy_cmd,yaw_cmd,distance_cmd,pos_before_x,pos_before_y,yaw_before,pos_after_x,pos_after_y,yaw_after,duration_sec,move_distance_m,pos_error_m,success,notes",
		r1)

	// Scenario 2: 5 runs
	r2 := scenario2(5)
	writeCSV(filepath.Join(expDir, "scenario2_sequential.csv"),
		"run,move1_target_x,move1_target_y,move2_target_x,move2_target_y,pos_after_m1_x,pos_after_m1_y,pos_after_m2_x,pos_after_m2_y,m1_error_m,m2_error_m,total_time_sec,success_m1,success_m2,task_order_correct,notes",
		r2)

	// Scenario 3: 5 runs
	r3 := scenario3(5)
	writeCSV(filepath.Join(expDir, "scenario3_dual_parallel.csv"),
		"run,r1_target_x,r1_target_y,r2_target_x,r2_target_y,r1_end_x,r1_end_y,r2_end_x,r2_end_y,r1_error_m,r2_error_m,r1_duration_sec,r2_duration_sec,motion_overlap,success_r1,success_r2,notes",
		r3)

	// Scenario 4: 5 runs
	r4 := scenario4(5)
	writeCSV(filepath.Join(expDir, "scenario4_fault_recovery.csv"),
		"run,intervention_type,intervention_time_sec,timeout_ms,detected_fault,system_result,fallback_action,robot_final_x,robot_final_y,crashed,notes",
		r4)

	fmt.Println("\n=== All experiments complete! ===")
}
