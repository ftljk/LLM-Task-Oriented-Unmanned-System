package main

import (
	"context"
	"fmt"
	"time"

	"robot/pkg/memory"
	"robot/pkg/robot"
	"robot/pkg/scheduler"
	"robot/pkg/task"
)

func main() {
	fmt.Println("=== GoSim + Scheduler Standalone Test ===")
	fmt.Println()

	adapter := robot.NewGOSimRobotAdapter()
	defer adapter.Close()

	mem := memory.NewInMemorySessionManager()
	mem.CreateSession(nil, "test-session")

	sched := scheduler.NewScheduler(adapter, mem)

	ctx := context.Background()

	fmt.Println("Test 1: Simple sequential tasks")
	fmt.Println("  robot1 moves forward (x=0.5), waits 2s, then stops")
	plan1 := &task.TaskPlan{
		Tasks: []*task.Task{
			{
				ID: "task-1", Description: "robot1 forward", Action: task.ActionMove,
				Target: "robot1", Params: map[string]interface{}{"x": 0.5},
				Config: task.TaskConfig{TimeoutMs: 10000, MaxRetries: 1, OnFailure: task.SkipAndContinue},
			},
			{
				ID: "task-2", Description: "wait 2s", Action: task.ActionWait,
				Params: map[string]interface{}{"duration": 2.0},
				Dependencies: []string{"task-1"},
				Config: task.TaskConfig{TimeoutMs: 10000, MaxRetries: 1, OnFailure: task.SkipAndContinue},
			},
			{
				ID: "task-3", Description: "robot1 stop", Action: task.ActionMove,
				Target: "robot1", Params: map[string]interface{}{"x": 0, "z": 0},
				Dependencies: []string{"task-2"},
				Config: task.TaskConfig{TimeoutMs: 10000, MaxRetries: 1, OnFailure: task.SkipAndContinue},
			},
		},
	}
	err := sched.ExecutePlan(ctx, plan1, "test-session")
	fmt.Print(scheduler.FormatResultSummary(plan1))
	if err != nil {
		fmt.Printf("Test 1 error: %v\n", err)
	}

	printRobotPositions(adapter)

	fmt.Println("\nTest 2: Two independent robots moving simultaneously")
	fmt.Println("  robot1 rotates, robot2 moves forward (parallel)")
	plan2 := &task.TaskPlan{
		Tasks: []*task.Task{
			{
				ID: "task-1", Description: "robot1 rotate", Action: task.ActionMove,
				Target: "robot1", Params: map[string]interface{}{"z": 1.0},
				Config: task.TaskConfig{TimeoutMs: 10000, MaxRetries: 1, OnFailure: task.SkipAndContinue},
			},
			{
				ID: "task-2", Description: "robot2 forward", Action: task.ActionMove,
				Target: "robot2", Params: map[string]interface{}{"x": 0.3},
				Config: task.TaskConfig{TimeoutMs: 10000, MaxRetries: 1, OnFailure: task.SkipAndContinue},
			},
			{
				ID: "task-3", Description: "collect data", Action: task.ActionCollectData,
				Target: "robot2", Params: map[string]interface{}{"type": "position"},
				Dependencies: []string{"task-2"},
				Config: task.TaskConfig{TimeoutMs: 10000, MaxRetries: 1, OnFailure: task.SkipAndContinue},
			},
		},
	}
	err = sched.ExecutePlan(ctx, plan2, "test-session")
	fmt.Print(scheduler.FormatResultSummary(plan2))
	if err != nil {
		fmt.Printf("Test 2 error: %v\n", err)
	}

	printRobotPositions(adapter)

	fmt.Println("\nTest 3: Fault tolerance - task timeout")
	fmt.Println("  non-existent robot3 should timeout/fail gracefully")
	plan3 := &task.TaskPlan{
		Tasks: []*task.Task{
			{
				ID: "task-1", Description: "robot3 move", Action: task.ActionMove,
				Target: "robot3", Params: map[string]interface{}{"x": 1.0},
				Config: task.TaskConfig{TimeoutMs: 1000, MaxRetries: 1, OnFailure: task.SkipAndContinue},
			},
			{
				ID: "task-2", Description: "robot1 move", Action: task.ActionMove,
				Target: "robot1", Params: map[string]interface{}{"x": -0.5},
				Config: task.TaskConfig{TimeoutMs: 10000, MaxRetries: 1, OnFailure: task.SkipAndContinue},
			},
		},
	}
	err = sched.ExecutePlan(ctx, plan3, "test-session")
	fmt.Print(scheduler.FormatResultSummary(plan3))
	if err != nil {
		fmt.Printf("Test 3 error: %v\n", err)
	}

	printRobotPositions(adapter)

	fmt.Println("\n=== All tests completed ===")
}

func printRobotPositions(adapter robot.RobotAdapter) {
	fmt.Println("\nRobot positions after execution:")
	for _, name := range []string{"robot1", "robot2"} {
		odo, err := adapter.GetOdometry(context.Background(), name)
		if err != nil {
			fmt.Printf("  %s: error - %v\n", name, err)
		} else {
			fmt.Printf("  %s: (%.2f, %.2f, %.2f)\n", name, odo.X, odo.Y, odo.Theta)
		}
	}
	time.Sleep(100 * time.Millisecond)
}
