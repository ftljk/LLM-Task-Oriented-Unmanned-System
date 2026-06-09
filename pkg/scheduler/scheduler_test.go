package scheduler

import (
	"context"
	"testing"
	"time"

	"robot/pkg/memory"
	"robot/pkg/robot"
	"robot/pkg/task"
)

func newTestPlan(tasks ...*task.Task) *task.TaskPlan {
	return &task.TaskPlan{Tasks: tasks}
}

func TestScheduler_EmptyPlan(t *testing.T) {
	mem := memory.NewInMemorySessionManager()
	adapter := robot.NewGOSimRobotAdapter()
	defer adapter.Close()

	sched := NewScheduler(adapter, mem)
	err := sched.ExecutePlan(context.Background(), newTestPlan(), "test-session")
	if err != nil {
		t.Errorf("unexpected error: %v", err)
	}
}

func TestScheduler_SingleMoveTask(t *testing.T) {
	mem := memory.NewInMemorySessionManager()
	adapter := robot.NewGOSimRobotAdapter()
	defer adapter.Close()

	sched := NewScheduler(adapter, mem)
	plan := newTestPlan(&task.Task{
		ID:     "task-1",
		Action: task.ActionMove,
		Target: "robot1",
		Params: map[string]interface{}{"x": 0.3},
	})

	err := sched.ExecutePlan(context.Background(), plan, "test-session")
	if err != nil {
		t.Errorf("unexpected error: %v", err)
	}
}

func TestScheduler_WaitTask(t *testing.T) {
	mem := memory.NewInMemorySessionManager()
	adapter := robot.NewGOSimRobotAdapter()
	defer adapter.Close()

	sched := NewScheduler(adapter, mem)
	plan := newTestPlan(&task.Task{
		ID:     "task-1",
		Action: task.ActionWait,
		Params: map[string]interface{}{"duration": 0.1},
	})

	err := sched.ExecutePlan(context.Background(), plan, "test-session")
	if err != nil {
		t.Errorf("unexpected error: %v", err)
	}
}

func TestScheduler_SequentialTasks(t *testing.T) {
	mem := memory.NewInMemorySessionManager()
	adapter := robot.NewGOSimRobotAdapter()
	defer adapter.Close()

	sched := NewScheduler(adapter, mem)
	plan := newTestPlan(
		&task.Task{
			ID:     "task-1",
			Action: task.ActionMove,
			Target: "robot1",
			Params: map[string]interface{}{"x": 0.3},
		},
		&task.Task{
			ID:           "task-2",
			Action:       task.ActionMove,
			Target:       "robot1",
			Params:       map[string]interface{}{"x": 0.3},
			Dependencies: []string{"task-1"},
		},
	)

	err := sched.ExecutePlan(context.Background(), plan, "test-session")
	if err != nil {
		t.Errorf("unexpected error: %v", err)
	}
}

func TestScheduler_StopTask(t *testing.T) {
	mem := memory.NewInMemorySessionManager()
	adapter := robot.NewGOSimRobotAdapter()
	defer adapter.Close()

	sched := NewScheduler(adapter, mem)
	plan := newTestPlan(&task.Task{
		ID:     "task-1",
		Action: task.ActionMove,
		Target: "robot1",
		Params: map[string]interface{}{},
	})

	err := sched.ExecutePlan(context.Background(), plan, "test-session")
	if err != nil {
		t.Errorf("unexpected error: %v", err)
	}
}

func TestScheduler_ParallelWaits_ShortCompletesFirst(t *testing.T) {
	// Two parallel Wait tasks with different durations.
	// The short Wait's dependent (stop) should execute WITHOUT waiting
	// for the long Wait to complete (regression: previously wg.Wait() batched them).
	mem := memory.NewInMemorySessionManager()
	adapter := robot.NewGOSimRobotAdapter()
	defer adapter.Close()

	sched := NewScheduler(adapter, mem)
	plan := newTestPlan(
		&task.Task{
			ID:     "move-1",
			Action: task.ActionMove,
			Target: "robot1",
			Params: map[string]interface{}{"z": 1.0},
		},
		&task.Task{
			ID:           "wait-1",
			Action:       task.ActionWait,
			Params:       map[string]interface{}{"duration": 0.05},
			Dependencies: []string{"move-1"},
		},
		&task.Task{
			ID:           "stop-1",
			Action:       task.ActionMove,
			Target:       "robot1",
			Params:       map[string]interface{}{},
			Dependencies: []string{"wait-1"},
		},
		&task.Task{
			ID:     "move-2",
			Action: task.ActionMove,
			Target: "robot2",
			Params: map[string]interface{}{"x": 0.1},
		},
		&task.Task{
			ID:           "wait-2",
			Action:       task.ActionWait,
			Params:       map[string]interface{}{"duration": 1.0},
			Dependencies: []string{"move-2"},
		},
		&task.Task{
			ID:           "stop-2",
			Action:       task.ActionMove,
			Target:       "robot2",
			Params:       map[string]interface{}{},
			Dependencies: []string{"wait-2"},
		},
	)

	start := time.Now()
	err := sched.ExecutePlan(context.Background(), plan, "test-session")
	elapsed := time.Since(start)

	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}

	// Total time should be ~max(wait-1, wait-2) = 1.0s, not ~sum = 1.05s
	if elapsed > 1200*time.Millisecond {
		t.Errorf("stop-1 should execute early without waiting for wait-2, elapsed=%v", elapsed)
	}

	// stop-1 must complete before stop-2 in results
	stop1Idx, stop2Idx := -1, -1
	for i, r := range plan.Results {
		if r.TaskID == "stop-1" {
			stop1Idx = i
		}
		if r.TaskID == "stop-2" {
			stop2Idx = i
		}
	}
	if stop1Idx < 0 || stop2Idx < 0 {
		t.Fatal("stop-1 or stop-2 not found in results")
	}
	if stop1Idx >= stop2Idx {
		t.Errorf("stop-1 should complete before stop-2, got indices %d >= %d", stop1Idx, stop2Idx)
	}
}

func TestScheduler_MultipleRobots(t *testing.T) {
	mem := memory.NewInMemorySessionManager()
	adapter := robot.NewGOSimRobotAdapter()
	defer adapter.Close()

	sched := NewScheduler(adapter, mem)
	plan := newTestPlan(
		&task.Task{
			ID:     "task-1",
			Action: task.ActionMove,
			Target: "robot1",
			Params: map[string]interface{}{"x": 0.3},
		},
		&task.Task{
			ID:     "task-2",
			Action: task.ActionMove,
			Target: "robot2",
			Params: map[string]interface{}{"x": 0.3},
		},
	)

	err := sched.ExecutePlan(context.Background(), plan, "test-session")
	if err != nil {
		t.Errorf("unexpected error: %v", err)
	}
}
