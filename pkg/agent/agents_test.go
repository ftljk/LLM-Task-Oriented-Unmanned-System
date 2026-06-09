package agent

import (
	"strings"
	"testing"

	"robot/pkg/task"
)

func TestToTaskPlan_SimpleMove(t *testing.T) {
	opt := PlanOption{
		Description: "forward 0.5m",
		Tasks: []SimpleTask{
			{Action: "Move", Target: "robot1", X: 0.5},
		},
	}
	plan := opt.ToTaskPlan()
	if len(plan.Tasks) != 1 {
		t.Fatalf("expected 1 task for Move without duration, got %d", len(plan.Tasks))
	}
	if plan.Tasks[0].Action != task.ActionMove {
		t.Errorf("expected ActionMove, got %s", plan.Tasks[0].Action)
	}
}

func TestToTaskPlan_MoveWithDuration_ExpandsToThree(t *testing.T) {
	opt := PlanOption{
		Description: "move with wait",
		Tasks: []SimpleTask{
			{Action: "Move", Target: "robot1", X: 0.5, Duration: 2.0},
		},
	}
	plan := opt.ToTaskPlan()
	if len(plan.Tasks) != 3 {
		t.Fatalf("expected 3 tasks (Move+Wait+Stop), got %d", len(plan.Tasks))
	}
	if plan.Tasks[0].Action != task.ActionMove {
		t.Errorf("task[0] expected Move, got %s", plan.Tasks[0].Action)
	}
	if plan.Tasks[1].Action != task.ActionWait {
		t.Errorf("task[1] expected Wait, got %s", plan.Tasks[1].Action)
	}
	if plan.Tasks[2].Action != task.ActionMove {
		t.Errorf("task[2] expected Move (stop), got %s", plan.Tasks[2].Action)
	}
}

func TestToTaskPlan_ExpansionDependencyForwarding(t *testing.T) {
	opt := PlanOption{
		Description: "rotate then move",
		Tasks: []SimpleTask{
			{Action: "Move", Target: "robot1", Z: -1.0, Duration: 1.57},
			{Action: "Move", Target: "robot1", X: 0.5, Dependencies: []string{"task-1"}},
		},
	}
	plan := opt.ToTaskPlan()
	if len(plan.Tasks) < 4 {
		t.Fatalf("expected 4+ tasks, got %d", len(plan.Tasks))
	}
	// task-1 expands to 3 tasks (ids: task-1, task-1-w, task-1-s)
	// task-2 should depend on task-1-s (the stop of the expanded sequence)
	lastIdx := len(plan.Tasks) - 1
	lastTask := plan.Tasks[lastIdx]
	if len(lastTask.Dependencies) == 0 {
		t.Fatal("expected last task to have dependencies after forwarding")
	}
	// The dependency should be the stop ID (task-1-s), not the original task-1
	if lastTask.Dependencies[0] == "task-1" {
		t.Errorf("dependency was not forwarded: still depends on task-1 instead of task-1-s")
	}
	if !strings.HasSuffix(lastTask.Dependencies[0], "-s") {
		t.Errorf("dependency should end with -s (stop task), got %s", lastTask.Dependencies[0])
	}
}

func TestToTaskPlan_WaitTask(t *testing.T) {
	opt := PlanOption{
		Description: "wait only",
		Tasks: []SimpleTask{
			{Action: "Wait", Duration: 5.0},
		},
	}
	plan := opt.ToTaskPlan()
	if len(plan.Tasks) != 1 {
		t.Fatalf("expected 1 task, got %d", len(plan.Tasks))
	}
	if plan.Tasks[0].Action != task.ActionWait {
		t.Errorf("expected ActionWait, got %s", plan.Tasks[0].Action)
	}
	dur, ok := plan.Tasks[0].Params["duration"]
	if !ok {
		t.Fatal("Wait task should have duration param")
	}
	if dur.(float64) != 5.0 {
		t.Errorf("expected duration 5.0, got %v", dur)
	}
}

func TestToTaskPlan_EmptyPlan(t *testing.T) {
	opt := PlanOption{
		Description: "empty",
		Tasks:       []SimpleTask{},
	}
	plan := opt.ToTaskPlan()
	if plan == nil {
		t.Fatal("plan should not be nil")
	}
	if len(plan.Tasks) != 0 {
		t.Errorf("expected empty task list, got %d", len(plan.Tasks))
	}
}

func TestToTaskPlan_PreservesStopActionParams(t *testing.T) {
	opt := PlanOption{
		Description: "move then stop cleanly",
		Tasks: []SimpleTask{
			{Action: "Move", Target: "robot1", X: 0.5, Duration: 3.0},
		},
	}
	plan := opt.ToTaskPlan()
	stopTask := plan.Tasks[2]
	if stopTask.Action != task.ActionMove {
		t.Errorf("stop task should be ActionMove, got %s", stopTask.Action)
	}
	if stopTask.Target != "robot1" {
		t.Errorf("stop task should preserve target, got %q", stopTask.Target)
	}
}

func TestToTaskPlan_MoveWaitPair_Expands(t *testing.T) {
	opt := PlanOption{
		Description: "move+wait pair becomes Move→Wait→Stop",
		Tasks: []SimpleTask{
			{Action: "Move", Target: "robot1", Z: -1.0},
			{Action: "Wait", Duration: 1.57},
		},
	}
	plan := opt.ToTaskPlan()
	if len(plan.Tasks) != 3 {
		t.Fatalf("expected 3 tasks (Move+Wait+Stop) from Move→Wait pair, got %d", len(plan.Tasks))
	}
	if plan.Tasks[0].Action != task.ActionMove {
		t.Errorf("task[0] expected Move, got %s", plan.Tasks[0].Action)
	}
	if plan.Tasks[1].Action != task.ActionWait {
		t.Errorf("task[1] expected Wait, got %s", plan.Tasks[1].Action)
	}
	dur, ok := plan.Tasks[1].Params["duration"]
	if !ok || dur.(float64) != 1.57 {
		t.Errorf("Wait duration should be 1.57, got %v", dur)
	}
	// Wait should depend on Move
	if len(plan.Tasks[1].Dependencies) == 0 {
		t.Fatal("Wait should have dependencies")
	}
	if plan.Tasks[1].Dependencies[0] != "task-1" {
		t.Errorf("Wait should depend on task-1, got %v", plan.Tasks[1].Dependencies)
	}
	if plan.Tasks[2].Action != task.ActionMove {
		t.Errorf("task[2] expected Move (stop), got %s", plan.Tasks[2].Action)
	}
}

func TestToTaskPlan_MoveWaitPair_NonConsecutive(t *testing.T) {
	// Move without following Wait should NOT be expanded
	opt := PlanOption{
		Description: "move only",
		Tasks: []SimpleTask{
			{Action: "Move", Target: "robot1", X: 0.5},
			{Action: "Move", Target: "robot2", X: 0.3},
		},
	}
	plan := opt.ToTaskPlan()
	if len(plan.Tasks) != 2 {
		t.Fatalf("expected 2 tasks for non-paired Move sequences, got %d", len(plan.Tasks))
	}
}

func TestToTaskPlan_MoveWaitPair_DependencyForwarding(t *testing.T) {
	opt := PlanOption{
		Description: "rotate then forward with pair expansion",
		Tasks: []SimpleTask{
			{Action: "Move", Target: "robot1", Z: -1.0},
			{Action: "Wait", Duration: 1.57},
			{Action: "Move", Target: "robot1", X: 0.5, Dependencies: []string{"task-1"}},
			{Action: "Wait", Duration: 1.0, Dependencies: []string{"task-2"}},
			{Action: "Move", Target: "robot1", Dependencies: []string{"task-3"}},
		},
	}
	plan := opt.ToTaskPlan()
	// task-1 → task-1-w → task-1-s, task-3 → task-3-w → task-3-s, task-5 as-is
	// Expected: 3 (expanded task-1) + 3 (expanded task-3) + 1 (task-5) = 7
	if len(plan.Tasks) != 7 {
		t.Fatalf("expected 7 tasks (3+3+1), got %d", len(plan.Tasks))
	}

	// Find task-3 and verify its dependency was forwarded to task-1-s
	var task3 *task.Task
	var task5 *task.Task
	for _, t := range plan.Tasks {
		if t.ID == "task-3" {
			task3 = t
		}
		if t.ID == "task-5" {
			task5 = t
		}
	}
	if task3 == nil {
		t.Fatal("task-3 should exist")
	}
	if len(task3.Dependencies) == 0 || task3.Dependencies[0] != "task-1-s" {
		t.Errorf("task-3 should depend on task-1-s (forwarded), got %v", task3.Dependencies)
	}
	if task5 == nil {
		t.Fatal("task-5 should exist")
	}
	if len(task5.Dependencies) == 0 || task5.Dependencies[0] != "task-3-s" {
		t.Errorf("task-5 should depend on task-3-s (forwarded), got %v", task5.Dependencies)
	}
}

func TestSimpleTaskActionPreservation(t *testing.T) {
	opt := PlanOption{
		Description: "collect data",
		Tasks: []SimpleTask{
			{Action: "CollectData", Target: "robot1"},
		},
	}
	plan := opt.ToTaskPlan()
	if len(plan.Tasks) != 1 {
		t.Fatalf("expected 1 task, got %d", len(plan.Tasks))
	}
	if string(plan.Tasks[0].Action) != "CollectData" {
		t.Errorf("expected CollectData action, got %s", plan.Tasks[0].Action)
	}
}
