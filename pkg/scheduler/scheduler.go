package scheduler

import (
	"context"
	"fmt"
	"robot/pkg/task"
	"robot/pkg/tools"
	"strings"
	"sync"
	"time"
)

type Scheduler struct {
}

func NewScheduler() *Scheduler {
	return &Scheduler{}
}

func (s *Scheduler) ExecutePlan(ctx context.Context, plan *task.TaskPlan) error {
	taskMap := make(map[string]*task.Task)
	for _, t := range plan.Tasks {
		t.Status = task.StatusPending
		taskMap[t.ID] = t
	}

	for {
		var pendingTasks []*task.Task
		allCompleted := true
		hasRunning := false
		hasFailed := false

		// Check status of all tasks
		for _, t := range plan.Tasks {
			if t.Status == task.StatusFailed {
				hasFailed = true
			}
			if t.Status == task.StatusRunning {
				hasRunning = true
			}
			if t.Status != task.StatusCompleted {
				allCompleted = false
			}

			if t.Status == task.StatusPending {
				canRun := true
				for _, depID := range t.Dependencies {
					depTask, exists := taskMap[depID]
					if !exists || depTask.Status != task.StatusCompleted {
						canRun = false
						break
					}
				}
				if canRun {
					pendingTasks = append(pendingTasks, t)
				}
			}
		}

		if hasFailed {
			return fmt.Errorf("plan execution failed")
		}

		if allCompleted {
			fmt.Println("All tasks completed successfully!")
			return nil
		}

		if len(pendingTasks) == 0 {
			if !hasRunning && !allCompleted {
				return fmt.Errorf("deadlock detected or dependency missing")
			}
			time.Sleep(100 * time.Millisecond)
			continue
		}

		var wg sync.WaitGroup
		for _, t := range pendingTasks {
			t.Status = task.StatusRunning
			wg.Add(1)
			go func(tsk *task.Task) {
				defer wg.Done()
				fmt.Printf("Starting task %s: %s\n", tsk.ID, tsk.Description)
				err := s.executeTask(ctx, tsk)
				if err != nil {
					fmt.Printf("Task %s failed: %v\n", tsk.ID, err)
					tsk.Status = task.StatusFailed
					tsk.Result = err.Error()
				} else {
					fmt.Printf("Task %s completed\n", tsk.ID)
					tsk.Status = task.StatusCompleted
					tsk.Result = "Success"
				}
			}(t)
		}
		wg.Wait()
	}
}

func (s *Scheduler) executeTask(ctx context.Context, t *task.Task) error {
	switch t.Action {
	case task.ActionMove:
		return s.executeMove(ctx, t)
	case task.ActionCollectData:
		return s.executeCollectData(ctx, t)
	case task.ActionWait:
		return s.executeWait(ctx, t)
	default:
		return fmt.Errorf("unknown action: %s", t.Action)
	}
}

func (s *Scheduler) executeMove(ctx context.Context, t *task.Task) error {
	topicType := "geometry_msgs/Twist"
	topics, err := tools.GetTopicsFunc(ctx, tools.TopicsForTypeRequest{Type: topicType})
	if err != nil {
		return err
	}

	var topicName string
	for _, topic := range topics {
		if strings.Contains(topic, t.Target) && strings.Contains(topic, "cmd_vel") {
			topicName = topic
			break
		}
	}
	
	if topicName == "" && len(topics) > 0 {
		// Just for testing/simulation if specific target not found, pick the first one if available
		// Or fail if you want strict checking
		// For now, let's try strict matching first, if not found, maybe retry or fail.
		// Given the environment, let's be strict but informative.
		return fmt.Errorf("cannot find topic for target %s in %v", t.Target, topics)
	}

	// 解析速度参数
	// linear_x, linear_y: 线性速度
	// angular_z: 角速度（用于旋转）
	linearX, _ := getFloat(t.Params, "x")
	linearY, _ := getFloat(t.Params, "y")
	angularZ, _ := getFloat(t.Params, "z")

	req := tools.VelRequest{
		Name: topicName,
		Msg: tools.Twist{
			Linear:  tools.Velocity{X: linearX, Y: linearY, Z: 0},
			Angular: tools.Velocity{X: 0, Y: 0, Z: angularZ},
		},
	}

	resp, err := tools.SetVelFunc(ctx, req)
	if err != nil {
		return err
	}
	fmt.Printf("[%s] Execution result: %s\n", t.ID, resp)
	return nil
}

func (s *Scheduler) executeCollectData(ctx context.Context, t *task.Task) error {
	topicType := "nav_msgs/Odometry"
	topics, err := tools.GetTopicsFunc(ctx, tools.TopicsForTypeRequest{Type: topicType})
	if err != nil {
		return err
	}

	var topicName string
	for _, topic := range topics {
		if strings.Contains(topic, t.Target) && strings.Contains(topic, "odom") {
			topicName = topic
			break
		}
	}
	
	if topicName == "" {
		return fmt.Errorf("cannot find topic for target %s in %v", t.Target, topics)
	}

	req := tools.PosResponse{Name: topicName}
	resp, err := tools.GetPosFunc(ctx, req)
	if err != nil {
		return err
	}
	fmt.Printf("[%s] Execution result: %s\n", t.ID, resp)
	return nil
}

// executeWait 执行等待任务，暂停指定的秒数
func (s *Scheduler) executeWait(ctx context.Context, t *task.Task) error {
	duration, ok := getFloat(t.Params, "duration")
	if !ok {
		duration = 1.0 // 默认等待1秒
	}
	
	waitTime := time.Duration(duration * float64(time.Second))
	fmt.Printf("[%s] Waiting for %.1f seconds...\n", t.ID, duration)
	
	select {
	case <-time.After(waitTime):
		fmt.Printf("[%s] Wait completed\n", t.ID)
		return nil
	case <-ctx.Done():
		return ctx.Err()
	}
}

func getFloat(m map[string]interface{}, key string) (float64, bool) {
	if m == nil {
		return 0, false
	}
	val, ok := m[key]
	if !ok {
		return 0, false
	}
	switch v := val.(type) {
	case float64:
		return v, true
	case int:
		return float64(v), true
	case int32:
		return float64(v), true
	case int64:
		return float64(v), true
	default:
		return 0, false
	}
}
