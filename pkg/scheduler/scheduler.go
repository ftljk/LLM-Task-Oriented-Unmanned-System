package scheduler

import (
	"context"
	"fmt"
	"log"
	"math"
	"strings"
	"sync"
	"time"

	"robot/pkg/memory"
	"robot/pkg/robot"
	"robot/pkg/task"
)

type Scheduler struct {
	adapter robot.RobotAdapter
	mem     memory.Memory
	mu      sync.Mutex
}

func NewScheduler(adapter robot.RobotAdapter, mem memory.Memory) *Scheduler {
	return &Scheduler{
		adapter: adapter,
		mem:     mem,
	}
}

func (s *Scheduler) ExecutePlan(ctx context.Context, plan *task.TaskPlan, sessionID string) error {
	taskMap := make(map[string]*task.Task)
	for _, t := range plan.Tasks {
		t.Status = task.StatusPending
		if t.Config == (task.TaskConfig{}) {
			t.Config = task.DefaultTaskConfig()
		}
		taskMap[t.ID] = t
	}

	plan.Results = make([]task.ExecutionResult, 0)

	doneCh := make(chan *task.Task, len(plan.Tasks))
	running := 0

	for {
		// Non-blocking drain: collect all recently completed tasks
		for running > 0 {
			select {
			case <-doneCh:
				running--
			default:
				goto schedule
			}
		}

	schedule:
		var readyTasks []*task.Task
		allDone := true

		for _, t := range plan.Tasks {
			switch t.Status {
			case task.StatusCompleted, task.StatusFailed, task.StatusSkipped:
				continue
			case task.StatusRunning:
				allDone = false
				continue
			}

			allDone = false

			if t.Status == task.StatusPending {
				canRun := true
				for _, depID := range t.Dependencies {
					dep, ok := taskMap[depID]
					if !ok {
						canRun = false
						break
					}
					if dep.Status == task.StatusFailed || dep.Status == task.StatusSkipped {
						t.Status = task.StatusSkipped
						t.Result = fmt.Sprintf("skipped: dependency %s %s", depID, dep.Status)
						s.appendResult(plan, task.ExecutionResult{
							TaskID: t.ID, Description: t.Description,
							Action: t.Action, Target: t.Target,
							Status: task.StatusSkipped, SkipReason: t.Result,
						})
						canRun = false
						break
					}
					if dep.Status != task.StatusCompleted {
						canRun = false
						break
					}
				}
				if canRun && t.Status == task.StatusPending {
					readyTasks = append(readyTasks, t)
				}
			}
		}

		if allDone {
			log.Println("[Scheduler] All tasks completed")
			return nil
		}

		if len(readyTasks) == 0 {
			if running == 0 {
				return fmt.Errorf("deadlock or unresolvable dependencies")
			}
			// Wait for at least one running task to complete
			<-doneCh
			running--
			continue
		}

		for _, t := range readyTasks {
			t.Status = task.StatusRunning
			running++
			tsk := t
			go func() {
				s.executeTaskWithRetry(ctx, tsk, sessionID, plan)
				doneCh <- tsk
			}()
		}
	}
}

func (s *Scheduler) appendResult(plan *task.TaskPlan, r task.ExecutionResult) {
	s.mu.Lock()
	defer s.mu.Unlock()
	plan.Results = append(plan.Results, r)
}

func (s *Scheduler) executeTaskWithRetry(ctx context.Context, t *task.Task, sessionID string, plan *task.TaskPlan) {
	log.Printf("[Scheduler] Starting %s: %s (target=%s)", t.ID, t.Description, t.Target)

	var lastErr error
	attempts := 0
	maxAttempts := t.Config.MaxRetries
	if maxAttempts < 1 {
		maxAttempts = 1
	}

	for attempt := 1; attempt <= maxAttempts; attempt++ {
		attempts = attempt

		taskCtx, cancel := context.WithTimeout(ctx, time.Duration(t.Config.TimeoutMs)*time.Millisecond)
		err := s.executeSingleTask(taskCtx, t, sessionID)
		cancel()

		if err == nil {
			t.Status = task.StatusCompleted
			t.Result = "Success"
			log.Printf("[Scheduler] %s completed (attempt %d/%d)", t.ID, attempt, maxAttempts)
			s.appendResult(plan, task.ExecutionResult{
				TaskID: t.ID, Description: t.Description,
				Action: t.Action, Target: t.Target,
				Status: task.StatusCompleted, Attempts: attempt,
			})
			return
		}

		lastErr = err
		log.Printf("[Scheduler] %s attempt %d/%d failed: %v", t.ID, attempt, maxAttempts, err)

		if attempt < maxAttempts {
			delay := time.Duration(t.Config.RetryDelayMs) * time.Millisecond
			log.Printf("[Scheduler] %s retrying after %v...", t.ID, delay)
			select {
			case <-time.After(delay):
			case <-ctx.Done():
				t.Status = task.StatusFailed
				t.Result = "cancelled"
				s.appendResult(plan, task.ExecutionResult{
					TaskID: t.ID, Description: t.Description,
					Action: t.Action, Target: t.Target,
					Status: task.StatusFailed, Error: "cancelled",
				})
				return
			}
		}
	}

	switch t.Config.OnFailure {
	case task.SkipAndContinue:
		t.Status = task.StatusSkipped
		t.Result = fmt.Sprintf("skipped after %d failed attempts: %v", attempts, lastErr)
		log.Printf("[Scheduler] %s skipped: %v", t.ID, lastErr)
		s.appendResult(plan, task.ExecutionResult{
			TaskID: t.ID, Description: t.Description,
			Action: t.Action, Target: t.Target,
			Status: task.StatusSkipped, Attempts: attempts,
			Error: lastErr.Error(), SkipReason: "max retries exceeded",
		})

	default:
		t.Status = task.StatusFailed
		t.Result = fmt.Sprintf("failed after %d attempts: %v", attempts, lastErr)
		log.Printf("[Scheduler] %s failed: %v", t.ID, lastErr)
		s.appendResult(plan, task.ExecutionResult{
			TaskID: t.ID, Description: t.Description,
			Action: t.Action, Target: t.Target,
			Status: task.StatusFailed, Attempts: attempts,
			Error: lastErr.Error(),
		})
	}
}

func (s *Scheduler) executeSingleTask(ctx context.Context, t *task.Task, sessionID string) error {
	switch t.Action {
	case task.ActionMove:
		return s.execMove(ctx, t, sessionID)
	case task.ActionCollectData:
		return s.execCollectData(ctx, t, sessionID)
	case task.ActionWait:
		return s.execWait(ctx, t)
	default:
		return fmt.Errorf("unknown action: %s", t.Action)
	}
}

func (s *Scheduler) execMove(ctx context.Context, t *task.Task, sessionID string) error {
	linearX, _ := getFloat(t.Params, "x")
	linearY, _ := getFloat(t.Params, "y")
	angularZ, _ := getFloat(t.Params, "z")

	err := s.adapter.SetVelocity(ctx, t.Target, linearX, linearY, angularZ)
	if err != nil {
		return fmt.Errorf("set_velocity failed: %w", err)
	}

	s.updateRobotState(sessionID, t.Target)
	return nil
}

func (s *Scheduler) execCollectData(ctx context.Context, t *task.Task, sessionID string) error {
	odo, err := s.adapter.GetOdometry(ctx, t.Target)
	if err != nil {
		return fmt.Errorf("get_odometry failed: %w", err)
	}

	result := fmt.Sprintf("%s position: x=%.3f, y=%.3f, theta=%.3f", t.Target, odo.X, odo.Y, odo.Theta)
	t.Result = result

	if sessionID != "" {
		s.mem.AddMessage(ctx, sessionID, memory.Message{
			Role:    memory.RoleTool,
			Content: fmt.Sprintf("Data collection result: %s", result),
		})
	}

	s.updateRobotState(sessionID, t.Target)
	return nil
}

func (s *Scheduler) execWait(ctx context.Context, t *task.Task) error {
	duration, ok := getFloat(t.Params, "duration")
	if !ok || duration <= 0 {
		duration = 1.0
	}

	waitDur := time.Duration(duration * float64(time.Second))

	targetX, hasTargetX := getFloat(t.Params, "target_x")
	targetY, hasTargetY := getFloat(t.Params, "target_y")
	hasTarget := hasTargetX && hasTargetY
	targetTheta, hasTargetTheta := getFloat(t.Params, "target_theta")
	hasClosedLoop := hasTarget || hasTargetTheta

	log.Printf("[Scheduler] %s waiting %.1fs (laser guard active)", t.ID, duration)

	// Pure wait (no robot, no closed-loop): just sleep
	if t.Target == "" && !hasClosedLoop {
		select {
		case <-time.After(waitDur):
			return nil
		case <-ctx.Done():
			return ctx.Err()
		}
	}

	// Pre-flight laser check: wait briefly for path to clear before forward movement
	movingForward := false
	if fv, ok := getFloat(t.Params, "x"); ok && fv > 0 {
		movingForward = true
	}
	if hasTarget {
		movingForward = true
	}
	preflightElapsed := time.Duration(0)
	if movingForward && t.Target != "" {
		preflightStart := time.Now()
		preflightOK := false
		preflightMax := waitDur
		if preflightMax > 5*time.Second {
			preflightMax = 5 * time.Second
		}
		for time.Since(preflightStart) < preflightMax {
			scan, err := s.adapter.GetLaserScan(ctx, t.Target)
			if err == nil && scan.Front >= 0.5 && scan.FrontLeft >= 0.4 && scan.FrontRight >= 0.4 {
				preflightOK = true
				break
			}
			select {
			case <-time.After(100 * time.Millisecond):
			case <-ctx.Done():
				return ctx.Err()
			}
		}
		preflightElapsed = time.Since(preflightStart)
		if !preflightOK {
			log.Printf("[Scheduler] %s path still blocked after pre-flight (%.1fs)", t.ID, preflightElapsed.Seconds())
			_ = s.adapter.SetVelocity(ctx, t.Target, 0, 0, 0)
		}
	}

	// Laser guard: poll every 50ms, emergency stop if obstacle too close
	ticker := time.NewTicker(50 * time.Millisecond)
	defer ticker.Stop()

	remaining := waitDur - preflightElapsed
	if remaining < 100*time.Millisecond {
		remaining = 100 * time.Millisecond
	}
	rotateEnd := time.Now().Add(waitDur * 3)   // rotation extended timeout: 3x
	forwardEnd := time.Now().Add(remaining)     // forward: stop at remaining time

	emergencyStop := func(reason string) error {
		log.Printf("[Scheduler] %s EMERGENCY STOP: %s", t.ID, reason)
		_ = s.adapter.SetVelocity(ctx, t.Target, 0, 0, 0)
		return fmt.Errorf("emergency stop: %s", reason)
	}

	var prevDiff float64
	hasPrevDiff := false

	// Forward heading hold: capture settings for straight-line correction
	var holdForwardVel float64
	var holdStartHeading float64
	hasHold := false
	if fv, ok := getFloat(t.Params, "x"); ok && fv > 0 {
		holdForwardVel = fv
		if odo, err := s.adapter.GetOdometry(ctx, t.Target); err == nil {
			holdStartHeading = odo.Theta
			hasHold = true
		}
	}

	// Auto-compute target_theta from z+duration when LLM omitted it
	if !hasTargetTheta {
		if rotZ, ok := getFloat(t.Params, "z"); ok && rotZ != 0 && duration > 0 {
			if odo, err := s.adapter.GetOdometry(ctx, t.Target); err == nil {
				targetTheta = odo.Theta + rotZ*duration
				hasTargetTheta = true
				// recompute hasClosedLoop
				hasClosedLoop = hasTarget || hasTargetTheta
				log.Printf("[Scheduler] %s auto-computed target_theta=%.3f (z=%.2f * %.1fs)", t.ID, targetTheta, rotZ, duration)
			}
		}
	}

	for {
		select {
		case <-ticker.C:
			now := time.Now()

			if odo, err := s.adapter.GetOdometry(ctx, t.Target); err == nil {
				// Closed-loop rotation check: stop when within 0.05 rad of target angle
				if hasTargetTheta {
					diff := targetTheta - odo.Theta
					for diff > math.Pi {
						diff -= 2 * math.Pi
					}
					for diff < -math.Pi {
						diff += 2 * math.Pi
					}
					// Catch entering the window OR passing through between samples
					if math.Abs(diff) < 0.05 || (hasPrevDiff && prevDiff*diff < 0) {
						log.Printf("[Scheduler] %s reached target angle %.2f, stopping rotation early", t.ID, targetTheta)
						_ = s.adapter.SetVelocity(ctx, t.Target, 0, 0, 0)
						return nil
					}
					prevDiff = diff
					hasPrevDiff = true
				}

				// Forward heading hold: keep robot straight, prevent drift into walls
				if hasHold {
					hdiff := holdStartHeading - odo.Theta
					for hdiff > math.Pi {
						hdiff -= 2 * math.Pi
					}
					for hdiff < -math.Pi {
						hdiff += 2 * math.Pi
					}
					if math.Abs(hdiff) > 0.04 {
						correction := hdiff * 2.0
						if correction > 0.3 {
							correction = 0.3
						} else if correction < -0.3 {
							correction = -0.3
						}
						_ = s.adapter.SetVelocity(ctx, t.Target, holdForwardVel, 0, correction)
					}
				}

				// Closed-loop position check: stop early if within 0.3m of target
				if hasTarget {
					dx := targetX - odo.X
					dy := targetY - odo.Y
					if math.Sqrt(dx*dx+dy*dy) < 0.3 {
						log.Printf("[Scheduler] %s arrived at target (%.2f,%.2f), stopping early", t.ID, targetX, targetY)
						_ = s.adapter.SetVelocity(ctx, t.Target, 0, 0, 0)
						return nil
					}
				}

			}

			// Timer logic:
			// - Pure wait (no closed-loop): stop at expected duration
			// - Rotation (hasTargetTheta): extend to 3x for target to be reached
			// - Forward (hasTarget only): stop at expected duration, let next rotation correct
			if !hasClosedLoop {
				if now.After(forwardEnd) {
					return nil
				}
			} else if hasTargetTheta {
				if now.After(rotateEnd) {
					return nil
				}
			} else {
				if now.After(forwardEnd) {
					return nil
				}
			}

			scan, err := s.adapter.GetLaserScan(ctx, t.Target)
			if err != nil {
				continue
			}
			if scan.Front < 0.3 {
				return emergencyStop(fmt.Sprintf("front obstacle at %.2fm", scan.Front))
			}
			if scan.FrontLeft < 0.25 {
				return emergencyStop(fmt.Sprintf("front-left obstacle at %.2fm", scan.FrontLeft))
			}
			if scan.FrontRight < 0.25 {
				return emergencyStop(fmt.Sprintf("front-right obstacle at %.2fm", scan.FrontRight))
			}

		case <-ctx.Done():
			_ = s.adapter.SetVelocity(ctx, t.Target, 0, 0, 0)
			return ctx.Err()
		}
	}
}

func (s *Scheduler) updateRobotState(sessionID, robotName string) {
	if sessionID == "" {
		return
	}

	odo, err := s.adapter.GetOdometry(context.Background(), robotName)
	if err != nil {
		return
	}

	s.mem.UpdateRobotState(nil, sessionID, &memory.RobotState{
		Name:   robotName,
		X:      odo.X,
		Y:      odo.Y,
		Theta:  odo.Theta,
		Status: string(memory.RobotIdle),
	})
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

func FormatResultSummary(plan *task.TaskPlan) string {
	var b strings.Builder
	b.WriteString("\n========== Execution Summary ==========\n")
	success, failed, skipped := 0, 0, 0
	for _, r := range plan.Results {
		switch r.Status {
		case task.StatusCompleted:
			success++
		case task.StatusFailed:
			failed++
		case task.StatusSkipped:
			skipped++
		}
		b.WriteString(fmt.Sprintf("  [%s] %s -> %s", r.TaskID, r.Description, r.Status))
		if r.Error != "" {
			b.WriteString(fmt.Sprintf(" (error: %s)", r.Error))
		}
		if r.SkipReason != "" {
			b.WriteString(fmt.Sprintf(" (reason: %s)", r.SkipReason))
		}
		if r.Attempts > 0 {
			b.WriteString(fmt.Sprintf(" [attempts: %d]", r.Attempts))
		}
		b.WriteString("\n")
	}
	b.WriteString("-------------------------------------\n")
	b.WriteString(fmt.Sprintf("Total: %d | Success: %d | Failed: %d | Skipped: %d\n",
		len(plan.Results), success, failed, skipped))
	b.WriteString("=========================================\n")
	return b.String()
}
