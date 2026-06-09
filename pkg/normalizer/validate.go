package normalizer

import (
	"fmt"
	"math"
	"strings"

	"robot/pkg/task"
)

const (
	defaultLinearSpeed        = 0.5  // m/s
	defaultAngularSpeed       = 1.0  // rad/s
	defaultUnknownMoveDur     = 0.1  // seconds (safety net for unmatched moves)
	defaultPositionMoveDur    = 10.0 // seconds (for "移动到位置" with unknown distance)
)

func validateAndFix(plan *task.TaskPlan, pp *PreprocessResult) *ValidationResult {
	result := &ValidationResult{
		Plan:        plan,
		Corrections: make([]string, 0),
	}

	if plan == nil || len(plan.Tasks) == 0 {
		return result
	}

	// Track which tasks have already been expanded
	expanded := make(map[string]bool)

	// --- Pass 1: Hint-based expansion ---
	if len(pp.Hints) > 0 {
		hintIdx := 0
		for _, t := range plan.Tasks {
			if t.Action != task.ActionMove {
				continue
			}
			if hintIdx >= len(pp.Hints) {
				break
			}

			hint := pp.Hints[hintIdx]

			// Match by target (if hint has one)
			if hint.Target != "" && t.Target != hint.Target {
				continue
			}

			// Check if already has a Wait dependent (LLM already handled it)
			if hasWaitDependent(t.ID, plan.Tasks) {
				hintIdx++
				continue
			}

			if expanded[t.ID] {
				continue
			}
			expanded[t.ID] = true

			switch hint.Type {
			case ActionRotate:
				expandRotation(plan, t, &hint, result)
			case ActionMoveForward, ActionMoveBackward:
				expandMovement(plan, t, &hint, result)
			}
			hintIdx++
		}
	}

	// --- Pass 2: Safety net ---
	// Any remaining Move with non-zero velocity and no Wait+Stop gets expanded
	for _, t := range plan.Tasks {
		if t.Action != task.ActionMove {
			continue
		}
		if expanded[t.ID] {
			continue
		}
		if hasWaitDependent(t.ID, plan.Tasks) {
			continue
		}
		if !hasVelocity(t) {
			continue
		}

		// Determine which axis has velocity
		_, hasX := getFloat(t.Params, "x")
		_, hasZ := getFloat(t.Params, "z")

		// Create a synthetic hint and expand
		if hasX || hasZ {
			expanded[t.ID] = true
			expandUnmatched(plan, t, result)
		}
	}

	if len(result.Corrections) > 0 {
		result.WasCorrected = true
	}
	return result
}

func expandRotation(plan *task.TaskPlan, t *task.Task, hint *ActionHint, result *ValidationResult) {
	speed := defaultAngularSpeed
	if hint.Direction == "cw" {
		speed = -defaultAngularSpeed
	}

	radians := hint.Value * math.Pi / 180.0
	duration := radians / math.Abs(speed)

	t.Params = make(map[string]interface{})
	t.Params["z"] = speed

	waitID := t.ID + "-w"
	stopID := t.ID + "-s"

	updateDependencies(plan.Tasks, t.ID, stopID)

	waitTask := &task.Task{
		ID:           waitID,
		Description:  fmt.Sprintf("Wait for rotation (%.1f° = %.2fs)", hint.Value, duration),
		Action:       task.ActionWait,
		Target:       "",
		Params:       map[string]interface{}{"duration": math.Round(duration*100) / 100},
		Dependencies: []string{t.ID},
		Config:       task.DefaultTaskConfig(),
	}

	stopTask := &task.Task{
		ID:           stopID,
		Description:  fmt.Sprintf("Stop rotation for %s", t.Target),
		Action:       task.ActionMove,
		Target:       t.Target,
		Params:       map[string]interface{}{"z": float64(0)},
		Dependencies: []string{waitID},
		Config:       task.DefaultTaskConfig(),
	}

	plan.Tasks = append(plan.Tasks, waitTask, stopTask)

	result.Corrections = append(result.Corrections,
		fmt.Sprintf("%s: rotation %.0f° → Move(z=%.1f) + Wait(%.2fs) + Move(z=0)",
			t.ID, hint.Value, speed, duration))
}

func expandMovement(plan *task.TaskPlan, t *task.Task, hint *ActionHint, result *ValidationResult) {
	speed := defaultLinearSpeed
	if hint.Direction == "backward" {
		speed = -defaultLinearSpeed
	}

	var duration float64
	desc := ""

	if hint.Value > 0 {
		// Known distance
		duration = hint.Value / math.Abs(speed)
		desc = fmt.Sprintf("%.1fm = %.2fs", hint.Value, duration)
	} else {
		// Unknown distance (e.g., "移动到位置")
		duration = defaultPositionMoveDur
		desc = fmt.Sprintf("position move (default %.1fs)", duration)
	}

	t.Params = make(map[string]interface{})
	t.Params["x"] = speed

	waitID := t.ID + "-w"
	stopID := t.ID + "-s"

	updateDependencies(plan.Tasks, t.ID, stopID)

	waitTask := &task.Task{
		ID:           waitID,
		Description:  fmt.Sprintf("Wait for movement (%s)", desc),
		Action:       task.ActionWait,
		Target:       "",
		Params:       map[string]interface{}{"duration": math.Round(duration*100) / 100},
		Dependencies: []string{t.ID},
		Config:       task.DefaultTaskConfig(),
	}

	stopTask := &task.Task{
		ID:           stopID,
		Description:  fmt.Sprintf("Stop movement for %s", t.Target),
		Action:       task.ActionMove,
		Target:       t.Target,
		Params:       map[string]interface{}{"x": float64(0)},
		Dependencies: []string{waitID},
		Config:       task.DefaultTaskConfig(),
	}

	plan.Tasks = append(plan.Tasks, waitTask, stopTask)

	result.Corrections = append(result.Corrections,
		fmt.Sprintf("%s: movement (%.0fm → Move(x=%.1f) + Wait(%.1fs) + Move(x=0))",
			t.ID, hint.Value, speed, duration))
}

// expandUnmatched handles Move tasks that have no matching hint.
// Used as a safety net to prevent infinite movement.
func expandUnmatched(plan *task.TaskPlan, t *task.Task, result *ValidationResult) {
	x, hasX := getFloat(t.Params, "x")
	z, hasZ := getFloat(t.Params, "z")

	duration := defaultUnknownMoveDur
	speed := 0.0
	axis := ""

	if hasX {
		speed = x
		if speed == 0 {
			speed = defaultLinearSpeed
		}
		axis = "x"
	} else if hasZ {
		speed = z
		if speed == 0 {
			speed = defaultAngularSpeed
		}
		axis = "z"
	}

	// Keep the original param value for the speed
	// Don't modify t.Params since the LLM set it

	waitID := t.ID + "-w"
	stopID := t.ID + "-s"

	updateDependencies(plan.Tasks, t.ID, stopID)

	waitTask := &task.Task{
		ID:           waitID,
		Description:  fmt.Sprintf("Safety wait (%.1fs) for unmatched %s-move", duration, axis),
		Action:       task.ActionWait,
		Target:       "",
		Params:       map[string]interface{}{"duration": duration},
		Dependencies: []string{t.ID},
		Config:       task.DefaultTaskConfig(),
	}

	stopParams := make(map[string]interface{})
	if hasX {
		stopParams["x"] = float64(0)
	}
	if hasZ {
		stopParams["z"] = float64(0)
	}

	stopTask := &task.Task{
		ID:           stopID,
		Description:  fmt.Sprintf("Safety stop for %s (unmatched move)", t.Target),
		Action:       task.ActionMove,
		Target:       t.Target,
		Params:       stopParams,
		Dependencies: []string{waitID},
		Config:       task.DefaultTaskConfig(),
	}

	plan.Tasks = append(plan.Tasks, waitTask, stopTask)

	result.Corrections = append(result.Corrections,
		fmt.Sprintf("%s: unmatched move %s=%.1f → Wait(%.1fs) + Stop (safety net)",
			t.ID, axis, speed, duration))
}

func updateDependencies(tasks []*task.Task, oldID, newID string) {
	for _, t := range tasks {
		for i, dep := range t.Dependencies {
			if dep == oldID {
				t.Dependencies[i] = newID
			}
		}
	}
}

func hasWaitDependent(taskID string, tasks []*task.Task) bool {
	for _, t := range tasks {
		for _, dep := range t.Dependencies {
			if dep == taskID && t.Action == task.ActionWait {
				return true
			}
		}
	}
	return false
}

func hasVelocity(t *task.Task) bool {
	if t.Params == nil {
		return false
	}
	for _, key := range []string{"x", "y", "z"} {
		if v, ok := t.Params[key]; ok {
			switch val := v.(type) {
			case float64:
				if val != 0 {
					return true
				}
			case int:
				if val != 0 {
					return true
				}
			}
		}
	}
	return false
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

func formatCorrections(result *ValidationResult) string {
	if !result.WasCorrected {
		return ""
	}
	var b strings.Builder
	b.WriteString("[Normalizer] Corrections applied:\n")
	for _, c := range result.Corrections {
		b.WriteString(fmt.Sprintf("  • %s\n", c))
	}
	return b.String()
}
