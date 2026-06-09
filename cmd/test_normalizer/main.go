package main

import (
	"fmt"
	"strings"

	"robot/pkg/normalizer"
	"robot/pkg/task"
)

func main() {
	norm := normalizer.NewNormalizer()

	testCases := []struct {
		name  string
		input string
		plan  *task.TaskPlan
	}{
		{
			name:  "robot aliases",
			input: "一号，逆时针旋转180度",
			plan: &task.TaskPlan{
				Tasks: []*task.Task{
					{ID: "task-1", Action: "Move", Target: "robot1", Params: map[string]interface{}{"z": 3.14}},
				},
			},
		},
		{
			name:  "rotation + movement",
			input: "一号，逆时针旋转180度，然后向前运动2米",
			plan: &task.TaskPlan{
				Tasks: []*task.Task{
					{ID: "task-1", Action: "Move", Target: "robot1", Params: map[string]interface{}{"z": 3.14}},
					{ID: "task-2", Action: "Move", Target: "robot1", Params: map[string]interface{}{"x": 0.5}, Dependencies: []string{"task-1"}},
				},
			},
		},
		{
			name:  "two robots",
			input: "一号前进3米，二号后退2米",
			plan: &task.TaskPlan{
				Tasks: []*task.Task{
					{ID: "task-1", Action: "Move", Target: "robot1", Params: map[string]interface{}{"x": 0.5}},
					{ID: "task-2", Action: "Move", Target: "robot2", Params: map[string]interface{}{"x": -0.5}},
				},
			},
		},
		{
			name:  "LLM already correct",
			input: "robot1以x=0.5前进2秒后停止",
			plan: &task.TaskPlan{
				Tasks: []*task.Task{
					{ID: "task-1", Action: "Move", Target: "robot1", Params: map[string]interface{}{"x": 0.5}},
					{ID: "task-2", Action: "Wait", Target: "", Params: map[string]interface{}{"duration": 2.0}, Dependencies: []string{"task-1"}},
					{ID: "task-3", Action: "Move", Target: "robot1", Params: map[string]interface{}{"x": 0.0}, Dependencies: []string{"task-2"}},
				},
			},
		},
		{
			name:  "2号机器人旋转90度",
			input: "2号机器人，顺时针旋转90度",
			plan: &task.TaskPlan{
				Tasks: []*task.Task{
					{ID: "task-1", Action: "Move", Target: "robot2", Params: map[string]interface{}{"z": 1.57}},
				},
			},
		},
	}

	for _, tc := range testCases {
		fmt.Println(strings.Repeat("=", 65))
		fmt.Printf("Test: %s\n", tc.name)
		fmt.Printf("Input: %q\n", tc.input)

		pp := norm.Preprocess(tc.input)
		fmt.Printf("Normalized: %q\n", pp.NormalizedInput)
		fmt.Printf("Robots: %v\n", pp.Robots)
		for _, h := range pp.Hints {
			switch h.Type {
			case normalizer.ActionRotate:
				fmt.Printf("  Hint: rotate %s %s %.0f°\n", h.Target, h.Direction, h.Value)
			case normalizer.ActionMoveForward:
				fmt.Printf("  Hint: move %s forward %.1fm\n", h.Target, h.Value)
			case normalizer.ActionMoveBackward:
				fmt.Printf("  Hint: move %s backward %.1fm\n", h.Target, h.Value)
			}
		}

		fmt.Println("\nLLM Output (before correction):")
		for _, t := range tc.plan.Tasks {
			deps := t.Dependencies
			if deps == nil {
				deps = []string{}
			}
			fmt.Printf("  %s: [%s] target=%s params=%v deps=%v\n", t.ID, t.Action, t.Target, t.Params, deps)
		}

		vr := norm.ValidateAndFix(tc.plan, pp)
		if vr.WasCorrected {
			fmt.Println("\n[!] Corrections applied:")
			for _, c := range vr.Corrections {
				fmt.Printf("  - %s\n", c)
			}
			fmt.Println("Corrected plan:")
			for _, t := range tc.plan.Tasks {
				deps := t.Dependencies
				if deps == nil {
					deps = []string{}
				}
				fmt.Printf("  %s: [%s] target=%s params=%v deps=%v\n", t.ID, t.Action, t.Target, t.Params, deps)
			}
		} else {
			fmt.Println("\n[✓] No corrections needed (LLM output already correct)")
		}
		fmt.Println()
	}
}
