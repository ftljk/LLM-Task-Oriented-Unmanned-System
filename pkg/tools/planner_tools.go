package tools

import (
	"context"
	"robot/pkg/task"
)

var PlanCallback func(*task.TaskPlan)

type SubmitPlanRequest struct {
	Plan task.TaskPlan `json:"plan" jsonschema:"description=任务规划结果"`
}

func SubmitPlanFunc(ctx context.Context, input SubmitPlanRequest) (*task.TaskPlan, error) {
	if PlanCallback != nil {
		PlanCallback(&input.Plan)
	}
	return &input.Plan, nil
}
