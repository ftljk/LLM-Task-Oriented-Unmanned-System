package tools

import (
	"context"
	"fmt"

	"robot/pkg/robot"
)

var DefaultRobotAdapter robot.RobotAdapter

type VelRequest struct {
	Name string   `json:"name" required:"true" jsonschema:"required,description=ros2话题名称"`
	Msg  Twist    `json:"msg" required:"true" jsonschema:"required,description=速度数据"`
}

type Twist struct {
	Linear  Velocity `json:"linear"`
	Angular Velocity `json:"angular"`
}

type Velocity struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`
}

func SetVelFunc(ctx context.Context, input VelRequest) (string, error) {
	if DefaultRobotAdapter == nil {
		return "机器人适配器未初始化", fmt.Errorf("robot adapter not initialized")
	}

	robotName := extractRobotName(input.Name)

	err := DefaultRobotAdapter.SetVelocity(ctx, robotName, input.Msg.Linear.X, input.Msg.Linear.Y, input.Msg.Angular.Z)
	if err != nil {
		return fmt.Sprintf("设置速度失败: %v", err), err
	}
	return fmt.Sprintf("已成功设置 %s 的速度 (linear: %.2f, %.2f; angular: %.2f)",
		input.Name, input.Msg.Linear.X, input.Msg.Linear.Y, input.Msg.Angular.Z), nil
}

type PosRequest struct {
	Name string `json:"name" required:"true" jsonschema:"required,description=ros2话题名称"`
}

func GetPosFunc(ctx context.Context, input PosRequest) (string, error) {
	if DefaultRobotAdapter == nil {
		return "机器人适配器未初始化", fmt.Errorf("robot adapter not initialized")
	}

	robotName := extractRobotName(input.Name)
	odo, err := DefaultRobotAdapter.GetOdometry(ctx, robotName)
	if err != nil {
		return fmt.Sprintf("获取位置失败: %v", err), err
	}
	return fmt.Sprintf("%s 当前位置: x=%.3f, y=%.3f, theta=%.3f", input.Name, odo.X, odo.Y, odo.Theta), nil
}

type TopicsForTypeRequest struct {
	Type string `json:"type" required:"true" jsonschema:"required,description=ros2话题类型"`
}

func GetTopicsFunc(ctx context.Context, input TopicsForTypeRequest) ([]string, error) {
	if DefaultRobotAdapter == nil {
		return nil, fmt.Errorf("robot adapter not initialized")
	}
	return DefaultRobotAdapter.ListTopics(ctx, input.Type)
}

func extractRobotName(topicName string) string {
	if len(topicName) > 0 && topicName[0] == '/' {
		topicName = topicName[1:]
	}
	for i := 0; i < len(topicName); i++ {
		if topicName[i] == '/' {
			return topicName[:i]
		}
	}
	return topicName
}
