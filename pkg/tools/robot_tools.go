package tools

import (
	"context"
	"fmt"
	"robot/pkg/ros"

	"github.com/mitchellh/mapstructure"
)

type VelRequest struct {
	Name string   `json:"name" jsonschema:"required,description=ros2话题名称"`
	Msg  Twist    `json:"msg" jsonschema:"required,description=速度数据"`
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
	var msg map[string]interface{}
	decoder, err := mapstructure.NewDecoder(&mapstructure.DecoderConfig{
		TagName: "json",
		Result:  &msg,
	})
	if err != nil {
		return "创建decoder失败", err
	}
	err = decoder.Decode(input.Msg)
	if err != nil {
		return "转换map失败", err
	}
	err = ros.DefaultRosBridgeClient.Publish(input.Name, "geometry_msgs/Twist", msg)
	if err != nil {
		return "发布失败", err
	}
	return "发布成功", nil
}

type PosResponse struct {
	Name string `json:"name" jsonschema:"required,description=ros2话题名称"`
}

func GetPosFunc(ctx context.Context, input PosResponse) (string, error) {
	err := ros.DefaultRosBridgeClient.Subscribe(input.Name, func(msg map[string]interface{}) {
		fmt.Printf("Received odom data: %v\n", msg)
	})
	if err != nil {
		return "订阅失败", err
	}
	return "订阅成功", nil
}

type TopicsForTypeRequest struct {
	Type string `json:"type" jsonschema:"required,description=ros2话题类型"`
}

func GetTopicsFunc(ctx context.Context, input TopicsForTypeRequest) ([]string, error) {
	topics, err := ros.DefaultRosBridgeClient.GetTopicsForType(input.Type)
	if err != nil {
		return nil, err
	}
	return topics, nil
}
