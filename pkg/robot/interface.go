package robot

import (
	"context"
)

type Odometry struct {
	X     float64
	Y     float64
	Theta float64
}

type LaserScan struct {
	Front      float64
	FrontLeft  float64
	FrontRight float64
	Left       float64
	Right      float64
	BackLeft   float64
	BackRight  float64
	Back       float64
}

type RobotAdapter interface {
	Name() string
	SetVelocity(ctx context.Context, robotName string, linearX, linearY, angularZ float64) error
	SetPosition(ctx context.Context, robotName string, x, y, theta float64) error
	GetOdometry(ctx context.Context, robotName string) (*Odometry, error)
	GetLaserScan(ctx context.Context, robotName string) (*LaserScan, error)
	ListTopics(ctx context.Context, topicType string) ([]string, error)
	HealthCheck(ctx context.Context) error
	Close() error
}
