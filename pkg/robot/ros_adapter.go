package robot

import (
	"context"
	"fmt"
	"log"
	"math"
	"sync"
	"time"

	"robot/pkg/ros"
)

type ROS2RobotAdapter struct {
	client  *ros.RosBridgeClient
	toolMu  sync.Mutex
}

func NewROS2RobotAdapter(wsURL string) (*ROS2RobotAdapter, error) {
	client, err := ros.NewRosBridgeClient(wsURL)
	if err != nil {
		return nil, fmt.Errorf("failed to connect to ROS bridge at %s: %w", wsURL, err)
	}
	log.Printf("[ROS2] Connected to ROS bridge at %s", wsURL)
	return &ROS2RobotAdapter{client: client}, nil
}

func (a *ROS2RobotAdapter) Name() string {
	return "ROS2RobotAdapter"
}

func (a *ROS2RobotAdapter) SetVelocity(ctx context.Context, robotName string, linearX, linearY, angularZ float64) error {
	topicName := fmt.Sprintf("/model/%s/cmd_vel", robotName)
	msg := map[string]interface{}{
		"linear":  map[string]interface{}{"x": linearX, "y": linearY, "z": 0.0},
		"angular": map[string]interface{}{"x": 0.0, "y": 0.0, "z": angularZ},
	}
	return a.client.Publish(topicName, "geometry_msgs/Twist", msg)
}

func quatToYaw(qx, qy, qz, qw float64) float64 {
	siny_cosp := 2.0 * (qw*qz + qx*qy)
	cosy_cosp := 1.0 - 2.0*(qy*qy+qz*qz)
	return math.Atan2(siny_cosp, cosy_cosp)
}

func (a *ROS2RobotAdapter) GetOdometry(ctx context.Context, robotName string) (*Odometry, error) {
	a.toolMu.Lock()
	defer a.toolMu.Unlock()

	topicName := fmt.Sprintf("/model/%s/odometry", robotName)

	ch := make(chan *Odometry, 1)
	err := a.client.Subscribe(topicName, func(msg map[string]interface{}) {
		// The bridge may send subscribe_response or subscribe_result;
		// only parse messages with pose data.
		rawMsg, ok := msg["msg"].(map[string]interface{})
		if !ok {
			return
		}
		pose, ok := rawMsg["pose"].(map[string]interface{})
		if !ok {
			return
		}
		innerPose, ok := pose["pose"].(map[string]interface{})
		if !ok {
			return
		}
		position, ok := innerPose["position"].(map[string]interface{})
		if !ok {
			return
		}
		orientation, ok := innerPose["orientation"].(map[string]interface{})
		if !ok {
			return
		}

		x, _ := position["x"].(float64)
		y, _ := position["y"].(float64)

		qx, _ := orientation["x"].(float64)
		qy, _ := orientation["y"].(float64)
		qz, _ := orientation["z"].(float64)
		qw, _ := orientation["w"].(float64)

		theta := quatToYaw(qx, qy, qz, qw)

		select {
		case ch <- &Odometry{X: x, Y: y, Theta: theta}:
		default:
		}
	})
	if err != nil {
		return nil, fmt.Errorf("failed to subscribe: %w", err)
	}

	select {
	case odo := <-ch:
		a.client.Unsubscribe(topicName)
		return odo, nil
	case <-time.After(7 * time.Second):
		a.client.Unsubscribe(topicName)
		return nil, fmt.Errorf("timeout waiting for odometry on %s", topicName)
	case <-ctx.Done():
		a.client.Unsubscribe(topicName)
		return nil, ctx.Err()
	}
}

func (a *ROS2RobotAdapter) GetLaserScan(ctx context.Context, robotName string) (*LaserScan, error) {
	a.toolMu.Lock()
	defer a.toolMu.Unlock()

	topicName := fmt.Sprintf("/%s/scan", robotName)

	ch := make(chan *LaserScan, 1)
	err := a.client.Subscribe(topicName, func(msg map[string]interface{}) {
		rawMsg, ok := msg["msg"].(map[string]interface{})
		if !ok {
			return
		}
		scanRaw, ok := rawMsg["scan"].(map[string]interface{})
		if !ok {
			return
		}
		scan := &LaserScan{}
		if v, ok := scanRaw["front"].(float64); ok {
			scan.Front = v
		}
		if v, ok := scanRaw["front_left"].(float64); ok {
			scan.FrontLeft = v
		}
		if v, ok := scanRaw["front_right"].(float64); ok {
			scan.FrontRight = v
		}
		if v, ok := scanRaw["left"].(float64); ok {
			scan.Left = v
		}
		if v, ok := scanRaw["right"].(float64); ok {
			scan.Right = v
		}
		if v, ok := scanRaw["back_left"].(float64); ok {
			scan.BackLeft = v
		}
		if v, ok := scanRaw["back_right"].(float64); ok {
			scan.BackRight = v
		}
		if v, ok := scanRaw["back"].(float64); ok {
			scan.Back = v
		}
		select {
		case ch <- scan:
		default:
		}
	})
	if err != nil {
		return nil, fmt.Errorf("failed to subscribe: %w", err)
	}

	select {
	case scan := <-ch:
		a.client.Unsubscribe(topicName)
		return scan, nil
	case <-time.After(7 * time.Second):
		a.client.Unsubscribe(topicName)
		return nil, fmt.Errorf("timeout waiting for laser scan on %s", topicName)
	case <-ctx.Done():
		a.client.Unsubscribe(topicName)
		return nil, ctx.Err()
	}
}

func (a *ROS2RobotAdapter) SetPosition(ctx context.Context, robotName string, x, y, theta float64) error {
	log.Printf("[ROS2] SetPosition(%s, %.2f, %.2f, %.2f) - not implemented via ROS", robotName, x, y, theta)
	return nil
}

func (a *ROS2RobotAdapter) ListTopics(ctx context.Context, topicType string) ([]string, error) {
	return a.client.GetTopicsForType(topicType)
}

func (a *ROS2RobotAdapter) HealthCheck(ctx context.Context) error {
	// Connection established at adapter creation; no additional check needed.
	return nil
}

func (a *ROS2RobotAdapter) Close() error {
	a.client.Close()
	log.Println("[ROS2] Disconnected from ROS bridge")
	return nil
}
