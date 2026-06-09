package robot

import (
	"context"
	"fmt"
	"log"
	"math"
	"sync"
	"time"
)

type simRobot struct {
	name       string
	x, y, theta float64
	vx, vy, vz  float64
	lastUpdate  time.Time
}

type GOSimRobotAdapter struct {
	mu      sync.RWMutex
	robots  map[string]*simRobot
	stopCh  chan struct{}
}

func NewGOSimRobotAdapter() *GOSimRobotAdapter {
	adapter := &GOSimRobotAdapter{
		robots: make(map[string]*simRobot),
		stopCh: make(chan struct{}),
	}

	adapter.robots["robot1"] = &simRobot{
		name:      "robot1",
		x:         0, y: 0, theta: 0,
		lastUpdate: time.Now(),
	}
	adapter.robots["robot2"] = &simRobot{
		name:      "robot2",
		x:         5, y: 0, theta: 0,
		lastUpdate: time.Now(),
	}

	go adapter.physicsLoop()

	log.Println("[GoSim] Robot simulator started (robot1 at (0,0), robot2 at (5,0))")
	return adapter
}

func (a *GOSimRobotAdapter) Name() string {
	return "GoSimRobotAdapter"
}

func (a *GOSimRobotAdapter) physicsLoop() {
	ticker := time.NewTicker(100 * time.Millisecond)
	defer ticker.Stop()

	for {
		select {
		case <-ticker.C:
			a.updatePhysics()
		case <-a.stopCh:
			return
		}
	}
}

func (a *GOSimRobotAdapter) updatePhysics() {
	a.mu.Lock()
	defer a.mu.Unlock()

	now := time.Now()
	for _, r := range a.robots {
		dt := now.Sub(r.lastUpdate).Seconds()
		if dt <= 0 || dt > 1.0 {
			r.lastUpdate = now
			continue
		}

		r.theta += r.vz * dt
		avgTheta := r.theta - r.vz*dt*0.5
		r.x += (r.vx*math.Cos(avgTheta) - r.vy*math.Sin(avgTheta)) * dt
		r.y += (r.vx*math.Sin(avgTheta) + r.vy*math.Cos(avgTheta)) * dt
		r.lastUpdate = now
	}
}

func (a *GOSimRobotAdapter) SetVelocity(ctx context.Context, robotName string, linearX, linearY, angularZ float64) error {
	a.mu.Lock()
	defer a.mu.Unlock()

	r, ok := a.robots[robotName]
	if !ok {
		return fmt.Errorf("robot %s not found", robotName)
	}

	r.vx = linearX
	r.vy = linearY
	r.vz = angularZ
	log.Printf("[GoSim] %s: set velocity (vx=%.2f, vy=%.2f, vz=%.2f)", robotName, linearX, linearY, angularZ)
	return nil
}

func (a *GOSimRobotAdapter) GetOdometry(ctx context.Context, robotName string) (*Odometry, error) {
	a.mu.RLock()
	defer a.mu.RUnlock()

	r, ok := a.robots[robotName]
	if !ok {
		return nil, fmt.Errorf("robot %s not found", robotName)
	}

	return &Odometry{
		X:     r.x,
		Y:     r.y,
		Theta: r.theta,
	}, nil
}

func (a *GOSimRobotAdapter) GetLaserScan(ctx context.Context, robotName string) (*LaserScan, error) {
	// Go simulator has no obstacles — return max range in all directions
	a.mu.RLock()
	defer a.mu.RUnlock()

	if _, ok := a.robots[robotName]; !ok {
		return nil, fmt.Errorf("robot %s not found", robotName)
	}
	return &LaserScan{
		Front:      8.0,
		FrontLeft:  8.0,
		FrontRight: 8.0,
		Left:       8.0,
		Right:      8.0,
		BackLeft:   8.0,
		BackRight:  8.0,
		Back:       8.0,
	}, nil
}

func (a *GOSimRobotAdapter) ListTopics(ctx context.Context, topicType string) ([]string, error) {
	var topics []string
	switch topicType {
	case "geometry_msgs/Twist":
		topics = []string{"/model/robot1/cmd_vel", "/model/robot2/cmd_vel"}
	case "nav_msgs/Odometry":
		topics = []string{"/model/robot1/odometry", "/model/robot2/odometry"}
	case "sensor_msgs/LaserScan":
		topics = []string{"/robot1/scan", "/robot2/scan"}
	default:
		topics = []string{}
	}
	return topics, nil
}

func (a *GOSimRobotAdapter) HealthCheck(ctx context.Context) error {
	a.mu.RLock()
	defer a.mu.RUnlock()

	if len(a.robots) == 0 {
		return fmt.Errorf("no robots available")
	}
	return nil
}

func (a *GOSimRobotAdapter) Close() error {
	close(a.stopCh)
	log.Println("[GoSim] Simulator stopped")
	return nil
}

func (a *GOSimRobotAdapter) SetPosition(ctx context.Context, robotName string, x, y, theta float64) error {
	a.mu.Lock()
	defer a.mu.Unlock()

	r, ok := a.robots[robotName]
	if !ok {
		return fmt.Errorf("robot %s not found", robotName)
	}

	r.x = x
	r.y = y
	r.theta = theta
	r.lastUpdate = time.Now()
	log.Printf("[GoSim] %s: set position (%.2f, %.2f, %.2f)", robotName, x, y, theta)
	return nil
}

func (a *GOSimRobotAdapter) GetRobotPosition(robotName string) (float64, float64, float64, error) {
	odo, err := a.GetOdometry(context.Background(), robotName)
	if err != nil {
		return 0, 0, 0, err
	}
	return odo.X, odo.Y, odo.Theta, nil
}
