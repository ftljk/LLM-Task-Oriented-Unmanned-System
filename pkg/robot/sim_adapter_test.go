package robot

import (
	"context"
	"testing"
	"time"
)

func TestGOSimRobotAdapter_SetVelocity(t *testing.T) {
	adapter := NewGOSimRobotAdapter()
	defer adapter.Close()

	err := adapter.SetVelocity(context.Background(), "robot1", 0.5, 0, 0)
	if err != nil {
		t.Fatalf("SetVelocity failed: %v", err)
	}
}

func TestGOSimRobotAdapter_GetOdometry(t *testing.T) {
	adapter := NewGOSimRobotAdapter()
	defer adapter.Close()

	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()

	odo, err := adapter.GetOdometry(ctx, "robot1")
	if err != nil {
		t.Fatalf("GetOdometry failed: %v", err)
	}
	if odo == nil {
		t.Fatal("GetOdometry returned nil")
	}
	if odo.X != 0 || odo.Y != 0 || odo.Theta != 0 {
		t.Errorf("expected initial (0,0,0), got (%.2f, %.2f, %.2f)", odo.X, odo.Y, odo.Theta)
	}
}

func TestGOSimRobotAdapter_GetLaserScan(t *testing.T) {
	adapter := NewGOSimRobotAdapter()
	defer adapter.Close()

	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()

	scan, err := adapter.GetLaserScan(ctx, "robot1")
	if err != nil {
		t.Fatalf("GetLaserScan failed: %v", err)
	}
	if scan == nil {
		t.Fatal("GetLaserScan returned nil")
	}
	if scan.Front != 8.0 {
		t.Errorf("expected front=8.0 (max range), got %.1f", scan.Front)
	}
}

func TestGOSimRobotAdapter_SimulateMovement(t *testing.T) {
	adapter := NewGOSimRobotAdapter()
	defer adapter.Close()

	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()

	// Set velocity forward
	err := adapter.SetVelocity(ctx, "robot1", 0.5, 0, 0)
	if err != nil {
		t.Fatalf("SetVelocity failed: %v", err)
	}

	// Wait for physics to step
	time.Sleep(300 * time.Millisecond)

	// Stop
	err = adapter.SetVelocity(ctx, "robot1", 0, 0, 0)
	if err != nil {
		t.Fatalf("SetVelocity(stop) failed: %v", err)
	}

	time.Sleep(100 * time.Millisecond)

	odo, err := adapter.GetOdometry(ctx, "robot1")
	if err != nil {
		t.Fatalf("GetOdometry failed: %v", err)
	}
	if odo.X <= 0 {
		t.Errorf("expected robot to move forward (x>0), got x=%.3f", odo.X)
	}
}

func TestGOSimRobotAdapter_SimulateRotation(t *testing.T) {
	adapter := NewGOSimRobotAdapter()
	defer adapter.Close()

	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()

	// Rotate clockwise (right turn, z negative = CW)
	err := adapter.SetVelocity(ctx, "robot1", 0, 0, -1.0)
	if err != nil {
		t.Fatalf("SetVelocity failed: %v", err)
	}

	time.Sleep(200 * time.Millisecond)

	err = adapter.SetVelocity(ctx, "robot1", 0, 0, 0)
	if err != nil {
		t.Fatalf("SetVelocity(stop) failed: %v", err)
	}

	time.Sleep(100 * time.Millisecond)

	odo, err := adapter.GetOdometry(ctx, "robot1")
	if err != nil {
		t.Fatalf("GetOdometry failed: %v", err)
	}
	// After CW rotation, theta should be negative
	if odo.Theta >= 0 {
		t.Errorf("expected negative theta after CW rotation, got %.3f", odo.Theta)
	}
}

func TestGOSimRobotAdapter_MultipleRobots(t *testing.T) {
	adapter := NewGOSimRobotAdapter()
	defer adapter.Close()

	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()

	// robot1 moves forward
	err := adapter.SetVelocity(ctx, "robot1", 0.5, 0, 0)
	if err != nil {
		t.Fatalf("SetVelocity robot1 failed: %v", err)
	}

	// robot2 moves backward
	err2 := adapter.SetVelocity(ctx, "robot2", -0.3, 0, 0)
	if err2 != nil {
		t.Fatalf("SetVelocity robot2 failed: %v", err2)
	}

	time.Sleep(300 * time.Millisecond)

	adapter.SetVelocity(ctx, "robot1", 0, 0, 0)
	adapter.SetVelocity(ctx, "robot2", 0, 0, 0)

	time.Sleep(100 * time.Millisecond)

	odo1, _ := adapter.GetOdometry(ctx, "robot1")
	odo2, _ := adapter.GetOdometry(ctx, "robot2")

	if odo1.X <= 0 {
		t.Errorf("robot1 should have moved forward (x>0), got x=%.3f", odo1.X)
	}
	if odo2.X >= 5.0 {
		t.Errorf("robot2 should have moved backward (x<5.0), got x=%.3f", odo2.X)
	}
}

func TestGOSimRobotAdapter_SetPosition(t *testing.T) {
	adapter := NewGOSimRobotAdapter()
	defer adapter.Close()

	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()

	err := adapter.SetPosition(ctx, "robot1", 5.0, 3.0, 1.57)
	if err != nil {
		t.Fatalf("SetPosition failed: %v", err)
	}

	odo, err := adapter.GetOdometry(ctx, "robot1")
	if err != nil {
		t.Fatalf("GetOdometry after SetPosition failed: %v", err)
	}
	if odo.X != 5.0 || odo.Y != 3.0 || odo.Theta != 1.57 {
		t.Errorf("expected (5.0, 3.0, 1.57), got (%.2f, %.2f, %.2f)", odo.X, odo.Y, odo.Theta)
	}
}

func TestGOSimRobotAdapter_ListTopics(t *testing.T) {
	adapter := NewGOSimRobotAdapter()
	defer adapter.Close()

	topics, err := adapter.ListTopics(context.Background(), "geometry_msgs/Twist")
	if err != nil {
		t.Fatalf("ListTopics failed: %v", err)
	}
	if len(topics) == 0 {
		t.Fatal("expected non-empty topic list")
	}
}

func TestGOSimRobotAdapter_HealthCheck(t *testing.T) {
	adapter := NewGOSimRobotAdapter()
	defer adapter.Close()

	err := adapter.HealthCheck(context.Background())
	if err != nil {
		t.Fatalf("HealthCheck failed: %v", err)
	}
}
