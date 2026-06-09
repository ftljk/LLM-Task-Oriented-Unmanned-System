package memory

import (
	"strings"
	"testing"
)

func TestRobotCapabilitiesString_Standard(t *testing.T) {
	s := RobotCapabilitiesString("robot1", RobotTypeStandard)
	if !strings.Contains(s, "标准") {
		t.Errorf("expected standard robot description, got: %s", s)
	}
	if !strings.Contains(s, "不可载货") {
		t.Errorf("standard robot should indicate no cargo, got: %s", s)
	}
}

func TestRobotCapabilitiesString_Delivery(t *testing.T) {
	s := RobotCapabilitiesString("robot2", RobotTypeDelivery)
	if !strings.Contains(s, "配送") {
		t.Errorf("expected delivery robot description, got: %s", s)
	}
	if !strings.Contains(s, "载货") {
		t.Errorf("delivery robot should mention cargo, got: %s", s)
	}
}

func TestRobotStateCapabilitiesMethod(t *testing.T) {
	state := &RobotState{
		Name: "robot1",
		Type: RobotTypeStandard,
	}
	caps := state.Capabilities()
	if !strings.Contains(caps, "标准") {
		t.Errorf("expected standard description, got: %s", caps)
	}

	state2 := &RobotState{
		Name: "robot2",
		Type: RobotTypeDelivery,
	}
	caps2 := state2.Capabilities()
	if !strings.Contains(caps2, "配送") {
		t.Errorf("expected delivery description, got: %s", caps2)
	}
}

func TestRobotTypeConstants(t *testing.T) {
	if RobotTypeStandard != "standard" {
		t.Errorf("RobotTypeStandard should be 'standard', got %q", RobotTypeStandard)
	}
	if RobotTypeDelivery != "delivery" {
		t.Errorf("RobotTypeDelivery should be 'delivery', got %q", RobotTypeDelivery)
	}
}
