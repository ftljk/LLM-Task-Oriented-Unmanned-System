package memory

import (
	"time"
)

type Role string

const (
	RoleUser      Role = "user"
	RoleAssistant Role = "assistant"
	RoleTool      Role = "tool"
	RoleSystem    Role = "system"
)

type Message struct {
	Role      Role      `json:"role"`
	Content   string    `json:"content"`
	CreatedAt time.Time `json:"created_at"`
}

type RobotType string

const (
	RobotTypeStandard  RobotType = "standard"
	RobotTypeDelivery  RobotType = "delivery"
)

type RobotState struct {
	Name      string             `json:"name"`
	Type      RobotType          `json:"type"`
	X         float64            `json:"x"`
	Y         float64            `json:"y"`
	Theta     float64            `json:"theta"`
	Status    string             `json:"status"`
	Laser     map[string]float64 `json:"laser,omitempty"`
	UpdatedAt time.Time          `json:"updated_at"`
}

func (r *RobotState) Capabilities() string {
	switch r.Type {
	case RobotTypeStandard:
		return "类型:标准巡逻机器人,最大速度:0.5m/s,可载货:否,适合任务:巡逻/运输/侦查"
	case RobotTypeDelivery:
		return "类型:配送机器人,最大速度:0.3m/s,可载货:是(载货量1kg),适合任务:配送/货物运输"
	default:
		return "类型:未知"
	}
}

func RobotCapabilitiesString(robotName string, robotType RobotType) string {
	switch robotType {
	case RobotTypeStandard:
		return "标准巡逻机器人,最高速度0.5m/s,不可载货,适合巡逻/运输"
	case RobotTypeDelivery:
		return "配送机器人,最高速度0.3m/s,可载货1kg,适合配送/货物运输"
	default:
		return "未知类型"
	}
}

type RobotStatus string

const (
	RobotIdle  RobotStatus = "idle"
	RobotBusy  RobotStatus = "busy"
	RobotError RobotStatus = "error"
)

type Session struct {
	ID          string                 `json:"id"`
	Messages    []Message              `json:"messages"`
	RobotStates map[string]*RobotState `json:"robot_states"`
	CreatedAt   time.Time              `json:"created_at"`
	UpdatedAt   time.Time              `json:"updated_at"`
}

type Memory interface {
	CreateSession(ctx interface{}, id string) (*Session, error)
	GetSession(ctx interface{}, id string) (*Session, bool)
	AddMessage(ctx interface{}, sessionID string, msg Message) error
	GetMessages(ctx interface{}, sessionID string) ([]Message, error)
	GetRecentMessages(ctx interface{}, sessionID string, n int) ([]Message, error)
	UpdateRobotState(ctx interface{}, sessionID string, state *RobotState) error
	GetRobotState(ctx interface{}, sessionID string, robotName string) (*RobotState, bool)
	Clear(ctx interface{}, sessionID string) error
	DeleteSession(ctx interface{}, id string) error
}
