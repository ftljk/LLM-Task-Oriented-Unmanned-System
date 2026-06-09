package memory

import (
	"fmt"
	"sync"
	"time"
)

type InMemorySessionManager struct {
	mu       sync.RWMutex
	sessions map[string]*Session
}

func NewInMemorySessionManager() *InMemorySessionManager {
	return &InMemorySessionManager{
		sessions: make(map[string]*Session),
	}
}

func (m *InMemorySessionManager) CreateSession(ctx interface{}, id string) (*Session, error) {
	m.mu.Lock()
	defer m.mu.Unlock()

	if _, exists := m.sessions[id]; exists {
		return nil, fmt.Errorf("session %s already exists", id)
	}

	now := time.Now()
	session := &Session{
		ID:          id,
		Messages:    make([]Message, 0),
		RobotStates: make(map[string]*RobotState),
		CreatedAt:   now,
		UpdatedAt:   now,
	}
	m.sessions[id] = session
	return session, nil
}

func (m *InMemorySessionManager) GetSession(ctx interface{}, id string) (*Session, bool) {
	m.mu.RLock()
	defer m.mu.RUnlock()

	session, ok := m.sessions[id]
	return session, ok
}

func (m *InMemorySessionManager) AddMessage(ctx interface{}, sessionID string, msg Message) error {
	m.mu.Lock()
	defer m.mu.Unlock()

	session, ok := m.sessions[sessionID]
	if !ok {
		return fmt.Errorf("session %s not found", sessionID)
	}

	msg.CreatedAt = time.Now()
	session.Messages = append(session.Messages, msg)
	session.UpdatedAt = time.Now()
	return nil
}

func (m *InMemorySessionManager) GetMessages(ctx interface{}, sessionID string) ([]Message, error) {
	m.mu.RLock()
	defer m.mu.RUnlock()

	session, ok := m.sessions[sessionID]
	if !ok {
		return nil, fmt.Errorf("session %s not found", sessionID)
	}

	result := make([]Message, len(session.Messages))
	copy(result, session.Messages)
	return result, nil
}

func (m *InMemorySessionManager) GetRecentMessages(ctx interface{}, sessionID string, n int) ([]Message, error) {
	m.mu.RLock()
	defer m.mu.RUnlock()

	session, ok := m.sessions[sessionID]
	if !ok {
		return nil, fmt.Errorf("session %s not found", sessionID)
	}

	if n <= 0 || n > len(session.Messages) {
		n = len(session.Messages)
	}

	result := make([]Message, n)
	copy(result, session.Messages[len(session.Messages)-n:])
	return result, nil
}

func (m *InMemorySessionManager) UpdateRobotState(ctx interface{}, sessionID string, state *RobotState) error {
	m.mu.Lock()
	defer m.mu.Unlock()

	session, ok := m.sessions[sessionID]
	if !ok {
		return fmt.Errorf("session %s not found", sessionID)
	}

	state.UpdatedAt = time.Now()
	session.RobotStates[state.Name] = state
	session.UpdatedAt = time.Now()
	return nil
}

func (m *InMemorySessionManager) GetRobotState(ctx interface{}, sessionID string, robotName string) (*RobotState, bool) {
	m.mu.RLock()
	defer m.mu.RUnlock()

	session, ok := m.sessions[sessionID]
	if !ok {
		return nil, false
	}

	state, ok := session.RobotStates[robotName]
	return state, ok
}

func (m *InMemorySessionManager) Clear(ctx interface{}, sessionID string) error {
	m.mu.Lock()
	defer m.mu.Unlock()

	session, ok := m.sessions[sessionID]
	if !ok {
		return fmt.Errorf("session %s not found", sessionID)
	}

	session.Messages = make([]Message, 0)
	session.RobotStates = make(map[string]*RobotState)
	session.UpdatedAt = time.Now()
	return nil
}

func (m *InMemorySessionManager) DeleteSession(ctx interface{}, id string) error {
	m.mu.Lock()
	defer m.mu.Unlock()

	if _, ok := m.sessions[id]; !ok {
		return fmt.Errorf("session %s not found", id)
	}

	delete(m.sessions, id)
	return nil
}
