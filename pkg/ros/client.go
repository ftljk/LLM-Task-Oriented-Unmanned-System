package ros

import (
	"fmt"
	"log"
	"sync"
	"time"

	"github.com/gorilla/websocket"
)

var DefaultRosBridgeClient *RosBridgeClient

type RosBridgeClient struct {
	conn     *websocket.Conn
	writeMu  sync.Mutex
	closeCh  chan struct{}

	handlers   map[string]func(map[string]interface{})
	handlersMu sync.RWMutex

	serviceMu sync.Mutex
	pending   map[string]chan<- map[string]interface{}
	pendingMu sync.Mutex
}

type CallService struct {
	Op           string      `json:"op"`
	Id           string      `json:"id,omitempty"`
	Service      string      `json:"service"`
	Args         interface{} `json:"args,omitempty"`
	FragmentSize int         `json:"fragment_size,omitempty"`
	Compression  string      `json:"compression,omitempty"`
	Timeout      float64     `json:"timeout,omitempty"`
}

func NewCallService(service string) *CallService {
	return &CallService{
		Op:      "call_service",
		Service: service,
	}
}

func NewRosBridgeClient(url string) (*RosBridgeClient, error) {
	conn, _, err := websocket.DefaultDialer.Dial(url, nil)
	if err != nil {
		return &RosBridgeClient{conn: nil, closeCh: make(chan struct{})}, err
	}
	c := &RosBridgeClient{
		conn:     conn,
		closeCh:  make(chan struct{}),
		handlers: make(map[string]func(map[string]interface{})),
		pending:  make(map[string]chan<- map[string]interface{}),
	}
	go c.reader()
	return c, nil
}

func (rc *RosBridgeClient) reader() {
	defer func() {
		if r := recover(); r != nil {
			log.Printf("[ROS client] reader recovered: %v", r)
		}
	}()
	for {
		var msg map[string]interface{}
		err := rc.conn.ReadJSON(&msg)
		if err != nil {
			select {
			case <-rc.closeCh:
				return
			default:
				log.Printf("[ROS client] read error: %v", err)
				return
			}
		}

		op, _ := msg["op"].(string)

		// Skip fire-and-forget ack messages before any handler lookup
		if op == "publish_result" || op == "subscribe_result" || op == "error" {
			continue
		}

		// Route pending service responses
		rc.pendingMu.Lock()
		if op == "service_response" {
			service, _ := msg["service"].(string)
			if ch, ok := rc.pending[service]; ok {
				delete(rc.pending, service)
				rc.pendingMu.Unlock()
				ch <- msg
				close(ch)
				continue
			}
		}
		rc.pendingMu.Unlock()

		// Route to topic handler
		topic, _ := msg["topic"].(string)
		if topic != "" {
			rc.handlersMu.RLock()
			h, ok := rc.handlers[topic]
			rc.handlersMu.RUnlock()
			if ok {
				h(msg)
				continue
			}
			log.Printf("[ROS client] no handler for topic=%s op=%s", topic, op)
			continue
		}

		log.Printf("[ROS client] unhandled: op=%s topic=%s", op, topic)
	}
}

func (rc *RosBridgeClient) writeJSON(v interface{}) error {
	rc.writeMu.Lock()
	defer rc.writeMu.Unlock()
	return rc.conn.WriteJSON(v)
}

func (rc *RosBridgeClient) GetTopicsForType(typeName string) ([]string, error) {
	rc.serviceMu.Lock()
	defer rc.serviceMu.Unlock()

	if rc.conn == nil {
		log.Println("[Mock] GetTopicsForType called")
		return []string{"/model/robot1/cmd_vel", "/model/robot2/cmd_vel",
			"/model/robot1/odometry", "/model/robot2/odometry",
			"/robot1/scan", "/robot2/scan"}, nil
	}

	ch := make(chan map[string]interface{}, 1)
	rc.pendingMu.Lock()
	rc.pending["/rosapi/topics"] = ch
	rc.pendingMu.Unlock()

	req := NewCallService("/rosapi/topics")
	req.Args = typeName
	err := rc.writeJSON(req)
	if err != nil {
		return nil, err
	}

	select {
	case resp := <-ch:
		values, ok := resp["values"].(map[string]interface{})
		if !ok {
			return nil, fmt.Errorf("unexpected service response: %v", resp)
		}
		topicsRaw, ok := values["topics"]
		if !ok {
			return nil, fmt.Errorf("no topics in response: %v", resp)
		}

		list, ok := topicsRaw.([]interface{})
		if !ok {
			return nil, fmt.Errorf("topics not a list: %T", topicsRaw)
		}
		result := make([]string, len(list))
		for i, t := range list {
			result[i], _ = t.(string)
		}
		return result, nil
	case <-time.After(10 * time.Second):
		rc.pendingMu.Lock()
		delete(rc.pending, "/rosapi/topics")
		rc.pendingMu.Unlock()
		return nil, fmt.Errorf("timeout waiting for service response")
	}
}

func (rc *RosBridgeClient) Publish(topic string, msgType string, msg interface{}) error {
	if rc.conn == nil {
		log.Printf("[Mock] Publish to %s: %v", topic, msg)
		return nil
	}

	cmd := map[string]interface{}{
		"op":    "publish",
		"topic": topic,
		"type":  msgType,
		"msg":   msg,
	}
	return rc.writeJSON(cmd)
}

func (rc *RosBridgeClient) Subscribe(topic string, callback func(map[string]interface{})) error {
	if rc.conn == nil {
		log.Printf("[Mock] Subscribe to %s", topic)
		return nil
	}

	rc.handlersMu.Lock()
	rc.handlers[topic] = callback
	rc.handlersMu.Unlock()

	cmd := map[string]interface{}{
		"op":    "subscribe",
		"topic": topic,
	}
	return rc.writeJSON(cmd)
}

func (rc *RosBridgeClient) Unsubscribe(topic string) error {
	rc.handlersMu.Lock()
	delete(rc.handlers, topic)
	rc.handlersMu.Unlock()

	if rc.conn == nil {
		return nil
	}
	return rc.writeJSON(map[string]interface{}{
		"op":    "unsubscribe",
		"topic": topic,
	})
}

func (rc *RosBridgeClient) Close() {
	close(rc.closeCh)
	if rc.conn != nil {
		rc.conn.Close()
	}
}

func InitGlobalClient(url string) error {
	var err error
	DefaultRosBridgeClient, err = NewRosBridgeClient(url)
	return err
}
