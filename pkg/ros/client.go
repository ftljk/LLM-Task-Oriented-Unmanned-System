package ros

import (
	"log"
	"sync"

	"github.com/gorilla/websocket"
)

var DefaultRosBridgeClient *RosBridgeClient

type RosBridgeClient struct {
	conn  *websocket.Conn
	mutex sync.Mutex // 保护 WebSocket 连接的并发写入
}

type TopicInfo struct {
	Name string
	Type string
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

type ServiceResponse struct {
	Id      string      `json:"id"`
	Service string      `json:"service"`
	Values  interface{} `json:"values"`
	Result  bool        `json:"result"`
}

func NewRosBridgeClient(url string) (*RosBridgeClient, error) {
	conn, _, err := websocket.DefaultDialer.Dial(url, nil)
	if err != nil {
		// Mock Mode: return client with nil conn, but pass err
		return &RosBridgeClient{conn: nil}, err
	}
	return &RosBridgeClient{conn: conn}, nil
}

func (rc *RosBridgeClient) GetTopicsForType(typeName string) ([]string, error) {
	if rc.conn == nil {
		log.Println("[Mock] GetTopicsForType called")
		return []string{"/robot1/cmd_vel", "/robot2/cmd_vel", "/robot1/odom", "/robot2/odom"}, nil
	}

	// 使用互斥锁保护并发读写
	rc.mutex.Lock()
	defer rc.mutex.Unlock()

	// 发送请求
	request := NewCallService("/rosapi/topics")
	request.Args = typeName

	err := rc.conn.WriteJSON(request)
	if err != nil {
		return nil, err
	}

	// 读取响应
	var response ServiceResponse
	err = rc.conn.ReadJSON(&response)
	if err != nil {
		return nil, err
	}
	topics := response.Values.(map[string]interface{})["topics"].([]interface{})
	result := make([]string, len(topics))
	for i, topic := range topics {
		result[i] = topic.(string)
	}
	return result, nil
}

func (rc *RosBridgeClient) Publish(topic string, msgType string, msg interface{}) error {
	if rc.conn == nil {
		log.Printf("[Mock] Publish to %s: %v\n", topic, msg)
		return nil
	}

	command := map[string]interface{}{
		"op":    "publish",
		"topic": topic,
		"type":  msgType,
		"msg":   msg,
	}

	// 使用互斥锁保护并发写入
	rc.mutex.Lock()
	defer rc.mutex.Unlock()
	return rc.conn.WriteJSON(command)
}

func (rc *RosBridgeClient) Subscribe(topic string, callback func(map[string]interface{})) error {
	if rc.conn == nil {
		log.Printf("[Mock] Subscribe to %s\n", topic)
		return nil
	}

	command := map[string]interface{}{
		"op":    "subscribe",
		"topic": topic,
	}

	if err := rc.conn.WriteJSON(command); err != nil {
		return err
	}

	go func() {
		for {
			var msg map[string]interface{}
			err := rc.conn.ReadJSON(&msg)
			if err != nil {
				log.Printf("Error reading message: %v", err)
				return
			}
			callback(msg)
		}
	}()

	return nil
}

func (rc *RosBridgeClient) Close() {
	if rc.conn != nil {
		rc.conn.Close()
	}
}

func InitGlobalClient(url string) error {
	var err error
	DefaultRosBridgeClient, err = NewRosBridgeClient(url)
	return err
}
