package main

import (
	"context"
	"fmt"
	"github.com/cloudwego/eino/adk"
	"github.com/cloudwego/eino/components/tool"
	"github.com/cloudwego/eino/compose"
	"github.com/mitchellh/mapstructure"
	"log"

	"github.com/cloudwego/eino-ext/components/model/ark"
	"github.com/cloudwego/eino/components/tool/utils"
	"github.com/gorilla/websocket"
)

var DefaultRosBridgeClient *RosBridgeClient

type RosBridgeClient struct {
	conn *websocket.Conn
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
		return nil, err
	}
	return &RosBridgeClient{conn: conn}, nil
}

func (rc *RosBridgeClient) GetTopicsForType(typeName string) ([]string, error) {
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
	command := map[string]interface{}{
		"op":    "publish",
		"topic": topic,
		"type":  msgType,
		"msg":   msg,
	}

	return rc.conn.WriteJSON(command)
}

func (rc *RosBridgeClient) Subscribe(topic string, callback func(map[string]interface{})) error {
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
	rc.conn.Close()
}

type VelRequest struct {
	Name string `json:"name" jsonschema:"required,description=ros2话题名称"`
	Msg  Twist  `json:"msg" jsonschema:"required,description=速度数据"`
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
	err = decoder.Decode(input.Msg)
	if err != nil {
		return "转换map失败", err
	}
	err = DefaultRosBridgeClient.Publish(input.Name, "geometry_msgs/Twist", msg)
	if err != nil {
		return "发布失败", err
	}
	return "发布成功", nil
}

type PosResponse struct {
	Name string `json:"name" jsonschema:"required,description=ros2话题名称"`
}

func GetPosFunc(ctx context.Context, input PosResponse) (string, error) {
	err := DefaultRosBridgeClient.Subscribe(input.Name, func(msg map[string]interface{}) {
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
	topics, err := DefaultRosBridgeClient.GetTopicsForType(input.Type)
	if err != nil {
		return nil, err
	}
	return topics, nil
}

func init() {
	DefaultRosBridgeClient, _ = NewRosBridgeClient("ws://192.168.124.130:9090")
}

func main() {
	setVelTool, _ := utils.InferTool("机器人运动速度控制工具", "向该工具传入ros2话题名称和具体速度数据，以实现机器人运动速度控制", SetVelFunc)
	getPosTool, _ := utils.InferTool("机器人位置信息采集工具", "向该工具传入ros2话题名称，返回值中将包含位置信息", GetPosFunc)
	getTopicsTool, _ := utils.InferTool("机器人同类型ros2话题名称查询工具", "向该工具传入话题类型，返回值为该类型所有话题名称列表", GetTopicsFunc)
	ctx := context.Background()
	model, _ := ark.NewChatModel(ctx, &ark.ChatModelConfig{
		APIKey: "03978d91-d03d-463a-bf1f-488c6307727d",
		Model:  "deepseek-v3-1-250821",
	})
	retriever, _ := adk.NewChatModelAgent(ctx, &adk.ChatModelAgentConfig{
		Name:        "TopicRetriever",
		Description: "获取ros2话题名称",
		Instruction: "分析请求目标：" +
			"1.机器人运动控制，则话题类型为geometry_msgs/Twist。" +
			"2.机器人信息采集，则话题类型为nav_msgs/Odometry。" +
			"将话题类型传给同类型话题名称查询工具，得到结果再根据请求中的机器人名称从中筛选出唯一正确的话题名称。" +
			"最后完善请求数据，与话题名称一并作为请求输出给下一节点",
		Model: model,
		ToolsConfig: adk.ToolsConfig{
			ToolsNodeConfig: compose.ToolsNodeConfig{
				Tools: []tool.BaseTool{getTopicsTool},
			},
		},
	})
	operator, _ := adk.NewChatModelAgent(ctx, &adk.ChatModelAgentConfig{
		Name:        "Operator",
		Description: "根据请求执行操作",
		Instruction: "你是一名操作员，需要根据请求内容判断并执行相应的操作。可以进行的操作如下：" +
			"1.机器人运动控制。从请求中获取ros2话题名称和具体速度数据并传入机器人运动速度控制工具。" +
			"2.机器人信息采集。从请求中获取ros2话题名称并传入机器人位置信息采集工具，输出工具返回值。",
		Model: model,
		ToolsConfig: adk.ToolsConfig{
			ToolsNodeConfig: compose.ToolsNodeConfig{
				Tools: []tool.BaseTool{setVelTool, getPosTool},
			},
		},
	})
	agent, _ := adk.NewSequentialAgent(ctx, &adk.SequentialAgentConfig{
		Name:        "RequestProcessor",
		Description: "请求处理流程：分析得到话题名称 → 执行操作",
		SubAgents:   []adk.Agent{retriever, operator},
	})
	runner := adk.NewRunner(ctx, adk.RunnerConfig{
		Agent: agent,
	})
	input := "让robot1以线速度x=1，角速度z=1的速度运动"
	iter := runner.Query(ctx, input)
	stepCount := 1
	for {
		event, ok := iter.Next()
		if !ok {
			break
		}

		if event.Err != nil {
			log.Fatal(event.Err)
		}

		if event.Output != nil && event.Output.MessageOutput != nil {
			fmt.Printf("\n=== 步骤 %d: %s ===\n", stepCount, event.AgentName)
			fmt.Printf("%s\n", event.Output.MessageOutput.Message.Content)
			stepCount++
		}
	}
	//client, err := NewRosBridgeClient("ws://192.168.124.130:9090")
	//if err != nil {
	//	panic(err)
	//}
	//defer client.Close()
	//
	//fmt.Println("=== 获取所有 Topics ===")
	//topics, err := client.GetTopics()
	//if err != nil {
	//	log.Fatal(err)
	//}
	//
	//for _, topic := range topics {
	//	fmt.Printf("Topic: %-40s Type: %s\n", topic.Name, topic.Type)
	//}

	//// 订阅激光雷达数据
	//err = client.Subscribe("/odom", func(msg map[string]interface{}) {
	//	fmt.Printf("Received odom data: %v\n", msg)
	//})
	//if err != nil {
	//	log.Fatal(err)
	//}
	//
	//// 发布速度命令
	//twist := map[string]interface{}{
	//	"linear":  map[string]float64{"x": 0.5, "y": 0, "z": 0},
	//	"angular": map[string]float64{"x": 0, "y": 0, "z": 0.1},
	//}
	//
	//err = client.Publish("/cmd_vel", "geometry_msgs/Twist", twist)
	//if err != nil {
	//	log.Fatal(err)
	//}
	//
	select {} // 保持运行
}
