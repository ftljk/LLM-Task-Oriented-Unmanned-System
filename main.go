package main

import (
	"context"
	"fmt"
	"log"
	"robot/pkg/agent"
	"robot/pkg/ros"
	"robot/pkg/scheduler"
	"robot/pkg/task"
	"robot/pkg/tools"

	"github.com/cloudwego/eino/adk"
)

func main() {
	// 初始化 ROS 客户端
	// 注意：如果本地没有运行 ROS Bridge，这一步会失败。
	err := ros.InitGlobalClient("ws://192.168.1.11:9090")
	if err != nil {
		log.Printf("Warning: Failed to connect to ROS Bridge: %v. Assuming simulation mode or network issue.", err)
	} else {
		defer ros.DefaultRosBridgeClient.Close()
	}

	ctx := context.Background()

	// 初始化 ChatModel
	model, err := agent.NewChatModel(ctx)
	if err != nil {
		log.Fatal(err)
	}

	// 初始化 TaskPlanner Agent
	plannerAgent, err := agent.NewTaskPlanner(ctx, model)
	if err != nil {
		log.Fatal(err)
	}

	// 设置回调
	planChan := make(chan *task.TaskPlan, 1)
	tools.PlanCallback = func(p *task.TaskPlan) {
		planChan <- p
	}

	runner := adk.NewRunner(ctx, adk.RunnerConfig{
		Agent: plannerAgent,
	})

	input := "robot1向前移动(x=0.5)，robot2向后移动(x=-0.5)；2秒后robot1原地旋转(z=1.0)，同时robot2停止。"
	fmt.Printf("User Input: %s\n", input)

	iter := runner.Query(ctx, input)
	
	// 处理 Agent 输出
	go func() {
		for {
			event, ok := iter.Next()
			if !ok {
				break
			}
			if event.Err != nil {
				log.Printf("Agent error: %v", event.Err)
				return
			}
			if event.Output != nil && event.Output.MessageOutput != nil {
				fmt.Printf("Agent message: %s\n", event.Output.MessageOutput.Message.Content)
			}
		}
	}()

	// 等待 Plan
	fmt.Println("Waiting for plan...")
	var finalPlan *task.TaskPlan
	select {
	case finalPlan = <-planChan:
		fmt.Println("Plan received!")
	// 这里可以加个 timeout，或者直接阻塞等待
	}

	if finalPlan != nil {
		fmt.Println("Executing Plan...")
		sched := scheduler.NewScheduler()
		err := sched.ExecutePlan(ctx, finalPlan)
		if err != nil {
			log.Printf("Plan execution failed: %v", err)
		} else {
			fmt.Println("Plan execution completed successfully.")
		}
	} else {
		fmt.Println("No plan generated.")
	}
}
