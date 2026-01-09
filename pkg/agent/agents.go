package agent

import (
	"context"
	"robot/pkg/tools"

	"github.com/cloudwego/eino-ext/components/model/ark"
	"github.com/cloudwego/eino/adk"
	"github.com/cloudwego/eino/components/tool"
	"github.com/cloudwego/eino/components/tool/utils"
	"github.com/cloudwego/eino/compose"
)

func NewChatModel(ctx context.Context) (*ark.ChatModel, error) {
	return ark.NewChatModel(ctx, &ark.ChatModelConfig{
		APIKey: "03978d91-d03d-463a-bf1f-488c6307727d",
		Model:  "deepseek-v3-1-250821",
	})
}

func NewRequestProcessor(ctx context.Context, model *ark.ChatModel) (adk.Agent, error) {
	setVelTool, _ := utils.InferTool("机器人运动速度控制工具", "向该工具传入ros2话题名称和具体速度数据，以实现机器人运动速度控制", tools.SetVelFunc)
	getPosTool, _ := utils.InferTool("机器人位置信息采集工具", "向该工具传入ros2话题名称，返回值中将包含位置信息", tools.GetPosFunc)
	getTopicsTool, _ := utils.InferTool("机器人同类型ros2话题名称查询工具", "向该工具传入话题类型，返回值为该类型所有话题名称列表", tools.GetTopicsFunc)

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
	agent, err := adk.NewSequentialAgent(ctx, &adk.SequentialAgentConfig{
		Name:        "RequestProcessor",
		Description: "请求处理流程：分析得到话题名称 → 执行操作",
		SubAgents:   []adk.Agent{retriever, operator},
	})
	return agent, err
}

func NewTaskPlanner(ctx context.Context, model *ark.ChatModel) (adk.Agent, error) {
	submitPlanTool, _ := utils.InferTool("提交任务规划结果工具", "将自然语言转换后的任务规划结果提交给系统", tools.SubmitPlanFunc)

	planner, err := adk.NewChatModelAgent(ctx, &adk.ChatModelAgentConfig{
		Name:        "TaskPlanner",
		Description: "将自然语言指令转换为结构化的任务规划",
		Instruction: `你是一个任务规划专家。你的任务是将用户的自然语言指令分解为一系列可执行的任务（Task）。

你可以创建的任务类型包括：
1. Move（移动）- 控制机器人移动或旋转
2. CollectData（数据采集）- 采集机器人传感器数据
3. Wait（等待）- 等待指定的时间（秒），用于在动作之间添加延时

重要规则：
- 请注意任务之间的依赖关系。如果任务B需要在任务A完成后才能开始，请将任务A的ID添加到任务B的Dependencies列表中。
- 请识别指令中的执行主体（例如 robot1, robot2）。
- 当一个机器人需要持续运动一段时间时，应该在 Move 任务后添加一个 Wait 任务来保持运动状态，然后再发送下一个命令。
- 例如：robot1移动 → Wait等待2秒 → robot1停止，这样robot1才能真正移动一段距离。

参数说明：
- Move 任务：params 中包含 x（前进/后退速度）, y（左右速度）, z（旋转角速度）
- Wait 任务：params 中包含 duration（等待秒数，例如 {"duration": 2.0}）
- CollectData 任务：通常不需要额外参数

你需要调用'提交任务规划结果工具'来提交你的规划结果。`,
		Model: model,
		ToolsConfig: adk.ToolsConfig{
			ToolsNodeConfig: compose.ToolsNodeConfig{
				Tools: []tool.BaseTool{submitPlanTool},
			},
		},
	})
	return planner, err
}
