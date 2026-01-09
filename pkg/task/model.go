package task

type TaskStatus string

const (
	StatusPending   TaskStatus = "Pending"
	StatusRunning   TaskStatus = "Running"
	StatusCompleted TaskStatus = "Completed"
	StatusFailed    TaskStatus = "Failed"
)

type TaskAction string

const (
	ActionMove        TaskAction = "Move"
	ActionCollectData TaskAction = "CollectData"
	ActionWait        TaskAction = "Wait" // 等待指定时间（秒）
)

type Task struct {
	ID           string                 `json:"id" jsonschema:"description=任务唯一标识符"`
	Description  string                 `json:"description" jsonschema:"description=任务自然语言描述"`
	Action       TaskAction             `json:"action" jsonschema:"description=任务动作类型,enum=Move,enum=CollectData"`
	Target       string                 `json:"target" jsonschema:"description=执行任务的目标机器人名称,例如robot1"`
	Params       map[string]interface{} `json:"params" jsonschema:"description=任务参数"`
	Dependencies []string               `json:"dependencies" jsonschema:"description=依赖的前置任务ID列表"`
	Status       TaskStatus             `json:"-"` // Internal status
	Result       string                 `json:"-"` // Execution result
}

type TaskPlan struct {
	Tasks []*Task `json:"tasks" jsonschema:"description=任务列表"`
}
