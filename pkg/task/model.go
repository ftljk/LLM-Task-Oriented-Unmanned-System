package task

type TaskStatus string

const (
	StatusPending   TaskStatus = "Pending"
	StatusRunning   TaskStatus = "Running"
	StatusCompleted TaskStatus = "Completed"
	StatusFailed    TaskStatus = "Failed"
	StatusSkipped   TaskStatus = "Skipped"
)

type TaskAction string

const (
	ActionMove        TaskAction = "Move"
	ActionCollectData TaskAction = "CollectData"
	ActionWait        TaskAction = "Wait"
)

type FailureStrategy string

const (
	FailImmediate  FailureStrategy = "fail"
	SkipAndContinue FailureStrategy = "skip"
	RetryWithBackoff FailureStrategy = "retry"
)

type TaskConfig struct {
	TimeoutMs    int             `json:"timeout_ms,omitempty"`
	MaxRetries   int             `json:"max_retries,omitempty"`
	RetryDelayMs int             `json:"retry_delay_ms,omitempty"`
	OnFailure    FailureStrategy `json:"on_failure,omitempty"`
}

func DefaultTaskConfig() TaskConfig {
	return TaskConfig{
		TimeoutMs:    30000,
		MaxRetries:   2,
		RetryDelayMs: 1000,
		OnFailure:    FailImmediate,
	}
}

type Task struct {
	ID           string                 `json:"id"`
	Description  string                 `json:"description"`
	Action       TaskAction             `json:"action"`
	Target       string                 `json:"target"`
	Params       map[string]interface{} `json:"params"`
	Dependencies []string               `json:"dependencies"`
	Status       TaskStatus             `json:"-"`
	Result       string                 `json:"-"`
	Config       TaskConfig             `json:"config,omitempty"`
}

type ExecutionResult struct {
	TaskID       string     `json:"task_id"`
	Description  string     `json:"description"`
	Action       TaskAction `json:"action"`
	Target       string     `json:"target"`
	Status       TaskStatus `json:"status"`
	Attempts     int        `json:"attempts"`
	Error        string     `json:"error,omitempty"`
	SkipReason   string     `json:"skip_reason,omitempty"`
}

type TaskPlan struct {
	Tasks   []*Task            `json:"tasks"`
	Results []ExecutionResult  `json:"results,omitempty"`
}
