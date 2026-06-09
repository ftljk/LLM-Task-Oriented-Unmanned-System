package normalizer

import "robot/pkg/task"

type ActionType int

const (
	ActionRotate      ActionType = iota
	ActionMoveForward
	ActionMoveBackward
)

type ActionHint struct {
	Type      ActionType
	Target    string
	Value     float64 // degrees for Rotate, meters for Move
	Direction string  // "cw"/"ccw" for Rotate, "forward"/"backward" for Move
}

type PreprocessResult struct {
	NormalizedInput string
	RawInput        string
	Robots          []string
	Hints           []ActionHint
}

type ValidationResult struct {
	Plan         *task.TaskPlan
	Corrections  []string
	WasCorrected bool
}

type Normalizer struct{}

func NewNormalizer() *Normalizer {
	return &Normalizer{}
}

func (n *Normalizer) Preprocess(input string) *PreprocessResult {
	return preprocess(input)
}

func (n *Normalizer) ValidateAndFix(plan *task.TaskPlan, pp *PreprocessResult) *ValidationResult {
	return validateAndFix(plan, pp)
}
