package normalizer

import (
	"fmt"
	"regexp"
	"strings"
)

// Robot name aliases: longer matches must come first for Replacer to work correctly
var robotReplacer = strings.NewReplacer(
	"机器人一号", "robot1",
	"一号机器人", "robot1",
	"机器人1号", "robot1",
	"1号机器人", "robot1",
	"机器人二号", "robot2",
	"二号机器人", "robot2",
	"机器人2号", "robot2",
	"2号机器人", "robot2",
	"一号", "robot1",
	"1号", "robot1",
	"二号", "robot2",
	"2号", "robot2",
	"car1", "robot1",
	"car2", "robot2",
	"robot1", "robot1",
	"robot2", "robot2",
)

var (
	// Rotation: [direction] + 旋转/转 + N度
	rotatePat = regexp.MustCompile(
		`(逆时针|顺时针)?\s*(?:旋转|转)\s*(\d+(?:\.\d+)?)\s*度`,
	)
	// Rotation: 旋转/转 + N度 + direction (only match if direction explicitly follows)
	rotatePostPat = regexp.MustCompile(
		`(?:旋转|转)\s*(\d+(?:\.\d+)?)\s*度\s*(逆时针|顺时针)`,
	)
	// Movement: direction word + distance + unit
	movePat = regexp.MustCompile(
		`(前进|向前|前移|向前运动|向前移动|后退|向后|后移|向后运动|向后移动)\s*(\d+(?:\.\d+)?)\s*(?:米|m)`,
	)
	// Generic movement: 移动 + distance + unit
	moveGenericPat = regexp.MustCompile(
		`移动\s*(\d+(?:\.\d+)?)\s*(?:米|m)`,
	)
)

type robotRef struct {
	name   string
	offset int
}

type hintMatch struct {
	hint   ActionHint
	offset int // byte offset in original input
}

func preprocess(input string) *PreprocessResult {
	result := &PreprocessResult{
		RawInput: input,
		Robots:   make([]string, 0),
		Hints:    make([]ActionHint, 0),
	}

	// Step 1: Find all robot mentions with byte positions
	var robots []robotRef
	robotSet := make(map[string]bool)

	for _, alias := range []string{
		"机器人一号", "一号机器人", "机器人1号", "1号机器人",
		"机器人二号", "二号机器人", "机器人2号", "2号机器人",
		"一号", "1号", "二号", "2号",
		"car1", "car2", "robot1", "robot2",
	} {
		offset := 0
		for {
			idx := strings.Index(input[offset:], alias)
			if idx < 0 {
				break
			}
			absIdx := offset + idx
			canonical := robotReplacer.Replace(alias)
			robots = append(robots, robotRef{name: canonical, offset: absIdx})
			if !robotSet[canonical] {
				robotSet[canonical] = true
				result.Robots = append(result.Robots, canonical)
			}
			offset = absIdx + len(alias)
		}
	}

	// Sort robots by position
	for i := 0; i < len(robots); i++ {
		for j := i + 1; j < len(robots); j++ {
			if robots[i].offset > robots[j].offset {
				robots[i], robots[j] = robots[j], robots[i]
			}
		}
	}

	// Step 2: Extract action hints with positions, assign nearest preceding robot
	var hints []hintMatch
	for _, m := range rotatePat.FindAllStringSubmatchIndex(input, -1) {
		if len(m) < 6 {
			continue
		}
		matchStart := m[0]
		dir := ""
		degStr := ""
		if m[2] >= 0 {
			dir = input[m[2]:m[3]]
		}
		if m[4] >= 0 {
			degStr = input[m[4]:m[5]]
		}
		h := buildRotateHint(degStr, dir, matchStart, robots)
		if h != nil {
			hints = append(hints, *h)
		}
	}
	for _, m := range rotatePostPat.FindAllStringSubmatchIndex(input, -1) {
		if len(m) < 6 {
			continue
		}
		matchStart := m[0]
		degStr := ""
		dir := ""
		if m[2] >= 0 {
			degStr = input[m[2]:m[3]]
		}
		if m[4] >= 0 {
			dir = input[m[4]:m[5]]
		}
		h := buildRotateHint(degStr, dir, matchStart, robots)
		if h != nil {
			hints = append(hints, *h)
		}
	}
	for _, m := range movePat.FindAllStringSubmatchIndex(input, -1) {
		if len(m) < 6 {
			continue
		}
		matchStart := m[0]
		dir := input[m[2]:m[3]]
		distStr := input[m[4]:m[5]]
		h := buildMoveHint(distStr, dir, matchStart, robots)
		if h != nil {
			hints = append(hints, *h)
		}
	}
	for _, m := range moveGenericPat.FindAllStringSubmatchIndex(input, -1) {
		if len(m) < 4 {
			continue
		}
		matchStart := m[0]
		distStr := input[m[2]:m[3]]
		h := buildMoveHint(distStr, "", matchStart, robots)
		if h != nil {
			hints = append(hints, *h)
		}
	}
	// Sort hints by position
	for i := 0; i < len(hints); i++ {
		for j := i + 1; j < len(hints); j++ {
			if hints[i].offset > hints[j].offset {
				hints[i], hints[j] = hints[j], hints[i]
			}
		}
	}

	// Deduplicate: skip hints with same type+value+direction for same robot
	seen := make(map[string]bool)
	for _, hm := range hints {
		key := fmt.Sprintf("%d-%s-%.1f-%s", hm.hint.Type, hm.hint.Target, hm.hint.Value, hm.hint.Direction)
		if seen[key] {
			continue
		}
		seen[key] = true
		result.Hints = append(result.Hints, hm.hint)
	}

	// Step 3: Normalize robot names in the input text
	result.NormalizedInput = robotReplacer.Replace(input)

	return result
}

func buildRotateHint(degStr, dir string, offset int, robots []robotRef) *hintMatch {
	var degrees float64
	_, err := fmt.Sscanf(degStr, "%f", &degrees)
	if err != nil || degrees <= 0 {
		return nil
	}
	if degrees > 720 {
		degrees = 360
	}

	hintDir := "ccw"
	if dir == "顺时针" {
		hintDir = "cw"
	}

	target := nearestPrecedingRobot(offset, robots)

	return &hintMatch{
		offset: offset,
		hint: ActionHint{
			Type:      ActionRotate,
			Target:    target,
			Value:     degrees,
			Direction: hintDir,
		},
	}
}

func buildMoveHint(distStr, dir string, offset int, robots []robotRef) *hintMatch {
	var distance float64
	_, err := fmt.Sscanf(distStr, "%f", &distance)
	if err != nil || distance <= 0 {
		return nil
	}
	if distance > 100 {
		distance = 100
	}

	actionType := ActionMoveForward
	moveDir := "forward"
	if strings.Contains(dir, "后") || strings.Contains(dir, "退") {
		actionType = ActionMoveBackward
		moveDir = "backward"
	}

	target := nearestPrecedingRobot(offset, robots)

	return &hintMatch{
		offset: offset,
		hint: ActionHint{
			Type:      actionType,
			Target:    target,
			Value:     distance,
			Direction: moveDir,
		},
	}
}

// nearestPrecedingRobot finds the robot mentioned closest before the given offset.
func nearestPrecedingRobot(offset int, robots []robotRef) string {
	best := ""
	bestDist := -1
	for _, r := range robots {
		if r.offset <= offset {
			dist := offset - r.offset
			if bestDist < 0 || dist < bestDist {
				bestDist = dist
				best = r.name
			}
		}
	}
	return best
}
