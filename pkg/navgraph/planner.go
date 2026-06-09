package navgraph

import (
	"fmt"
	"math"
)

type Segment struct {
	Angle           float64 `json:"angle"`
	TargetTheta     float64 `json:"target_theta"`
	RotateZ         float64 `json:"rotate_z"`
	RotateDuration  float64 `json:"rotate_duration"`
	ForwardX        float64 `json:"forward_x"`
	ForwardDuration float64 `json:"forward_duration"`
	Distance        float64 `json:"distance"`
	FromX           float64 `json:"from_x"`
	FromY           float64 `json:"from_y"`
	ToX             float64 `json:"to_x"`
	ToY             float64 `json:"to_y"`
	FromName        string  `json:"from_name,omitempty"`
	ToName          string  `json:"to_name,omitempty"`
	Speed           float64 `json:"speed"`
}

// PlanPath decomposes a path (ordered vertex indices) into movement segments.
// The first segment starts from (robotX, robotY, robotTheta) to the first path vertex.
// Subsequent segments go vertex-to-vertex along the path.
func (g *Graph) PlanPath(robotX, robotY, robotTheta float64, path []int, speed float64) []Segment {
	if len(path) == 0 {
		return nil
	}
	if speed <= 0 {
		speed = 0.5
	}

	var segments []Segment

	cx, cy, ctheta := robotX, robotY, robotTheta

	for i := 0; i < len(path); i++ {
		vi := g.Vertices[path[i]]
		tx, ty := vi.X, vi.Y
		toName := vi.Name

		// If we're already at this vertex (within 0.3m), skip
		dx := tx - cx
		dy := ty - cy
		dist := math.Sqrt(dx*dx + dy*dy)
		if dist < 0.3 {
			continue
		}

		// Angle to face the target
		targetAngle := math.Atan2(dy, dx)
		angleDelta := targetAngle - ctheta
		for angleDelta > math.Pi {
			angleDelta -= 2 * math.Pi
		}
		for angleDelta < -math.Pi {
			angleDelta += 2 * math.Pi
		}

		rotateZ := -1.0
		if angleDelta > 0 {
			rotateZ = 1.0
		}
		rotateDur := math.Abs(angleDelta)
		if math.Abs(angleDelta) < 0.01 {
			rotateZ = 0
			rotateDur = 0
		}
		forwardDur := dist / speed

		segments = append(segments, Segment{
			Angle:           angleDelta,
			TargetTheta:     targetAngle,
			RotateZ:         rotateZ,
			RotateDuration:  rotateDur,
			ForwardX:        speed,
			ForwardDuration: forwardDur,
			Distance:        dist,
			FromX:           cx,
			FromY:           cy,
			ToX:             tx,
			ToY:             ty,
			FromName:        g.nearestVertexName(cx, cy),
			ToName:          toName,
			Speed:           speed,
		})

		cx, cy = tx, ty
		ctheta = targetAngle
	}

	return segments
}

func (g *Graph) nearestVertexName(x, y float64) string {
	idx, _ := g.Nearest(x, y)
	if idx >= 0 && idx < len(g.Vertices) {
		return g.Vertices[idx].Name
	}
	return ""
}

// PatrolWaypoints returns a predefined patrol circuit: patrol_A1 → patrol_D1 → patrol_A2 → patrol_D2 → back to patrol_A1.
func PatrolWaypoints() []string {
	return []string{"patrol_A1", "patrol_D1", "patrol_A2", "patrol_D2"}
}

// PatrolRoute finds the shortest patrol circuit starting from the robot's nearest waypoint.
// It returns the patrol waypoints ordered so the robot visits them in a loop.
func (g *Graph) PatrolRoute(robotX, robotY float64) ([]int, float64, error) {
	patrolNames := PatrolWaypoints()
	// Find indices
	patrolIndices := make([]int, 0, len(patrolNames))
	for _, name := range patrolNames {
		idx, ok := g.FindByName(name)
		if !ok {
			continue
		}
		patrolIndices = append(patrolIndices, idx)
	}
	if len(patrolIndices) < 2 {
		return nil, 0, fmt.Errorf("not enough patrol waypoints found")
	}

	// Find nearest patrol waypoint to robot
	nearestIdx := 0
	nearestDist := math.MaxFloat64
	for i, pi := range patrolIndices {
		v := g.Vertices[pi]
		dx := v.X - robotX
		dy := v.Y - robotY
		d := dx*dx + dy*dy
		if d < nearestDist {
			nearestDist = d
			nearestIdx = i
		}
	}

	// Build route starting from nearest patrol waypoint
	ordered := make([]int, 0, len(patrolIndices)+1)
	for i := 0; i < len(patrolIndices); i++ {
		idx := (nearestIdx + i) % len(patrolIndices)
		ordered = append(ordered, patrolIndices[idx])
	}
	ordered = append(ordered, patrolIndices[nearestIdx]) // loop back

	// Compute total path distance by concatenating shortest paths between consecutive waypoints
	totalDist := 0.0
	fullPath := []int{ordered[0]}
	for i := 0; i < len(ordered)-1; i++ {
		seg, segDist, err := g.ShortestPath(ordered[i], ordered[i+1])
		if err != nil {
			return nil, 0, err
		}
		if segDist > 0 {
			fullPath = append(fullPath, seg[1:]...)
		}
		totalDist += segDist
	}

	return fullPath, totalDist, nil
}
