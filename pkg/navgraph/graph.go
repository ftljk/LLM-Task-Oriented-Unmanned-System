package navgraph

import (
	"fmt"
	"math"
	"os"

	"gopkg.in/yaml.v2"
)

type Vertex struct {
	Index            int
	X, Y             float64
	Name             string
	IsCharger        bool
	IsHoldingPoint   bool
	IsParkingSpot    bool
	PickupDispenser  string
	DropoffIngestor  string
	SpawnRobotName   string
	SpawnRobotType   string
}

type Edge struct {
	From, To int
	DoorName string
}

type Graph struct {
	Vertices  []Vertex
	Edges     []Edge
	NameIndex map[string]int
	Adj       map[int][]Edge
}

type navGraphRaw struct {
	Levels map[string]struct {
		Lanes    []interface{} `yaml:"lanes"`
		Vertices []interface{} `yaml:"vertices"`
	} `yaml:"levels"`
}

func Load(path string) (*Graph, error) {
	data, err := os.ReadFile(path)
	if err != nil {
		return nil, fmt.Errorf("read nav graph: %w", err)
	}

	var raw navGraphRaw
	if err := yaml.Unmarshal(data, &raw); err != nil {
		return nil, fmt.Errorf("parse nav graph YAML: %w", err)
	}

	level, ok := raw.Levels["L1"]
	if !ok {
		return nil, fmt.Errorf("no L1 level in nav graph")
	}

	g := &Graph{
		NameIndex: make(map[string]int),
		Adj:       make(map[int][]Edge),
	}

	// Parse vertices
	for i, v := range level.Vertices {
		entry, ok := v.([]interface{})
		if !ok || len(entry) < 3 {
			continue
		}
		x, ok := toFloat(entry[0])
		if !ok {
			continue
		}
		y, ok := toFloat(entry[1])
		if !ok {
			continue
		}

		vt := Vertex{
			Index: i,
			X:     x,
			Y:     y,
		}

		meta, ok := entry[2].(map[interface{}]interface{})
		if ok {
			if name, ok := meta["name"]; ok {
				vt.Name, _ = name.(string)
			}
			if v, ok := meta["is_charger"]; ok {
				vt.IsCharger, _ = v.(bool)
			}
			if v, ok := meta["is_holding_point"]; ok {
				vt.IsHoldingPoint, _ = v.(bool)
			}
			if v, ok := meta["is_parking_spot"]; ok {
				vt.IsParkingSpot, _ = v.(bool)
			}
			if v, ok := meta["pickup_dispenser"]; ok {
				vt.PickupDispenser, _ = v.(string)
			}
			if v, ok := meta["dropoff_ingestor"]; ok {
				vt.DropoffIngestor, _ = v.(string)
			}
			if v, ok := meta["spawn_robot_name"]; ok {
				vt.SpawnRobotName, _ = v.(string)
			}
			if v, ok := meta["spawn_robot_type"]; ok {
				vt.SpawnRobotType, _ = v.(string)
			}
		}

		g.Vertices = append(g.Vertices, vt)
		if vt.Name != "" {
			g.NameIndex[vt.Name] = i
		}
	}

	// Parse lanes
	for _, l := range level.Lanes {
		entry, ok := l.([]interface{})
		if !ok || len(entry) < 2 {
			continue
		}
		from, ok := toInt(entry[0])
		if !ok {
			continue
		}
		to, ok := toInt(entry[1])
		if !ok {
			continue
		}

		e := Edge{From: from, To: to}

		if len(entry) >= 3 {
			if meta, ok := entry[2].(map[interface{}]interface{}); ok {
				if dn, ok := meta["door_name"]; ok {
					e.DoorName, _ = dn.(string)
				}
			}
		}

		g.Edges = append(g.Edges, e)
		g.Adj[from] = append(g.Adj[from], e)
	}

	return g, nil
}

func (g *Graph) Nearest(x, y float64) (int, float64) {
	best := -1
	bestDist := math.MaxFloat64
	for i, v := range g.Vertices {
		dx := v.X - x
		dy := v.Y - y
		dist := dx*dx + dy*dy
		if dist < bestDist {
			bestDist = dist
			best = i
		}
	}
	return best, math.Sqrt(bestDist)
}

func (g *Graph) FindByName(name string) (int, bool) {
	idx, ok := g.NameIndex[name]
	return idx, ok
}

func (g *Graph) Distance(a, b int) float64 {
	if a < 0 || a >= len(g.Vertices) || b < 0 || b >= len(g.Vertices) {
		return math.MaxFloat64
	}
	dx := g.Vertices[a].X - g.Vertices[b].X
	dy := g.Vertices[a].Y - g.Vertices[b].Y
	return math.Sqrt(dx*dx + dy*dy)
}

func toFloat(v interface{}) (float64, bool) {
	switch val := v.(type) {
	case float64:
		return val, true
	case int:
		return float64(val), true
	case int64:
		return float64(val), true
	default:
		return 0, false
	}
}

func toInt(v interface{}) (int, bool) {
	switch val := v.(type) {
	case int:
		return val, true
	case int64:
		return int(val), true
	case float64:
		return int(val), true
	default:
		return 0, false
	}
}
