package navgraph

import (
	"testing"
)

func TestLoadGraph(t *testing.T) {
	g, err := Load("/home/mofus/rmf_ws/install/rmf_demos_maps/share/rmf_demos_maps/maps/office/nav_graphs/0.yaml")
	if err != nil {
		t.Fatalf("failed to load nav graph: %v", err)
	}
	if len(g.Vertices) == 0 {
		t.Fatal("no vertices loaded")
	}
	if len(g.Edges) == 0 {
		t.Fatal("no edges loaded")
	}
}

func TestFindByName(t *testing.T) {
	g, err := Load("/home/mofus/rmf_ws/install/rmf_demos_maps/share/rmf_demos_maps/maps/office/nav_graphs/0.yaml")
	if err != nil {
		t.Fatalf("failed to load nav graph: %v", err)
	}
	idx, ok := g.FindByName("pantry")
	if !ok {
		t.Fatal("'pantry' not found")
	}
	if g.Vertices[idx].Name != "pantry" {
		t.Fatalf("expected 'pantry', got '%s'", g.Vertices[idx].Name)
	}
}

func TestNearest(t *testing.T) {
	g, err := Load("/home/mofus/rmf_ws/install/rmf_demos_maps/share/rmf_demos_maps/maps/office/nav_graphs/0.yaml")
	if err != nil {
		t.Fatalf("failed to load nav graph: %v", err)
	}
	// pantry is at (16.85, -5.40)
	idx, dist := g.Nearest(16.85, -5.40)
	if dist > 1.0 {
		t.Fatalf("nearest to pantry should be close, got dist=%.2f", dist)
	}
	if g.Vertices[idx].Name != "pantry" {
		t.Fatalf("expected pantry, got '%s'", g.Vertices[idx].Name)
	}
}

func TestShortestPath(t *testing.T) {
	g, err := Load("/home/mofus/rmf_ws/install/rmf_demos_maps/share/rmf_demos_maps/maps/office/nav_graphs/0.yaml")
	if err != nil {
		t.Fatalf("failed to load nav graph: %v", err)
	}
	// pantry index
	pantry, _ := g.FindByName("pantry")
	coe, _ := g.FindByName("coe")

	path, dist, err := g.ShortestPath(pantry, coe)
	if err != nil {
		t.Fatalf("shortest path failed: %v", err)
	}
	if len(path) < 2 {
		t.Fatal("path should have at least 2 vertices")
	}
	if dist <= 0 {
		t.Fatal("distance should be positive")
	}
	// First vertex should be pantry, last should be coe
	if g.Vertices[path[0]].Name != "pantry" {
		t.Fatalf("first vertex should be pantry, got '%s'", g.Vertices[path[0]].Name)
	}
	if g.Vertices[path[len(path)-1]].Name != "coe" {
		t.Fatalf("last vertex should be coe, got '%s'", g.Vertices[path[len(path)-1]].Name)
	}
}

func TestPlanPath(t *testing.T) {
	g, err := Load("/home/mofus/rmf_ws/install/rmf_demos_maps/share/rmf_demos_maps/maps/office/nav_graphs/0.yaml")
	if err != nil {
		t.Fatalf("failed to load nav graph: %v", err)
	}
	pantry, _ := g.FindByName("pantry")
	coe, _ := g.FindByName("coe")

	path, _, err := g.ShortestPath(pantry, coe)
	if err != nil {
		t.Fatalf("shortest path failed: %v", err)
	}

	// Start at pantry position facing east
	segments := g.PlanPath(16.85, -5.40, 0, path, 0.5)
	if len(segments) == 0 {
		t.Fatal("expected at least one segment")
	}
	// Each segment should have an angle and distance
	for i, s := range segments {
		if s.Distance <= 0 {
			t.Fatalf("segment %d has non-positive distance", i)
		}
		if s.Speed <= 0 {
			t.Fatalf("segment %d has non-positive speed", i)
		}
	}
}
