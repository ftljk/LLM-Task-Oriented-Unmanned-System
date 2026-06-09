package navgraph

import (
	"container/heap"
	"fmt"
	"math"
)

type pathItem struct {
	index    int
	vertex   int
	distance float64
	prev     int
}

type priorityQueue []*pathItem

func (pq priorityQueue) Len() int            { return len(pq) }
func (pq priorityQueue) Less(i, j int) bool  { return pq[i].distance < pq[j].distance }
func (pq priorityQueue) Swap(i, j int)       { pq[i], pq[j] = pq[j], pq[i]; pq[i].index = i; pq[j].index = j }
func (pq *priorityQueue) Push(x interface{}) { n := len(*pq); item := x.(*pathItem); item.index = n; *pq = append(*pq, item) }
func (pq *priorityQueue) Pop() interface{}   { old := *pq; n := len(old); item := old[n-1]; old[n-1] = nil; item.index = -1; *pq = old[:n-1]; return item }

func (g *Graph) ShortestPath(from, to int) ([]int, float64, error) {
	return g.dijkstra(from, to, nil)
}

func (g *Graph) ShortestPathAvoiding(from, to int, avoid []int) ([]int, float64, error) {
	avoidSet := make(map[int]bool, len(avoid))
	for _, v := range avoid {
		avoidSet[v] = true
	}
	return g.dijkstra(from, to, avoidSet)
}

func (g *Graph) ShortestPathMinimizeOverlap(from, to int, avoid []int) ([]int, float64, error) {
	avoidSet := make(map[int]bool, len(avoid))
	for _, v := range avoid {
		avoidSet[v] = true
	}
	return g.dijkstraPenalized(from, to, avoidSet, 100.0)
}

func (g *Graph) dijkstra(from, to int, avoidSet map[int]bool) ([]int, float64, error) {
	return g.dijkstraInternal(from, to, avoidSet, 0)
}

func (g *Graph) dijkstraPenalized(from, to int, avoidSet map[int]bool, penalty float64) ([]int, float64, error) {
	return g.dijkstraInternal(from, to, avoidSet, penalty)
}

func (g *Graph) dijkstraInternal(from, to int, avoidSet map[int]bool, penalty float64) ([]int, float64, error) {
	if from < 0 || from >= len(g.Vertices) {
		return nil, 0, fmt.Errorf("from vertex %d out of range", from)
	}
	if to < 0 || to >= len(g.Vertices) {
		return nil, 0, fmt.Errorf("to vertex %d out of range", to)
	}
	if from == to {
		return []int{from}, 0, nil
	}

	dist := make([]float64, len(g.Vertices))
	prev := make([]int, len(g.Vertices))
	visited := make([]bool, len(g.Vertices))
	for i := range dist {
		dist[i] = math.Inf(1)
		prev[i] = -1
	}
	if avoidSet[from] {
		return nil, 0, fmt.Errorf("from vertex %d is in avoid set", from)
	}
	dist[from] = 0

	pq := &priorityQueue{}
	heap.Init(pq)
	heap.Push(pq, &pathItem{vertex: from, distance: 0})

	for pq.Len() > 0 {
		item := heap.Pop(pq).(*pathItem)
		u := item.vertex
		if visited[u] {
			continue
		}
		visited[u] = true
		if u == to {
			break
		}

		for _, edge := range g.Adj[u] {
			v := edge.To
			if visited[v] {
				continue
			}
			if penalty == 0 && avoidSet[v] {
				continue
			}
			nd := dist[u] + g.Distance(u, v)
			if penalty > 0 && avoidSet[v] {
				nd += penalty
			}
			if nd < dist[v] {
				dist[v] = nd
				prev[v] = u
				heap.Push(pq, &pathItem{vertex: v, distance: nd})
			}
		}
	}

	if math.IsInf(dist[to], 1) {
		return nil, 0, fmt.Errorf("no path from vertex %d to %d", from, to)
	}

	path := []int{}
	for cur := to; cur != -1; cur = prev[cur] {
		path = append(path, cur)
	}
	for i, j := 0, len(path)-1; i < j; i, j = i+1, j-1 {
		path[i], path[j] = path[j], path[i]
	}

	return path, dist[to], nil
}
