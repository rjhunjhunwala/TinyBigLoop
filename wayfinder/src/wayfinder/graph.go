package wayfinder

import (
	"fmt"
	"slices"
	"strconv"
	"strings"
)

type Graph struct {
	Edges map[NodeID]map[NodeID]float64
}

func newGraph() *Graph {
	return &Graph{
		Edges: make(map[NodeID]map[NodeID]float64),
	}
}

func (g *Graph) addEdge(i, j NodeID, dist float64) {
	if _, ok := g.Edges[i]; !ok {
		g.Edges[i] = make(map[NodeID]float64)
	}
	g.Edges[i][j] = dist
}

func (a *Area) makeBackRefs() map[NodeID][]WayID {
	backRefs := make(map[NodeID][]WayID)
	for nodeId := range a.nodes {
		backRefs[nodeId] = nil
	}

	for _, way := range a.ways {
		for _, nodeId := range way.nodeIds {
			backRefs[nodeId] = append(backRefs[nodeId], way.id)
		}
	}

	return backRefs
}

func GraphFromArea(area *Area) *Graph {
	backRefs := area.makeBackRefs()

	graph := newGraph()

	for _, way := range area.ways {
		if len(way.nodeIds) == 0 {
			continue
		}
		var prevCrossId NodeID = 0
		var prevCrossDist float64 = 0
		cumDist := 0.0

		if way.isCircular() {
			prevCrossId = way.nodeIds[0]
		}
		for i, nodeId := range way.nodeIds {
			if i != 0 {
				node := area.nodes[nodeId]
				prevNode := area.nodes[way.nodeIds[i-1]]

				delta := node.GreatCircleDistance(prevNode)
				cumDist += delta
			}

			// for _, brefId := range backRefs[nodeId] {
			// 	if brefId == way.id {
			// 		continue
			// 	}

			// 	if prevCrossId != 0 {
			// 		// if way.id == 603162778 {
			// 		// 	fmt.Printf("WAY=%d adding %d %d, %f\n", way.id, prevCrossId, nodeId, dist)
			// 		// 	fmt.Printf("node: %+v\n", area.nodes[nodeId])
			// 		// 	fmt.Printf("prev: %+v\n", area.nodes[prevCrossId])
			// 		// 	fmt.Printf("GCD=%f\n", area.nodes[nodeId].GreatCircleDistance(area.nodes[prevCrossId]))
			// 		// }
			// 	}
			// }

			if len(backRefs[nodeId]) > 1 {
				if prevCrossId != 0 {
					dist := cumDist - prevCrossDist
					graph.addEdge(prevCrossId, nodeId, dist)
					graph.addEdge(nodeId, prevCrossId, dist)
				}

				prevCrossId = nodeId
				prevCrossDist = cumDist
			}

		}
	}

	return graph
}

func (g *Graph) GenPython(a *Area) string {
	var sb strings.Builder
	sb.WriteString("V = [")
	for nodeId := range g.Edges {
		sb.WriteString(a.nodes[nodeId].tuple())
		sb.WriteString(",\n")
	}
	sb.WriteString("]\n")

	sb.WriteString("E = {\n")
	for i, adj := range g.Edges {
		for j, dist := range adj {
			sb.WriteString("(")
			sb.WriteString(a.nodes[i].tuple())
			sb.WriteString(",")
			sb.WriteString(a.nodes[j].tuple())
			sb.WriteString("):")

			sb.WriteString(strconv.FormatFloat(dist, 'g', -1, 64))
			sb.WriteString(",\n")
		}
	}
	sb.WriteString("}\n")

	return sb.String()
}

func (g *Graph) GenSvg(a *Area) string {
	var sb strings.Builder

	seen := false
	var minLat, maxLat, minLon, maxLon float64
	for id := range g.Edges {
		node := a.nodes[id]
		if !seen {
			minLat = node.lat
			maxLat = node.lat
			minLon = node.lon
			maxLon = node.lon
			seen = true
		}
		if node.lat < minLat {
			minLat = node.lat
		}
		if node.lat > maxLat {
			maxLat = node.lat
		}

		if node.lon < minLon {
			minLon = node.lon
		}
		if node.lon > maxLon {
			maxLon = node.lon
		}
	}
	lonRange := maxLon - minLon
	latRange := maxLat - minLat

	brefs := a.makeBackRefs()

	// viewBox := fmt.Sprintf("%f %f %f %f", minLon, minLat, maxLon, maxLat)

	width := 800.0
	height := width * latRange / lonRange

	svg := fmt.Sprintf("<svg width=\"%f\" height=\"%f\" xmlns=\"http://www.w3.org/2000/svg\">", width, height)
	sb.WriteString(svg)

	for i, adj := range g.Edges {
		for j, _ := range adj {
			u := a.nodes[i]
			v := a.nodes[j]

			x1 := (u.lon - minLon) / lonRange * width
			x2 := (v.lon - minLon) / lonRange * width

			y1 := (maxLat - u.lat) / latRange * width
			y2 := (maxLat - v.lat) / latRange * width

			uback := brefs[u.id]
			vback := brefs[v.id]

			xsect := slices.Clone(uback)
			xsect = slices.DeleteFunc(xsect, func(id WayID) bool {
				return !slices.Contains(vback, id)
			})

			sb.WriteString(fmt.Sprintf("<line x1=\"%f\" y1=\"%f\" x2=\"%f\" y2=\"%f\" stroke=\"black\" data-uid=\"%d\" data-vid=\"%d\" brefs=\"%v+\" />\n", x1, y1, x2, y2, u.id, v.id, xsect))
		}
	}
	sb.WriteString("</svg>\n")
	return sb.String()
}
