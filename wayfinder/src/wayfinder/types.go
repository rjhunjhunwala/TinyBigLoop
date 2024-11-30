package wayfinder

import (
	"fmt"
	"math"
)

type NodeID uint64
type Node struct {
	id  NodeID
	lat float64
	lon float64
}

type WayID uint64
type Way struct {
	id      WayID
	nodeIds []NodeID
}

type Area struct {
	nodes map[NodeID]Node
	ways  map[WayID]Way
}

func (w Way) isCircular() bool {
	return w.nodeIds[0] == w.nodeIds[len(w.nodeIds)-1]
}

func degToRad(deg float64) float64 {
	return deg * math.Pi / 180
}

func (n Node) tuple() string {
	return fmt.Sprintf("(%f,%f)", n.lat, n.lon)
}

// haversin calculates the haversine of a value (sin²(value / 2))
func haversin(theta float64) float64 {
	return math.Pow(math.Sin(theta/2), 2)
}

// GreatCircleDistance calculates the great-circle distance between two points
// lat1, lon1, lat2, lon2 should be in degrees
// The distance is returned in meters
func (n Node) GreatCircleDistance(o Node) float64 {
	const earthRadiusMeters = 6371.0 * 1000

	// Convert degrees to radians
	lat1Rad := n.lat * math.Pi / 180
	lon1Rad := n.lon * math.Pi / 180
	lat2Rad := o.lat * math.Pi / 180
	lon2Rad := o.lon * math.Pi / 180

	// Haversine formula
	dLat := lat2Rad - lat1Rad
	dLon := lon2Rad - lon1Rad

	a := haversin(dLat) + math.Cos(lat1Rad)*math.Cos(lat2Rad)*haversin(dLon)
	c := 2 * math.Atan2(math.Sqrt(a), math.Sqrt(1-a))

	// Distance
	return earthRadiusMeters * c
}

func (a *Area) Validate() {
	var empty struct{}
	dupes := make(map[string]struct{})
	for id, node := range a.nodes {
		if id != node.id {
			panic(fmt.Sprintf("bad node id %d vs %d", id, node.id))
		}
		key := node.tuple()
		if _, ok := dupes[key]; ok {
			panic(fmt.Sprintf("duplicate %s", key))
		}
		dupes[key] = empty
	}

	for id, way := range a.ways {
		if id != way.id {
			panic(fmt.Sprintf("bad way id %d vs %d", id, way.id))
		}
		// seenNodes := make(map[NodeID]struct{})
		for _, nodeId := range way.nodeIds {
			// seenNodes[nodeId] = empty
			if _, ok := a.nodes[nodeId]; !ok {
				panic(fmt.Sprintf("bad node id %d in way %d", nodeId, way.id))
			}
		}
		// if len(seenNodes) != len(way.nodeIds) {
		// 	panic(fmt.Sprintf("way %d had dupe nodes", way.id))
		// }
	}
}
