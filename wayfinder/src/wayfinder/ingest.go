package wayfinder

import (
	"strconv"

	"github.com/beevik/etree"
)

func attrUint64(a *etree.Attr) uint64 {
	val, err := strconv.ParseUint(a.Value, 10, 64)
	exitIf(err)
	return val
}
func attrFloat64(a *etree.Attr) float64 {
	val, err := strconv.ParseFloat(a.Value, 64)
	exitIf(err)
	return val
}
func IngestNode(el *etree.Element) Node {
	id := attrUint64(el.SelectAttr("id"))
	return Node{
		id:  NodeID(id),
		lat: attrFloat64(el.SelectAttr("lat")),
		lon: attrFloat64(el.SelectAttr("lon")),
	}
}
func IngestWay(el *etree.Element) Way {
	id := attrUint64(el.SelectAttr("id"))
	var nodeIds []NodeID

	for _, child := range el.SelectElements("nd") {
		id := attrUint64(child.SelectAttr("ref"))
		nodeIds = append(nodeIds, NodeID(id))
	}
	return Way{
		id:      WayID(id),
		nodeIds: nodeIds,
	}
}

func IngestArea(doc *etree.Document) *Area {
	osm := doc.SelectElement("osm")
	area := &Area{
		nodes: make(map[NodeID]Node),
		ways:  make(map[WayID]Way),
	}

	for _, el := range osm.SelectElements("node") {
		node := IngestNode(el)
		area.nodes[node.id] = node
	}

	for _, el := range osm.SelectElements("way") {
		way := IngestWay(el)
		area.ways[way.id] = way
	}

	area.Validate()
	return area
}
