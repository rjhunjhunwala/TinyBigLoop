package main

import (
	"fmt"
	"os"

	"github.com/beevik/etree"
	"github.com/pimlu/wayfinder/src/wayfinder"
)

func main() {
	path := os.Args[1]

	doc := etree.NewDocument()

	err := doc.ReadFromFile(path)
	if err != nil {
		panic(err)
	}

	area := wayfinder.IngestArea(doc)

	graph := wayfinder.GraphFromArea(area)

	// fmt.Printf("verts: %d\n", len(graph.Edges))

	// fmt.Printf("%+v\n", graph)

	// fmt.Printf("%s", graph.GenPython(area))

	fmt.Printf("%s", graph.GenSvg(area))

	println("done")
}
