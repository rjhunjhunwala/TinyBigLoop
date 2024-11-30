package wayfinder

func exitIf(err error) {
	if err != nil {
		panic(err)
	}
}
