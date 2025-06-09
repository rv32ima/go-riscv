package main

import (
	"fmt"
	"os"

	"github.com/rv32ima/go-riscv/internal/cmd"
)

func main() {
	cmd := cmd.NewRootCmd()
	if err := cmd.Execute(); err != nil {
		fmt.Fprintf(os.Stderr, "error: %v\n", err)
		os.Exit(1)
	}
}
