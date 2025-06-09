package cmd

import (
	"bytes"
	"fmt"
	"os"

	"github.com/rv32ima/go-riscv/internal/riscv"
	"github.com/spf13/cobra"
	"golang.org/x/exp/constraints"
)

type flags struct {
	ramSize uint64
}

func Abs[T constraints.Integer](x T) T {
	if x < 0 {
		return -x
	}
	return x
}

func NewRootCmd() *cobra.Command {
	var flags flags
	cmd := &cobra.Command{
		Short:         "riscv",
		Args:          cobra.ExactArgs(1),
		SilenceUsage:  true,
		SilenceErrors: true,
		RunE: func(cmd *cobra.Command, args []string) error {
			return run(cmd, args, flags)
		},
	}

	cmd.Flags().Uint64Var(&flags.ramSize, "ram-size", 536870912, "size of the ram")

	return cmd
}

func run(cmd *cobra.Command, args []string, flags flags) error {
	f, err := os.OpenFile(args[0], os.O_RDONLY, 0)
	if err != nil {
		return err
	}

	ram := bytes.NewBuffer(nil)

	fileLen, err := f.WriteTo(ram)
	if err != nil {
		return err
	}

	ram.Write(make([]byte, Abs(int(flags.ramSize)-int(fileLen))))

	if err := f.Close(); err != nil {
		return err
	}

	r := riscv.NewRISCVEmulator(ram)
	defer func() {
		if e, ok := recover().(error); ok {
			fmt.Printf("caught error %v\n", e.Error())
			fmt.Printf("state of emulator at crash:\n")
			r.DumpState()
		}
	}()

	r.RunSingleStep()

	return nil
}
