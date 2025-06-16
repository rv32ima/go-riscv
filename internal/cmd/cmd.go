package cmd

import (
	"fmt"
	"log/slog"
	"net/http"
	"os"
	"strconv"
	"sync"

	"github.com/rv32ima/go-riscv/internal/riscv"
	"github.com/spf13/cobra"
)

type flags struct {
	ramSize uint64
}

func NewRootCmd() *cobra.Command {
	var flags flags
	cmd := &cobra.Command{
		Short:         "riscv",
		Args:          cobra.ExactArgs(0),
		SilenceUsage:  true,
		SilenceErrors: true,
		RunE: func(cmd *cobra.Command, args []string) error {
			return run(cmd, args, flags)
		},
	}

	cmd.Flags().Uint64Var(&flags.ramSize, "ram-size", 536870912, "size of the ram")

	return cmd
}

func run(_ *cobra.Command, _ []string, flags flags) error {
	h := http.NewServeMux()

	var rv *riscv.RISCVEmulator
	var mu sync.Mutex
	h.HandleFunc("/create", func(w http.ResponseWriter, r *http.Request) {
		mu.Lock()
		defer mu.Unlock()

		file := r.URL.Query().Get("file")
		f, err := os.OpenFile(file, os.O_RDONLY, 0)
		if err != nil {
			slog.Error("error loading file", "err", err)
			http.Error(w, "failed to open file", http.StatusInternalServerError)
			return
		}
		defer f.Close()

		rv, err = riscv.FromELF(flags.ramSize, f)
		if err != nil {
			slog.Error("error creating emulator", "error", err)
			http.Error(w, "failed to create emulator", http.StatusInternalServerError)
			return
		}
		w.Write([]byte("ok"))
	})

	h.HandleFunc("/singlestep", func(w http.ResponseWriter, r *http.Request) {
		if rv == nil {
			slog.Error("need to create emulator before singlestep")
			http.Error(w, "create the fucking emulator", http.StatusInternalServerError)
			return
		}

		count := r.URL.Query().Get("count")
		if count == "" {
			count = "1"
		}

		c, err := strconv.ParseUint(count, 0, 64)
		if err != nil {
			slog.Error("error parsing count", "error", err)
			http.Error(w, err.Error(), http.StatusInternalServerError)
			return
		}

		rc := http.NewResponseController(w)

		rv.Logf = func(s string, args ...any) {
			w.Write(fmt.Appendf(nil, s, args...))
			rc.Flush()
		}

		for range c {
			rv.PrintCurrentInstr(w)

			if err := rv.RunSingleStep(r.Context()); err != nil {
				slog.Error("error single-stepping", "error", err)
				http.Error(w, err.Error(), http.StatusInternalServerError)
				return
			}

			rv.DumpState(w)
			fmt.Fprintf(w, "\n\n")
		}

		rv.Logf = func(s string, args ...any) {}
	})

	h.HandleFunc("/dumpinstrs", func(w http.ResponseWriter, r *http.Request) {
		mu.Lock()
		defer mu.Unlock()

		if rv == nil {
			slog.Error("need to create emulator before singlestep")
			http.Error(w, "create the fucking emulator", http.StatusInternalServerError)
			return
		}

		base, err := strconv.ParseUint(r.URL.Query().Get("base"), 0, 64)
		if err != nil {
			slog.Error("error parsing base", "error", err)
			http.Error(w, err.Error(), http.StatusInternalServerError)
			return
		}

		length, err := strconv.ParseUint(r.URL.Query().Get("length"), 0, 64)
		if err != nil {
			slog.Error("error parsing length", "error", err)
			http.Error(w, err.Error(), http.StatusInternalServerError)
			return
		}

		for i := uint64(0); i < length*4; i += 4 {
			rv.DumpInstrAt(w, base+i)
		}
	})

	h.HandleFunc("/run", func(w http.ResponseWriter, r *http.Request) {
		mu.Lock()
		defer mu.Unlock()

		if rv == nil {
			slog.Error("need to create emulator before singlestep")
			http.Error(w, "create the fucking emulator", http.StatusInternalServerError)
			return
		}

		w.WriteHeader(http.StatusOK)

		rc := http.NewResponseController(w)

		rv.Logf = func(s string, args ...any) {
			w.Write(fmt.Appendf(nil, s, args...))
			rc.Flush()
		}

		if err := rv.Run(r.Context()); err != nil {
			rv.Logf("error: %v\n", err)
			return
		}

		rv.Logf = func(s string, args ...any) {}
	})

	h.HandleFunc("/state", func(w http.ResponseWriter, r *http.Request) {
		mu.Lock()
		defer mu.Unlock()

		if rv == nil {
			slog.Error("need to create emulator before singlestep")
			http.Error(w, "create the fucking emulator", http.StatusInternalServerError)
			return
		}

		rv.DumpState(w)
	})

	if err := http.ListenAndServe("localhost:8081", h); err != nil {
		return err
	}

	return nil
}
