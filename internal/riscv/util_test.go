package riscv

import (
	"fmt"
	"testing"
)

func TestSignExtend(t *testing.T) {
	ir := uint32(0xfe01_0113)
	imm := ir >> 20
	expected := (imm | 0xffff_f000)
	if signExtend(imm, 11) != expected {
		fmt.Printf("%08x\n", imm)
		fmt.Printf("%08x\n", expected)

		t.Fatal("wrong sign extend")
	}
}

func TestBitwiseNot(t *testing.T) {
	t.Logf("%b", bitwiseNot[uint32](1))
}
