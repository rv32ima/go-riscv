package riscv

import (
	"fmt"
	"testing"
)

func TestSignExtend(t *testing.T) {
	ir := uint32(0xffe)
	expected := int32(-2)
	if int32(signExtend(ir, 11)) != expected {
		fmt.Printf("%08x\n", ir)
		fmt.Printf("%08x\n", uint32(expected))

		t.Fatal("wrong sign extend")
	}
}

func TestBitwiseNot(t *testing.T) {
	t.Logf("%b", bitwiseNot[uint32](1))
}
