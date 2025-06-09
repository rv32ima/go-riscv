package riscv

import "testing"

func TestBasicMemoryOps(t *testing.T) {
	e := RISCVEmulator{
		image: make([]byte, 32),
	}

	magic := uint32(0xDEAD_BEEF)

	e.store4BE(4, magic)
	if e.load4BE(4) != magic {
		t.Fatal("unexpected magic")
	}

	e.store4BE(4, 0)
	if e.load4BE(4) != 0 {
		t.Fatal("hmmmm")
	}

	e.store4LE(4, magic)
	if e.load4LE(4) != magic {
		t.Fatal("unexpected")
	}

	if e.load4BE(8) != 0 {
		t.Fatal("spilled over")
	}

	for i := range 4 {
		e.store1(uint64(4+i), uint8(0xFE))
	}

	if e.load4LE(4) != 0xFEFEFEFE {
		t.Fatalf("didn't work, got %x instead", e.load4LE(4))
	}

	for i := range 4 {
		if e.load1(uint64(4+i)) != 0xFE {
			t.Fatal("didn't work")
		}
	}
}
