package riscv

func bitwiseNot[T uint8 | uint16 | uint32 | uint64](bit uint8) T {
	return T(^uint(bit))
}

func abs[T int8 | int16 | int32](x T) T {
	if x < 0 {
		return -x
	}
	return x
}

func signExtend[T uint8 | uint16 | uint32 | uint64](x T, bit uint8) T {
	mask := T(1 << bit)
	if (x & mask) != 0 {
		x |= (^mask + 1) ^ mask
	}
	return x
}
