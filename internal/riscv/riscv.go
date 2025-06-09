package riscv

import (
	"bufio"
	"bytes"
	"encoding/binary"
	"fmt"
	"os"
	"os/signal"
	"strings"
	"time"
)

type CSR string

const (
	Pc         CSR = "pc"
	Mstatus    CSR = "mstatus"
	Cycle      CSR = "cycle"
	Timer      CSR = "timer"
	TimerMatch CSR = "timermatch"
	Mscratch   CSR = "mscratch"
	Mtvec      CSR = "mtvec"
	Mie        CSR = "mie"
	Mip        CSR = "mip"
	Mepc       CSR = "mepc"
	Mtval      CSR = "mtval"
	Mcause     CSR = "mcause"
	Extraflags CSR = "extraflags"
)

var AllCSRs = []CSR{
	Pc,
	Mstatus,
	Cycle,
	Timer,
	TimerMatch,
	Mscratch,
	Mtvec,
	Mie,
	Mip,
	Mepc,
	Mtval,
	Mcause,
	Extraflags,
}

type Register string

const (
	X0  Register = "x0"
	X1  Register = "x1"
	X2  Register = "x2"
	X3  Register = "x3"
	X4  Register = "x4"
	X5  Register = "x5"
	X6  Register = "x6"
	X7  Register = "x7"
	X8  Register = "x8"
	X9  Register = "x9"
	X10 Register = "x10"
	X11 Register = "x11"
	X12 Register = "x12"
	X13 Register = "x13"
	X14 Register = "x14"
	X15 Register = "x15"
	X16 Register = "x16"
	X17 Register = "x17"
	X18 Register = "x18"
	X19 Register = "x19"
	X20 Register = "x20"
	X21 Register = "x21"
	X22 Register = "x22"
	X23 Register = "x23"
	X24 Register = "x24"
	X25 Register = "x25"
	X26 Register = "x26"
	X27 Register = "x27"
	X28 Register = "x28"
	X29 Register = "x29"
	X30 Register = "x30"
	X31 Register = "x31"
)

var AllRegisters = []Register{
	X0,
	X1,
	X2,
	X3,
	X4,
	X5,
	X6,
	X7,
	X8,
	X9,
	X10,
	X11,
	X12,
	X13,
	X14,
	X15,
	X16,
	X17,
	X18,
	X19,
	X20,
	X21,
	X22,
	X23,
	X24,
	X25,
	X26,
	X27,
	X28,
	X29,
	X30,
	X31,
}

type Instruction struct {
	opcode uint32
	funct3 uint32
	rs1    uint32
	rs2    uint32
	csr    int64
	funct7 uint32
}

var Lui = Instruction{}

func RegisterByNum(id uint8) Register {
	if int(id) >= len(AllRegisters) {
		panic(fmt.Sprintf("unknown register %d", id))
	}

	return AllRegisters[id]
}

const RAMOffset = 0x8000_0000

type RISCVEmulator struct {
	image     []byte
	registers map[Register]uint32
	csrs      map[CSR]uint64
}

func (r *RISCVEmulator) store4LE(offset uint64, val uint32) {
	b := r.image[offset:]
	binary.LittleEndian.PutUint32(b, val)
}

func (r *RISCVEmulator) store4BE(offset uint64, val uint32) {
	b := r.image[offset:]
	binary.BigEndian.PutUint32(b, val)
}

func (r *RISCVEmulator) load4LE(offset uint64) uint32 {
	b := r.image[offset:]
	return binary.LittleEndian.Uint32(b)
}

func (r *RISCVEmulator) load4BE(offset uint64) uint32 {
	b := r.image[offset:]
	return binary.BigEndian.Uint32(b)
}

func (r *RISCVEmulator) store2LE(offset uint64, val uint16) {
	b := r.image[offset:]
	binary.LittleEndian.PutUint16(b, val)
}

func (r *RISCVEmulator) store2BE(offset uint64, val uint16) {
	b := r.image[offset:]
	binary.BigEndian.PutUint16(b, val)
}

func (r *RISCVEmulator) load2LE(offset uint64) uint16 {
	b := r.image[offset:]
	return binary.LittleEndian.Uint16(b)
}

func (r *RISCVEmulator) load2BE(offset uint64) uint16 {
	b := r.image[offset:]
	return binary.BigEndian.Uint16(b)
}

func (r *RISCVEmulator) store1(offset uint64, val uint8) {
	r.image[offset] = val
}

func (r *RISCVEmulator) load1(offset uint64) uint8 {
	return r.image[offset]
}

func (r *RISCVEmulator) storeCSR(csr CSR, val uint64) {
	r.csrs[csr] = val
}

func (r *RISCVEmulator) checkCSRBit(csr CSR, bit uint8) bool {
	v := r.loadCSR(csr)
	if (v & (1 << bit)) == 1 {
		return true
	}
	return false
}

func (r *RISCVEmulator) storeCSRBit(csr CSR, bit uint8) {
	v := r.loadCSR(csr)
	v |= (1 << bit)
	r.storeCSR(csr, v)
}

func (r *RISCVEmulator) clearCSRBit(csr CSR, bit uint8) {
	v := r.loadCSR(csr)
	v &= bitwiseNot[uint64](bit)
	r.storeCSR(csr, v)
}

func (r *RISCVEmulator) loadCSR(csr CSR) uint64 {
	return r.csrs[csr]
}

func (r *RISCVEmulator) loadReg(reg Register) uint32 {
	if reg == X0 {
		return 0
	}

	return r.registers[reg]
}

func (r *RISCVEmulator) storeReg(reg Register, val uint32) {
	r.registers[reg] = val
}

func (r *RISCVEmulator) RunSingleStep() error {
	rd := bufio.NewReader(os.Stdin)

	stepC := make(chan struct{}, 1)
	go func() {
		defer close(stepC)
		for {
			_, err := rd.ReadString('\n')
			if err != nil {
				return
			}

			stepC <- struct{}{}
		}
	}()

	intC := make(chan os.Signal, 1)
	signal.Notify(intC, os.Interrupt)

	tick := time.Now()
	for {
		select {
		case <-intC:
			return nil
		case _, ok := <-stepC:
			if !ok {
				return nil
			}

			if err := r.step(uint64(time.Now().Sub(tick).Microseconds())); err != nil {
				fmt.Printf("error: %v\n", err)
			}
			tick = time.Now()
		}
	}
}

func (r *RISCVEmulator) step(elapsed uint64) error {
	r.storeCSR(Timer, r.loadCSR(Timer)+elapsed)

	if r.loadCSR(Timer) == r.loadCSR(TimerMatch) {
		r.clearCSRBit(Extraflags, 3)
		r.storeCSRBit(Mip, 7)
	} else {
		r.clearCSRBit(Mip, 7)
	}

	pc := r.loadCSR(Pc)
	cycle := r.loadCSR(Cycle)

	if r.checkCSRBit(Extraflags, 3) {
		return nil
	}

	ofs_pc := pc - RAMOffset
	ir := r.load4LE(ofs_pc)
	instr, err := MatchInstr(ir)
	if err != nil {
		pc += 4
		r.storeCSR(Pc, pc)
		return err
	}

	r.DumpInstr(pc, ir)

	rd := AllRegisters[(ir>>7)&0b11111]
	switch instr {
	case AUIPC:
		imm := uint64(ir & (bitwiseNot[uint32](0) << 12))
		r.storeReg(rd, uint32(pc+imm))
	case JAL:
		rv := uint32(pc + 4)
		// J-type instructions are laid out like so:
		// [ 20 ] [ imm:10:1 ] [ 11 ] [ imm:19:12 ]
		//   31   30        21   20   19         12
		reladdr := ((ir & 0x8000_0000 >> 11) | ((ir & 0x7fe0_0000) >> 20) | ((ir & 0x0010_0000) >> 9) | (ir & 0x000f_f000))
		// Sign bit is always stored in the 20th bit of the immediate
		// If this is true, we need to do a sign extension
		reladdr = signExtend(reladdr, 20)
		reladdr_s := int32(reladdr)
		if reladdr_s < 0 {
			pc -= uint64(-reladdr_s)
		} else {
			pc += uint64(reladdr_s)
		}
		r.storeReg(rd, rv)
		pc -= 4
	case JALR:
		imm := uint32(ir >> 20)
		imm = signExtend(imm, 12)
		rs1 := r.loadReg(AllRegisters[((ir >> 15) & 0b11111)])
		jumpAddr := ((rs1 + imm) & bitwiseNot[uint32](1))
		r.storeReg(rd, jumpAddr+4)
		pc = uint64(jumpAddr)
		pc -= 4
	case LUI:
		imm := uint32(ir & (bitwiseNot[uint32](0) << 12))
		fmt.Printf("store %08x -> %v\n", imm, rd)
		r.storeReg(rd, imm)
	case ADDI:
		imm := int32(signExtend((ir >> 20), 12))
		rs1 := AllRegisters[((ir >> 15) & 0b11111)]
		src := r.loadReg(rs1)
		if imm < 0 {
			src -= uint32(-imm)
		} else {
			src += uint32(imm)
		}
		r.storeReg(rd, src)
	case SB, SH, SW:
		rs1 := AllRegisters[((ir >> 15) & 0b11111)]
		rs2 := AllRegisters[((ir >> 20) & 0b11111)]
		rs2v := r.loadReg(rs2)
		offset := ((ir >> 7) & 0x1f) | (((ir >> 25) & 0b1111111) << 5)
		offset_s := int32(signExtend(offset, 12))
		addr := r.loadReg(rs1)
		if offset_s < 0 {
			addr -= uint32(-offset_s)
		} else {
			addr += uint32(offset_s)
		}
		addr -= RAMOffset

		switch instr {
		case SB:
			r.store1(uint64(addr), uint8(rs2v))
		case SH:
			r.store2LE(uint64(addr), uint16(rs2v))
		case SW:
			r.store4LE(uint64(addr), rs2v)
		}
	case LW, LH, LB:
		rs1 := AllRegisters[((ir >> 15) & 0b11111)]
		imm := int32(signExtend((ir >> 20), 12))
		addr := r.loadReg(rs1)
		if imm < 0 {
			addr -= uint32(-imm)
		} else {
			addr += uint32(imm)
		}
		addr -= RAMOffset

		fmt.Printf("%x\n", addr)

		switch instr {
		case LW:
			v := r.load4LE(uint64(addr))
			r.storeReg(rd, v)
		case LH:
			v := r.load2LE(uint64(addr))
			r.storeReg(rd, signExtend(uint32(v), 15))
		case LB:
			v := r.load1(uint64(addr))
			r.storeReg(rd, signExtend(uint32(v), 8))
		}
	case CSRRW:
		rs1 := AllRegisters[((ir >> 15) & 0b11111)]
		csr := (ir >> 20) & 0b111111111111
		var rval uint64
		switch csr {
		case 0x340:
			rval = r.loadCSR(Mscratch)
		case 0x305:
			rval = r.loadCSR(Mtvec)
		case 0x138:
			addr := r.loadReg(rs1)
			ptr := addr - RAMOffset
			var str []byte
			for {
				b := r.image[ptr]
				if b == 0 {
					break
				}
				str = append(str, b)
				ptr += 1
			}
			fmt.Printf("%v\n", string(str))
		case 0x137:
			val := r.loadReg(rs1)
			fmt.Printf("%08x\n", val)
		default:
			fmt.Printf("unhandled csr write to %x (val: %x)\n", csr, r.loadReg(rs1))
		}

		r.storeReg(rd, uint32(rval))
	}

	pc += 4

	r.storeCSR(Cycle, cycle)
	r.storeCSR(Pc, pc)

	return nil
}

func (r *RISCVEmulator) DumpInstr(pc uint64, ir uint32) {
	outf := func(f string, rest ...any) {
		str := fmt.Sprintf(f, rest...)

		fmt.Printf("(pc: 0x%08x): %s\n", pc, str)
	}

	instr, err := MatchInstr(ir)
	if err != nil {
		return
	}

	rd := AllRegisters[(ir>>7)&0b11111]

	switch instr {
	case AUIPC:
		imm := uint64(ir & (bitwiseNot[uint32](0) << 12))
		outf("auipc %v, 0x%08x", rd, imm)
	case JAL:
		// J-type instructions are laid out like so:
		// [ 20 ] [ imm:10:1 ] [ 11 ] [ imm:19:12 ]
		//   31   30        21   20   19         12
		reladdr := ((ir & 0x8000_0000 >> 11) | ((ir & 0x7fe0_0000) >> 20) | ((ir & 0x0010_0000) >> 9) | (ir & 0x000f_f000))
		// Sign bit is always stored in the 20th bit of the immediate
		reladdr_s := int32(signExtend(reladdr, 20))
		pc := r.loadCSR(Pc)
		if reladdr_s < 0 {
			pc -= uint64(abs(reladdr_s))
		} else {
			pc += uint64(reladdr_s)
		}
		pc -= 4

		outf("jal %v, 0x%08x (new pc: 0x%08x)", rd, reladdr_s, pc+4)
	case JALR:
		imm := uint32(ir >> 20)
		imm = signExtend(imm, 12)
		rs1 := AllRegisters[((ir >> 15) & 0b11111)]
		outf("jalr %v, %d(%v)", rd, imm, rs1)
	case ADDI:
		imm := uint32((ir >> 20))
		imm_s := int32(signExtend(imm, 12))
		rs1 := AllRegisters[((ir >> 15) & 0b11111)]
		outf("addi %v, %v, %x", rd, rs1, imm_s)
	case SW, SB, SH:
		rs1 := AllRegisters[((ir >> 15) & 0x1f)]
		rs2 := AllRegisters[((ir >> 20) & 0x1f)]
		offset := ((ir >> 7) & 0x1f) | (((ir >> 25) & 0b1111111) << 5)
		offset_s := int32(signExtend(offset, 12))

		outf("%s %v, %d(%v)", strings.ToLower(instr.name), rs2, offset_s, rs1)
	case LUI:
		imm := uint32(ir & (bitwiseNot[uint32](0) << 12))
		outf("lui %v, %x", rd, imm)
	case LW, LH, LB:
		rs1 := AllRegisters[((ir >> 15) & 0b11111)]
		imm := int32(signExtend((ir >> 20), 12))
		outf("%v %v, %d(%v)", strings.ToLower(instr.name), rd, imm, rs1)
	case CSRRW:
		rs1 := AllRegisters[((ir >> 15) & 0b11111)]
		csr := (ir >> 20) & 0b111111111111
		outf("csrrw %v, %v, 0x%x", rd, rs1, csr)

	default:
		outf("unhandled instr: %v", instr.name)
	}
}

func (r *RISCVEmulator) DumpState() {
	for _, register := range AllRegisters {
		val := r.loadReg(register)
		fmt.Printf("%v: %x\n", register, val)
	}
}

func NewRISCVEmulator(image *bytes.Buffer) *RISCVEmulator {
	csrs := map[CSR]uint64{}
	for _, csr := range AllCSRs {
		csrs[csr] = 0
	}

	registers := map[Register]uint32{}
	for _, register := range AllRegisters {
		registers[register] = 0
	}

	csrs[Pc] = RAMOffset
	registers[X10] = 0x00
	csrs[Extraflags] |= 3

	return &RISCVEmulator{
		image:     image.Bytes(),
		csrs:      csrs,
		registers: registers,
	}
}
