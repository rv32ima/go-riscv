package riscv

import (
	"bytes"
	"context"
	"debug/elf"
	"encoding/binary"
	"errors"
	"fmt"
	"io"
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

	Trap     CSR = "trap"
	TrapRval CSR = "traprval"
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
	Trap,
	TrapRval,
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

func isMMIORange(offset uint32) bool {
	return 0x1000_0000 <= offset && offset <= 0x1200_0000
}

type LogfFunction func(s string, args ...any)

type RISCVEmulator struct {
	Logf LogfFunction

	image     []byte
	registers map[Register]uint32
	csrs      map[CSR]uint64
	poweroff  bool
	lastTick  time.Time
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
	v &= ^uint64(1 << bit)
	r.storeCSR(csr, v)
}

func (r *RISCVEmulator) loadCSR(csr CSR) uint64 {
	v := r.csrs[csr]

	switch csr {
	case Mip:
		v &= ^uint64(1 << 2)
	case Mie:
		v &= ^uint64(1 << 2)
	case Mstatus:
		v &= ^uint64(1 << 8)
		v &= ^uint64(1 << 11)
	}

	return v
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

func (r *RISCVEmulator) RunSingleStep(ctx context.Context) error {
	err := r.run(ctx, 1)
	if err != nil {
		return err
	}

	return nil
}

func (r *RISCVEmulator) Run(ctx context.Context) error {
	for rt := 0; true; rt += 1024 {
		if err := r.run(ctx, 1024); err != nil {
			return err
		}
	}

	return nil
}

var (
	trapError     = errors.New("trap")
	wfiError      = errors.New("wfi")
	poweroffError = errors.New("power off")
)

func (r *RISCVEmulator) handleTrap() {
	trap := r.loadCSR(Trap)
	rval := r.loadCSR(TrapRval)

	r.Logf("TRAP: %08x %08x, PC: %08x\n", trap, rval, r.loadCSR(Pc))

	if trap&0x80000000 == 1 {
		r.storeCSR(Mcause, uint64(trap))
		r.storeCSR(Mtval, 0)
		r.storeCSR(Pc, r.loadCSR(Pc)+4)
	} else {
		r.storeCSR(Mcause, uint64(trap-1))
		if trap > 5 && trap <= 8 {
			r.storeCSR(Mtval, uint64(rval))
		} else if trap == 5 {
			r.storeCSR(Mtval, 0)
		} else {
			r.storeCSR(Mtval, uint64(r.load4LE(r.loadCSR(Pc)-RAMOffset)))
		}
	}

	r.storeCSR(Mepc, r.loadCSR(Pc))
	r.storeCSR(Mstatus, ((r.loadCSR(Mstatus)&0x08)<<4)|((r.loadCSR(Extraflags)&3)<<11))
	r.storeCSR(Pc, r.loadCSR(Mtvec))
	r.storeCSR(Extraflags, r.loadCSR(Extraflags)|3)
	r.storeCSR(Trap, 0)
}

func (r *RISCVEmulator) processorStep() error {
	if r.lastTick.IsZero() {
		r.lastTick = time.Now()
	}
	defer func() {
		r.lastTick = time.Now()
	}()

	if r.poweroff {
		return poweroffError
	}

	elapsed := uint64(time.Now().Sub(r.lastTick).Microseconds())

	r.storeCSR(Timer, r.loadCSR(Timer)+elapsed)

	if r.loadCSR(Timer) == r.loadCSR(TimerMatch) {
		r.clearCSRBit(Extraflags, 3)
		r.storeCSRBit(Mip, 7)
	} else {
		r.clearCSRBit(Mip, 7)
	}

	// If WFI, don't run the processor
	if r.checkCSRBit(Extraflags, 3) {
		return wfiError
	}

	if r.checkCSRBit(Mip, 7) && r.checkCSRBit(Mie, 7) && r.checkCSRBit(Mstatus, 4) {
		r.storeCSR(Trap, 0x8000_0007)
		r.storeCSR(Pc, r.loadCSR(Pc)-4)
		return trapError
	}

	pc := r.loadCSR(Pc)
	ofs_pc := pc - RAMOffset

	// Access violation
	if ofs_pc >= RAMOffset {
		r.storeCSR(Trap, 1+1)
		r.storeCSR(TrapRval, uint64(0))
		return trapError
	}

	// PC-misaligned access
	if (ofs_pc & 3) == 1 {
		r.storeCSR(Trap, 1+0)
		r.storeCSR(TrapRval, uint64(0))
		return trapError
	}

	ir := r.load4LE(ofs_pc)
	instr, err := MatchInstr(ir)
	if err != nil {
		r.storeCSR(Trap, 2+1)
		r.storeCSR(TrapRval, uint64(0))
		return trapError
	}

	r.storeCSR(Cycle, r.loadCSR(Cycle)+1)

	rdid := (ir >> 7) & 0b11111
	var rval uint32
	switch instr {
	case AUIPC:
		imm := uint64(ir & (bitwiseNot[uint32](0) << 12))
		rval = uint32(pc + imm)
	case JAL:
		rval = uint32(pc + 4)
		reladdr := ((ir & 0x8000_0000 >> 11) | ((ir & 0x7fe0_0000) >> 20) | ((ir & 0x0010_0000) >> 9) | (ir & 0x000f_f000))
		reladdr_s := int32(signExtend(reladdr, 20))
		pc = add(pc, reladdr_s)
		pc -= 4
	case JALR:
		imm := signExtend(uint32(ir>>20), 12)
		rs1 := r.loadReg(AllRegisters[((ir >> 15) & 0b11111)])
		jumpAddr := ((rs1 + imm) & bitwiseNot[uint32](1))
		rval = jumpAddr + 4
		pc = uint64(jumpAddr)
		pc -= 4
	case LUI:
		imm := uint32(ir & (bitwiseNot[uint32](0) << 12))
		rval = imm
	case ADDI:
		imm := int32(signExtend((ir >> 20), 12))
		rs1 := AllRegisters[((ir >> 15) & 0b11111)]
		src := r.loadReg(rs1)
		src = add(src, imm)
		rval = src
	case SB, SH, SW:
		rdid = 0
		rs1 := AllRegisters[((ir >> 15) & 0b11111)]
		rs2 := AllRegisters[((ir >> 20) & 0b11111)]
		rs2v := r.loadReg(rs2)
		offset := ((ir >> 7) & 0x1f) | (((ir >> 25) & 0b1111111) << 5)
		offset_s := int32(signExtend(offset, 12))
		addr := r.loadReg(rs1)
		addr = add(addr, offset_s)
		addr -= RAMOffset

		if addr >= uint32(len(r.image)-3) {
			addr += RAMOffset
			if isMMIORange(addr) {
				r.handleMMIOStore(addr, rs2v)
				break
			}
			r.storeCSR(Trap, 7+1)
			r.storeCSR(TrapRval, uint64(addr))
			return trapError
		}

		if (addr & 3) != 0 {
			r.storeCSR(Trap, 6+1)
			r.storeCSR(TrapRval, uint64(0))
			return trapError
		}

		switch instr {
		case SB:
			r.store1(uint64(addr), uint8(rs2v))
		case SH:
			r.store2LE(uint64(addr), uint16(rs2v))
		case SW:
			r.store4LE(uint64(addr), rs2v)
		}

	case LW, LH, LHU, LB, LBU:
		rs1 := AllRegisters[((ir >> 15) & 0b11111)]
		imm := int32(signExtend((ir >> 20), 12))
		addr := r.loadReg(rs1)
		addr = add(addr, imm)
		addr -= RAMOffset

		if addr >= uint32(len(r.image)-3) {
			addr += RAMOffset
			if isMMIORange(addr) {
				rval = r.handleMMIOLoad(addr)
				break
			}
			r.storeCSR(Trap, 5+1)
			r.storeCSR(TrapRval, uint64(addr))
			return trapError
		}

		if (addr & 3) != 0 {
			r.storeCSR(Trap, 4+1)
			r.storeCSR(TrapRval, uint64(0))
			return trapError
		}

		switch instr {
		case LW:
			v := r.load4LE(uint64(addr))
			rval = v
		case LH:
			v := r.load2LE(uint64(addr))
			rval = signExtend(uint32(v), 15)
		case LHU:
			v := r.load2LE(uint64(addr))
			rval = uint32(v)
		case LB:
			v := r.load1(uint64(addr))
			rval = signExtend(uint32(v), 8)
		case LBU:
			v := r.load1(uint64(addr))
			rval = uint32(v)
		}

	case BEQ, BNE, BLT, BGE, BLTU, BGEU:
		rs1 := int32(r.loadReg(AllRegisters[(ir>>15)&0b11111]))
		rs2 := int32(r.loadReg(AllRegisters[(ir>>20)&0b11111]))
		imm := ((ir & 0xf00) >> 7) | ((ir & 0x7e000000) >> 20) | ((ir & 0x80) << 4) | ((ir >> 31) << 12)
		imm = add(uint32(pc), int32(signExtend(imm, 12))) - 4
		rdid = 0
		switch instr {
		case BEQ:
			if rs1 == rs2 {
				pc = uint64(imm)
			}
		case BNE:
			if rs1 != rs2 {
				pc = uint64(imm)
			}
		case BLT:
			if rs1 < rs2 {
				pc = uint64(imm)
			}
		case BGE:
			if rs1 >= rs2 {
				pc = uint64(imm)
			}
		case BLTU:
			if uint32(rs1) < uint32(rs2) {
				pc = uint64(imm)
			}
		case BGEU:
			if uint32(rs1) < uint32(rs2) {
				pc = uint64(imm)
			}
		}

	case CSRRW, CSRRS, CSRRC, CSRRWI, CSRRSI, CSRRCI:
		rs1 := ((ir >> 15) & 0b11111)
		csr := (ir >> 20) & 0b111111111111
		rval = r.handleCSRRead(csr)

		var writeval uint32
		switch instr {
		case CSRRW:
			writeval = r.loadReg(AllRegisters[rs1])
		case CSRRS:
			writeval = rval | r.loadReg(AllRegisters[rs1])
		case CSRRC:
			writeval = rval & ^r.loadReg(AllRegisters[rs1])
		case CSRRWI:
			writeval = rs1
		case CSRRSI:
			writeval = rval | rs1
		case CSRRCI:
			writeval = rval & ^rs1
		}

		r.handleCSRWrite(csr, writeval)
	case ADD, SUB, AND, OR, XOR, SLL, SRL, SLT, SLTU:
		rs1 := r.loadReg(AllRegisters[((ir >> 15) & 0b11111)])
		rs2 := r.loadReg(AllRegisters[((ir >> 20) & 0b11111)])
		switch instr {
		case ADD:
			rval = rs1 + rs2
		case SUB:
			rval = rs1 - rs2
		case AND:
			rval = rs1 & rs2
		case OR:
			rval = rs1 | rs2
		case XOR:
			rval = rs1 ^ rs2
		case SLL:
			rval = rs1 << (rs2 & 0x1F)
		case SRL:
			rval = rs1 >> (rs2 & 0x1F)
		case SRA:
			rval = uint32(int32(rs1) >> int32(rs2&0x1F))
		case SLT:
			v := int32(rs1) < int32(rs2)
			if v {
				rval = 1
			} else {
				rval = 0
			}
		case SLTU:
			v := rs1 < rs2
			if v {
				rval = 1
			} else {
				rval = 0
			}
		}
	case ADDI, ANDI, SLTIU, SLTI, ORI, XORI:
		imm := int32(signExtend(ir>>20, 12))
		rs1 := r.loadReg(AllRegisters[((ir >> 15) & 0b11111)])
		switch instr {
		case ADDI:
			rval = add(rs1, imm)
		case ANDI:
			rval = uint32(int32(rs1) & imm)
		case ORI:
			rval = uint32(int32(rs1) | imm)
		case XORI:
			rval = uint32(int32(rs1) ^ imm)
		case SLTI:
			v := int32(rs1) < imm
			if v {
				rval = 1
			} else {
				rval = 0
			}
		case SLTIU:
			if imm == 1 && rs1 == 0 {
				rval = 1
				break
			}

			v := uint32(rs1) < uint32(imm)
			if v {
				rval = 1
			} else {
				rval = 0
			}
		}
	case SLLI, SRLI, SRAI:
		rs1 := r.loadReg(AllRegisters[((ir >> 15) & 0b11111)])
		shamt := (ir >> 20) & 0b11111
		switch instr {
		case SLLI:
			rval = rs1 << shamt
		case SRLI:
			rval = rs1 >> shamt
		case SRAI:
			rval = uint32(int32(rs1) >> int32(shamt))
		}
	case MRET:
		rdid = 0
		mstatus := r.loadCSR(Mstatus)
		extraflags := r.loadCSR(Extraflags)
		r.storeCSR(Mstatus, ((mstatus&0x80)>>4)|((extraflags&3)<<11)|0x80)
		r.storeCSR(Extraflags, (extraflags & ^uint64(3))|((mstatus>>11)&3))
		pc = r.loadCSR(Mepc) - 4
	case ECALL:
		if r.loadCSR(Extraflags)&3 == 1 {
			r.storeCSR(Trap, 11+1)
		} else {
			r.storeCSR(Trap, 8+1)
		}
	case WFI:
		r.storeCSRBit(Mstatus, 4)
		r.storeCSRBit(Extraflags, 3)
		r.storeCSR(Pc, pc+4)
		return wfiError
	case EBREAK:
		r.storeCSR(Trap, 3+1)
	case MUL, MULH, MULHSU, MULHU, DIV, DIVU, REM, REMU:
		rs1 := AllRegisters[(ir>>15)&0b11111]
		rs2 := AllRegisters[(ir>>20)&0b11111]
		switch instr {
		case MUL:
			rval = r.loadReg(rs1) * r.loadReg(rs2)
		case MULH:
			rval = uint32(mulhss(int64(r.loadReg(rs1)), int64(r.loadReg(rs2))) >> 32)
		case MULHU:
			rval = uint32(mulhuu(uint64(r.loadReg(rs1)), uint64(r.loadReg(rs2))) >> 32)
		case MULHSU:
			rval = uint32(mulhsu(int64(r.loadReg(rs1)), uint64(r.loadReg(rs2))) >> 32)
		case DIV:
			rs2v := int32(r.loadReg(rs2))
			if rs2v == 0 {
				rval = 0xFFFF_FFFF
			} else {
				rval = uint32(int32(r.loadReg(rs1)) / rs2v)
			}
		case DIVU:
			rs2v := r.loadReg(rs2)
			if rs2v == 0 {
				rval = 0xFFFF_FFFF
			} else {
				rval = r.loadReg(rs1) / rs2v
			}
		case REM:
			rval = uint32(int32(r.loadReg(rs1)) % int32(r.loadReg(rs2)))
		case REMU:
			rval = r.loadReg(rs1) % r.loadReg(rs2)
		}
	case FENCE:
		rdid = 0
	}

	if r.loadCSR(Trap) != 0 {
		r.storeCSR(TrapRval, uint64(rval))
		return trapError
	}

	if rdid != 0 {
		r.storeReg(AllRegisters[rdid], rval)
	}

	r.storeCSR(Pc, pc+4)

	return nil
}

func (r *RISCVEmulator) run(ctx context.Context, instrCount uint32) error {
	t := time.NewTicker(time.Second / (1 * 1000000))
	defer t.Stop()

	var cycleCount uint32
	for {
		select {
		case <-ctx.Done():
			return fmt.Errorf("context canceled")
		case <-t.C:
			if cycleCount == instrCount {
				return nil
			}

			err := r.processorStep()
			if err == trapError {
				r.handleTrap()
				return err
			} else if err != nil {
				return err
			}

			cycleCount += 1
		}
	}
}

func (r *RISCVEmulator) handleCSRRead(csr uint32) uint32 {
	switch csr {
	case 0x340:
		return uint32(r.loadCSR(Mscratch))
	case 0x305:
		return uint32(r.loadCSR(Mtvec))
	case 0x304:
		return uint32(r.loadCSR(Mie))
	case 0xC00:
		return uint32(r.loadCSR(Cycle))
	case 0x344:
		return uint32(r.loadCSR(Mip))
	case 0x300:
		return uint32(r.loadCSR(Mstatus))
	case 0x341:
		return uint32(r.loadCSR(Mepc))
	case 0x342:
		return uint32(r.loadCSR(Mcause))
	case 0x343:
		return uint32(r.loadCSR(Mtval))
	case 0xf11:
		return 0xff0f_f0ff
	case 0x301:
		return 0x40401101
	case 0x136:
		return 0
	case 0x138:
		return 0
	default:
		r.Logf("unhandled csr read: %08x\n", csr)
		return 0
	}
}

func (r *RISCVEmulator) handleCSRWrite(csr uint32, val uint32) {
	switch csr {
	case 0x340:
		r.storeCSR(Mscratch, uint64(val))
	case 0x305:
		r.storeCSR(Mtvec, uint64(val))
	case 0x304:
		r.storeCSR(Mie, uint64(val))
	case 0x344:
		r.storeCSR(Mip, uint64(val))
	case 0x341:
		r.storeCSR(Mepc, uint64(val))
	case 0x300:
		r.storeCSR(Mstatus, uint64(val))
	case 0x342:
		r.storeCSR(Mcause, uint64(val))
	case 0x343:
		r.storeCSR(Mtval, uint64(val))
	case 0x138:
		ptr := val - RAMOffset
		var str []byte
		for {
			b := r.image[ptr]
			if b == 0 {
				break
			}
			str = append(str, b)
			ptr += 1
		}
		r.Logf("%v", string(str))
	case 0x137:
		r.Logf("%08x", val)
	case 0x136:
		r.Logf("%d", val)
	default:
		r.Logf("unhandled csr write: %08x -> %08x\n", csr, val)
	}
}

func (r *RISCVEmulator) handleMMIOStore(addr uint32, val uint32) {
	switch addr {
	case 0x1110_0000:
		if val == 0x5555 {
			r.poweroff = true
		}
	}
}

func (r *RISCVEmulator) handleMMIOLoad(addr uint32) uint32 {
	switch addr {
	case 0x1100bffc:
		return uint32(r.loadCSR(Timer) >> 32)
	case 0x1100bff8:
		return uint32(r.loadCSR(Timer))
	default:
		fmt.Printf("unhandled MMIO load from %08x\n", addr)
	}
	return 0
}

func (r *RISCVEmulator) PrintCurrentInstr(w io.Writer) {
	pc := r.loadCSR(Pc)
	r.DumpInstrAt(w, pc)
}

func (r *RISCVEmulator) DumpInstrAt(w io.Writer, pc uint64) {
	outf := func(f string, rest ...any) {
		str := fmt.Sprintf(f, rest...)

		fmt.Fprintf(w, "(pc: 0x%08x): %s\n", pc, str)
	}

	ofs_pc := pc - RAMOffset
	if ofs_pc > uint64(len(r.image)) {
		outf("out of range memory address (trap 1+1)")
		return
	}

	ir := r.load4LE(ofs_pc)
	instr, err := MatchInstr(ir)
	if err != nil {
		outf("invalid instruction: %v", err)
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
		pc := add(pc, reladdr_s)
		pc -= 4

		outf("jal %v, 0x%08x", rd, pc+4)
	case JALR:
		imm := uint32(ir >> 20)
		imm = signExtend(imm, 12)
		rs1 := AllRegisters[((ir >> 15) & 0b11111)]
		outf("jalr %v, %d(%v)", rd, imm, rs1)
	case ADDI, ANDI, SLTIU, SLTI, ORI, XORI:
		imm := uint32((ir >> 20))
		imm_s := int32(signExtend(imm, 12))
		rs1 := AllRegisters[((ir >> 15) & 0b11111)]
		outf("%s %v, %v, 0x%x", strings.ToLower(instr.name), rd, rs1, imm_s)
	case SW, SB, SH:
		rs1 := AllRegisters[((ir >> 15) & 0x1f)]
		rs2 := AllRegisters[((ir >> 20) & 0x1f)]
		offset := ((ir >> 7) & 0x1f) | (((ir >> 25) & 0b1111111) << 5)
		offset_s := int32(signExtend(offset, 12))

		outf("%s %v, %d(%v)", strings.ToLower(instr.name), rs2, offset_s, rs1)
	case LUI:
		imm := uint32(ir & (bitwiseNot[uint32](0) << 12))
		outf("lui %v, 0x%x", rd, imm)
	case LW, LH, LHU, LB, LBU:
		rs1 := AllRegisters[((ir >> 15) & 0b11111)]
		imm := int32(signExtend((ir >> 20), 11))
		outf("%v %v, 0x%x(%v)", strings.ToLower(instr.name), rd, imm, rs1)
	case CSRRW, CSRRC, CSRRS, CSRRCI, CSRRSI, CSRRWI:
		rs1 := ((ir >> 15) & 0b11111)
		csr := (ir >> 20) & 0b111111111111
		switch instr {
		case CSRRW, CSRRC, CSRRS:
			rs1reg := AllRegisters[rs1]
			outf("%s %v, %v, 0x%x", strings.ToLower(instr.name), rd, rs1reg, csr)
		case CSRRCI, CSRRSI, CSRRWI:
			outf("%s %v, %v, 0x%x", strings.ToLower(instr.name), rd, rs1, csr)
		}
	case BEQ, BNE, BLT, BGE, BLTU, BGEU:
		rs1 := AllRegisters[(ir>>15)&0b11111]
		rs2 := AllRegisters[(ir>>20)&0b11111]
		imm := ((ir & 0xf00) >> 7) | ((ir & 0x7e000000) >> 20) | ((ir & 0x80) << 4) | ((ir >> 31) << 12)
		imm_s := int32(signExtend(imm, 12))
		outf("%s %v, %v, 0x%x", strings.ToLower(instr.name), rs1, rs2, imm_s)
	case ADD, SUB, AND, OR, XOR, SLL, SRL, SLT, SLTU:
		rs1 := AllRegisters[((ir >> 15) & 0b11111)]
		rs2 := AllRegisters[((ir >> 20) & 0b11111)]
		outf("%s %v, %v, %v", strings.ToLower(instr.name), rd, rs1, rs2)
	case ADDI, ANDI, SLTIU, SLTI, ORI, XORI:
		imm := int32(signExtend(ir>>20, 12))
		rs1 := AllRegisters[((ir >> 15) & 0b11111)]
		outf("%s %v, 0x%x(%v)", strings.ToLower(instr.name), rd, imm, rs1)
	case MUL, MULH, MULHSU, MULHU, DIV, DIVU, REM, REMU:
		rs1 := AllRegisters[((ir >> 15) & 0b11111)]
		rs2 := AllRegisters[((ir >> 20) & 0b11111)]
		outf("%s %v, %v, %v", strings.ToLower(instr.name), rd, rs1, rs2)
	case SLLI, SRLI, SRAI:
		rs1 := AllRegisters[((ir >> 15) & 0b11111)]
		shamt := (ir >> 20) & 0b11111
		outf("%s %v, %v, 0x%x", strings.ToLower(instr.name), rd, rs1, shamt)
	case FENCE:
		outf("fence")
	case ECALL:
		outf("ecall")
	case MRET:
		outf("mret")
	default:
		outf("unhandled instr: %v", instr.name)
	}
}

func (r *RISCVEmulator) DumpState(w io.Writer) {
	for _, register := range AllRegisters {
		val := r.loadReg(register)
		fmt.Fprintf(w, "%v: %x ", register, val)
	}
	fmt.Fprintf(w, "\n")
	for _, csr := range AllCSRs {
		val := r.loadCSR(csr)
		fmt.Fprintf(w, "%v: %x ", csr, val)
	}
	fmt.Fprintf(w, "\n")
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
		Logf: func(s string, args ...any) {
			fmt.Printf(s, args...)
		},
		image:     image.Bytes(),
		csrs:      csrs,
		registers: registers,
	}
}

func FromELF(ramSize uint64, rd io.ReaderAt) (*RISCVEmulator, error) {
	if ramSize == 0 {
		ramSize = 536870912
	}

	ram := bytes.NewBuffer(nil)
	if _, err := ram.Write(make([]byte, ramSize)); err != nil {
		return nil, err
	}

	e, err := elf.NewFile(rd)
	if err != nil {
		return nil, err
	}

	for _, sect := range e.Progs {
		if sect.Type != elf.PT_LOAD {
			continue
		}

		data := make([]byte, sect.Filesz)
		if _, err := rd.ReadAt(data, int64(sect.Off)); err != nil {
			return nil, err
		}

		addr := sect.Paddr - RAMOffset
		copy(ram.Bytes()[addr:addr+sect.Filesz+1], data)
	}

	rv := NewRISCVEmulator(ram)
	return rv, nil
}
