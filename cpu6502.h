/*
 Created by hachi on 11/5/2019.

 Emulation of the Rockwell 6502 CPU

 Code based on javid9x's implementation of the 6502 for the One Lone Coder NES Emulator

 Since it's taking code, whole or in part, from OLC implementation, here's a license:

 License (OLC-3)
	~~~~~~~~~~~~~~~
	Copyright 2018-2019 OneLoneCoder.com
	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:
	1. Redistributions or derivations of source code must retain the above
	copyright notice, this list of conditions and the following disclaimer.
	2. Redistributions or derivative works in binary form must reproduce
	the above copyright notice. This list of conditions and the following
	disclaimer must be reproduced in the documentation and/or other
	materials provided with the distribution.
	3. Neither the name of the copyright holder nor the names of its
	contributors may be used to endorse or promote products derived
	from this software without specific prior written permission.
	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
	A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
	HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
	SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
	LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
	DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
	THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
	OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */


#ifndef NESDEV_CPU6502_H
#define NESDEV_CPU6502_H

#include <vector>
#include "bus.h"

class cpu6502 {
public:
    cpu6502();
    ~cpu6502();

public:
    enum FLAGS6502 {
        C = (1 << 0),       // Carry bit - 1 = True
        Z = (1 << 1),       // Zero - 1 = Result Zero
        I = (1 << 2),       // Disable Interrupts - 1 = Disable
        D = (1 << 3),       // Decimal Mode (currently unused) - 1 = True
        B = (1 << 4),       // Break - 1 = BRK
        U = (1 << 5),       // Unused
        V = (1 << 6),       // Overflow - 1 = True
        M = (1 << 7)        // Negative - 1 = Negative
    };

    uint8_t a = 0x00;       // Accumulator Register
    uint8_t x = 0x00;       // X Register
    uint8_t y = 0x00;       // Y Register
    uint8_t stkp = 0x00;    // Stack Pointer (points to a location on the bus) - TODO: prepended with a 1?
    uint16_t pc = 0x0000;   // Program Counter
    uint8_t status = 0x00;  // Status Register

    void ConnectBus(Bus *n) {
        bus = n;
    }

    // Addressing Modes
    uint8_t IMP();  uint8_t IMM();
    uint8_t ZP0();  uint8_t ZPX();
    uint8_t ZPY();  uint8_t REL();
    uint8_t ABS();  uint8_t ABX();
    uint8_t ABY();  uint8_t IND();
    uint8_t IZX();  uint8_t IZY();

    // Opcodes
    uint8_t ADC();  uint8_t AND();  uint8_t ASL();  uint8_t BCC();
    uint8_t BCS();  uint8_t BEQ();  uint8_t BIT();  uint8_t BMI();
    uint8_t BNE();  uint8_t BPL();  uint8_t BRK();  uint8_t BVC();
    uint8_t BVS();  uint8_t CLC();  uint8_t CLD();  uint8_t CLI();
    uint8_t CLV();  uint8_t CMP();  uint8_t CPX();  uint8_t CPY();
    uint8_t DEC();  uint8_t DEX();  uint8_t DEY();  uint8_t EOR();
    uint8_t INC();  uint8_t INX();  uint8_t INY();  uint8_t JMP();
    uint8_t JSR();  uint8_t LDA();  uint8_t LDX();  uint8_t LDY();
    uint8_t LSR();  uint8_t NOP();  uint8_t ORA();  uint8_t PHA();
    uint8_t PHP();  uint8_t PLA();  uint8_t PLP();  uint8_t ROL();
    uint8_t ROR();  uint8_t RTI();  uint8_t RTS();  uint8_t SBC();
    uint8_t SEC();  uint8_t SED();  uint8_t SEI();  uint8_t STA();
    uint8_t STX();  uint8_t STY();  uint8_t TAX();  uint8_t TAY();
    uint8_t TSX();  uint8_t TXA();  uint8_t TXS();  uint8_t TYA();

    uint8_t XXX();  // Illegal Opcode

    void clock();   // Indicate to the CPU that we want one clock cycle to occur
    void reset();   // Reset signal
    void irq();     // Interrupt request signal - can be ignored if the interrupt flag is set
    void nmi();     // Non-maskable Interrupt request signal - can never be ignored

    uint8_t fetch();
    uint8_t fetchedData = 0x00;

    uint16_t addr_abs = 0x0000; // Absolute location in memory
    uint16_t addr_rel = 0x0000; // Relative address for branches
    uint8_t opcode = 0x00;      // Current opcode
    uint8_t cycles = 0;         // Current number of clock cycles

private:
    Bus *bus = nullptr;
    uint8_t read(uint16_t address);
    void write (uint16_t address, uint8_t data);

    // Convenience functions to access status registers
    uint8_t GetFlag(FLAGS6502 flag);
    void SetFlag(FLAGS6502 flag, bool value);

    struct INSTRUCTION
    {
        std::string name;                               // Mnemonic for the instruction
        uint8_t(cpu6502::*operate)(void) = nullptr;     // Function pointer to operation to be performed
        uint8_t(cpu6502::*addrmode)(void) = nullptr;    // Function pointer to address mode
        uint8_t cycles = 0;                             // Number of clock cycles needed for the instruction to operate
    };

    std::vector<INSTRUCTION> lookup;
};

#endif //NESDEV_CPU6502_H
