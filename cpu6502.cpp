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


#include "cpu6502.h"


/***********************
 * CLASS INITIALIZATION
 ***********************/
cpu6502::cpu6502() {
    // Assembles the translation table.

    // It is 16x16 entries. This gives 256 instructions. It is arranged to that the bottom
    // 4 bits of the instruction choose the column, and the top 4 bits choose the row.

    // The table is one big initialiser list of initialiser lists...

    // Entry format: { Mnemonic, Function pointer to function implementing operation, Function pointer to address mode, number of clock cycles}
    // Any entry marked as "???" refers to a blank space in the 6502's instruction set data sheet
    // TODO: find out what the &a entries are and if there's a way to make them more descriptive
    using a = olc6502;
    lookup = {
            { "BRK", &a::BRK, &a::IMM, 7 },{ "ORA", &a::ORA, &a::IZX, 6 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "???", &a::NOP, &a::IMP, 3 },{ "ORA", &a::ORA, &a::ZP0, 3 },{ "ASL", &a::ASL, &a::ZP0, 5 },{ "???", &a::XXX, &a::IMP, 5 },{ "PHP", &a::PHP, &a::IMP, 3 },{ "ORA", &a::ORA, &a::IMM, 2 },{ "ASL", &a::ASL, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::NOP, &a::IMP, 4 },{ "ORA", &a::ORA, &a::ABS, 4 },{ "ASL", &a::ASL, &a::ABS, 6 },{ "???", &a::XXX, &a::IMP, 6 },
            { "BPL", &a::BPL, &a::REL, 2 },{ "ORA", &a::ORA, &a::IZY, 5 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "???", &a::NOP, &a::IMP, 4 },{ "ORA", &a::ORA, &a::ZPX, 4 },{ "ASL", &a::ASL, &a::ZPX, 6 },{ "???", &a::XXX, &a::IMP, 6 },{ "CLC", &a::CLC, &a::IMP, 2 },{ "ORA", &a::ORA, &a::ABY, 4 },{ "???", &a::NOP, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 7 },{ "???", &a::NOP, &a::IMP, 4 },{ "ORA", &a::ORA, &a::ABX, 4 },{ "ASL", &a::ASL, &a::ABX, 7 },{ "???", &a::XXX, &a::IMP, 7 },
            { "JSR", &a::JSR, &a::ABS, 6 },{ "AND", &a::AND, &a::IZX, 6 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "BIT", &a::BIT, &a::ZP0, 3 },{ "AND", &a::AND, &a::ZP0, 3 },{ "ROL", &a::ROL, &a::ZP0, 5 },{ "???", &a::XXX, &a::IMP, 5 },{ "PLP", &a::PLP, &a::IMP, 4 },{ "AND", &a::AND, &a::IMM, 2 },{ "ROL", &a::ROL, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 2 },{ "BIT", &a::BIT, &a::ABS, 4 },{ "AND", &a::AND, &a::ABS, 4 },{ "ROL", &a::ROL, &a::ABS, 6 },{ "???", &a::XXX, &a::IMP, 6 },
            { "BMI", &a::BMI, &a::REL, 2 },{ "AND", &a::AND, &a::IZY, 5 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "???", &a::NOP, &a::IMP, 4 },{ "AND", &a::AND, &a::ZPX, 4 },{ "ROL", &a::ROL, &a::ZPX, 6 },{ "???", &a::XXX, &a::IMP, 6 },{ "SEC", &a::SEC, &a::IMP, 2 },{ "AND", &a::AND, &a::ABY, 4 },{ "???", &a::NOP, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 7 },{ "???", &a::NOP, &a::IMP, 4 },{ "AND", &a::AND, &a::ABX, 4 },{ "ROL", &a::ROL, &a::ABX, 7 },{ "???", &a::XXX, &a::IMP, 7 },
            { "RTI", &a::RTI, &a::IMP, 6 },{ "EOR", &a::EOR, &a::IZX, 6 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "???", &a::NOP, &a::IMP, 3 },{ "EOR", &a::EOR, &a::ZP0, 3 },{ "LSR", &a::LSR, &a::ZP0, 5 },{ "???", &a::XXX, &a::IMP, 5 },{ "PHA", &a::PHA, &a::IMP, 3 },{ "EOR", &a::EOR, &a::IMM, 2 },{ "LSR", &a::LSR, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 2 },{ "JMP", &a::JMP, &a::ABS, 3 },{ "EOR", &a::EOR, &a::ABS, 4 },{ "LSR", &a::LSR, &a::ABS, 6 },{ "???", &a::XXX, &a::IMP, 6 },
            { "BVC", &a::BVC, &a::REL, 2 },{ "EOR", &a::EOR, &a::IZY, 5 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "???", &a::NOP, &a::IMP, 4 },{ "EOR", &a::EOR, &a::ZPX, 4 },{ "LSR", &a::LSR, &a::ZPX, 6 },{ "???", &a::XXX, &a::IMP, 6 },{ "CLI", &a::CLI, &a::IMP, 2 },{ "EOR", &a::EOR, &a::ABY, 4 },{ "???", &a::NOP, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 7 },{ "???", &a::NOP, &a::IMP, 4 },{ "EOR", &a::EOR, &a::ABX, 4 },{ "LSR", &a::LSR, &a::ABX, 7 },{ "???", &a::XXX, &a::IMP, 7 },
            { "RTS", &a::RTS, &a::IMP, 6 },{ "ADC", &a::ADC, &a::IZX, 6 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "???", &a::NOP, &a::IMP, 3 },{ "ADC", &a::ADC, &a::ZP0, 3 },{ "ROR", &a::ROR, &a::ZP0, 5 },{ "???", &a::XXX, &a::IMP, 5 },{ "PLA", &a::PLA, &a::IMP, 4 },{ "ADC", &a::ADC, &a::IMM, 2 },{ "ROR", &a::ROR, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 2 },{ "JMP", &a::JMP, &a::IND, 5 },{ "ADC", &a::ADC, &a::ABS, 4 },{ "ROR", &a::ROR, &a::ABS, 6 },{ "???", &a::XXX, &a::IMP, 6 },
            { "BVS", &a::BVS, &a::REL, 2 },{ "ADC", &a::ADC, &a::IZY, 5 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "???", &a::NOP, &a::IMP, 4 },{ "ADC", &a::ADC, &a::ZPX, 4 },{ "ROR", &a::ROR, &a::ZPX, 6 },{ "???", &a::XXX, &a::IMP, 6 },{ "SEI", &a::SEI, &a::IMP, 2 },{ "ADC", &a::ADC, &a::ABY, 4 },{ "???", &a::NOP, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 7 },{ "???", &a::NOP, &a::IMP, 4 },{ "ADC", &a::ADC, &a::ABX, 4 },{ "ROR", &a::ROR, &a::ABX, 7 },{ "???", &a::XXX, &a::IMP, 7 },
            { "???", &a::NOP, &a::IMP, 2 },{ "STA", &a::STA, &a::IZX, 6 },{ "???", &a::NOP, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 6 },{ "STY", &a::STY, &a::ZP0, 3 },{ "STA", &a::STA, &a::ZP0, 3 },{ "STX", &a::STX, &a::ZP0, 3 },{ "???", &a::XXX, &a::IMP, 3 },{ "DEY", &a::DEY, &a::IMP, 2 },{ "???", &a::NOP, &a::IMP, 2 },{ "TXA", &a::TXA, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 2 },{ "STY", &a::STY, &a::ABS, 4 },{ "STA", &a::STA, &a::ABS, 4 },{ "STX", &a::STX, &a::ABS, 4 },{ "???", &a::XXX, &a::IMP, 4 },
            { "BCC", &a::BCC, &a::REL, 2 },{ "STA", &a::STA, &a::IZY, 6 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 6 },{ "STY", &a::STY, &a::ZPX, 4 },{ "STA", &a::STA, &a::ZPX, 4 },{ "STX", &a::STX, &a::ZPY, 4 },{ "???", &a::XXX, &a::IMP, 4 },{ "TYA", &a::TYA, &a::IMP, 2 },{ "STA", &a::STA, &a::ABY, 5 },{ "TXS", &a::TXS, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 5 },{ "???", &a::NOP, &a::IMP, 5 },{ "STA", &a::STA, &a::ABX, 5 },{ "???", &a::XXX, &a::IMP, 5 },{ "???", &a::XXX, &a::IMP, 5 },
            { "LDY", &a::LDY, &a::IMM, 2 },{ "LDA", &a::LDA, &a::IZX, 6 },{ "LDX", &a::LDX, &a::IMM, 2 },{ "???", &a::XXX, &a::IMP, 6 },{ "LDY", &a::LDY, &a::ZP0, 3 },{ "LDA", &a::LDA, &a::ZP0, 3 },{ "LDX", &a::LDX, &a::ZP0, 3 },{ "???", &a::XXX, &a::IMP, 3 },{ "TAY", &a::TAY, &a::IMP, 2 },{ "LDA", &a::LDA, &a::IMM, 2 },{ "TAX", &a::TAX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 2 },{ "LDY", &a::LDY, &a::ABS, 4 },{ "LDA", &a::LDA, &a::ABS, 4 },{ "LDX", &a::LDX, &a::ABS, 4 },{ "???", &a::XXX, &a::IMP, 4 },
            { "BCS", &a::BCS, &a::REL, 2 },{ "LDA", &a::LDA, &a::IZY, 5 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 5 },{ "LDY", &a::LDY, &a::ZPX, 4 },{ "LDA", &a::LDA, &a::ZPX, 4 },{ "LDX", &a::LDX, &a::ZPY, 4 },{ "???", &a::XXX, &a::IMP, 4 },{ "CLV", &a::CLV, &a::IMP, 2 },{ "LDA", &a::LDA, &a::ABY, 4 },{ "TSX", &a::TSX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 4 },{ "LDY", &a::LDY, &a::ABX, 4 },{ "LDA", &a::LDA, &a::ABX, 4 },{ "LDX", &a::LDX, &a::ABY, 4 },{ "???", &a::XXX, &a::IMP, 4 },
            { "CPY", &a::CPY, &a::IMM, 2 },{ "CMP", &a::CMP, &a::IZX, 6 },{ "???", &a::NOP, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "CPY", &a::CPY, &a::ZP0, 3 },{ "CMP", &a::CMP, &a::ZP0, 3 },{ "DEC", &a::DEC, &a::ZP0, 5 },{ "???", &a::XXX, &a::IMP, 5 },{ "INY", &a::INY, &a::IMP, 2 },{ "CMP", &a::CMP, &a::IMM, 2 },{ "DEX", &a::DEX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 2 },{ "CPY", &a::CPY, &a::ABS, 4 },{ "CMP", &a::CMP, &a::ABS, 4 },{ "DEC", &a::DEC, &a::ABS, 6 },{ "???", &a::XXX, &a::IMP, 6 },
            { "BNE", &a::BNE, &a::REL, 2 },{ "CMP", &a::CMP, &a::IZY, 5 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "???", &a::NOP, &a::IMP, 4 },{ "CMP", &a::CMP, &a::ZPX, 4 },{ "DEC", &a::DEC, &a::ZPX, 6 },{ "???", &a::XXX, &a::IMP, 6 },{ "CLD", &a::CLD, &a::IMP, 2 },{ "CMP", &a::CMP, &a::ABY, 4 },{ "NOP", &a::NOP, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 7 },{ "???", &a::NOP, &a::IMP, 4 },{ "CMP", &a::CMP, &a::ABX, 4 },{ "DEC", &a::DEC, &a::ABX, 7 },{ "???", &a::XXX, &a::IMP, 7 },
            { "CPX", &a::CPX, &a::IMM, 2 },{ "SBC", &a::SBC, &a::IZX, 6 },{ "???", &a::NOP, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "CPX", &a::CPX, &a::ZP0, 3 },{ "SBC", &a::SBC, &a::ZP0, 3 },{ "INC", &a::INC, &a::ZP0, 5 },{ "???", &a::XXX, &a::IMP, 5 },{ "INX", &a::INX, &a::IMP, 2 },{ "SBC", &a::SBC, &a::IMM, 2 },{ "NOP", &a::NOP, &a::IMP, 2 },{ "???", &a::SBC, &a::IMP, 2 },{ "CPX", &a::CPX, &a::ABS, 4 },{ "SBC", &a::SBC, &a::ABS, 4 },{ "INC", &a::INC, &a::ABS, 6 },{ "???", &a::XXX, &a::IMP, 6 },
            { "BEQ", &a::BEQ, &a::REL, 2 },{ "SBC", &a::SBC, &a::IZY, 5 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "???", &a::NOP, &a::IMP, 4 },{ "SBC", &a::SBC, &a::ZPX, 4 },{ "INC", &a::INC, &a::ZPX, 6 },{ "???", &a::XXX, &a::IMP, 6 },{ "SED", &a::SED, &a::IMP, 2 },{ "SBC", &a::SBC, &a::ABY, 4 },{ "NOP", &a::NOP, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 7 },{ "???", &a::NOP, &a::IMP, 4 },{ "SBC", &a::SBC, &a::ABX, 4 },{ "INC", &a::INC, &a::ABX, 7 },{ "???", &a::XXX, &a::IMP, 7 },
        };
}

cpu6502::~cpu6502() {}

/************************
 * ADDRESSING MODES
 ************************/
uint8_t cpu6502::IMP() {
    fetched = a;
    return 0;
}

uint8_t cpu6502::IMM() {
    addr_abs = pc++;
    return 0;
}

uint8_t cpu6502::ZP0() {
    return 0;
}

uint8_t cpu6502::ZPX() {
    return 0;
}

uint8_t cpu6502::ZPY() {
    return 0;
}

uint8_t cpu6502::REL() {
    return 0;
}

uint8_t cpu6502::ABS() {
    return 0;
}

uint8_t cpu6502::ABX() {
    return 0;
}

uint8_t cpu6502::ABY() {
    return 0;
}

uint8_t cpu6502::IND() {
    return 0;
}

uint8_t cpu6502::IZX() {
    return 0;
}

uint8_t cpu6502::IZY() {
    return 0;
}

/***********************
 * OPCODES
 ***********************/
// Add Memory to Accumulator with Carry
uint8_t cpu6502::ADC() {
    return 0;
}

// 'AND' Memory with Accumulator
uint8_t cpu6502::AND() {
    return 0;
}

// Shift Left One Bit (Memory or Accumulator)
uint8_t cpu6502::ASL() {
    return 0;
}

// Branch on Carry Clear
uint8_t cpu6502::BCC() {
    return 0;
}

// Branch on Carry Set
uint8_t cpu6502::BCS() {
    return 0;
}

// Branch on Result Zero
uint8_t cpu6502::BEQ() {
    return 0;
}

// Test Bits in Memory with Accumulator
uint8_t cpu6502::BIT() {
    return 0;
}

// Branch on Return Minus
uint8_t cpu6502::BMI() {
    return 0;
}

// Branch on Return Not Zero
uint8_t cpu6502::BNE() {
    return 0;
}

// Branch on Result Plus
uint8_t cpu6502::BPL() {
    return 0;
}

// Force Break
uint8_t cpu6502::BRK() {
    return 0;
}

// Branch on Overflow Clear
uint8_t cpu6502::BVC() {
    return 0;
}

// Branch on Overflow Set
uint8_t cpu6502::BVS() {
    return 0;
}

// Clear Carry Flag
uint8_t cpu6502::CLC() {
    return 0;
}

// Clear Decimal Mode
uint8_t cpu6502::CLD() {
    return 0;
}

// Clear Interrupt Disable Bit
uint8_t cpu6502::CLI() {
    return 0;
}

// Clear Overflow Flag
uint8_t cpu6502::CLV() {
    return 0;
}

// Compare Memory and Accumulator
uint8_t cpu6502::CMP() {
    return 0;
}

// Compare Memory and Index X
uint8_t cpu6502::CPX() {
    return 0;
}

// Compare Memory and Index Y
uint8_t cpu6502::CPY() {
    return 0;
}

// Decrement Memory by One
uint8_t cpu6502::DEC() {
    return 0;
}

// Decrement Index X by One
uint8_t cpu6502::DEX() {
    return 0;
}

// Decrement Index Y by One
uint8_t cpu6502::DEY() {
    return 0;
}

// "Exclusive-OR" Memory with Accumulator
uint8_t cpu6502::EOR() {
    return 0;
}

// Increment Memory by One
uint8_t cpu6502::INC() {
    return 0;
}

// Increment Index X by One
uint8_t cpu6502::INX() {
    return 0;
}

// Increment Index Y by One
uint8_t cpu6502::INY() {
    return 0;
}

// Jump to New Location
uint8_t cpu6502::JMP() {
    return 0;
}

// Jump to New Location Saving Return Address
uint8_t cpu6502::JSR() {
    return 0;
}

// Load Accumulator with Memory
uint8_t cpu6502::LDA() {
    return 0;
}

// Load Index X with Memory
uint8_t cpu6502::LDX() {
    return 0;
}

// Load Index Y with Memory
uint8_t cpu6502::LDY() {
    return 0;
}

// Shift One Bit Right (Memory or Accumulator)
uint8_t cpu6502::LSR() {
    return 0;
}

// No Operation
uint8_t cpu6502::NOP() {
    return 0;
}

// "OR" Memory with Accumulator
uint8_t cpu6502::ORA() {
    return 0;
}

// Push Accumulator on Stack
uint8_t cpu6502::PHA() {
    return 0;
}

// Push Processor Status on Stack
uint8_t cpu6502::PHP() {
    return 0;
}

// Pull Accumulator from Stack
uint8_t cpu6502::PLA() {
    return 0;
}

// Pull Processor Status from Stack
uint8_t cpu6502::PLP() {
    return 0;
}

// Rotate One Bit Left (Memory or Accumulator)
uint8_t cpu6502::ROL() {
    return 0;
}

// Rotate One Bit Right (Memory or Accumulator)
uint8_t cpu6502::ROR() {
    return 0;
}

// Return to Interrupt
uint8_t cpu6502::RTI() {
    return 0;
}

// Return to Subroutine
uint8_t cpu6502::RTS() {
    return 0;
}

// Subtract Memory from Accumulator with Borrow
uint8_t cpu6502::SBC() {
    return 0;
}

// Set Carry Flag
uint8_t cpu6502::SEC() {
    return 0;
}

// Set Decimal Mode
uint8_t cpu6502::SED() {
    return 0;
}

// Set Interrupt Disable Status
uint8_t cpu6502::SEI() {
    return 0;
}

// Store Accumulator in Memory
uint8_t cpu6502::STA() {
    return 0;
}

// Store Index X in Memory
uint8_t cpu6502::STX() {
    return 0;
}

// Store Index Y in Memory
uint8_t cpu6502::STY() {
    return 0;
}

// Transfer Accumulator to Index X
uint8_t cpu6502::TAX() {
    return 0;
}

// Transfer Accumulator to Index Y
uint8_t cpu6502::TAY() {
    return 0;
}

// Transfer Stack Pointer to Index X
uint8_t cpu6502::TSX() {
    return 0;
}

// Transfer Index X to Accumulator
uint8_t cpu6502::TXA() {
    return 0;
}

// Transfer Index X to Stack Register
uint8_t cpu6502::TXS() {
    return 0;
}

// Transfer Index Y to Accumulator
uint8_t cpu6502::TYA() {
    return 0;
}


/***********************
 * HELPER FUNCTIONS
 ***********************/
uint8_t cpu6502::XXX() {
    return 0;
}

void cpu6502::clock() {
    if (cycles == 0) {
        // Get opcode at current program counter location and increment the program counter
        opcode = read(pc);
        pc++;

        // Get starting number of cycles for the instruction
        cycles = lookup[opcode].cycles;

        // Call the address mode and operation for the requested opcode
        uint8_t  additional_cycles1 = (this -> *lookup[opcode].addrmode)();
        uint8_t  additional_cycles2 = (this -> *lookup[opcode].operate)();

        // If either the address mode or opcode indicate they need additional clock cycles to operate, add them
        cycles += (additional_cycles1 & additional_cycles2);
    }
}

void cpu6502::reset() {

}

void cpu6502::irq() {

}

void cpu6502::nmi() {

}

uint8_t cpu6502::fetch() {
    return 0;
}


// Bus read/write
uint8_t cpu6502::read(uint16_t address) {
    return bus->read(address, false);
}

void cpu6502::write(uint16_t address, uint8_t data) {
    return bus->write(address, data);
}

// Convenience functions to access status registers
uint8_t cpu6502::GetFlag(cpu6502::FLAGS6502 flag) {
    return 0;
}

void cpu6502::SetFlag(cpu6502::FLAGS6502 flag, bool value) {
    if (value) {
        status |= flag;
    }
    else {
        status &= ~flag;    // FIXME: Clang-tidy: use of a signed integer operand with a bitwise binary operand
    }
}
