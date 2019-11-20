/*
 Created by hachi on 11/5/2019.

 Emulation of a bus connected to the Rockwell 6502 CPU

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


#include "bus.h"

Bus::Bus() {
    // Clear RAM contents, just in case
    for (auto &i : ram) {  // TODO: what does auto keyword mean?
        i = 0x00;
    }
}

Bus::~Bus() {}

void Bus::write(uint16_t addr, uint8_t data) {
    // Write data to a given 16bit address in memory
    if (addr >= 0x0000 && addr <= 0xFFFF) {
        ram[addr] = data;
    }
}

uint8_t Bus::read(uint16_t addr, bool bReadOnly) {
    if (addr >= 0x0000 && addr <= 0xFFFF) {
        return ram[addr];
    }
    else {
        return 0x00;
    }
}
