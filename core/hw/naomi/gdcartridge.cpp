/*
 * gdcartridge.cpp
 *
 *  Created on: Nov 16, 2018
 *      Author: flyinghead
 *
 * From mame naomigd.cpp
 * // license:BSD-3-Clause
 * // copyright-holders:Olivier Galibert
 *
 */

#include "gdcartridge.h"
#include "stdclass.h"
#include "emulator.h"
#include "oslib/storage.h"
#include "naomi_regs.h"
#include "hw/holly/sb.h"
#include "hw/holly/holly_intc.h"
#include "hw/mem/addrspace.h"
#include "serialize.h"
#include "hw/sh4/sh4_sched.h"
#include "naomi.h"
#include <algorithm>

/*

  GPIO pins(main board: EEPROM, DIMM SPDs, option board: PIC16, JPs)
   |
  SH4 <-> 315-6154 <-> PCI bus -> Sega 315-6322 -> Host system interface (NAOMI, Triforce, Chihiro)
   |         |                                  -> 2x DIMM RAM modules
  RAM       RAM                -> Altera (PCI IDE Bus Master Controller) -> IDE bus -> GD-ROM or CF
 16MB       4MB                -> PCnet-FAST III -> Ethernet

315-6154 - SH4 CPU to PCI bridge and SDRAM controller, also used in Sega Hikaru (2x)
315-6322 - DIMM SDRAM controller, DES decryption, host system communication

 SH4 address space
-------------------
00000000 - 001FFFFF Flash ROM (1st half - stock firmware, 2nd half - updated firmware)
04000000 - 040000FF memory/PCI bridge registers (Sega 315-6154)
0C000000 - 0CFFFFFF SH4 local RAM
10000000 - 103FFFFF memory/PCI controller RAM
14000000 - 1BFFFFFF 8x banked pages

internal / PCI memory space
-------------------
00000000 - 000000FF DIMM controller registers (Sega 315-6322)
10000000 - 4FFFFFFF DIMM memory, upto 1GB (if register 28 bit 1 is 0, otherwise some unknown MMIO)
70000000 - 70FFFFFF SH4 local RAM
78000000 - 783FFFFF 315-6154 PCI bridge RAM
C00001xx   IDE registers                 \
C00003xx   IDE registers                  | software configured in VxWorks, preconfigured or hardcoded in 1.02
C000CCxx   IDE Bus Master DMA registers  /
C1xxxxxx   Network registers

PCI configuration space (enabled using memctl 1C reg)
-------------------
00000000 - 00000FFF unknown, write 142 to reg 04 at very start
00001000 - 00001FFF PCI IDE controller (upper board Altera Flex) Vendor 11db Device 189d
00002000 - 00002FFF AMD AM79C973BVC PCnet-FAST III Network

DIMM controller registers
-------------------
14 5F703C |
18 5F7040 |
1C 5F7044 | 16bit  4x Communication registers
20 5F7048 |
24 5F704C   16bit  Interrupt register
                   -------c ---b---a
                    a - IRQ to DIMM (SH4 IRL3): 0 set / 1 clear
                    b - unk, mask of a ???
                    c - IRQ to NAOMI (HOLLY EXT 3): 0 set / 1 clear (write 0 from NAOMI seems ignored)

28          16bit  dddd---c ------ba
                    a - 0->1 NAOMI reset
                    b - 1 seems disable DIMM RAM access, followed by write 01010101 to bank 10 offset 000110 or 000190 (some MMIO?)
                    c - unk, set to 1 in VxWorks, 0 in 1.02
                    d - unk, checked for == 1 in 1.02

2A           8bit  possible DES decryption area size 8 MSB bits (16MB units number)
                   VxWorks firmwares set this to ((DIMMsize >> 24) - 1), 1.02 set it to FF

2C          32bit  SDRAM config
30          32bit  DES key low
34          32bit  DES key high

SH4 IO port A bits
-------------------
9 select input, 0 - main/lower board, 1 - option/upper board (IDE, Net, PIC)
     0             1
0 DIMM SPD clk   JP? 0 - enable IDE
1 DIMM SPD data  JP? 0 - enable Network
2 93C46 DI       PIC16 D0
3 93C46 CS       PIC16 D1
4 93C46 CLK      PIC16 D2
5 93C46 DO       PIC16 CLK



    Dimm board communication registers software level usage:

    Name:                   Naomi   Dimm Bd.
    NAOMI_DIMM_COMMAND    = 5f703c  14000014 (16 bit):
        if bits all 1 no dimm board present and other registers not used
        bit 15: during an interrupt is 1 if the dimm board has a command to be executed
        bit 14-9: 6 bit command number (naomi bios understands 0 1 3 4 5 6 8 9 a)
        bit 7-0: higher 8 bits of 24 bit offset parameter
    NAOMI_DIMM_OFFSETL    = 5f7040  14000018 (16 bit):
        bit 15-0: lower 16 bits of 24 bit offset parameter
    NAOMI_DIMM_PARAMETERL = 5f7044  1400001c (16 bit)
    NAOMI_DIMM_PARAMETERH = 5f7048  14000020 (16 bit)
    NAOMI_DIMM_STATUS     = 5f704c  14000024 (16 bit):
        bit 0: when 0 signal interrupt from naomi to dimm board
        bit 8: when 0 signal interrupt from dimm board to naomi

*/

const u32 GDCartridge::DES_LEFTSWAP[] = {
	0x00000000, 0x00000001, 0x00000100, 0x00000101, 0x00010000, 0x00010001, 0x00010100, 0x00010101,
	0x01000000, 0x01000001, 0x01000100, 0x01000101, 0x01010000, 0x01010001, 0x01010100, 0x01010101
};

const u32 GDCartridge::DES_RIGHTSWAP[] = {
	0x00000000, 0x01000000, 0x00010000, 0x01010000, 0x00000100, 0x01000100, 0x00010100, 0x01010100,
	0x00000001, 0x01000001, 0x00010001, 0x01010001, 0x00000101, 0x01000101, 0x00010101, 0x01010101,
};

const u32 GDCartridge::DES_SBOX1[] = {
	0x00808200, 0x00000000, 0x00008000, 0x00808202, 0x00808002, 0x00008202, 0x00000002, 0x00008000,
	0x00000200, 0x00808200, 0x00808202, 0x00000200, 0x00800202, 0x00808002, 0x00800000, 0x00000002,
	0x00000202, 0x00800200, 0x00800200, 0x00008200, 0x00008200, 0x00808000, 0x00808000, 0x00800202,
	0x00008002, 0x00800002, 0x00800002, 0x00008002, 0x00000000, 0x00000202, 0x00008202, 0x00800000,
	0x00008000, 0x00808202, 0x00000002, 0x00808000, 0x00808200, 0x00800000, 0x00800000, 0x00000200,
	0x00808002, 0x00008000, 0x00008200, 0x00800002, 0x00000200, 0x00000002, 0x00800202, 0x00008202,
	0x00808202, 0x00008002, 0x00808000, 0x00800202, 0x00800002, 0x00000202, 0x00008202, 0x00808200,
	0x00000202, 0x00800200, 0x00800200, 0x00000000, 0x00008002, 0x00008200, 0x00000000, 0x00808002
};

const u32 GDCartridge::DES_SBOX2[] = {
	0x40084010, 0x40004000, 0x00004000, 0x00084010, 0x00080000, 0x00000010, 0x40080010, 0x40004010,
	0x40000010, 0x40084010, 0x40084000, 0x40000000, 0x40004000, 0x00080000, 0x00000010, 0x40080010,
	0x00084000, 0x00080010, 0x40004010, 0x00000000, 0x40000000, 0x00004000, 0x00084010, 0x40080000,
	0x00080010, 0x40000010, 0x00000000, 0x00084000, 0x00004010, 0x40084000, 0x40080000, 0x00004010,
	0x00000000, 0x00084010, 0x40080010, 0x00080000, 0x40004010, 0x40080000, 0x40084000, 0x00004000,
	0x40080000, 0x40004000, 0x00000010, 0x40084010, 0x00084010, 0x00000010, 0x00004000, 0x40000000,
	0x00004010, 0x40084000, 0x00080000, 0x40000010, 0x00080010, 0x40004010, 0x40000010, 0x00080010,
	0x00084000, 0x00000000, 0x40004000, 0x00004010, 0x40000000, 0x40080010, 0x40084010, 0x00084000
};

const u32 GDCartridge::DES_SBOX3[] = {
	0x00000104, 0x04010100, 0x00000000, 0x04010004, 0x04000100, 0x00000000, 0x00010104, 0x04000100,
	0x00010004, 0x04000004, 0x04000004, 0x00010000, 0x04010104, 0x00010004, 0x04010000, 0x00000104,
	0x04000000, 0x00000004, 0x04010100, 0x00000100, 0x00010100, 0x04010000, 0x04010004, 0x00010104,
	0x04000104, 0x00010100, 0x00010000, 0x04000104, 0x00000004, 0x04010104, 0x00000100, 0x04000000,
	0x04010100, 0x04000000, 0x00010004, 0x00000104, 0x00010000, 0x04010100, 0x04000100, 0x00000000,
	0x00000100, 0x00010004, 0x04010104, 0x04000100, 0x04000004, 0x00000100, 0x00000000, 0x04010004,
	0x04000104, 0x00010000, 0x04000000, 0x04010104, 0x00000004, 0x00010104, 0x00010100, 0x04000004,
	0x04010000, 0x04000104, 0x00000104, 0x04010000, 0x00010104, 0x00000004, 0x04010004, 0x00010100
};

const u32 GDCartridge::DES_SBOX4[] = {
	0x80401000, 0x80001040, 0x80001040, 0x00000040, 0x00401040, 0x80400040, 0x80400000, 0x80001000,
	0x00000000, 0x00401000, 0x00401000, 0x80401040, 0x80000040, 0x00000000, 0x00400040, 0x80400000,
	0x80000000, 0x00001000, 0x00400000, 0x80401000, 0x00000040, 0x00400000, 0x80001000, 0x00001040,
	0x80400040, 0x80000000, 0x00001040, 0x00400040, 0x00001000, 0x00401040, 0x80401040, 0x80000040,
	0x00400040, 0x80400000, 0x00401000, 0x80401040, 0x80000040, 0x00000000, 0x00000000, 0x00401000,
	0x00001040, 0x00400040, 0x80400040, 0x80000000, 0x80401000, 0x80001040, 0x80001040, 0x00000040,
	0x80401040, 0x80000040, 0x80000000, 0x00001000, 0x80400000, 0x80001000, 0x00401040, 0x80400040,
	0x80001000, 0x00001040, 0x00400000, 0x80401000, 0x00000040, 0x00400000, 0x00001000, 0x00401040
};

const u32 GDCartridge::DES_SBOX5[] = {
	0x00000080, 0x01040080, 0x01040000, 0x21000080, 0x00040000, 0x00000080, 0x20000000, 0x01040000,
	0x20040080, 0x00040000, 0x01000080, 0x20040080, 0x21000080, 0x21040000, 0x00040080, 0x20000000,
	0x01000000, 0x20040000, 0x20040000, 0x00000000, 0x20000080, 0x21040080, 0x21040080, 0x01000080,
	0x21040000, 0x20000080, 0x00000000, 0x21000000, 0x01040080, 0x01000000, 0x21000000, 0x00040080,
	0x00040000, 0x21000080, 0x00000080, 0x01000000, 0x20000000, 0x01040000, 0x21000080, 0x20040080,
	0x01000080, 0x20000000, 0x21040000, 0x01040080, 0x20040080, 0x00000080, 0x01000000, 0x21040000,
	0x21040080, 0x00040080, 0x21000000, 0x21040080, 0x01040000, 0x00000000, 0x20040000, 0x21000000,
	0x00040080, 0x01000080, 0x20000080, 0x00040000, 0x00000000, 0x20040000, 0x01040080, 0x20000080
};

const u32 GDCartridge::DES_SBOX6[] = {
	0x10000008, 0x10200000, 0x00002000, 0x10202008, 0x10200000, 0x00000008, 0x10202008, 0x00200000,
	0x10002000, 0x00202008, 0x00200000, 0x10000008, 0x00200008, 0x10002000, 0x10000000, 0x00002008,
	0x00000000, 0x00200008, 0x10002008, 0x00002000, 0x00202000, 0x10002008, 0x00000008, 0x10200008,
	0x10200008, 0x00000000, 0x00202008, 0x10202000, 0x00002008, 0x00202000, 0x10202000, 0x10000000,
	0x10002000, 0x00000008, 0x10200008, 0x00202000, 0x10202008, 0x00200000, 0x00002008, 0x10000008,
	0x00200000, 0x10002000, 0x10000000, 0x00002008, 0x10000008, 0x10202008, 0x00202000, 0x10200000,
	0x00202008, 0x10202000, 0x00000000, 0x10200008, 0x00000008, 0x00002000, 0x10200000, 0x00202008,
	0x00002000, 0x00200008, 0x10002008, 0x00000000, 0x10202000, 0x10000000, 0x00200008, 0x10002008
};

const u32 GDCartridge::DES_SBOX7[] = {
	0x00100000, 0x02100001, 0x02000401, 0x00000000, 0x00000400, 0x02000401, 0x00100401, 0x02100400,
	0x02100401, 0x00100000, 0x00000000, 0x02000001, 0x00000001, 0x02000000, 0x02100001, 0x00000401,
	0x02000400, 0x00100401, 0x00100001, 0x02000400, 0x02000001, 0x02100000, 0x02100400, 0x00100001,
	0x02100000, 0x00000400, 0x00000401, 0x02100401, 0x00100400, 0x00000001, 0x02000000, 0x00100400,
	0x02000000, 0x00100400, 0x00100000, 0x02000401, 0x02000401, 0x02100001, 0x02100001, 0x00000001,
	0x00100001, 0x02000000, 0x02000400, 0x00100000, 0x02100400, 0x00000401, 0x00100401, 0x02100400,
	0x00000401, 0x02000001, 0x02100401, 0x02100000, 0x00100400, 0x00000000, 0x00000001, 0x02100401,
	0x00000000, 0x00100401, 0x02100000, 0x00000400, 0x02000001, 0x02000400, 0x00000400, 0x00100001
};

const u32 GDCartridge::DES_SBOX8[] = {
	0x08000820, 0x00000800, 0x00020000, 0x08020820, 0x08000000, 0x08000820, 0x00000020, 0x08000000,
	0x00020020, 0x08020000, 0x08020820, 0x00020800, 0x08020800, 0x00020820, 0x00000800, 0x00000020,
	0x08020000, 0x08000020, 0x08000800, 0x00000820, 0x00020800, 0x00020020, 0x08020020, 0x08020800,
	0x00000820, 0x00000000, 0x00000000, 0x08020020, 0x08000020, 0x08000800, 0x00020820, 0x00020000,
	0x00020820, 0x00020000, 0x08020800, 0x00000800, 0x00000020, 0x08020020, 0x00000800, 0x00020820,
	0x08000800, 0x00000020, 0x08000020, 0x08020000, 0x08020020, 0x08000000, 0x00020000, 0x08000820,
	0x00000000, 0x08020820, 0x00020020, 0x08000020, 0x08020000, 0x08000800, 0x08000820, 0x00000000,
	0x08020820, 0x00020800, 0x00020800, 0x00000820, 0x00000820, 0x00020020, 0x08000000, 0x08020800
};

const u32 GDCartridge::DES_MASK_TABLE[] = {
	0x24000000, 0x10000000, 0x08000000, 0x02080000, 0x01000000,
	0x00200000, 0x00100000, 0x00040000, 0x00020000, 0x00010000,
	0x00002000, 0x00001000, 0x00000800, 0x00000400, 0x00000200,
	0x00000100, 0x00000020, 0x00000010, 0x00000008, 0x00000004,
	0x00000002, 0x00000001, 0x20000000, 0x10000000, 0x08000000,
	0x04000000, 0x02000000, 0x01000000, 0x00200000, 0x00100000,
	0x00080000, 0x00040000, 0x00020000, 0x00010000, 0x00002000,
	0x00001000, 0x00000808, 0x00000400, 0x00000200, 0x00000100,
	0x00000020, 0x00000011, 0x00000004, 0x00000002
};

const u8 GDCartridge::DES_ROTATE_TABLE[16] = {
	1, 1, 2, 2, 2, 2, 2, 2, 1, 2, 2, 2, 2, 2, 2, 1
};

void GDCartridge::permutate(u32 &a, u32 &b, u32 m, int shift)
{
	u32 temp = ((a>>shift) ^ b) & m;
	a ^= temp<<shift;
	b ^= temp;
}

void GDCartridge::des_generate_subkeys(const u64 key, u32 *subkeys)
{
	u32 l = key >> 32;
	u32 r = key;

	permutate(r, l, 0x0f0f0f0f, 4);
	permutate(r, l, 0x10101010, 0);

	l = (DES_LEFTSWAP[(l >> 0)  & 0xf] << 3) |
		(DES_LEFTSWAP[(l >> 8)  & 0xf] << 2) |
		(DES_LEFTSWAP[(l >> 16) & 0xf] << 1) |
		(DES_LEFTSWAP[(l >> 24) & 0xf] << 0) |
		(DES_LEFTSWAP[(l >> 5)  & 0xf] << 7) |
		(DES_LEFTSWAP[(l >> 13) & 0xf] << 6) |
		(DES_LEFTSWAP[(l >> 21) & 0xf] << 5) |
		(DES_LEFTSWAP[(l >> 29) & 0xf] << 4);

	r = (DES_RIGHTSWAP[(r >> 1)  & 0xf] << 3) |
		(DES_RIGHTSWAP[(r >> 9)  & 0xf] << 2) |
		(DES_RIGHTSWAP[(r >> 17) & 0xf] << 1) |
		(DES_RIGHTSWAP[(r >> 25) & 0xf] << 0) |
		(DES_RIGHTSWAP[(r >> 4)  & 0xf] << 7) |
		(DES_RIGHTSWAP[(r >> 12) & 0xf] << 6) |
		(DES_RIGHTSWAP[(r >> 20) & 0xf] << 5) |
		(DES_RIGHTSWAP[(r >> 28) & 0xf] << 4);

	l &= 0x0fffffff;
	r &= 0x0fffffff;


	for(int round = 0; round < 16; round++) {
		l = ((l << DES_ROTATE_TABLE[round]) | (l >> (28 - DES_ROTATE_TABLE[round]))) & 0x0fffffff;
		r = ((r << DES_ROTATE_TABLE[round]) | (r >> (28 - DES_ROTATE_TABLE[round]))) & 0x0fffffff;

		subkeys[round*2] =
			((l << 4)  & DES_MASK_TABLE[0]) |
			((l << 28) & DES_MASK_TABLE[1]) |
			((l << 14) & DES_MASK_TABLE[2]) |
			((l << 18) & DES_MASK_TABLE[3]) |
			((l << 6)  & DES_MASK_TABLE[4]) |
			((l << 9)  & DES_MASK_TABLE[5]) |
			((l >> 1)  & DES_MASK_TABLE[6]) |
			((l << 10) & DES_MASK_TABLE[7]) |
			((l << 2)  & DES_MASK_TABLE[8]) |
			((l >> 10) & DES_MASK_TABLE[9]) |
			((r >> 13) & DES_MASK_TABLE[10])|
			((r >> 4)  & DES_MASK_TABLE[11])|
			((r << 6)  & DES_MASK_TABLE[12])|
			((r >> 1)  & DES_MASK_TABLE[13])|
			((r >> 14) & DES_MASK_TABLE[14])|
			((r >> 0)  & DES_MASK_TABLE[15])|
			((r >> 5)  & DES_MASK_TABLE[16])|
			((r >> 10) & DES_MASK_TABLE[17])|
			((r >> 3)  & DES_MASK_TABLE[18])|
			((r >> 18) & DES_MASK_TABLE[19])|
			((r >> 26) & DES_MASK_TABLE[20])|
			((r >> 24) & DES_MASK_TABLE[21]);

		subkeys[round*2+1] =
			((l << 15) & DES_MASK_TABLE[22])|
			((l << 17) & DES_MASK_TABLE[23])|
			((l << 10) & DES_MASK_TABLE[24])|
			((l << 22) & DES_MASK_TABLE[25])|
			((l >> 2)  & DES_MASK_TABLE[26])|
			((l << 1)  & DES_MASK_TABLE[27])|
			((l << 16) & DES_MASK_TABLE[28])|
			((l << 11) & DES_MASK_TABLE[29])|
			((l << 3)  & DES_MASK_TABLE[30])|
			((l >> 6)  & DES_MASK_TABLE[31])|
			((l << 15) & DES_MASK_TABLE[32])|
			((l >> 4)  & DES_MASK_TABLE[33])|
			((r >> 2)  & DES_MASK_TABLE[34])|
			((r << 8)  & DES_MASK_TABLE[35])|
			((r >> 14) & DES_MASK_TABLE[36])|
			((r >> 9)  & DES_MASK_TABLE[37])|
			((r >> 0)  & DES_MASK_TABLE[38])|
			((r << 7)  & DES_MASK_TABLE[39])|
			((r >> 7)  & DES_MASK_TABLE[40])|
			((r >> 3)  & DES_MASK_TABLE[41])|
			((r << 2)  & DES_MASK_TABLE[42])|
			((r >> 21) & DES_MASK_TABLE[43]);
	}
}

template<bool decrypt>
u64 GDCartridge::des_encrypt_decrypt(u64 src, const u32 *des_subkeys)
{
	u32 r = (src & 0x00000000ffffffffULL) >> 0;
	u32 l = (src & 0xffffffff00000000ULL) >> 32;

	permutate(l, r, 0x0f0f0f0f, 4);
	permutate(l, r, 0x0000ffff, 16);
	permutate(r, l, 0x33333333, 2);
	permutate(r, l, 0x00ff00ff, 8);
	permutate(l, r, 0x55555555, 1);

	int subkey;
	if constexpr (decrypt)
		subkey = 30;
	else
		subkey = 0;

	for(int i = 0; i < 32 ; i+=4) {
		u32 temp;

		temp = ((r<<1) | (r>>31)) ^ des_subkeys[subkey];
		l ^= DES_SBOX8[ (temp>>0)  & 0x3f ];
		l ^= DES_SBOX6[ (temp>>8)  & 0x3f ];
		l ^= DES_SBOX4[ (temp>>16) & 0x3f ];
		l ^= DES_SBOX2[ (temp>>24) & 0x3f ];
		subkey++;

		temp = ((r>>3) | (r<<29)) ^ des_subkeys[subkey];
		l ^= DES_SBOX7[ (temp>>0)  & 0x3f ];
		l ^= DES_SBOX5[ (temp>>8)  & 0x3f ];
		l ^= DES_SBOX3[ (temp>>16) & 0x3f ];
		l ^= DES_SBOX1[ (temp>>24) & 0x3f ];
		subkey++;
		if constexpr (decrypt)
			subkey -= 4;

		temp = ((l<<1) | (l>>31)) ^ des_subkeys[subkey];
		r ^= DES_SBOX8[ (temp>>0)  & 0x3f ];
		r ^= DES_SBOX6[ (temp>>8)  & 0x3f ];
		r ^= DES_SBOX4[ (temp>>16) & 0x3f ];
		r ^= DES_SBOX2[ (temp>>24) & 0x3f ];
		subkey++;

		temp = ((l>>3) | (l<<29)) ^ des_subkeys[subkey];
		r ^= DES_SBOX7[ (temp>>0)  & 0x3f ];
		r ^= DES_SBOX5[ (temp>>8)  & 0x3f ];
		r ^= DES_SBOX3[ (temp>>16) & 0x3f ];
		r ^= DES_SBOX1[ (temp>>24) & 0x3f ];
		subkey++;
		if constexpr (decrypt)
			subkey -= 4;
	}

	permutate(r, l, 0x55555555, 1);
	permutate(l, r, 0x00ff00ff, 8);
	permutate(l, r, 0x33333333, 2);
	permutate(r, l, 0x0000ffff, 16);
	permutate(r, l, 0x0f0f0f0f, 4);

	return (u64(r) << 32) | u64(l);
}

u64 GDCartridge::rev64(u64 src)
{
	u64 ret;

	ret = ((src & 0x00000000000000ffULL) << 56)
		| ((src & 0x000000000000ff00ULL) << 40)
		| ((src & 0x0000000000ff0000ULL) << 24)
		| ((src & 0x00000000ff000000ULL) << 8 )
		| ((src & 0x000000ff00000000ULL) >> 8 )
		| ((src & 0x0000ff0000000000ULL) >> 24)
		| ((src & 0x00ff000000000000ULL) >> 40)
		| ((src & 0xff00000000000000ULL) >> 56);

	return ret;
}

void GDCartridge::find_file(const char *name, const u8 *dir_sector, u32 &file_start, u32 &file_size)
{
	file_start = 0;
	file_size = 0;
	DEBUG_LOG(NAOMI, "Looking for file [%s]", name);
	for (u32 pos = 0; pos < 2048 && dir_sector[pos] != 0; pos += dir_sector[pos])
	{
		if (dir_sector[pos + 25] & 2)
			continue;

		char fname[FILENAME_LENGTH + 1] {};
		const int len = std::min<int>(dir_sector[pos + 32], FILENAME_LENGTH);
		for (int i = 0; i < len; i++)
		{
			u8 c = dir_sector[pos + 33 + i];
			if (c == ';')
				break;
			fname[i] = c;
		}
		//printf("file: [%s]\n", fname);
		bool found = false;
		if (name[0] == '*')
		{
			char *p = strchr(fname, name[1]);
			if (p != nullptr && !strcmp(p, &name[1]))
				found = true;
		}
		else
		{
			found = strcmp(fname, name) == 0;
		}
		if (found)
		{
			// start sector and size of file
			file_start = *(u32 *)&dir_sector[pos + 2];
			file_size = *(u32 *)&dir_sector[pos + 10];

			DEBUG_LOG(NAOMI, "start %d size %d", file_start, file_size);
			return;
		}
	}
}

void GDCartridge::read_gdrom(Disc *gdrom, u32 sector, u8* dst, u32 count, LoadProgress *progress)
{
	gdrom->ReadSectors(sector + 150, count, dst, 2048, false, progress);
}

void GDCartridge::device_start(LoadProgress *progress, std::vector<u8> *digest)
{
	if (dimm_data != NULL)
	{
		free(dimm_data);
		dimm_data = NULL;
	}
	dimm_data_size = 0;
	loadedSegments.clear();

	char name[128];
	memset(name,'\0',128);

	u64 key;
	u8 netpic = 0;

	const u8 *picdata = this->RomPtr;

	if (RomSize > 0 && gdrom_name != NULL)
	{
		if (RomSize >= 0x4000) {
			DEBUG_LOG(NAOMI, "Real PIC binary found");
			for(int i=0;i<7;i++)
				name[i] = picdata[0x7c0+i*2];
			for(int i=0;i<7;i++)
				name[i+7] = picdata[0x7e0+i*2];

			key = 0;
			for(int i=0;i<7;i++)
				key |= u64(picdata[0x780+i*2]) << (56 - i*8);

			key |= picdata[0x7a0];

			netpic = picdata[0x6ee]; // TODO dragntr[2] seem to prefer a 0 here
		} else {
			// use extracted pic data
			//printf("This PIC key hasn't been converted to a proper PIC binary yet!\n");
			memcpy(name, picdata+33, 7);
			memcpy(name+7, picdata+25, 7);

			key =((u64(picdata[0x31]) << 56) |
					(u64(picdata[0x32]) << 48) |
					(u64(picdata[0x33]) << 40) |
					(u64(picdata[0x34]) << 32) |
					(u64(picdata[0x35]) << 24) |
					(u64(picdata[0x36]) << 16) |
					(u64(picdata[0x37]) << 8)  |
					(u64(picdata[0x29]) << 0));
		}

		DEBUG_LOG(NAOMI, "key is %08x%08x, name is %s", (u32)(key >> 32), (u32)key, name);

		u8 buffer[2048];
		std::string parent = hostfs::storage().getParentPath(settings.content.path);
		std::string gdrom_path = get_file_basename(settings.content.fileName) + "/" + gdrom_name;
		try {
			gdrom_path = hostfs::storage().getSubPath(parent, gdrom_path);
			gdrom = std::unique_ptr<Disc>(OpenDisc(gdrom_path + ".chd", digest));
		}
		catch (const FlycastException& e)
		{
			WARN_LOG(NAOMI, "Opening chd failed: %s", e.what());
			if (gdrom_parent_name != nullptr)
			{
				try {
					std::string gdrom_parent_path = hostfs::storage().getSubPath(parent, std::string(gdrom_parent_name) + "/" + gdrom_name);
					gdrom = std::unique_ptr<Disc>(OpenDisc(gdrom_parent_path + ".chd", digest));
				} catch (const FlycastException& e) {
					WARN_LOG(NAOMI, "Opening parent chd failed: %s", e.what());
				}
			}
			if (gdrom == nullptr)
				throw NaomiCartException("Naomi GDROM: Cannot open " + gdrom_path + ".chd");
		}

		// primary volume descriptor
		// read frame 0xb06e (frame=sector+150)
		// dimm board firmware starts straight from this frame
		read_gdrom(gdrom.get(), (netpic ? 0 : 45000) + 16, buffer);
		u32 path_table = ((buffer[0x8c+0] << 0) |
								(buffer[0x8c+1] << 8) |
								(buffer[0x8c+2] << 16) |
								(buffer[0x8c+3] << 24));
		// path table
		read_gdrom(gdrom.get(), path_table, buffer);

		// directory
		u8 dir_sector[2048];
		// find data of file
		u32 file_size;

		if (netpic == 0) {
			u32 dir = ((buffer[0x2 + 0] << 0) |
				(buffer[0x2 + 1] << 8) |
				(buffer[0x2 + 2] << 16) |
				(buffer[0x2 + 3] << 24));

			read_gdrom(gdrom.get(), dir, dir_sector);
			find_file(name, dir_sector, file_start, file_size);

			if (file_start && (file_size == 0x100)) {
				// read file
				read_gdrom(gdrom.get(), file_start, buffer);
				// get "rom" file name
				memset(name, '\0', 128);
				memcpy(name, buffer + 0xc0, FILENAME_LENGTH - 1);
			}
		} else {
			bool found = false;
			u32 i = 0;
			while (i < 2048 && buffer[i] != 0)
			{
				if (buffer[i] == 3 && buffer[i + 8] == 'R' && buffer[i + 9] == 'O' && buffer[i + 10] == 'M')    // find ROM dir
				{
					u32 dir = ((buffer[i + 2] << 0) |
						(buffer[i + 3] << 8) |
						(buffer[i + 4] << 16) |
						(buffer[i + 5] << 24));
					memcpy(name, "ROM.BIN", 7);
					read_gdrom(gdrom.get(), dir, dir_sector);
					found = true;
					break;
				}
				i += buffer[i] + 8 + (buffer[i] & 1);
			}
			if (!found)
			{
				u32 dir = *(u32 *)&buffer[2];
				read_gdrom(gdrom.get(), dir, dir_sector);
			}
		}

		find_file(name, dir_sector, file_start, file_size);
		if (file_start == 0)
			// mj1: filename in the pic is incorrect, probably because the disk isn't supposed to be run like this?
			// so grab the first .BIN and load it.
			find_file("*.BIN", dir_sector, file_start, file_size);

		if (file_start != 0)
		{
			u32 file_rounded_size = (file_size + 2047) & ~2048;
			for (dimm_data_size = 4096; dimm_data_size < file_rounded_size; dimm_data_size <<= 1)
				;
			dimm_data = (u8 *)malloc(dimm_data_size);
			if (dimm_data == nullptr)
				throw NaomiCartException("Memory allocation failed");
			if (dimm_data_size != file_rounded_size)
				memset(dimm_data + file_rounded_size, 0, dimm_data_size - file_rounded_size);

			loadedSegments.resize(dimm_data_size / SEGMENT_SIZE);
			std::fill(loadedSegments.begin() + (file_rounded_size + SEGMENT_SIZE - 1) / SEGMENT_SIZE,
					loadedSegments.end(), true);

			des_generate_subkeys(rev64(key), des_subkeys);
		}

		if (!dimm_data)
			throw NaomiCartException("Naomi GDROM: Could not find the file to decrypt.");
	}
}

void GDCartridge::loadSegments(u32 offset, u32 size)
{
	const u32 lastSegment = (offset + size - 1) / SEGMENT_SIZE;
	for (u32 segment = offset / SEGMENT_SIZE; segment <= lastSegment; segment++)
	{
		if (loadedSegments[segment])
			continue;
		DEBUG_LOG(NAOMI, "Loading segment %d", segment);
		// load data
		read_gdrom(gdrom.get(), file_start + (segment * SEGMENT_SIZE) / 2048,
				dimm_data + segment * SEGMENT_SIZE,
				SEGMENT_SIZE / 2048,
				nullptr);
		// decrypt loaded data
		u64 *pData = (u64 *)(dimm_data + segment * SEGMENT_SIZE);
		for (u32 i = 0; i < SEGMENT_SIZE; i += 8, pData++)
			*pData = des_encrypt_decrypt<true>(*pData, des_subkeys);
		loadedSegments[segment] = true;
	}
}

void GDCartridge::device_reset()
{
	dimm_cur_address = 0;
}
void *GDCartridge::GetDmaPtr(u32 &size)
{
	if (dimm_data == NULL)
	{
		size = 0;
		return NULL;
	}
	dimm_cur_address = DmaOffset & (dimm_data_size-1);
	size = std::min(size, dimm_data_size - dimm_cur_address);
	loadSegments(dimm_cur_address, size);
	return dimm_data + dimm_cur_address;
}

bool GDCartridge::Read(u32 offset, u32 size, void *dst)
{
	if (dimm_data == NULL)
	{
		*(u32 *)dst = 0;
		return true;
	}
	u32 addr = offset & (dimm_data_size - 1);
	size = std::min(size, dimm_data_size - addr);
	loadSegments(addr, size);
	memcpy(dst, &dimm_data[addr], size);
	return true;
}

GDCartridge::GDCartridge(u32 size) : NaomiCartridge(size)
{
	schedId = sh4_sched_register(0, [](int tag, int sch_cycl, int jitter, void *arg){
		return ((GDCartridge *)arg)->schedCallback();
	}, this);
}

GDCartridge::~GDCartridge()
{
	free(dimm_data);
	sh4_sched_unregister(schedId);
}

u32 GDCartridge::ReadMem(u32 address, u32 size)
{
	switch (address)
	{
	case NAOMI_DIMM_COMMAND:
		DEBUG_LOG(NAOMI, "DIMM COMMAND read -> %x", dimm_command);
		return dimm_command;
	case NAOMI_DIMM_OFFSETL:
		DEBUG_LOG(NAOMI, "DIMM OFFSETL read -> %x", dimm_offsetl);
		return dimm_offsetl;
	case NAOMI_DIMM_PARAMETERL:
		DEBUG_LOG(NAOMI, "DIMM PARAMETERL read -> %x", dimm_parameterl);
		return dimm_parameterl;
	case NAOMI_DIMM_PARAMETERH:
		DEBUG_LOG(NAOMI, "DIMM PARAMETERH read -> %x", dimm_parameterh);
		return dimm_parameterh;
	case NAOMI_DIMM_STATUS:
		{
			u32 rc =  DIMM_STATUS & ~(((SB_ISTEXT >> 3) & 1) << 8);
			static u32 lastRc;
			if (rc != lastRc)
				DEBUG_LOG(NAOMI, "DIMM STATUS read -> %x", rc);
			lastRc = rc;
			return rc;
		}
	default:
		return NaomiCartridge::ReadMem(address, size);
	}
}

void GDCartridge::WriteMem(u32 address, u32 data, u32 size)
{
	switch (address)
	{
	case NAOMI_DIMM_COMMAND:
		dimm_command = data;
		DEBUG_LOG(NAOMI, "DIMM COMMAND Write<%d>: %x", size, data);
		return;

	case NAOMI_DIMM_OFFSETL:
		dimm_offsetl = data;
		DEBUG_LOG(NAOMI, "DIMM OFFSETL Write<%d>: %x", size, data);
		return;
	case NAOMI_DIMM_PARAMETERL:
		dimm_parameterl = data;
		DEBUG_LOG(NAOMI, "DIMM PARAMETERL Write<%d>: %x", size, data);
		return;
	case NAOMI_DIMM_PARAMETERH:
		dimm_parameterh = data;
		DEBUG_LOG(NAOMI, "DIMM PARAMETERH Write<%d>: %x", size, data);
		return;

	case NAOMI_DIMM_STATUS:
		DEBUG_LOG(NAOMI, "DIMM STATUS Write<%d>: %x", size, data);
		if (data & 0x100)
			// write 0 seems ignored
			asic_CancelInterrupt(holly_EXP_PCI);
		if ((data & 1) == 0)
			// irq to dimm
			process();
		return;

	default:
		NaomiCartridge::WriteMem(address, data, size);
		return;
	}
}

void GDCartridge::process()
{
	INFO_LOG(NAOMI, "NetDIMM cmd %04x sock %d offset %04x paramh/l %04x %04x", (dimm_command >> 9) & 0x3f,
			dimm_command & 0xff, dimm_offsetl, dimm_parameterh, dimm_parameterl);

	int cmdGroup = (dimm_command >> 13) & 3;
	int cmd = (dimm_command >> 9) & 0xf;
	switch (cmdGroup)
	{
	case 0:	// system commands
		systemCmd(cmd);
		break;
	case 1: // network commands
		WARN_LOG(NAOMI, "Network command received cmd %x. Need full NetDIMM?", cmd);
		returnToNaomi(true, 0, -1);
		break;
	default:
		WARN_LOG(NAOMI, "Unknown DIMM command group %d cmd %x", cmdGroup, cmd);
		returnToNaomi(true, 0, -1);
		break;
	}
}

void GDCartridge::returnToNaomi(bool failed, u16 offsetl, u32 parameter)
{
	dimm_command = ((dimm_command & 0x7e00) + 0x400) | (failed ? 0xff : 0x4);
	dimm_offsetl = offsetl;
	dimm_parameterh = parameter >> 16;
	dimm_parameterl = parameter;
	verify(((SB_ISTEXT >> 3) & 1) == 0);
	asic_RaiseInterrupt(holly_EXP_PCI);
}

void GDCartridge::systemCmd(int cmd)
{
	switch (cmd)
	{
	case 0xf:	// startup
		INFO_LOG(NAOMI, "NetDIMM startup");
		// bit 16,17: dimm0 size (none, 128, 256, 512)
		// bit 18,19: dimm1 size
		// bit 28: network enabled (network settings appear in bios menu)
		// bit 29: set
		// bit 30: gd-rom connected
		// bit 31: mobile/ppp network?
		// (| 30, 70, F0, 1F0, 3F0, 7F0)
		// | offset >> 20 (dimm buffers offset @ size - 16MB)
		// offset = (64MB << 0-5) - 16MB
		// vf4 forces this value to 0f000000 (256MB) if != 1f000000 (512MB)
		if (dimm_data_size == 512_MB)
			addrspace::write32(0xc01fc04, (3 << 16) | 0x60000000 | (dimm_data_size >> 20));	// dimm board config 1 x 512 MB
		else if (dimm_data_size == 256_MB)
			addrspace::write32(0xc01fc04, (2 << 16) | 0x60000000 | (dimm_data_size >> 20));	// dimm board config 1 x 256 MB
		else
			addrspace::write32(0xc01fc04, (1 << 16) | 0x60000000 | (dimm_data_size >> 20));	// dimm board config 1 x 128 MB
		addrspace::write32(0xc01fc0c, 0x1020000 | 0x264);		// fw version 1.02
		// DIMM board serial Id
		{
			const u32 *serial = (u32 *)(getGameSerialId() + 0x20);	// get only the serial id
			addrspace::write32(0xc01fc40, *serial++);
			addrspace::write32(0xc01fc44, *serial++);
			addrspace::write32(0xc01fc48, *serial++);
			addrspace::write32(0xc01fc4c, *serial++);
		}
		// SET_BASE_ADDRESS(0c000000, 0)
		dimm_command = 0x8600;
		dimm_offsetl = 0;
		dimm_parameterl = 0;
		dimm_parameterh = 0x0c00;
		asic_RaiseInterrupt(holly_EXP_PCI);
		sh4_sched_request(schedId, SH4_MAIN_CLOCK);

		break;

	case 0:		// nop
	case 1:		// control read
	case 3:		// set base address
	case 4:		// peek8
	case 5:		// peek16
	case 6:		// peek32
	case 8:		// poke8
	case 9:		// poke16
	case 10:	// poke32
		// These are callbacks from naomi
		INFO_LOG(NAOMI, "System callback command %x", cmd);
		break;

	default:
		WARN_LOG(NAOMI, "Unknown system command %x", cmd);
		break;
	}
}

void GDCartridge::Serialize(Serializer &ser) const
{
	NaomiCartridge::Serialize(ser);
	ser << dimm_command;
	ser << dimm_offsetl;
	ser << dimm_parameterl;
	ser << dimm_parameterh;
	sh4_sched_serialize(ser, schedId);
}

void GDCartridge::Deserialize(Deserializer &deser)
{
	NaomiCartridge::Deserialize(deser);
	if (deser.version() >= Deserializer::V53)
	{
		deser >> dimm_command;
		deser >> dimm_offsetl;
		deser >> dimm_parameterl;
		deser >> dimm_parameterh;
		sh4_sched_deserialize(deser, schedId);
	}
}

int GDCartridge::schedCallback()
{
	if (SB_ISTEXT & 8)	// holly_EXP_PCI
		return SH4_MAIN_CLOCK;

	// regularly peek the test request address
	peek<u32>(0xc01fc08);
	asic_RaiseInterrupt(holly_EXP_PCI);

	u32 testRequest = addrspace::read32(0xc01fc08);
	if (testRequest & 1)
	{
		// bios dimm (fake) test
		addrspace::write32(0xc01fc08, testRequest & ~1);
		bool isMem;
		char *p = (char *)addrspace::writeConst(0xc01fd00, isMem, 4);
		strcpy(p, "CHECKING DIMM BD");
		p = (char *)addrspace::writeConst(0xc01fd10, isMem, 4);
		strcpy(p, "DIMM0 - GOOD");
		p = (char *)addrspace::writeConst(0xc01fd20, isMem, 4);
		strcpy(p, "DIMM1 - GOOD");
		p = (char *)addrspace::writeConst(0xc01fd30, isMem, 4);
		strcpy(p, "--- COMPLETED---");
		addrspace::write32(0xc01fc0c, 0x0102a264);
	}
	else if (testRequest != 0)
	{
		addrspace::write32(0xc01fc08, 0);
		addrspace::write32(0xc01fc0c, 0x03170100);
		INFO_LOG(NAOMI, "TEST REQUEST %x", testRequest);
	}

	return SH4_MAIN_CLOCK;
}
