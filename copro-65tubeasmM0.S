/*TODO

NB instructions like LDA ($00),Y don't check for wrapping around in memory


Changes for M0+ core

 * This file is part of the a6502 project.
 *
 * Copyright (C) 2012 Ed Spittles <ed.spittles@gmail.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */
//#define TRACE6502
#include "copro-65tubeasm.h"
/*
2012-06-11  Ed Spittles  started (as outgrowth of usb_cdcacm example in libopencm3)
  FIXME: decimal mode, of course

Using deeply nested macros - can get visibility for debug using
  arm-none-eabi-gcc -E *.S > x.tmp
  arm-none-eabi-as -amlh=x.list  x.tmp

r0  operand byte, opcode
r1  effective address
*/
#define rflagsNZ   r2  /* N and Z flags      */
#define rAcc       r3  /* a (accumulator)       */
#define rYreg      r4  /* y index register      */
#define rPC        r5  /* pc program counter    */
#define rmem       r6  /* memory base pointer   */
#define rCarry     r7  /* Carry bit in LSB */

#define rSP        r8  /* sp stack pointer      */
#define insttable  r9  /* pointer instruction table */
#define rPbyteLo   r10 /* Process status byte Lo */ /*V D and I in 6502 order: nv-bdizc */
#define rXreg      r11 /* x index register */
#define tregs      r12 // constant 0xFEF8>>3
#define temp       lr

#define rCarrybit 0

#define armNflag (0x80<<24)   /* negative NZCV---- */
#define armZflag (0x40<<24)   /* zero     NZCV---- */
#define armCflag (0x20<<24)   /* carry    NZCV---- */
#define armVflag (0x10<<24)   /* overflow NZCV---- */

#define pByteNflag 0x80    /* negative   nv-bdizc */
#define pByteVflag 0x40    /* overflow   nv-bdizc */
#define pByteXflag 0x20    /* stuck      nv-bdizc */
#define pByteBflag 0x10    /* break      nv-bdizc */
#define pByteDflag 0x08    /* decimal    nv-bdizc */
#define pByteIflag 0x04    /* interrupt  nv-bdizc */
#define pByteZflag 0x02    /* zero       nv-bdizc */
#define pByteCflag 0x01    /* carry      nv-bdizc */

#define I_ALIGN (1<<I_ALIGN_BITS)
#define INSTALIGN .balign I_ALIGN,0

#define EVENT_HANDLER_FLAG I_ALIGN*256
#define EVENT_HANDLER_SLOW_FLAG EVENT_HANDLER_FLAG<<1

.thumb
@.syntax unified
.text

// used to create a LO temperary register
.macro push_r reg
   mov   temp,\reg
.endm

.macro pop_r reg
   mov   \reg, temp
.endm

.macro SET_FLAGS_NZ reg=rAcc
   sxtb  rflagsNZ, \reg
.endm

.macro CLEAR_ZFLAG
   lsr    rflagsNZ, #8
   lsl    rflagsNZ, #8  // clear Z flag
.endm

.macro SETUP_CARRY reg=rflagsNZ
   LSR   \reg, rCarry, #1
.endm

.macro SAVE_CARRY
// Carry stored in the LSB
// Corrupts V flag
// if rcarry was clear then adding itself to itself is still clear
// if rcarry was set then adding itself to itself is cleared
// so ADC is always adding new carry to a clear bit 0
   ADC   rCarry, rCarry, rCarry
.endm

.macro SAVE_VFLAG_FAST reg=r0
   bvc   1f
   mov   \reg, #pByteVflag // Corrupts NZ but not CV
   add   rPbyteLo, rPbyteLo, \reg
1:
.endm

.macro CLEARV_PART2
   LSR  r1,#28
   mov  rPbyteLo,r1  // Vflag now clear
.endm

// NB BRK and PHP sets B flags on stacking
// IRQ and NMI B =0
// Other interrupts don't set the the B flag

.macro statustoR0 bflag
   LSL   r0, rflagsNZ,#24   // lower bytes cleared
   BNE   1f
   add   r0, #pByteZflag
1:
   cmp   rflagsNZ,#0xff
   BLS   2f
   add   r0, #pByteNflag
2:
   SETUP_CARRY r1
   BCC   3f
   add   r0, #pByteCflag
3:
.ifc \bflag,bflag
   add   r0, #pByteXflag+pByteBflag
.else
   add   r0, #pByteXflag
.endif
   mov   r1, rPbyteLo
   orr   r0, r1  // add in V, D and I flags
.endm

@ convert from 6502 status byte (nv-bdizc) in r0 to ARMs Pbyte (NZCV)
//  entry with rCarry preset with byte as we get that for free
.macro r0toPByte

   SET_FLAGS_NZ rCarry  // set Nflag

   mov   r1,#255-pByteZflag
   bic   rflagsNZ,rflagsNZ,r1
   mov   r1,#pByteZflag
   eor   rflagsNZ, rflagsNZ, r1

   movs  r1, #(pByteDflag+pByteIflag+pByteVflag)
   and   r1, rCarry, r1
   mov   rPbyteLo, r1 // we leave r1 with rPbyteLo so check IRQ can use it
.endm

// used for instructions that can enable interrupts
// CLI PLP RTI ( shorter version used for CLI
// r1 is rPbyteLo
.macro CHECK_IRQ
   // check for external IRQ
   LDR   r0,=tube_irq
   LDRb  r0,[r0]
   LSR   r0,r0,#1
   BCC   1f
   // check if 6502 IRQs enabled
   LSR   r1,r1, #3  // bit 2 =0 means enabled
   BCS   1f
   BL    handleinterrupt
1:
   NEXT_INSTRUCTION 0
.endm

.macro GET_ZP_LOCATION   // $00
   load_operand_byte r1
.endm

.macro GET_ZPX_LOCATION
   load_operand_byte r1
   add   r1, rXreg
   UXTB  r1, r1 @ AND #255
.endm

.macro GET_ZPY_LOCATION
   load_operand_byte r1
   add   r1, rYreg
   UXTB  r1, r1 @ AND #255
.endm

.macro GET_IND_LOCATION
   load_operand_byte r1
   ldrb  r0, [r1, rmem]
   ADD   r1, r1, #1
   ldrb  r1, [r1, rmem]
   LSL   r1, r1, #8
   orr   r1, r0
.endm

.macro GET_ABS_LOCATION reg=r1
.ifc \reg,r0
   ldrb  \reg, [rPC]
   ldrb  r1 , [rPC, #1]
   LSL   r1, r1,#8
   orr   \reg, \reg, r1
.else
   ldrb  \reg, [rPC]
   ldrb  r0 , [rPC, #1]
   LSL   r0, r0,#8
   orr   \reg, \reg, r0
.endif
.endm

.macro GET_INY_LOCATION
   GET_IND_LOCATION
   add   r1, r1, rYreg
.endm

.macro GET_INX_LOCATION // ($00,X)
   load_operand_byte r1
   add   r1, rXreg
   UXTB  r1, r1 @ AND #255
   ldrb  r0, [r1, rmem]
   ADD   r1, #1
   ldrb  r1, [r1, rmem]
   LSL   r1, #8
   orr   r1, r0
.endm

.macro  GET_ABY_LOCATION // $0000,Y
   GET_ABS_LOCATION r1
   add   r1, rYreg
.endm

.macro  GET_ABX_LOCATION // $0000,X
   GET_ABS_LOCATION r1
   add   r1, rXreg
.endm

.macro load_operand_byte reg=r0
   ldrb  \reg, [rPC]
.endm

.macro   load_2_operand_bytes_jmp /* and adjust PC */
   GET_ABS_LOCATION r0
   add   rPC, r0, rmem
.endm

.macro LOAD_INX reg=r0
   GET_INX_LOCATION
   LOAD_BYTE \reg , r1
.endm

.macro LOAD_IMM reg=r0
   load_operand_byte \reg
.endm

.macro LOAD_ZP reg=r0
   GET_ZP_LOCATION
   LOAD_BYTE \reg, r1
.endm

.macro LOAD_ABS reg=r0
   GET_ABS_LOCATION r1
   LOAD_BYTE \reg, r1
.endm

.macro LOAD_ABS_IO reg=r0
   GET_ABS_LOCATION r0
   // Check for IO access
   LSR   r1,r0,#3
   cmp   r1, tregs   // need to compare with 0xFEF8>>3
   bne   1f
.ifnc \reg,rAcc
   push  {rAcc}
.endif
   blx   tube_parasite_read
.ifnc \reg,rAcc
   pop   {rAcc}
.endif
   ldr   r1,=0xfef8>>3
   mov   tregs,r1

.ifnc \reg,r0
   mov   \reg, r0
.endif
   b     2f
1:
   LOAD_BYTE \reg, r0
2:
.endm

.macro LOAD_INY reg=r0 // ($00),Y
   GET_INY_LOCATION
   LOAD_BYTE \reg, r1
.endm

.macro LOAD_IND reg=r0
   GET_IND_LOCATION
   LOAD_BYTE \reg, r1
.endm

.macro LOAD_ZPX reg=r0 // $00,X
   GET_ZPX_LOCATION
   LOAD_BYTE \reg, r1
.endm

.macro LOAD_ZPY reg=r0
   GET_ZPY_LOCATION
   LOAD_BYTE \reg, r1
.endm

.macro LOAD_ABY reg=r0 // $0000,Y
   GET_ABY_LOCATION
   LOAD_BYTE \reg, r1
.endm

.macro LOAD_ABX reg=r0 // $0000,X
   GET_ABX_LOCATION
   LOAD_BYTE \reg, r1
.endm

.macro LOAD_BYTE rdst=r0 rsrc=r1
   ldrb  \rdst,[ rmem, \rsrc]
.endm

.macro STORE_BYTE_ONLY
   strb r0,[ rmem, r1]
.endm

.macro STORE_BYTE inc=1 rsrc=r0 rdst=r1
   strb    \rsrc,[ rmem, \rdst ]
   NEXT_INSTRUCTION \inc
.endm

.macro STORE_BYTE_ABS rsrc rdst
   LSR   r1, \rdst, #3
   CMP   r1, tregs
   bne   1f
   push  {r2,r3}
   mov   r1,\rsrc
   // r0 = address r1 = data
   blx   tube_parasite_write
   pop   {r2,r3}
   ldr   r1,=0xfef8>>3
   mov   r12,r1
   NEXT_INSTRUCTION 2 noalign
1:
.ifc \rsrc,rXreg
   mov   r1,rXreg
   strb  r1,[ rmem, \rdst ]
.else
   strb  \rsrc,[ rmem, \rdst ]
.endif
   NEXT_INSTRUCTION 2
.endm

/* branches */
.macro BRANCH condition inc=0 zero=nozero
   B\condition 1f
   NEXT_INSTRUCTION 1+\inc noalign
1:
.ifc \zero,nozero
   mov r0,#\inc
.endif
   ldrsb r0, [rPC,r0 ]  @ load a signed operand
   add   rPC, r0        @ ignore case of wrapping at 0xffff
   NEXT_INSTRUCTION 1+\inc
.endm

/* jumps and returns */

.macro INTR vector bflag

   mov   r0, rSP
   sub   r0, #3
   mov   rSP, r0
   sub   rPC, rmem
   strb  rPC, [r0, #2]
   LSR   rPC, rPC,#8
   strb  rPC, [r0, #3]
   mov   rPC, r0
   @ push pByte in 6502 format
   statustoR0 \bflag

   strb  r0, [rPC,#1]

   mov   r0, #pByteIflag  @ BRK sets interrupt disable just as IRQ would
   mov   rPbyteLo, r0 // I set B clear and clear DEC flag

   ldr   rPC, =0x10000-\vector
   ldrh  rPC, [rPC, rmem]  // vectors are aligned so we can use ldrh
   add   rPC, rmem
.endm

/* more stack operations */

.macro pushbyte reg=r0 wrap=nowrap
   mov   r1, rSP
   strb  \reg, [r1]
   sub   r1, #1
.ifc \wrap,wrap
   sub   r1,rmem
   cmp   r1,#255
   BNE   1f
   mov  r1,#0xff
   lsl  r1,#1
   add  r1,#1
1:
   add   r1,rmem
.endif
   mov   rSP, r1
.endm

.macro pullbyte reg
   mov   r1, rSP
   add   r1, #1
   ldrb  \reg, [r1]
   mov   rSP, r1
.endm

.macro NEXT_INSTRUCTION inc align=align
   ldrb  r0, [rPC, #\inc]
   add   rPC, #\inc+1    @ no auto increment in thumb

#ifdef TRACE6502
   BL   trace6502code
#else
   LSL   r0, #I_ALIGN_BITS
   add   r0, r0,insttable
   bx    r0
#endif
.ifc \align,align
   .ltorg
   INSTALIGN
.endif
.endm

.macro LOGICAL inc logic
   \logic rAcc, rAcc, r0
   SET_FLAGS_NZ
   NEXT_INSTRUCTION \inc
.endm

.macro NOP inc
   NEXT_INSTRUCTION \inc
.endm

.macro RMB bitnum
   LOAD_ZP
   mov   r1,#1<<\bitnum
   bic   r0, r0, r1
   GET_ZP_LOCATION //reload the address as R1 has been corrupted
   STORE_BYTE
.endm

.macro SMB bitnum
   LOAD_ZP
   mov    r1, #1<<\bitnum
   orr    r0, r0, r1
   GET_ZP_LOCATION //reload the address as R1 has been corrupted
   STORE_BYTE
.endm

.macro BBR bitnum
   LOAD_ZP
   LSR   r0, r0, #\bitnum+1
   BRANCH CC 1
.endm

.macro BBS bitnum
   LOAD_ZP
   LSR   r0, r0, #\bitnum+1
   BRANCH CS 1
.endm

.macro ASL6 inc=1 reg=r0
// unsigned input
   lsl   \reg,\reg,#1
   lsr   rCarry, \reg, #8
   SET_FLAGS_NZ \reg
   STORE_BYTE \inc
.endm

.macro ROL6 inc=1 reg=r0
//unsigned input
   SETUP_CARRY
   adc   \reg,\reg,\reg  // corrupts V flag ( clears )
   lsr   rCarry, \reg, #8
   SET_FLAGS_NZ \reg
.ifc \reg,r0
   STORE_BYTE \inc
.else
   uxtb  \reg,\reg
   NEXT_INSTRUCTION \inc
.endif
.endm

.macro LSR6 inc=1
   //NB unsigned input in rCarry as that sets it up for free
  // mov   rCarry, \reg // Save new carry
   lsr   r0,rCarry, #1
   SET_FLAGS_NZ r0
   STORE_BYTE \inc r0
.endm

.macro ROR6 inc reg=r0
   // NB unsigned input register
   lsl   rCarry, rCarry, #8
   orr   \reg, \reg, rCarry
   LSR   \reg,\reg,#1
   SAVE_CARRY
   SET_FLAGS_NZ \reg
.ifc \reg,r0
   STORE_BYTE \inc
.else
   uxtb  \reg,\reg
   NEXT_INSTRUCTION 0
.endif
.endm

.macro CMP6 inc=1 reg=rAcc
.ifc \reg,rXreg
   mov   r1, rXreg
   sub   rflagsNZ, r1, r0
.else
   sub   rflagsNZ, \reg, r0
.endif
   SET_FLAGS_NZ rflagsNZ // setup N flag nb needed because 255-1
   SAVE_CARRY
   NEXT_INSTRUCTION \inc
.endm

.macro ADC6 inc=1 bounce=nobounce decbounce=nodecbounce
   mov  r1,rPbyteLo
   LSL  r1,#24+4
.ifc \decbounce,nodecbounce
   bmi  2f
.else
   bmi   \decbounce
.endif
   CLEARV_PART2
   mvn   r0, r0
   lsl   r0, r0, #24
   mvn   r0, r0  // set lower bits to 0xzzFFFFFF
   lsl   rAcc, rAcc, #24
   SETUP_CARRY rCarry
   adc   rAcc, rAcc, r0
   SAVE_VFLAG_FAST
   SAVE_CARRY
   lsr   rAcc, rAcc, #24
   SET_FLAGS_NZ
.ifc \bounce,nobounce
   NEXT_INSTRUCTION \inc noalign
.else
   b  \bounce
.endif

.ifc \decbounce,nodecbounce
2:
.ifc \inc,1
   BL adc_decimal
.else
   BL adc_decimal2
.endif

.endif
   INSTALIGN
.endm

.macro SBC6 inc=1 bounce=nobounce decbounce=nodecbounce
   mov  r1,rPbyteLo
   LSL  r1,#24+4
.ifc \decbounce,nodecbounce
   Bmi  2f
.else
   Bmi  \decbounce
.endif
   CLEARV_PART2
   lsl   r0, r0, #24
   lsl   rAcc, rAcc, #24
   SETUP_CARRY
   sbc   rAcc, rAcc, r0
   SAVE_VFLAG_FAST
   SAVE_CARRY
   lsr   rAcc, rAcc, #24
   SET_FLAGS_NZ
.ifc \bounce,nobounce
   NEXT_INSTRUCTION \inc noalign
.else
   b  \bounce
.endif

.ifc \decbounce,nodecbounce
2:
.ifc \inc,1
   BL sbc_decimal
.else
   BL sbc_decimal2
.endif

.endif
   INSTALIGN
.endm

// BIT
// flags N bit = bit7
//       v bit = bit 6
// flags Z bit = r0 & ACC

.macro BIT inc=1

   mov   r1, #pByteVflag
   mov   rflagsNZ, rPbyteLo
   bic   rflagsNZ, rflagsNZ, r1  // clear Vflag
   and   r1,r1,r0  // get bit 6
   orr   rflagsNZ, r1
   mov   rPbyteLo, rflagsNZ

   LSR   rflagsNZ, R0, #7 // set up N flag
   LSL   rflagsNZ, #8 // ( clear lower 8 bits)

   and   R0, r0, rAcc
   orr   rflagsNZ, r0 // if r0 is zero the rflagsNZ will be zero

   NEXT_INSTRUCTION \inc
.endm
.section .time_critical.6502, "ax"
.balign (I_ALIGN)*256*4 , 0
     /* 6502 instruction set */
l_00:
breakinstruction:
   add rPC,#1
   INTR 2 bflag
   NEXT_INSTRUCTION 0 noalign //NB we don't want .ltorg here or we overflow
   .balign I_ALIGN, 0

l_01: // Opcode 01 - ORA ($00,X)
   LOAD_INX
   LOGICAL 1 orr

l_02:
   NOP 1

l_03:
   NOP 0

l_04: // Opcode 04 - TSB $00
   LOAD_ZP
   CLEAR_ZFLAG
   tst   r0, rAcc
   beq   1f
   add   rflagsNZ, #1
1:
   orr   r0, rAcc
   STORE_BYTE

l_05: // Opcode 05 - ORA $00
   LOAD_ZP
   LOGICAL 1 orr

l_06: // Opcode 06 - ASL $00
   LOAD_ZP
   ASL6

l_07: // Opcode 07 - RMB0 $00
   RMB 0

l_08: // Opcode 08 - PHP
   statustoR0 bflag
   pushbyte r0 wrap
   NEXT_INSTRUCTION 0

l_09: // Opcode 09 - ORA #$00
   LOAD_IMM
   LOGICAL 1 orr

l_0a: // Opcode 0A - ASL A
   lsl   rAcc,rAcc, #1
   lsr   rCarry, rAcc, #8
   uxtb  rAcc, rAcc
   SET_FLAGS_NZ
   NEXT_INSTRUCTION 0

l_0b:
   NOP 0

l_0c: // Opcode 0C - TSB $0000
   LOAD_ABS
   CLEAR_ZFLAG
   tst   r0, rAcc
   beq   1f
   add   rflagsNZ, #1
1:
   orr   r0, rAcc
   STORE_BYTE 2

l_0d: // Opcode 0D - ORA $0000
   LOAD_ABS
   LOGICAL 2 orr

l_0e: // Opcode 0E - ASL $0000
   LOAD_ABS
   ASL6 2

l_0f: // Opcode 0F - BBR0
   BBR 0

l_10: // Opcode 10 - BPL
   LSR  r0,rflagsNZ,#8 // EQ =+ve  NE=-ve
   BRANCH EQ 0 zero

l_11: // Opcode 11 - ORA ($00),Y
   LOAD_INY
   LOGICAL 1 orr

l_12: // Opcode 12 - ORA ($00)
   LOAD_IND
   LOGICAL 1 orr

l_13:
   NOP 0

l_14: // Opcode 14 - TRB $00
   LOAD_ZP
   CLEAR_ZFLAG
   tst   r0, rAcc
   beq   l_14_exit    // if zero there is no need to update the byte
   add   rflagsNZ, #1
   bic   r0, rAcc
   STORE_BYTE_ONLY
l_14_exit:
   NEXT_INSTRUCTION 1

l_15: // Opcode 15 - ORA $00,X
   LOAD_ZPX
   LOGICAL 1 orr

l_16: // Opcode 16 - ASL $00, X
   LOAD_ZPX
   ASL6

l_17: // Opcode 17 - RMB1 $00
   RMB 1

l_18: // Opcode 18 - CLC
   mov    rCarry, #0
   NEXT_INSTRUCTION 0

l_19: // Opcode 19 - ORA $0000,Y
   LOAD_ABY
   LOGICAL 2 orr

l_1a: // Opcode 1A - INC A
   add   rAcc, rAcc, #1
   uxtb  rAcc, rAcc
   SET_FLAGS_NZ
   NEXT_INSTRUCTION 0

l_1b:
   NOP 0

l_1c: // Opcode 1C - TRB $0000
   LOAD_ABS
   CLEAR_ZFLAG
   tst   r0, rAcc
   beq   l_1c_exit    // if the bits are already then no need to clear them
   add   rflagsNZ, #1
   bic   r0, rAcc
   STORE_BYTE_ONLY
l_1c_exit:
   NEXT_INSTRUCTION 2

l_1d: // Opcode 1D - ORA $0000,X
   LOAD_ABX
   LOGICAL 2 orr

l_1e: // Opcode 1E - ASL $0000,X
   LOAD_ABX
   ASL6 2

l_1f: // Opcode 1F - BBR1
   BBR 1

l_20: // Opcode 20 - JSR $0000
   add   r0, rPC, #1
   sub   r0, r0, rmem
   mov   r1, rSP
   sub   r1, r1, #2
   strb  r0, [r1, #1]
   LSR   r0, r0, #8
   strb  r0, [r1, #2]
   mov   rSP, r1

   load_2_operand_bytes_jmp
   NEXT_INSTRUCTION 0

l_21: // Opcode 21 - AND ($00,X)
   LOAD_INX
   LOGICAL 1 and

l_22:
   NOP 1

l_23:
   NOP 0

l_24: // Opcode 24 - BIT $00
   LOAD_ZP
   BIT

l_25: // Opcode 25 - AND $00
   LOAD_ZP
   LOGICAL 1 and

l_26: // Opcode 26 - ROL $00
   LOAD_ZP
   ROL6

l_27: // Opcode 27 - RMB2 $00
   RMB 2

l_28: // Opcode 28 - PLP
   pullbyte rCarry
   r0toPByte
   CHECK_IRQ
   INSTALIGN

l_29: // Opcode 29 - AND #$00
   LOAD_IMM
   LOGICAL 1 and

l_2a: // Opcode 2A - ROL A
   ROL6 0 rAcc

l_2b:
   BL l_ea
bitjumpload:
   push  {r3}
   blx   tube_parasite_read
   pop   {r3}
   ldr   r1,=0xfef8>>3
   mov   r12, r1
   BIT 2

l_2c: // Opcode 2C - BIT $0000
   GET_ABS_LOCATION r0
   // Check for IO access
   LSR   r1, r0, #3
   cmp   r1, tregs   // need to compare with 0xFEF8>>3
   beq   bitjumpload
   LOAD_BYTE r0, r0
   BIT 2

l_2d: // Opcode 2D - AND $0000
   LOAD_ABS
   LOGICAL 2 and

l_2e: // Opcode 2E - ROL $0000
   LOAD_ABS
   ROL6 2

l_2f: // Opcode 2F - BBR2
   BBR 2

l_30: // Opcode 30 - BMI
   LSR  r0,rflagsNZ,#8   //  EQ = +ve NE= -ve GT =-ve LE=+ve
   BRANCH NE

l_31: // Opcode 31 - AND ($00),Y
   LOAD_INY
   LOGICAL 1 and

l_32: // Opcode 32 - AND ($00)
   LOAD_IND
   LOGICAL 1 and

l_33:
   NOP 0

l_34: // Opcode 34 - BIT $00,X
   LOAD_ZPX
   BIT

l_35: // Opcode 35 - AND $00,X
   LOAD_ZPX
   LOGICAL 1 and

l_36: // Opcode 36 - ROL $00, X
   LOAD_ZPX
   ROL6

l_37: // Opcode 37 - RMB3 $00
   RMB 3

l_38: // Opcode 38 - SEC
   mov    rCarry, #1
   NEXT_INSTRUCTION 0

l_39: // Opcode 39 - AND $0000,Y
   LOAD_ABY
   LOGICAL 2 and

l_3a: // Opcode 3A - DEC A
   sub   rAcc, rAcc, #1
   uxtb  rAcc, rAcc
   SET_FLAGS_NZ
   NEXT_INSTRUCTION 0

l_3b:
   NOP 0

l_3c: // Opcode 3C - BIT $0000,X
   LOAD_ABX
   BIT 2

l_3d: // Opcode 3D - AND $0000,X
   LOAD_ABX
   LOGICAL 2 and

l_3e: // Opcode 3E - ROL $0000,X
   LOAD_ABX
   ROL6 2

l_3f: // Opcode 3F - BBR3
   BBR 3

l_40: // Opcode 40 - RTI
   mov   r0, rSP
   ldrb  rCarry, [r0, #1]
   ldrb  rPC, [r0, #2]  @ ignore case of wrapping
   ldrb  r1, [r0, #3]   @ ignore case of wrapping
   add   r0, #3
   mov   rSP, r0
   LSL   r1, #8
   orr   rPC, r1
   add   rPC, rPC, rmem
   r0toPByte // need to check if IRQs have now become enabled
   CHECK_IRQ

l_41: // Opcode 41 - EOR ($00,X)
   LOAD_INX
   LOGICAL 1 eor

l_42:
   NOP 1

l_43:
   NOP 0

l_44:
   NOP 1

l_45: // Opcode 45 - EOR $00
   LOAD_ZP
   LOGICAL 1 eor

l_46: // Opcode 46 - LSR $00
   LOAD_ZP rCarry
   LSR6

l_47: // Opcode 47 - RMB4 $00
   RMB 4

l_48: // Opcode 48 - PHA
   pushbyte rAcc
   NEXT_INSTRUCTION 0

l_49: // Opcode 49 - EOR #$00
   LOAD_IMM
   LOGICAL 1 eor

l_4a: // Opcode 4A - LSR A
   lsr   rAcc,rAcc, #1
   SAVE_CARRY
   SET_FLAGS_NZ rAcc
   NEXT_INSTRUCTION 0

l_4b:
   NOP 0

l_4c: // Opcode 4C - JMP $0000
   load_2_operand_bytes_jmp
   NEXT_INSTRUCTION 0

l_4d: // Opcode 4D - EOR $0000
   LOAD_ABS
   LOGICAL 2 eor

l_4e: // Opcode 4E - LSR $0000
   LOAD_ABS rCarry
   LSR6 2

l_4f: // Opcode 4F - BBR4
   BBR 4

l_50: // Opcode 30 - BVC
   mov   r0,rPbyteLo
   lsr   r0,r0,#7
   BRANCH CC 0 zero

l_51: // Opcode 51 - EOR ($00),Y
   LOAD_INY
   LOGICAL 1 eor

l_52: // Opcode 52 - EOR ($00)
   LOAD_IND
   LOGICAL 1 eor

l_53:
   NOP 0

l_54: // Opcode 54 - NOP
   NOP 1

l_55: // Opcode 55 - EOR $00,X
   LOAD_ZPX
   LOGICAL 1 eor

l_56: // Opcode 56 - LSR $00, X
   LOAD_ZPX rCarry
   LSR6

l_57: // Opcode 57 - RMB5 $00
   RMB 5

l_58: // Opcode 58 - CLI
   mov r1,rPbyteLo
   mov r0,#pByteIflag
   bic r1,r1,r0
   mov rPbyteLo, r1

   // check for external IRQ
   LDR   r0,=tube_irq
   LDRb   r0,[r0]
   LSR   r0,r0,#1
   BCC   1f
   BL    handleinterrupt
1:
   NEXT_INSTRUCTION 0

l_59: // Opcode 59 - EOR $0000,Y
   LOAD_ABY
   LOGICAL 2 eor

l_5a: // Opcode 5A - PHY
   pushbyte rYreg
   NEXT_INSTRUCTION 0

l_5b:
   NOP 0

l_5c: // Opcode 5C - NOP $0000,X
   NOP 2

l_5d: // Opcode 5D - EOR $0000,X
   LOAD_ABX
   LOGICAL 2 eor

l_5e: // Opcode 5E - LSR $0000,X
   LOAD_ABX rCarry
   LSR6 2

l_5f: // Opcode 5F - BBR5
   BBR 5

l_60: // Opcode 60 - RTS
   mov   r1, rSP
   ldrb  r0, [r1, #1]  @ ignore case of wrapping
   ldrb  rPC, [r1, #2]   @ ignore case of wrapping
   LSL   rPC, #8
   orr   rPC, r0
   add   r1,#2
   mov   rSP, r1
   add   rPC,rPC,rmem
   NEXT_INSTRUCTION 1

l_61: // Opcode 61 - ADC ($00,X)
   LOAD_INX
   ADC6 1 nobounce adcdecbounce

l_62:
   NEXT_INSTRUCTION 1 noalign
adcdecbounce:
   BL adc_decimal
   INSTALIGN

l_63:
   NOP 0

l_64: // Opcode 64 - STZ $00
   GET_ZP_LOCATION
   mov r0,#0
   STORE_BYTE

l_65: // Opcode 65 - ADC $00
   LOAD_ZP
   ADC6 1

l_66: // Opcode 66 - ROR $00
   LOAD_ZP
   ROR6

l_67: // Opcode 67 - RMB6 $00
   RMB 6

l_68: // Opcode 68 - PLA
   pullbyte rAcc
   SET_FLAGS_NZ
   NEXT_INSTRUCTION 0

l_69: // Opcode 69 - ADC #$00
   LOAD_IMM
   ADC6 1

l_6a: // Opcode 6A - ROR A
   ROR6 0 rAcc

l_6b:
   NOP 0

l_6c: // Opcode 6C - JMP ($0000)
   load_2_operand_bytes_jmp
   load_2_operand_bytes_jmp
   NEXT_INSTRUCTION 0

l_6d: // Opcode 6D - ADC6 $0000
   LOAD_ABS
   ADC6 2

l_6e: // Opcode 6E - ROR $0000
   LOAD_ABS
   ROR6 2

l_6f: // Opcode 6F - BBR6
   BBR 6

l_70: // Opcode 70 - BVS
   mov   r0, rPbyteLo
   lsr   r0, r0, #7
   BRANCH CS 0 zero

l_71: // Opcode 71 - ADC ($00),Y
   LOAD_INY
   ADC6 1

l_72: // Opcode 72 - ADC ($00)
   LOAD_IND
   ADC6 1

l_73:
   NOP 0

l_74: // Opcode 74 - STZ $00,X
   GET_ZPX_LOCATION
   mov   r0, #0 // may be able to optimsed this out
   STORE_BYTE

l_75: // Opcode 75 - ADC $00,X
   LOAD_ZPX
   ADC6 1

l_76: // Opcode 76 - ROR $00, X
   LOAD_ZPX
   ROR6

l_77: // Opcode 77 - RMB7 $00
   RMB 7

l_78: // Opcode 78 - SEI
   mov r0,rPbyteLo
   mov r1,#pByteIflag
   orr r0,r0,r1
   mov rPbyteLo, r0
   NEXT_INSTRUCTION 0

l_79: // Opcode 79 - ADC $0000,Y
   LOAD_ABY
   ADC6 2

l_7a: // Opcode 7A - PLY
   pullbyte rYreg
   SET_FLAGS_NZ rYreg
   NEXT_INSTRUCTION 0

l_7b:
   NOP 0

l_7c: // Opcode 7C - JMP $0000,X
   load_2_operand_bytes_jmp
   add   rPC, rPC, rXreg
   load_2_operand_bytes_jmp
   NEXT_INSTRUCTION 0

l_7d: // Opcode 7D - ADC $0000,X
   LOAD_ABX
   ADC6 2

l_7e: // Opcode 7E - ROR $0000,X
   LOAD_ABX
   ROR6 2

l_7f: // Opcode 7F - BBR7
   BBR 7

l_80: // Opcode 80 - BRA
   mov   r0, #0
   ldrsb r0, [rPC,r0]   @ load a signed operand
   add   rPC, r0        @ ignore case of wrapping at 0xffff
   NEXT_INSTRUCTION 1

l_81: // Opcode 81 - STA ($00,X)
   GET_INX_LOCATION
   STORE_BYTE 1 rAcc

l_82:
   NOP 1

l_83:
   NOP 0

l_84: // Opcode 84 - STY $00
   GET_ZP_LOCATION
   STORE_BYTE 1 rYreg

l_85: // Opcode 85 - STA $00
   GET_ZP_LOCATION
   STORE_BYTE 1 rAcc

l_86: // Opcode 86 - STX $00
   GET_ZP_LOCATION
   mov r0, rXreg
   STORE_BYTE 1 r0

l_87: // Opcode 87 - SMB0 $00
   SMB 0

l_88: // Opcode 88 - DEY
   sub rYreg, rYreg, #1
   uxtb rYreg, rYreg
   SET_FLAGS_NZ rYreg
   NEXT_INSTRUCTION 0

l_89: // Opcode 89 - BIT #$00 ( only changes Z flag )
   LOAD_IMM
   and r0,r0,rAcc
   CLEAR_ZFLAG
   orr rflagsNZ, rflagsNZ, r0
   NEXT_INSTRUCTION 1

l_8a: // Opcode 8A - TXA
   mov   rAcc, rXreg
   SET_FLAGS_NZ
   NEXT_INSTRUCTION 0

l_8b: // Opcode 8B - NOP
   NOP 0

l_8c: // Opcode 8C - STY $0000
   GET_ABS_LOCATION r0
   STORE_BYTE_ABS rYreg r0

l_8d: // Opcode 8D - STA $0000
   GET_ABS_LOCATION r0
   STORE_BYTE_ABS rAcc r0

l_8e: // Opcode 8E - STX $0000
   GET_ABS_LOCATION r0
   STORE_BYTE_ABS rXreg r0

l_8f: // Opcode 8F - BBS0
   BBS 0

l_90: // Opcode 90 - BCC
   //SETUP_CARRY r0
   LSL r0, rCarry,#31
   BRANCH EQ 0 zero

l_91: // Opcode 91 - STA ($00),Y
   GET_INY_LOCATION
   STORE_BYTE 1 rAcc

l_92: // Opcode 92 - STA ($00)
   GET_IND_LOCATION
   STORE_BYTE 1 rAcc

l_93:
   NOP 0

l_94: // Opcode 94 - STY $00,X
   GET_ZPX_LOCATION
   STORE_BYTE 1 rYreg

l_95: // Opcode 95 - STA $00,X
   GET_ZPX_LOCATION
   STORE_BYTE 1 rAcc

l_96: // Opcode 96 - STX $00,Y
   GET_ZPY_LOCATION
   mov r0, rXreg
   STORE_BYTE 1 r0

l_97: // Opcode 97 - SMB1 $00
   SMB 1

l_98: // Opcode 98 - TYA
   mov   rAcc, rYreg
   SET_FLAGS_NZ
   NEXT_INSTRUCTION 0

l_99: // Opcode 99 - STA $0000,Y
   GET_ABY_LOCATION
   STORE_BYTE 2 rAcc

l_9a: // Opcode 9A - TXS
   mov   r0, #1
   lsl   r0, r0, #8
   add   r0, r0, rmem
   add   r0, r0, rXreg
   mov   rSP, r0
   NEXT_INSTRUCTION 0

l_9b: // Opcode 9B - NOP
   NOP 0

l_9c: // Opcode 9C - STZ $0000
   GET_ABS_LOCATION
   mov  r0, #0
   STORE_BYTE 2 r0

l_9d: // Opcode 9D - STA $0000,X
   GET_ABX_LOCATION
   STORE_BYTE 2 rAcc

l_9e: // Opcode 9E - STZ $0000,X
   GET_ABX_LOCATION
   mov r0, #0
   STORE_BYTE 2 r0

l_9f: // Opcode 9F - BBS1
   BBS 1

l_a0: // Opcode A0 - LDY #$00
   LOAD_IMM rYreg
   SET_FLAGS_NZ rYreg
   NEXT_INSTRUCTION 1

l_a1: // Opcode A1 - LDA ($00,X)
   LOAD_INX rAcc
   SET_FLAGS_NZ
   NEXT_INSTRUCTION 1

l_a2: // Opcode A2 - LDX #$00
   LOAD_IMM r0
   mov   rXreg, r0
   SET_FLAGS_NZ r0
   NEXT_INSTRUCTION 1

l_a3: // Opcode a3 - NOP
   NOP 0

l_a4: // Opcode A4 - LDY $00
   LOAD_ZP rYreg
   SET_FLAGS_NZ rYreg
   NEXT_INSTRUCTION 1

l_a5: // Opcode A5 - LDA $00
   LOAD_ZP rAcc
   SET_FLAGS_NZ
   NEXT_INSTRUCTION 1

l_a6: // Opcode A4 - LDX $00
   LOAD_ZP
   mov  rXreg, r0
   SET_FLAGS_NZ r0
   NEXT_INSTRUCTION 1

l_a7: // Opcode a7 - SMB2 $00
   SMB 2

l_a8: // Opcode A8 - TAY
   mov  rYreg, rAcc
   SET_FLAGS_NZ
   NEXT_INSTRUCTION 0

l_a9: // Opcode A9 - LDA #$00
   LOAD_IMM rAcc
   SET_FLAGS_NZ
   NEXT_INSTRUCTION 1

l_aa: // Opcode AA - TAX
   mov  rXreg, rAcc
   SET_FLAGS_NZ
   NEXT_INSTRUCTION 0

l_ab: // Opcode ab - NOP
   NOP 0

l_ac: // Opcode AC - LDY $0000
   LOAD_ABS_IO rYreg
   SET_FLAGS_NZ rYreg
   NEXT_INSTRUCTION 2

l_ad: // Opcode AD - LDA $0000
   LOAD_ABS_IO rAcc
   SET_FLAGS_NZ
   NEXT_INSTRUCTION 2

l_ae: // Opcode AE - LDX $0000
   LOAD_ABS_IO r0
   mov  rXreg, r0
   SET_FLAGS_NZ r0
   NEXT_INSTRUCTION 2

l_af: // Opcode AF - BBS2
   BBS 2

l_b0: // Opcode B0 - BCS
   SETUP_CARRY r0
   BRANCH CS

l_b1: // Opcode B1 - LDA ($00), Y
   LOAD_INY rAcc
   SET_FLAGS_NZ
   NEXT_INSTRUCTION 1

l_b2: // Opcode B2 - LDA ($00)
   LOAD_IND rAcc
   SET_FLAGS_NZ
   NEXT_INSTRUCTION 1

l_b3: // Opcode B3 - NOP
   NOP 0

l_b4: // Opcode B4 - LDY $00,X
   LOAD_ZPX rYreg
   SET_FLAGS_NZ rYreg
   NEXT_INSTRUCTION 1

l_b5: // Opcode B5 - LDA $00,X
   LOAD_ZPX rAcc
   SET_FLAGS_NZ
   NEXT_INSTRUCTION 1

l_b6: // Opcode B6 - LDX $00,Y
   LOAD_ZPY
   mov  rXreg, r0
   SET_FLAGS_NZ r0
   NEXT_INSTRUCTION 1

l_b7: // Opcode B7 - SMB3 $00
   SMB 3

l_b8: // Opcode B8 - CLV
   mov  r0,rPbyteLo
   mov  r1, #pByteVflag
   bic  r0,r0,r1
   mov  rPbyteLo,r0
   NEXT_INSTRUCTION 0

l_b9: // Opcode B9 - LDA $0000,Y
   LOAD_ABY rAcc
   SET_FLAGS_NZ
   NEXT_INSTRUCTION 2

l_ba: // Opcode BA - TSX
   mov   r0, rSP
   sub   r0, rmem
   uxtb  r0,r0
   mov   rXreg, r0
   SET_FLAGS_NZ r0
   NEXT_INSTRUCTION 0

l_bb: // Opcode BB - NOP
   NOP 0

l_bc: // Opcode BC - LDY $0000,X
   LOAD_ABX rYreg
   SET_FLAGS_NZ rYreg
   NEXT_INSTRUCTION 2

l_bd: // Opcode BD - LDA $0000,X
   LOAD_ABX rAcc
   SET_FLAGS_NZ
   NEXT_INSTRUCTION 2

l_be: // Opcode BE - LDX $0000,Y
   LOAD_ABY r0
   mov  rXreg, r0
   SET_FLAGS_NZ r0
   NEXT_INSTRUCTION 2

l_bf: // Opcode BF - BBS3
   BBS 3

l_c0: // Opcode C0 - CPY #$00
   LOAD_IMM
   CMP6 1 rYreg

l_c1: // Opcode C1 - CMP ($00,X)
   LOAD_INX
   CMP6 1 rAcc

l_c2:
   NOP 1

l_c3:
   NOP 0

l_c4: // Opcode C4 - CPY $00
   LOAD_ZP
   CMP6 1 rYreg

l_c5: // Opcode C5 - CMP $00
   LOAD_ZP
   CMP6 1 rAcc

l_c6: // Opcode C6 - DEC $00
   LOAD_ZP
   SUB r0,r0,#1
   SET_FLAGS_NZ r0
   STORE_BYTE

l_c7: // Opcode C7 - SMB4 $00
   SMB 4

l_c8: // Opcode C8 - INY
   add  rYreg, rYreg, #1
   uxtb rYreg, rYreg
   SET_FLAGS_NZ rYreg
   NEXT_INSTRUCTION 0

l_c9: // Opcode C9 - CMP #$00
   LOAD_IMM
   CMP6 1 rAcc

l_ca: // Opcode CA - DEX
   mov  r0, rXreg
   sub  r0, r0, #1
   uxtb r0, r0
   mov  rXreg,r0
   SET_FLAGS_NZ r0
   NEXT_INSTRUCTION 0

l_cb: // Opcode CB - WAI not supported and not used on the 65c02 second proc
   NOP 0

l_cc: // Opcode CC - CPY $0000
   LOAD_ABS
   CMP6 2 rYreg

l_cd: // Opcode CD - CMP $0000
   LOAD_ABS
   CMP6 2 rAcc

l_ce: // Opcode CE - DEC $0000
   LOAD_ABS
   SUB r0,r0,#1
   SET_FLAGS_NZ r0
   STORE_BYTE 2

l_cf: // Opcode CF - BBS4
   BBS 4

l_d0: // Opcode D0 - BNE
   LSL  r0,rflagsNZ,#24
   BRANCH NE

l_d1: // Opcode D1 - CMP ($00),Y
   LOAD_INY
   CMP6 1 rAcc

l_d2: // Opcode D2 - CMP ($00)
   LOAD_IND
   CMP6 1 rAcc

l_d3:
   NOP 0

l_d4:
   NOP 1

l_d5: // Opcode D5 - CMP $00,X
   LOAD_ZPX
   CMP6 1 rAcc

l_d6: // Opcode D6 - DEC $00,X
   LOAD_ZPX
   SUB r0,r0,#1
   SET_FLAGS_NZ r0
   STORE_BYTE

l_d7: // Opcode D7 - SMB5 $00
   SMB 5

l_d8: // Opcode D8 - CLD
   mov r0, rPbyteLo
   mov r1, #pByteDflag
   bic r0, r0, r1
   mov rPbyteLo, r0
   NEXT_INSTRUCTION 0

l_d9: // Opcode D9 - CMP $0000,Y
   LOAD_ABY
   CMP6 2 rAcc

l_da: // Opcode DA - PHX
   mov r0, rXreg
   pushbyte r0
   NEXT_INSTRUCTION 0

l_db: // Opcode DB - STP not used
   NOP 0

l_dc:
   NOP 2

l_dd: // Opcode DD - CMP $0000,X
   LOAD_ABX
   CMP6 2 rAcc

l_de: // Opcode DE - DEC $0000,X
   LOAD_ABX
   SUB r0,r0,#1
   SET_FLAGS_NZ r0
   STORE_BYTE 2

l_df: // Opcode DF - BBS5
   BBS 5

l_e0: // Opcode E0 - CPX #$00
   LOAD_IMM
   CMP6 1 rXreg

l_e1: // Opcode E1 - SBC ($00,X)
   LOAD_INX
   SBC6 1

l_e2:
   NOP 1

l_e3:
   NOP 0

l_e4: // Opcode E4 - CPX $00
   LOAD_ZP
   CMP6 1 rXreg

l_e5: // Opcode E5 - SBC $00
   LOAD_ZP
   SBC6

l_e6: // Opcode E6 - INC $00
   LOAD_ZP
   add r0,r0,#1
   SET_FLAGS_NZ r0
   STORE_BYTE

l_e7: // Opcode E7 - SMB6 $00
   SMB 6

l_e8: // Opcode E8 - INX
   mov r0, rXreg
   add r0,r0,#1
   uxtb  r0, r0
   mov rXreg, r0
   SET_FLAGS_NZ r0
   NEXT_INSTRUCTION 0

l_e9: // Opcode E9 - SBC #$00
   LOAD_IMM
   SBC6

l_ea:
   NOP 0

l_eb: // NOP borrow for instruction timing
   ldr r0,=0xe000e018
   ldr r0,[r0]
   mov  r1,#0xE4
   str r0,[rmem,r1]
   NEXT_INSTRUCTION 0

l_ec: // Opcode EC - CPX $0000
   LOAD_ABS
   CMP6 2 rXreg

l_ed: // Opcode ED - SBC $0000
   LOAD_ABS
   SBC6 2

l_ee: // Opcode EE - INC $0000
   LOAD_ABS
   add r0,r0,#1
   SET_FLAGS_NZ r0
   STORE_BYTE 2

l_ef: // Opcode EF - BBS6
   BBS 6

l_f0: // Opcode F0 - BEQ
   LSL  r0,rflagsNZ,#24
   BRANCH EQ 0 zero

l_f1: // Opcode F1 - SBC ($00),Y
   LOAD_INY
   SBC6

l_f2: // Opcode F2 - SBC ($00)
   LOAD_IND
   SBC6

l_f3:
   NOP 0

l_f4:
   NOP 1

l_f5: // Opcode F5 - SBC $00,X
   LOAD_ZPX
   SBC6

l_f6: // Opcode F6 - INC $00,X
   LOAD_ZPX
   ADD r0,r0,#1
   SET_FLAGS_NZ r0
   STORE_BYTE

l_f7: // Opcode F7 - SMB7 $00
   SMB 7

l_f8: // Opcode F8 - SED
   mov r0, rPbyteLo
   mov r1, #pByteDflag
   orr r0, r0, r1
   mov rPbyteLo, r0
   NEXT_INSTRUCTION 0

l_f9: // Opcode F9 - SBC $0000,Y
   LOAD_ABY
   SBC6 2

l_fa: // Opcode FA - PLX
   pullbyte r0
   mov rXreg, r0
   SET_FLAGS_NZ r0
   NEXT_INSTRUCTION 0

l_fb: // Opcode FB - STP not used Stolen for timing
   ldr r0, =0xe000e018
   ldr r0, [r0]
   mov r1, #0xE0
   str r0, [rmem,r1]
   NEXT_INSTRUCTION 0

l_fc:
   NOP 2

l_fd: // Opcode FD - SBC $0000,X
   LOAD_ABX
   SBC6 2

l_fe: // Opcode FE - INC $0000,X
   LOAD_ABX
   add r0,r0,#1
   SET_FLAGS_NZ r0
   STORE_BYTE 2

l_ff: // Opcode FF - BBS7
   BBS 7
tablend:
. = 256*I_ALIGN
.print "Instruction table is the correct length."

// **********************************************
// End of opcode implementations
//
// Now the event handler ( 256 times)
// **********************************************
.rept 256
   mov   r1, #EVENT_HANDLER_FLAG>>8
   lsl   r1,r1,#8
   mov   r0, insttable
   bic   r0, r0, r1        // ack events
   mov   insttable, r0

   ldr   r0, =tube_irq
   ldrb  r0, [r0]

   sub   rPC, rPC, #1        // set the instruction back as we haven't executed it

   lsl   r0,r0,#29
   lsr   r0,r0,#29-2 // and the 3 bits and multiply by 4
   adr   r1,0f
   ldr   r0,[r1,r0]
   bx    r0
.align
0:
   .word    handle_nextinstruction+1
   .word    handle_irq+1
   .word    handle_nmi+1
   .word    handle_nmi+1
   .word    exec_65tube_exit +1
   .word    exec_65tube_exit +1
   .word    exec_65tube_exit +1
   .word    exec_65tube_exit +1
   .ltorg
   INSTALIGN
.endr

slowdown:
   push  {r2,r3}
   ldrb  r0, [rPC]             // get next instruction
   adr   r1, timing_table
   lsl   r0, #1
   ldrb  r3, [r1,r0]  // get instruction time
   add   r0, #1
   ldrb  r0, [r1,r0]  // get instruction length

   ldr   r1, lastPC
   cmp   r1, rPC

   beq   nojumptime
   ADD   r3, #1
nojumptime:
   add   r0, rPC
   adr   r1, lastPC
   str   r0, [r1]

   ldr   r1, copro_speed6502     // load copro_speed
   MUL   r3, r1, r3
   LDR   r2, targettime

   // add on new time
   LSR   r3, r3,#8
   sub   r0, r2,r0
   // save time new time.
   adr   r1, targettime
   str   r0, [r1]

   // loop until current time(r3) > nexttime(R1) ( nb deal with wrap around)
   ldr   r1, =0xe000e018
waste_time:
   ldr   r0, [r1]

   sub   r2, r0      // if this goes negative we are running too slow
   CMP   r2, r3
   BCS   waste_time
   pop   {r2-r3}
   mov   r0,#(EVENT_HANDLER_FLAG+EVENT_HANDLER_SLOW_FLAG)>>8     // ack any events
   lsl   r0,r0,#8
   mov   r1,insttable
   bic   r1,r1,r0
   ldr   r0,[rPC]
   LSL   r0,#I_ALIGN_BITS
   add   rPC,rPC,#1
   add   r0,r0,r1
   bx    r0
.align
.ltorg
lastPC:
   .word 0
targettime:
   .word 0
copro_speed6502:
   .word 0

// **********************************************
// Instruction timings
// **********************************************

// First byte is the number of cycles the instruction takes ( minus 1 for known jmps RTS as new pc will be detected and so extra cycle will be added)
// Second byte signifies number of bytes of instruction ) ie where PC will be

timing_table:
.byte   7   ,   1   //  0
.byte   6   ,   2   //  1
.byte   2   ,   2   //  2
.byte   1   ,   1   //  3
.byte   5   ,   2   //  4
.byte   3   ,   2   //  5
.byte   5   ,   2   //  6
.byte   5   ,   2   //  7
.byte   3   ,   1   //  8
.byte   2   ,   2   //  9
.byte   2   ,   1   //  A
.byte   1   ,   1   //  B
.byte   6   ,   3   //  C
.byte   4   ,   3   //  D
.byte   6   ,   3   //  E
.byte   5   ,   3   //  F
.byte   2   ,   2   //  10
.byte   5   ,   2   //  11
.byte   5   ,   2   //  12
.byte   1   ,   1   //  13
.byte   5   ,   2   //  14
.byte   4   ,   2   //  15
.byte   6   ,   2   //  16
.byte   5   ,   2   //  17
.byte   2   ,   1   //  18
.byte   4   ,   3   //  19
.byte   2   ,   1   //  1A
.byte   1   ,   1   //  1B
.byte   6   ,   3   //  1C
.byte   4   ,   3   //  1D
.byte   7   ,   3   //  1E
.byte   5   ,   3   //  1F
.byte   5   ,   3   //  20
.byte   6   ,   2   //  21
.byte   2   ,   2   //  22
.byte   1   ,   1   //  23
.byte   3   ,   2   //  24
.byte   3   ,   2   //  25
.byte   5   ,   2   //  26
.byte   5   ,   2   //  27
.byte   4   ,   1   //  28
.byte   2   ,   2   //  29
.byte   2   ,   1   //  2A
.byte   1   ,   1   //  2B
.byte   4   ,   3   //  2C
.byte   4   ,   3   //  2D
.byte   6   ,   3   //  2E
.byte   5   ,   3   //  2F
.byte   2   ,   2   //  30
.byte   5   ,   2   //  31
.byte   5   ,   2   //  32
.byte   1   ,   1   //  33
.byte   4   ,   2   //  34
.byte   4   ,   2   //  35
.byte   6   ,   2   //  36
.byte   5   ,   2   //  37
.byte   2   ,   1   //  38
.byte   4   ,   3   //  39
.byte   2   ,   1   //  3A
.byte   1   ,   1   //  3B
.byte   4   ,   3   //  3C
.byte   4   ,   3   //  3D
.byte   7   ,   3   //  3E
.byte   5   ,   3   //  3F
.byte   5   ,   1   //  40
.byte   6   ,   2   //  41
.byte   2   ,   2   //  42
.byte   1   ,   1   //  43
.byte   3   ,   2   //  44
.byte   3   ,   2   //  45
.byte   5   ,   2   //  46
.byte   5   ,   2   //  47
.byte   3   ,   1   //  48
.byte   2   ,   2   //  49
.byte   2   ,   1   //  4A
.byte   1   ,   1   //  4B
.byte   2   ,   3   //  4C
.byte   4   ,   3   //  4D
.byte   6   ,   3   //  4E
.byte   5   ,   3   //  4F
.byte   2   ,   2   //  50
.byte   5   ,   2   //  51
.byte   5   ,   2   //  52
.byte   1   ,   1   //  53
.byte   4   ,   2   //  54
.byte   4   ,   2   //  55
.byte   6   ,   2   //  56
.byte   5   ,   2   //  57
.byte   2   ,   1   //  58
.byte   4   ,   3   //  59
.byte   3   ,   1   //  5A
.byte   1   ,   1   //  5B
.byte   8   ,   3   //  5C
.byte   4   ,   3   //  5D
.byte   7   ,   3   //  5E
.byte   5   ,   3   //  5F
.byte   5   ,   1   //  60
.byte   6   ,   2   //  61
.byte   2   ,   2   //  62
.byte   1   ,   1   //  63
.byte   3   ,   2   //  64
.byte   3   ,   2   //  65
.byte   5   ,   2   //  66
.byte   5   ,   2   //  67
.byte   4   ,   1   //  68
.byte   2   ,   2   //  69
.byte   2   ,   1   //  6A
.byte   1   ,   1   //  6B
.byte   5   ,   3   //  6C
.byte   4   ,   3   //  6D
.byte   6   ,   3   //  6E
.byte   5   ,   3   //  6F
.byte   2   ,   2   //  70
.byte   5   ,   2   //  71
.byte   5   ,   2   //  72
.byte   1   ,   1   //  73
.byte   4   ,   2   //  74
.byte   4   ,   2   //  75
.byte   6   ,   2   //  76
.byte   5   ,   2   //  77
.byte   2   ,   1   //  78
.byte   4   ,   3   //  79
.byte   4   ,   1   //  7A
.byte   1   ,   1   //  7B
.byte   5   ,   3   //  7C
.byte   4   ,   3   //  7D
.byte   7   ,   3   //  7E
.byte   5   ,   3   //  7F
.byte   3   ,   2   //  80
.byte   6   ,   2   //  81
.byte   2   ,   2   //  82
.byte   1   ,   1   //  83
.byte   3   ,   2   //  84
.byte   3   ,   2   //  85
.byte   3   ,   2   //  86
.byte   5   ,   2   //  87
.byte   2   ,   1   //  88
.byte   2   ,   2   //  89
.byte   2   ,   1   //  8A
.byte   1   ,   1   //  8B
.byte   4   ,   3   //  8C
.byte   4   ,   3   //  8D
.byte   4   ,   3   //  8E
.byte   5   ,   3   //  8F
.byte   2   ,   2   //  90
.byte   6   ,   2   //  91
.byte   5   ,   2   //  92
.byte   1   ,   1   //  93
.byte   4   ,   2   //  94
.byte   4   ,   2   //  95
.byte   4   ,   2   //  96
.byte   5   ,   2   //  97
.byte   2   ,   1   //  98
.byte   5   ,   3   //  99
.byte   2   ,   1   //  9A
.byte   1   ,   1   //  9B
.byte   4   ,   3   //  9C
.byte   5   ,   3   //  9D
.byte   5   ,   3   //  9E
.byte   5   ,   3   //  9F
.byte   2   ,   2   //  A0
.byte   6   ,   2   //  A1
.byte   2   ,   2   //  A2
.byte   1   ,   1   //  A3
.byte   3   ,   2   //  A4
.byte   3   ,   2   //  A5
.byte   3   ,   2   //  A6
.byte   5   ,   2   //  A7
.byte   2   ,   1   //  A8
.byte   2   ,   2   //  A9
.byte   2   ,   1   //  AA
.byte   1   ,   1   //  AB
.byte   4   ,   3   //  AC
.byte   4   ,   3   //  AD
.byte   4   ,   3   //  AE
.byte   5   ,   3   //  AF
.byte   2   ,   2   //  B0
.byte   5   ,   2   //  B1
.byte   5   ,   2   //  B2
.byte   1   ,   1   //  B3
.byte   4   ,   2   //  B4
.byte   4   ,   2   //  B5
.byte   4   ,   2   //  B6
.byte   5   ,   2   //  B7
.byte   2   ,   1   //  B8
.byte   4   ,   3   //  B9
.byte   2   ,   1   //  BA
.byte   1   ,   1   //  BB
.byte   4   ,   3   //  BC
.byte   4   ,   3   //  BD
.byte   4   ,   3   //  BE
.byte   5   ,   3   //  BF
.byte   2   ,   2   //  C0
.byte   6   ,   2   //  C1
.byte   2   ,   2   //  C2
.byte   1   ,   1   //  C3
.byte   3   ,   2   //  C4
.byte   3   ,   2   //  C5
.byte   5   ,   2   //  C6
.byte   5   ,   2   //  C7
.byte   2   ,   1   //  C8
.byte   2   ,   2   //  C9
.byte   2   ,   1   //  CA
.byte   2   ,   1   //  CB
.byte   4   ,   3   //  CC
.byte   4   ,   3   //  CD
.byte   6   ,   3   //  CE
.byte   5   ,   3   //  CF
.byte   2   ,   2   //  D0
.byte   5   ,   2   //  D1
.byte   5   ,   2   //  D2
.byte   1   ,   1   //  D3
.byte   4   ,   2   //  D4
.byte   4   ,   2   //  D5
.byte   6   ,   2   //  D6
.byte   5   ,   2   //  D7
.byte   2   ,   1   //  D8
.byte   4   ,   3   //  D9
.byte   3   ,   1   //  DA
.byte   2   ,   1   //  DB
.byte   4   ,   3   //  DC
.byte   4   ,   3   //  DD
.byte   7   ,   3   //  DE
.byte   5   ,   3   //  DF
.byte   2   ,   2   //  E0
.byte   6   ,   2   //  E1
.byte   2   ,   2   //  E2
.byte   1   ,   1   //  E3
.byte   3   ,   2   //  E4
.byte   3   ,   2   //  E5
.byte   5   ,   2   //  E6
.byte   5   ,   2   //  E7
.byte   2   ,   1   //  E8
.byte   2   ,   2   //  E9
.byte   2   ,   1   //  EA
.byte   1   ,   1   //  EB
.byte   4   ,   3   //  EC
.byte   4   ,   3   //  ED
.byte   6   ,   3   //  EE
.byte   5   ,   3   //  EF
.byte   2   ,   2   //  F0
.byte   5   ,   2   //  F1
.byte   5   ,   2   //  F2
.byte   1   ,   1   //  F3
.byte   4   ,   2   //  F4
.byte   4   ,   2   //  F5
.byte   6   ,   2   //  F6
.byte   5   ,   2   //  F7
.byte   2   ,   1   //  F8
.byte   4   ,   3   //  F9
.byte   4   ,   1   //  FA
.byte   1   ,   1   //  FB
.byte   4   ,   3   //  FC
.byte   4   ,   3   //  FD
.byte   7   ,   3   //  FE
.byte   5   ,   3   //  FF

handle_irq:
   mov   r0,rPbyteLo
   lsr   r0,r0,#3  // check if 6502 IRQs are enabled
   BCS   handle_nextinstruction
handleinterrupt:
   INTR 2
handle_nextinstruction:
   NEXT_INSTRUCTION 0 noalign

handle_nmi:
   push_r   r2
   mov      r2,#2
   ldr      r0, =tube_irq
   CPSID i             // disable IRQs probably should disable FIQ ???
   ldrb     r1,[r0]     // load tube_irq again as it might have changed
   BIC     r1,r1,r2    //  NMI flag
   strb     r1,[r0]     // Store it back again
   CPSIE i             // re-enable ARM interrupts
   pop_r r2
   INTR    6
   NEXT_INSTRUCTION 0 noalign

adc_decimal:
   CLEARV_PART2
   lsl   r1,rAcc,#28
   lsr   r1,#28 // Get lower nibble only

   lsl   rflagsNZ,r0,#28
   lsr   rflagsNZ, rflagsNZ, #28// get other lower nibble

   lsr   rAcc,#4  // prep upper nibble
   lsl   rAcc,#28

   lsl   r0, r0, #24

   SETUP_CARRY rCarry // recover Carry
   adc   r1,rflagsNZ
   cmp   r1,#10
   bcc   adc_firstnibble_carry1   // lower than 10

   sub   r1,#10

   mov   rflagsNZ,#0xF
   and   r1,rflagsNZ // <- lower nibble

   ldr   rflagsNZ,=0x0FFFFFFF
   orr   r0,rflagsNZ
   orr   rAcc,rflagsNZ  // 0x0FFFFFFF

adc_firstnibble_carry1:

   add   rAcc,r0

   SAVE_VFLAG_FAST rflagsNZ
   ldr   rflagsNZ, =10<<28
   bcs   adc_wrap_upper_nibble1

   cmp   rAcc,rflagsNZ
   bcc   adc_nowrap_upper_nibble1

adc_wrap_upper_nibble1:
   SAVE_CARRY // Carry always set here
   sub   rAcc,rAcc,rflagsNZ
   b adc_skip_carry1

adc_nowrap_upper_nibble1:
   SAVE_CARRY  // Carry always clear here
adc_skip_carry1:
   lsr   rAcc, #28
   lsl   rAcc, #4
   orr   rAcc,rAcc,r1
   SET_FLAGS_NZ
   NEXT_INSTRUCTION 1 noalign

adc_decimal2:
   CLEARV_PART2
   lsl   r1,rAcc,#28
   lsr   r1,#28 // Get lower nibble only

   lsl   rflagsNZ,r0,#28
   lsr   rflagsNZ, rflagsNZ, #28// get other lower nibble

   lsr   rAcc,#4  // prep upper nibble
   lsl   rAcc,#28

   lsl   r0, r0, #24

   SETUP_CARRY rCarry // recover Carry
   adc   r1,rflagsNZ
   cmp   r1,#10
   bcc   adc_firstnibble_carry2   // lower than 10

   sub   r1,#10

   mov   rflagsNZ,#0xF
   and   r1,rflagsNZ // <- lower nibble

   ldr   rflagsNZ,=0x0FFFFFFF
   orr   r0,rflagsNZ
   orr   rAcc,rflagsNZ  // 0x0FFFFFFF

adc_firstnibble_carry2:

   add   rAcc,r0

   SAVE_VFLAG_FAST  r2
   ldr   rflagsNZ, =10<<28
   bcs   adc_wrap_upper_nibble2

   cmp   rAcc,rflagsNZ
   bcc   adc_nowrap_upper_nibble2

adc_wrap_upper_nibble2:
   SAVE_CARRY // Carry always set here
   sub   rAcc,rAcc,rflagsNZ
   b adc_skip_carry2

adc_nowrap_upper_nibble2:
   SAVE_CARRY  // Carry always clear here
adc_skip_carry2:
   lsr   rAcc, #28
   lsl   rAcc, #4
   orr   rAcc,rAcc,r1
   SET_FLAGS_NZ
   NEXT_INSTRUCTION 2 noalign

sbc_decimal :
   CLEARV_PART2
   push_r r5
   lsl   r1,rAcc,#28
   LSL   r5,r0,#28

   SETUP_CARRY rflagsNZ
   sbc   r1,r1,r5
   adc   r1,r1,r1   // save the carry for later

   lsl   rAcc,rAcc,#24 // 0xaa000000
   LSL   r0,r0,#24     // 0xzn000000

   SETUP_CARRY rflagsNZ
   sbc   rAcc,rAcc,r0

   SAVE_VFLAG_FAST rflagsNZ
   SAVE_CARRY  // corrupts carry flag

   lsr   rAcc,rAcc,#24

   SETUP_CARRY rflagsNZ
   bcs   sbc_fixup1
   sub   rAcc,rAcc,#0x60
sbc_fixup1:

   LSR r1,#1
   bcs sbc_positive1
   sub   rAcc,rAcc,#6
sbc_positive1:
   SET_FLAGS_NZ
   pop_r r5
   NEXT_INSTRUCTION 1 noalign

sbc_decimal2:
   CLEARV_PART2
   push_r r5
   lsl   r1,rAcc,#28
   LSL   r5,r0,#28

   SETUP_CARRY rflagsNZ
   sbc   r1,r1,r5
   adc   r1,r1,r1   // save the carry for later

   lsl   rAcc,rAcc,#24 // 0xaa000000
   LSL   r0,r0,#24     // 0xzn000000

   SETUP_CARRY rflagsNZ
   sbc   rAcc,rAcc,r0

   SAVE_VFLAG_FAST rflagsNZ
   SAVE_CARRY  // corrupts carry flag

   lsr   rAcc,rAcc,#24

   SETUP_CARRY rflagsNZ
   bcs   sbc_fixup2
   sub   rAcc, #0x60
sbc_fixup2:

   LSR   r1, #1
   bcs   sbc_positive2
   sub   rAcc, #6
sbc_positive2:
   SET_FLAGS_NZ
   pop_r r5
   NEXT_INSTRUCTION 2 noalign

// entry r0 = pointer to memory
//       r1 = 1 slow 6502
.global exec_65tube
.type exec_65tube,%function
.thumb_func
exec_65tube:
   push {r4-r7,r14}
   mov  r2,r8
   mov  r3,r9
   mov  r4,r10
   mov  r5,r11
   mov  r6,r12
   push {r2-r6}

   mov  rmem,r0

   mov  r2,#0xff  // set stack pointer to 0x1fe
   LSL  r2,r2,#1
   add  r2,r2,r0
   mov  rSP,r2

   ldr  r2,=(l_00)+1
   mov  insttable, r2

   mov  r1,r1
   BEQ  fast6502
   mov  r1,#(EVENT_HANDLER_FLAG+EVENT_HANDLER_SLOW_FLAG)>>8
   lsl  r1,r1,#8
   orr  r2,r2,r1
   mov  insttable,r2

fast6502:
   ldr   r0,=tube_enable_fast6502
   blx   r0
   ldr   r0, =(0xfef8)>>3
   mov   tregs, r0

   mov   rflagsNZ,#0  // Clear N and Z flag
   mov   rCarry, #0   // Clear C
   mov   rAcc,#0
   mov   rXreg,rAcc
   mov   rYreg,rAcc

// Set up cycling counting *****
   mov   r0,#5
   ldr   r1,=0xe000e010
   str   r0,[r1]

   mov   r0, #pByteIflag   // set I flag
   mov   rPbyteLo, r0  // Set I flag clear D B and V

   ldr   r0,=0xFFFC
   ldrh  rPC,[r0,rmem]  // load data at 0xFFFC ( we know it is aligned)
   add   rPC,rPC,rmem
   NEXT_INSTRUCTION 0 noalign

exec_65tube_exit:
   ldr   r0,=tube_disable_fast6502
   blx   r0
   pop   {r2-r6}
   mov   r8,r2
   mov   r9,r3
   mov   r10,r4
   mov   r11,r5
   mov   r12,r6
   pop   {r4-r7,pc}

.ltorg


/* it would be neat to enable tracing if the user pushbutton is pressed */
/* also to have other triggering conditions, such as instruction count, PC value, memory access */
/* we'll put up with an emulator (wdm) operation */
#ifdef TRACE6502
@ print newline and carriage return
.macro printnlcr
   push    {r0-r3}  @ could be clobbered
   movs     r0, #'\n'
   bl     putchar
   movs     r0, #'\r'
   bl     putchar
   pop     {r0-r3}
.endm
.align
counter:
   .word 0

trace6502code:
   push {r0-r3}
   // print PC
   push {r0-r2}
   ldr r1,=counter
   ldr r0,[r1]
   add r0,#1
   LSr r2,r0,#6
   CMP r2,#250
lockup:
   //BEQ lockup
   str   r0,[r1]
   pop     {r0-r2}

   push    {r0-r3}  @ could be clobbered
   mov     r0, rPC
   sub     r0, r0,#1
   sub     r0,r0,rmem
   bl     printhex16
   bl    printspace
   pop     {r0-r3}
   // print instruction

   push {r0-r3}  // in R0
   bl    printhex8
   bl    printspace
   pop {r0-r3}

   push {r0-r3}
   ldrb  r0,[rPC,#0]
   bl    printhex8
   bl    printspace
   pop {r0-r3}

   push {r0-r3}
   ldrb  r0,[rPC,#1]
   bl    printhex8
   bl    printspace
   bl    printspace
   pop {r0-r3}

// Print text version instruction

   push {r0-r3}
   mov  r0,rAcc
   bl    printhex8
   bl    printspace
   pop {r0-r3}

   push {r0-r3}
   mov  r0,rXreg
   bl    printhex8
   bl    printspace
   pop {r0-r3}

   push {r0-r3}
   mov  r0,rYreg
   bl    printhex8
   bl    printspace
   pop {r0-r3}

   statustoR0
   bl    printhex8
   bl    printspace

   mov   r0,rSP
   sub   r0,r0,rmem
   bl    printhex8
   bl    printspace

   printnlcr

   pop {r0-r3}
   LSL   r0, #I_ALIGN_BITS
   add   r0, r0,insttable
   bx    r0

printhex16:
   push {r0-r1,r14}
   mov r1,r0
   LSR r0,#8
   bl printhex8
   mov r0,r1
   bl printhex8
   pop  {r0-r1,pc}

printhex8:
   push {r0-r1,r14}
   mov r1,r0
   lsr r0,#4
   bl  printnibble
   mov r0,r1
   bl  printnibble
   pop {r0-r1,pc}

printnibble:
   push {r0-r3,r14}
   lsl  r0,r0,#28
   lsr r0,r0,#28 // clear upper bits
   CMP r0, #10
   blt nibble_less_than_10
   mov r1,#'a'-'0'-10
   add r0,r1
nibble_less_than_10:
   mov r1,#'0'
   add r0,r1
   BL putchar
   pop {r0-r3,pc}

printspace:
   push {r14}
   mov r0,#32
   BL putchar
   pop {pc}
.ltorg

#endif

handle_irq2:
   INTR    2
   b       execute_one_instruction

handle_nmi2:
   push_r  r2
   mov     r2,#2
   ldr      r0, =tube_irq
   CPSID i            // disable IRQs
   ldrb     r1,[r0]    // load tube_irq again as it might have changed
   BIC     r1,r1,r2   // clear NMI flag
   strb     r1,[r0]    // Store it back again
   CPSIE i            // re-enable ARM interrupts
   pop_r   r2
   INTR    6

execute_one_instruction:
   ldr   r1,=(l_00)+1
   ldr   r0,[rPC]
   LSL   r0,#I_ALIGN_BITS
   add   rPC,rPC,#1
   add   r0,r0,r1
   bx    r0

   .ltorg

// Slow event handler
.balign I_ALIGN*256 , 0
.rept 256
   ldr   r0, =tube_irq
   ldrb  r0,[r0]
   sub   rPC, rPC, #1        // set the instruction back as we haven't executed it

   mov   r1,#7
   tst   r0,r1
   bne   1f
   BL    slowdown
1:
   lsr   r1,r0,#3                // Bit 2 set indicate RST is active
   bcc   2f
   bl    exec_65tube_exit        // exit immediately if active edge seen
2:
   lsr   r1,r0,#2
   bcc   3f
   BL    handle_nmi2
3:
   lsr   r1,r0,#1
   bcc   4f
   mov   r0,rPbyteLo
   lsr   r0,r0,#3  // check if 6502 IRQs are enabled
   BCS   4f
   BL    handle_irq2
4:
   BL slowdown
   .ltorg
   INSTALIGN
.endr
