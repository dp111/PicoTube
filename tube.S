#include "tube-defs.h"
#include "copro-65tubeasm.h"

.text

.global arm_irq_handler
.global arm_fiq_handler
.global arm_fiq_handler_flag1

// =================================================
// ISR CODE
// =================================================

#define GPIO_IN         0x04
#define GPIO_OUT_SET    0x14
#define GPIO_OUT_CLR    0x18
#define GPIO_OE_SET     0x24
#define GPIO_OE_CLR     0x28

#define FIFO_WR         0x54
#define FIFO_RD         0x58

#define DEBUG_PIN1      18
#define DEBUG_PIN2      20

.section .scratch_y.tubeirq, "ax"
.global picofifo
.type picofifo,%function
.thumb_func
picofifo:
      push {lr}

#ifdef USE_PIO

      mov   r1,#0x50
      lsl   r1,#24
      mov   r0,#0x20
      lsl   r0,#16
      orr   r1, r0
      LDR   r0,[r1,#0x2C] // read data out of RX FIFO for PIO0 SM3

#else

      mov   r1,#0xd0
      lsl   r1,#24
      LDR   r0,[r1,#FIFO_RD] // read data out of fifo

      mov   r2,#15
      str   r2,[r1,#0x50] // Clear FIFO errors
#endif

      BLX   tube_io_handler

      LDR   r0,=tube_irq
      LDRb  r0,[r0]

      LSR   r1,r0,#8 // Get FAST_6502 bit into carry
      BCC   picofifoexit
      mov   r1,#RESET_BIT+NMI_BIT+IRQ_BIT
      TST   r1,r0
      BEQ   picofifoexit
      mov   r1,#copro6502asm_instruction_size>>8
      lsl   r1,#8
      mov   r0,copro6502asm_instruction_table
      orr   r0,r0,r1
      mov   copro6502asm_instruction_table,r0

picofifoexit:
      pop {pc}


// picotubecore
//

.section .scratch_x.tubecore, "ax"
.global tube_regs
.global picotubecore
#ifndef USE_PIO
.type picotubecore,%function
.thumb_func
picotubecore:
      mov   r4, #0xd0            // GPIO Base address
      lsl   r4, #24
      adr   r5, tube_regs        // Shared memory base address 8x bytes
      mov   r7, #255             // GPIO bits

poll_tube_low:

poll_tube_low_loop:
      ldr   r0, [r4,#GPIO_IN]    // Read GPIO
      LSR   r1, r0,#PHI2_PIN+1
      BCC   poll_tube_low_loop

      LSR   r1, r0,#NRST_PIN+1
      BCC   tube_reset

      LSR   r1, r0,#NTUBE_PIN+1
      BCS   poll_tube_low_loop

      LSR   r1, r0,#RNW_PIN+1
      BCC   write_cycle

// so we are in a read cycle

      LSL   r1, r0,#31-A2_PIN
      LSR   r1, r1,#(A0_PIN+(31-A2_PIN))    // Get just the address bits ( clear higher bits)
      LDRB  r2, [r5,r1]             // tube regs
      STR   r2, [r4,#GPIO_OUT_SET]  // Write data to bus
      str   r7, [r4,#GPIO_OE_SET]   // Make bus an output

      LSR   r1, r0,#A0_PIN+1
      BCC   read_wait_for_clk_low   // status registers aren't read sensitive so don't post mail

      str   r0, [r4,#FIFO_WR]       // Post mail FIFO

read_wait_for_clk_low:
      ldr   r0, [r4,#GPIO_IN]
      LSR   r1, r0,#PHI2_PIN+1
      BCS   read_wait_for_clk_low

      str   r7, [r4,#GPIO_OE_CLR]    // Stop driving the databus
      str   r7, [r4,#GPIO_OUT_CLR]   // clear databus to zero ready for next time

// detect dummy read
read_wait_for_clk_high2:
      ldr   r0, [r4,#GPIO_IN]  // Read GPIO
      LSR   r1, r0,#PHI2_PIN+1
      BCC   read_wait_for_clk_high2
      LSR   r1, r0,#NTUBE_PIN+1
      BCS   poll_tube_low
      LSR   r1, r0,#RNW_PIN+1
      BCS   poll_tube_low

wrloop:
      ldr   r0, [r4,#GPIO_IN]  // Read GPIO
      LSR   r1, r0,#PHI2_PIN+1
      BCC   wrloop

write_cycle:
      ldr   r0, [r4,#GPIO_IN]  // Read GPIO
      LSR   r1, r0,#PHI2_PIN+1
      BCS   write_cycle

      // check clock edge is real and not a glitch
      ldr   r1, [r4,#GPIO_IN]  // Read GPIO
      LSR   r1, r1,#PHI2_PIN+1
      BCS   write_cycle

      str   r0, [r4,#FIFO_WR] // FIFO
      b     poll_tube_low

tube_reset:
// Post mail signal reset
      str   r0, [r4, #FIFO_WR] // FIFO
post_reset_loop:
      ldr   r0, [r4, #GPIO_IN]  // Read GPIO
      LSR   r1, r0, #NRST_PIN+1
      BCC   post_reset_loop
      b     poll_tube_low
#endif

.align
tube_regs: // 8 bytes
      .word 0
      .word 0
