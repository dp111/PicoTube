/*
 * Tube ULA Emulation
 *
 * (c) 2016 David Banks and Ed Spittles
 *
 * Based on code from B-em v2.2 by Tom Walker
 *
 */

#include <stdio.h>
#include <inttypes.h>
#include "tube-defs.h"
#include "tube.h"
#include "tube-ula.h"

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/irq.h"

inline void _disable_interrupts()
{
   __asm volatile ("cpsid i");
}

inline void _enable_interrupts()
{
   __asm volatile ("cpsie i");
}

// For predictable timing (i.e. stalling to to cache or memory contention)
// we need to find somewhere in I/O space to place the tube registers.
//
// This makes a MASSIVE improvement to the timing, compared to tube_regs
// being placed in L2 cached memory. With this change we have ~+50ns of
// setup margin, without it we had ~-150ns in the worst (rare) case.
//
// These locations must allow 32 bit reads/writes of arbitrary data
//
// See http://elinux.org/BCM2835_registers
//
// We have chosen MS_MBOX_0..MS_MBOX_7, which are 8 consecutive words.
//
// These locations are possibly the 8 words of the ARM-GPU mailbox,
// so we may need to change when we want to use mailboxes.
//
// Another option if we go back to 8-bit values tube_regs is to use
// CPG_Param0..CPG_Param1

int test_pin;
static int led_type=0;

extern uint8_t tube_regs[8];

#define HBIT_7 (1 << 7)
#define HBIT_6 (1 << 6)
#define HBIT_5 (1 << 5)
#define HBIT_4 (1 << 4)
#define HBIT_3 (1 << 3)
#define HBIT_2 (1 << 2)
#define HBIT_1 (1 << 1)
#define HBIT_0 (1 << 0)

#define BYTE_TO_WORD(data) (data)
#define WORD_TO_BYTE(data) (data)

static char copro_command =0;

static uint8_t ph1[24],ph3_1;
static uint8_t hp1,hp2,hp3[2],hp4;
static uint8_t pstat[4];
static uint8_t ph3pos,hp3pos;
static uint8_t ph1rdpos,ph1wrpos,ph1len;
volatile int tube_irq;

// Host end of the fifos are the ones read by the tube isr
#define PH1_0 tube_regs[1]
#define PH2   tube_regs[3]
#define PH3_0 tube_regs[5]
#define PH3_1 ph3_1
#define PH4   tube_regs[7]

// Host status registers are the ones read by the tube isr
#define HSTAT1 tube_regs[0]
#define HSTAT2 tube_regs[2]
#define HSTAT3 tube_regs[4]
#define HSTAT4 tube_regs[6]

// parasite status registers are local to this file
#define PSTAT1 pstat[0]
#define PSTAT2 pstat[1]
#define PSTAT3 pstat[2]
#define PSTAT4 pstat[3]

#if defined(USE_PIO)

#include "hardware/pio.h"
#include "bus6502.pio.h"

#define NUM_PINS 15

static void pio_init(PIO p0, PIO p1, uint pin) {

   // Load the Control program
   uint offset_control0 = pio_add_program(p0, &bus6502_control0_program);
   uint offset_control1 = pio_add_program(p0, &bus6502_control1_program);
   uint offset_control2 = pio_add_program(p0, &bus6502_control2_program);
   uint offset_control3 = pio_add_program(p0, &bus6502_control3_program);

   // Load the PINDIRS program
   uint offset_pindirs = pio_add_program(p1, &bus6502_pindirs_program);

   // Load the PINS program
   uint offset_pins = pio_add_program(p1, &bus6502_pins_program);

   // Load the READ program
   uint offset_a2 = pio_add_program(p1, &bus6502_a2_program);

   // Set the GPIO Function Select to connect the pin to the PIO
   for (uint i = 0; i < NUM_PINS; i++) {
      pio_gpio_init(p0, pin + i);
      pio_gpio_init(p1, pin + i);
   }

   // Set the default pindirs of all state machines to input
   for (uint sm = 0; sm < 4; sm++) {
      pio_sm_set_consecutive_pindirs(p0, sm, pin, NUM_PINS, false);
      pio_sm_set_consecutive_pindirs(p1, sm, pin, NUM_PINS, false);
   }

   // Configure P0 / SM0 (the control state machine)
   pio_sm_config c00 = bus6502_control0_program_get_default_config(offset_control0);
   sm_config_set_in_pins (&c00, pin + 12); // mapping for IN and WAIT (nRST)
   sm_config_set_jmp_pin (&c00, pin + 13); // mapping for JMP (nTUBE)
   sm_config_set_in_shift(&c00, true, false, 0); // shift right, no auto push
   pio_sm_init(p0, 0, offset_control0 + bus6502_control0_offset_entry_point, &c00);

   // Configure P0 / SM1 (the control state machine)
   pio_sm_config c01 = bus6502_control1_program_get_default_config(offset_control1);
   sm_config_set_in_pins (&c01, pin     ); // mapping for IN and WAIT
   sm_config_set_jmp_pin (&c01, pin + 11); // mapping for JMP (RnW)
   pio_sm_init(p0, 1, offset_control1 + bus6502_control1_offset_entry_point, &c01);

   // Configure P0 / SM2 (the control state machine)
   pio_sm_config c02 = bus6502_control2_program_get_default_config(offset_control2);
   sm_config_set_in_pins (&c02, pin     ); // mapping for IN and WAIT
   sm_config_set_jmp_pin (&c02, pin + 8 ); // mapping for JMP (A0)
   pio_sm_init(p0, 2, offset_control2 + bus6502_control2_offset_entry_point, &c02);

   // Configure P0 / SM3 (the control state machine)
   pio_sm_config c03 = bus6502_control3_program_get_default_config(offset_control3);
   sm_config_set_in_pins (&c03, pin     ); // mapping for IN and WAIT
   sm_config_set_in_shift(&c03, false, false, 0); // shift left, no auto push
   sm_config_set_fifo_join(&c03, PIO_FIFO_JOIN_RX);
   pio_sm_init(p0, 3, offset_control3 + bus6502_control3_offset_entry_point, &c03);

   // Configure P1 / SM0 (the Read state machine, detecting a2)
   pio_sm_config p1c0 = bus6502_a2_program_get_default_config(offset_a2);
   sm_config_set_in_pins (&p1c0, pin     ); // mapping for IN and WAIT
   sm_config_set_jmp_pin (&p1c0, pin + 10); // mapping for JMP (A2)
   pio_sm_init(p1, 0, offset_a2 + bus6502_a2_offset_entry_point, &p1c0);

   // Configure P1 / SM1 (the PINDIRS state machine controlling the direction of D7:0)
   pio_sm_config p1c1 = bus6502_pindirs_program_get_default_config(offset_pindirs);
   sm_config_set_in_pins (&p1c1, pin       ); // mapping for IN and WAIT
   sm_config_set_jmp_pin (&p1c1, pin + 11  ); // mapping for JMP (RnW)
   sm_config_set_out_pins(&p1c1, pin,     8); // mapping for OUT (D7:0)
   pio_sm_init(p1, 1, offset_pindirs + bus6502_pindirs_offset_entry_point, &p1c1);

   // Configure P1 / SM2 (the PIN state machine controlling the data output to D7:0)
   pio_sm_config p1c2 = bus6502_pins_program_get_default_config(offset_pins);
   sm_config_set_in_pins (&p1c2, pin + 8   ); // mapping for IN and WAIT (A1:0)
   sm_config_set_out_pins(&p1c2, pin,     8); // mapping for OUT (D7:0)
   sm_config_set_in_shift(&p1c2, true, false, 0); // shift right, no auto push
   pio_sm_init(p1, 2, offset_pins + bus6502_pins_offset_entry_point, &p1c2);

   // Configure P1/ SM3 (the PIN state machine controlling the data output to D7:0)
   pio_sm_config p1c3 = bus6502_pins_program_get_default_config(offset_pins);
   sm_config_set_in_pins (&p1c3, pin + 8   ); // mapping for IN and WAIT (A1:0)
   sm_config_set_out_pins(&p1c3, pin,     8); // mapping for OUT (D7:0)
   sm_config_set_in_shift(&p1c3, true, false, 0); // shift right, no auto push
   pio_sm_init(p1, 3, offset_pins + bus6502_pins_offset_entry_point, &p1c3);

   // Enable all the state machines
   for (uint sm = 0; sm < 4; sm++) {
      pio_sm_set_enabled(p0, sm, true);
   }
   for (uint sm = 0; sm < 4; sm++) {
      pio_sm_set_enabled(p1, sm, true);
   }
}

static inline void set_x(PIO pio, uint sm, uint32_t x) {
   // Write to the TX FIFO
   pio_sm_put(pio, sm, x);
   // execute: pull
   pio_sm_exec(pio, sm, pio_encode_pull(false, false));
}

static inline void FLUSH_TUBE_REGS() {
   uint32_t *p = (uint32_t *)(&tube_regs);
   set_x(pio1, 2, *p++);
   set_x(pio1, 3, *p++);
}

#else
#define FLUSH_TUBE_REGS(...)
#endif

void tube_enable_fast6502(void)
{
   _disable_interrupts();
   tube_irq |= FAST6502_BIT;
   _enable_interrupts();
}

void tube_disable_fast6502(void)
{
   _disable_interrupts();
   tube_irq &= ~FAST6502_BIT;
   _enable_interrupts();
}

void tube_ack_nmi(void)
{
   _disable_interrupts();
   tube_irq &= ~NMI_BIT;
   _enable_interrupts();
}

void copro_command_excute(unsigned char copro_command,unsigned char val)
{
    switch (copro_command)
    {
      case 0 :
          if (val == 0)
             copro_speed = 0;
          else
             copro_speed = (arm_speed/(1000000/256) / val);
          LOG_DEBUG("New Copro speed= %u, %u\r\n", val, copro_speed);
          return;
      case 1 : // *fx 151,226,1 followed by *fx 151,228,val
               // Select memory size
               copro = copro | 128 ;  // Set bit 7 to signal full reset of core
               return;
      default :
          break;
    }
}

static void tube_reset()
{
   tube_irq |= TUBE_ENABLE_BIT;
   tube_irq &= ~(RESET_BIT + NMI_BIT + IRQ_BIT);
   hp3pos = 0;
   ph1rdpos = ph1wrpos = ph1len = 0;
   ph3pos = 1;
   PSTAT1 = 0x40;
   PSTAT2 = 0x7F;
   PSTAT3 = PSTAT2;
   PSTAT4 = PSTAT2;
   HSTAT1 = HBIT_6;
   HSTAT2 = HBIT_6 | HBIT_5 | HBIT_4 | HBIT_3 | HBIT_2 | HBIT_1 | HBIT_0;
   HSTAT3 = HSTAT2 | HBIT_7;
   HSTAT4 = HSTAT2;
   // On the Model B the initial write of &8E to FEE0 is missed
   // If the Pi is slower in starting than the Beeb. A work around
   // is to have the tube emulation reset to a state with interrupts
   // enabled.
   HSTAT1 |= HBIT_3 | HBIT_2 | HBIT_1;
   //tube_updateints_IRQ();
   //tube_updateints_NMI();
   FLUSH_TUBE_REGS();
}

// 6502 Host reading the tube registers
//
// This function implements read-ahead, so the next values
// to be read are already pre-loaded into the tube_regs[]
// array ready for the FIQ handler to read without any delay.
// This is why there is no return value.
//
// Reading of status registers has no side effects, so nothing to
// do here for even registers (all handled in the FIQ handler).

static void __time_critical_func(tube_host_read)(uint32_t addr)
{
   switch (addr & 7)
   {
   case 1: /*Register 1*/
      if (ph1len > 0) {
         PH1_0 = BYTE_TO_WORD(ph1[ph1rdpos]);
         ph1len--;
         if ( ph1len != 0)
         {
            if (ph1rdpos== 23)
               ph1rdpos =0;
            else
               ph1rdpos++;
         }
         if (!ph1len) HSTAT1 &= ~HBIT_7;
         PSTAT1 |= 0x40;
      }
      break;
   case 3: /*Register 2*/
      if (HSTAT2 & HBIT_7)
      {
         HSTAT2 &= ~HBIT_7;
         PSTAT2 |=  0x40;
      }
      break;
   case 5: /*Register 3*/
      if (ph3pos > 0)
      {
         PH3_0 = BYTE_TO_WORD(PH3_1);
         ph3pos--;
         PSTAT3 |= 0xC0;
         if (!ph3pos) HSTAT3 &= ~HBIT_7;
         if ((HSTAT1 & HBIT_3) && (ph3pos == 0)) tube_irq|=NMI_BIT;
      }
      break;
   case 7: /*Register 4*/
      if (HSTAT4 & HBIT_7)
      {
         HSTAT4 &= ~HBIT_7;
         PSTAT4 |=  0x40;
      }
      break;
   }
   FLUSH_TUBE_REGS();
}

static void __time_critical_func(tube_host_write)(uint32_t addr, uint8_t val)
{
   switch (addr & 7)
   {
   case 0: /*Register 1 control/status*/

      if (!(tube_irq & TUBE_ENABLE_BIT))
         return;

      // Evaluate NMI before the control register written
      int nmi1 = 0;
      if (!(HSTAT1 & HBIT_4) && ((hp3pos > 0) || (ph3pos == 0))) nmi1 = 1;
      if ( (HSTAT1 & HBIT_4) && ((hp3pos > 1) || (ph3pos == 0))) nmi1 = 1;
      int nmi1_m = ((HSTAT1 & HBIT_3) && nmi1) ? 1 : 0;

      if (val & 0x80) {
         // Implement software tube reset
         if (val & 0x40) {
            tube_reset();
         } else {
            HSTAT1 |= BYTE_TO_WORD(val & 0x3F);
         }
      } else {
         HSTAT1 &= ~BYTE_TO_WORD(val & 0x3F);
      }


      if ( HSTAT1 & HBIT_5) {
         tube_irq |= RESET_BIT;
      } else {
         tube_irq &= ~RESET_BIT;
      }

      // Evaluate NMI again after the control register written
      int nmi2 = 0;
      if (!(HSTAT1 & HBIT_4) && ((hp3pos > 0) || (ph3pos == 0))) nmi2 = 1;
      if ( (HSTAT1 & HBIT_4) && ((hp3pos > 1) || (ph3pos == 0))) nmi2 = 1;
      int nmi2_m = ((HSTAT1 & HBIT_3) && nmi2) ? 1 : 0;

      // Ensure PSTAT3.7 (N) stays consistent with internal NMI when ever the control register is written
      // (e.g. if we switch between one and two byte mode)
      if (nmi2) {
         PSTAT3 |= 0x80;
      } else {
         PSTAT3 &= 0x7F;
      }

      // Only propagate significant rising edges
      if (!nmi1_m && nmi2_m) tube_irq |= NMI_BIT;

      // And disable regardless
      if (!nmi2_m) tube_irq &= ~(NMI_BIT);

      tube_irq &= ~(IRQ_BIT);
      if ((HSTAT1 & HBIT_1) && (PSTAT1 & 128)) tube_irq  |= IRQ_BIT;
      if ((HSTAT1 & HBIT_2) && (PSTAT4 & 128)) tube_irq  |= IRQ_BIT;

      break;
   case 1: /*Register 1*/
      hp1 = val;
      PSTAT1 |=  0x80;
      HSTAT1 &= ~HBIT_6;
      if (HSTAT1 & HBIT_1) tube_irq  |= IRQ_BIT;
      break;
   case 2:
      copro_command = val;
      break;
   case 3: /*Register 2*/
      hp2 = val;
      PSTAT2 |=  0x80;
      HSTAT2 &= ~HBIT_6;
      break;
   case 4:
      copro_command_excute(copro_command,val);
      break;
   case 5: /*Register 3*/
      if (HSTAT1 & HBIT_4)
      {
         if (hp3pos < 2)
            hp3[hp3pos++] = val;
         if (hp3pos == 2)
         {
            PSTAT3 |=  0x80;
            HSTAT3 &= ~HBIT_6;
         }
         if ((HSTAT1 & HBIT_3) && (hp3pos > 1)) tube_irq |= NMI_BIT;
      }
      else
      {
         hp3[0] = val;
         hp3pos = 1;
         PSTAT3 |=  0x80;
         HSTAT3 &= ~HBIT_6;
         if (HSTAT1 & HBIT_3) tube_irq |= NMI_BIT;
      }
      break;
   case 6:
      copro = val;
      LOG_DEBUG("New Copro = %u\r\n", copro);
      return;
   case 7: /*Register 4*/
      hp4 = val;
      PSTAT4 |=  0x80;
      HSTAT4 &= ~HBIT_6;
      if (HSTAT1 & HBIT_2) tube_irq |= IRQ_BIT;
      break;
   }
   FLUSH_TUBE_REGS();
}

uint8_t __time_critical_func(tube_parasite_read)(uint32_t addr)
{

   uint8_t temp ;
   switch (addr & 7)
   {
   case 0: /*Register 1 stat*/
      temp = PSTAT1 | (WORD_TO_BYTE(HSTAT1) & 0x3F);
      break;
   case 1: /*Register 1*/
       _disable_interrupts();
      temp = hp1;
      if (PSTAT1 & 0x80)
      {
         PSTAT1 &= ~0x80;
         HSTAT1 |=  HBIT_6;
         if (!(PSTAT4 & 128)) tube_irq &= ~IRQ_BIT;
      }
      _enable_interrupts();
      break;
   case 2: /*Register 2 stat*/
      temp = PSTAT2;
      break;
   case 3: /*Register 2*/
       _disable_interrupts();
      temp = hp2;
      if (PSTAT2 & 0x80)
      {
         PSTAT2 &= ~0x80;
         HSTAT2 |=  HBIT_6;
      }
      _enable_interrupts();
      break;
   case 4: /*Register 3 stat*/
      temp = PSTAT3;
      break;
   case 5: /*Register 3*/
      _disable_interrupts();
      temp = hp3[0];
      if (hp3pos>0)
      {
         hp3[0] = hp3[1];
         hp3pos--;
         if (!hp3pos)
         {
            HSTAT3 |=  HBIT_6;
            PSTAT3 &= ~0x80;
         }
         // here we want to only clear NMI if required
         if ( ( !(ph3pos == 0) ) && ( (!(HSTAT1 & HBIT_4) && (!(hp3pos >0))) || (HSTAT1 & HBIT_4) ) ) tube_irq &= ~NMI_BIT;
      }
      _enable_interrupts();
      break;
   case 6: /*Register 4 stat*/
      temp = PSTAT4;
      break;
   case 7: /*Register 4*/
       _disable_interrupts();
      temp = hp4;
      if (PSTAT4 & 0x80)
      {
         PSTAT4 &= ~0x80;
         HSTAT4 |=  HBIT_6;
         if (!(PSTAT1 & 128)) tube_irq &= ~IRQ_BIT;
      }
      _enable_interrupts();
      break;
   }
   if (addr & 1) {
      FLUSH_TUBE_REGS();
   }
   return temp;
}

// Special IO write wrapper for the 65Tube Co Pro:
// - the tube registers are accessed at 0xFEF8-0xFEFF
// - the bank select registers are accessed at 0xFEE0-0xFEE7
void tube_parasite_write_banksel(uint32_t addr, uint8_t val)
{

}

void __time_critical_func(tube_parasite_write)(uint32_t addr, uint8_t val)
{
   _disable_interrupts();

   switch (addr & 7)
   {
   case 1: /*Register 1*/
      if (ph1len < 24)
      {
         if (ph1len == 0) {
            PH1_0 = BYTE_TO_WORD(val);
         } else {
            ph1[ph1wrpos] = val;
            if (ph1wrpos== 23)
               ph1wrpos =0;
            else
               ph1wrpos++;
         }

         ph1len++;
         HSTAT1 |= HBIT_7;
         if (ph1len == 24) PSTAT1 &= ~0x40;
      }
      break;
   case 3: /*Register 2*/
      PH2 = BYTE_TO_WORD(val);
      HSTAT2 |=  HBIT_7;
      PSTAT2 &= ~0x40;
      break;
   case 5: /*Register 3*/
      if (HSTAT1 & HBIT_4)
      {
         if (ph3pos < 2) {
            if (ph3pos == 0) {
               PH3_0 = BYTE_TO_WORD(val);
            } else {
               PH3_1 = val;
            }
            ph3pos++;
         }
         if (ph3pos == 2)
         {
            HSTAT3 |=  HBIT_7;
            PSTAT3 &= ~0xC0;
         }
         //NMI if other case isn't setting it
         if (!(hp3pos > 1) ) tube_irq &= ~NMI_BIT;
      }
      else
      {
         PH3_0 = BYTE_TO_WORD(val);
         ph3pos = 1;
         HSTAT3 |=  HBIT_7;
         PSTAT3 &= ~0xC0;
         //NMI if other case isn't setting it
         if (!(hp3pos > 0) ) tube_irq &= ~NMI_BIT;
      }

      break;
   case 7: /*Register 4*/
      PH4 = BYTE_TO_WORD(val);
      HSTAT4 |=  HBIT_7;
      PSTAT4 &= ~0x40;
      break;
   }
   FLUSH_TUBE_REGS();
   _enable_interrupts();
}

// Returns bit 0 set if IRQ is asserted by the tube
// Returns bit 1 set if NMI is asserted by the tube
// Returns bit 2 set if RST is asserted by the host or tube

void __time_critical_func(tube_io_handler)(uint32_t mail)
{
   if (((mail >> NRST_PIN) & 1) == 0)        // Check for Reset
   {
      tube_irq |= RESET_BIT;
   }
   else
   {
      uint32_t addr = (mail>>A0_PIN) & 7;
      if ( ( (mail >>RNW_PIN ) & 1) == 0) {  // Check read write flag
         tube_host_write(addr, mail );
      } else {
         tube_host_read(addr);
      }
   }
}


void tube_init_hardware()
{

#ifdef USE_PIO
   pio_init(pio0, pio1, 0);
#else
   gpio_init(D0_PIN);
   gpio_init(D1_PIN);
   gpio_init(D2_PIN);
   gpio_init(D3_PIN);
   gpio_init(D4_PIN);
   gpio_init(D5_PIN);
   gpio_init(D6_PIN);
   gpio_init(D7_PIN);
   gpio_init(A0_PIN);
   gpio_init(A1_PIN);
   gpio_init(A2_PIN);
   gpio_init(RNW_PIN);
   gpio_init(NRST_PIN);
   gpio_init(NTUBE_PIN);
   gpio_init(PHI2_PIN);
#endif

   //gpio_init(18);
   //gpio_init(20);

   hp1 = hp2 = hp4 = hp3[0]= hp3[1]=0;

}

int tube_is_rst_active() {
   return (gpio_get(NRST_PIN)==0);
}

// Debounce RST

// On my Model B the characterisc of RST bounce on release is a burst
// of short (2us) high pulses approx 2ms before a clean rising RST edge

// On my Master 128 there is no RST bounce, and RST is clean

// This debounce code waits for RST to go high and stay high for a period
// set by DEBOUNCE_TIME (a of 10000 on a Pi 3 was measured at 690us)

// On the Model B
// - the first tube accesses are ~15ms after RST is released

// On the Master
// - the first tube accesses are ~13ms after RST is released

#define DEBOUNCE_TIME 10000

void tube_wait_for_rst_release() {
   volatile int i;
   do {
      // Wait for reset to be released
      while (tube_is_rst_active());
      // Make sure RST stays continuously high for a further 100us
      for (i = 0; i < DEBOUNCE_TIME && !tube_is_rst_active(); i++);
      // Loop back if we exit the debouce loop prematurely because RST has gone active again
   } while (i < DEBOUNCE_TIME);
   // Reset all the TUBE ULA registers
   tube_reset();
}

void tube_reset_performance_counters() {
}

void tube_log_performance_counters() {
}

void disable_tube() {
   int i;
   tube_irq &= ~TUBE_ENABLE_BIT;
   for (i = 0; i < 8; i++) {
      tube_regs[i] = 0xfe;
   }
}

void start_ula()
{
#ifdef USE_PIO
   irq_set_exclusive_handler(PIO0_IRQ_0, picofifo);
   irq_set_enabled(PIO0_IRQ_0, true);
   pio0->inte0 = PIO_IRQ0_INTE_SM3_RXNEMPTY_BITS;
#else
   multicore_launch_core1(picotubecore);
   irq_set_exclusive_handler(SIO_IRQ_PROC0, picofifo);
   irq_set_enabled(SIO_IRQ_PROC0, true);
#endif

}
