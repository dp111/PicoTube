#include <stdio.h>
#include "pico/stdlib.h"
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "tube-defs.h"
#include "tube.h"
#include "tube-ula.h"

typedef void (*func_ptr)();

extern int test_pin;

#include "copro-65tube.h"
#include "copro-null.h"
#include "hardware/regs/clocks.h"
#include "hardware/platform_defs.h"
#include "hardware/resets.h"
#include "hardware/clocks.h"
#include "hardware/watchdog.h"
#include "hardware/pll.h"
#include "hardware/xosc.h"
#include "pico/stdlib.h"

static const char * emulator_names[] = {
   "65C02 (fast)",           // 0
   "65C02 (3MHz)",           // 1
   "65C102 (fast)",          // 2
   "65C102 (4MHz)",          // 3
   "Null",                   // 5
   "Null",                   // 6
   "Null",                   // 7
   "Null",                   // 8
   "Null",                   // 9
   "Null",                   // 10
   "Null",                   // 11
   "Null",                   // 12
   "Null"                    // 13
   "Null",                   // 14
   "Null",                   // 15
   "Null",                   // 16
   "Null",                   // 17
   "Null",                   // 18
   "Null",                   // 19
   "Null",                   // 20
   "Null",                   // 21
   "Null"                    // 22
   "Null",                   // 23
   "Null",                   // 24
   "Null",                   // 25
   "Null",                   // 26
   "Null",                   // 27
   "Null",                   // 28
   "Null",                   // 29
   "Null",                   // 30
   "Null"                    // 31
};

static const func_ptr emulator_functions[] = {
   copro_65tube_emulator,    // 0
   copro_65tube_emulator,    // 1
   copro_65tube_emulator,    // 2
   copro_65tube_emulator,    // 3
   copro_null_emulator,      // 4
   copro_null_emulator,      // 5
   copro_null_emulator,      // 6
   copro_null_emulator,      // 7
   copro_null_emulator,      // 8
   copro_null_emulator,      // 9
   copro_null_emulator,      // 10
   copro_null_emulator,      // 11
   copro_null_emulator,      // 12
   copro_null_emulator,      // 13
   copro_null_emulator,      // 14
   copro_null_emulator,      // 15
   copro_null_emulator,      // 16
   copro_null_emulator,      // 17
   copro_null_emulator,      // 18
   copro_null_emulator,      // 19
   copro_null_emulator,      // 20
   copro_null_emulator,      // 21
   copro_null_emulator,      // 22
   copro_null_emulator,      // 23
   copro_null_emulator,      // 24
   copro_null_emulator,      // 25
   copro_null_emulator,      // 26
   copro_null_emulator,      // 27
   copro_null_emulator,      // 28
   copro_null_emulator,      // 29
   copro_null_emulator,      // 30
   copro_null_emulator       // 31
};

volatile unsigned int copro;
volatile unsigned int copro_speed;

int arm_speed = 133;

static func_ptr emulator;

unsigned char mpu_memory[64*1024];

unsigned char * copro_mem_reset(int length)
{
   // Wipe memory
   memset(mpu_memory, 0, length);

   // return pointer to memory
   return mpu_memory;
}

void init_emulator() {
   emulator = emulator_functions[copro];
}

static unsigned int get_copro_number() {
   unsigned int copro = DEFAULT_COPRO ;
   return copro;
}

static void get_copro_speed() {
   copro_speed = 0; // default
   // Note: Co Pro Speed is only implemented in the 65tube Co Processors (copros 0/1/2/3)
   if (copro == COPRO_65TUBE_1) {
      copro_speed = 3; // default to 3MHz (65C02)
   } else if (copro == COPRO_65TUBE_3) {
      copro_speed = 4; // default to 4MHz (65C102)
   }
}

#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 16
#define UART_RX_PIN 17

void main(void)
{
   int last_copro = -1;
    
   set_sys_clock_khz( arm_speed * 1000, false);

   stdio_uart_init_full(UART_ID, BAUD_RATE, UART_TX_PIN, UART_RX_PIN);

   tube_init_hardware();

   start_ula();

   init_emulator();

  do {

     // When changing Co Processors, reset the Co Pro speed and memory back to the
     // configured default for that Co Pro.
     if (copro != last_copro) {
        get_copro_speed();
     }
     last_copro = copro;

     // Run the emulator
     printf("Pico Tube start\r\n");
     emulator();

     // Clear top bit which is used to signal full reset
     copro &= 127 ;

     // Reload the emulator as copro may have changed
     init_emulator();

  } while (1);

}
