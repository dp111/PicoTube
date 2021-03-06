// tube-ula.h

#ifndef TUBE_ULA_H
#define TUBE_ULA_H

#include <inttypes.h>
#include "tube.h"
#include "tube-defs.h"

// Uncomment to checksum tube transfers
// #define DEBUG_TRANSFERS

// Uncomment to log all tube FIFO reads/writes (excluding status only)
// #define DEBUG_TUBE

extern volatile int tube_irq;

extern void disable_tube();

//extern void tube_host_read(uint16_t addr);

//extern void tube_host_write(uint16_t addr, uint8_t val);

void tube_ack_nmi(void);

extern uint8_t tube_parasite_read(uint32_t addr);

extern void tube_parasite_write(uint32_t addr, uint8_t val);

extern void tube_parasite_write_banksel(uint32_t addr, uint8_t val);

//extern void tube_reset();

extern void tube_io_handler(uint32_t mail);

extern void tube_init_hardware();

extern int tube_is_rst_active();

//extern void tube_wait_for_rst_active();

extern void tube_wait_for_rst_release();

extern void start_ula();

#endif
