// tube-defs.h

#ifndef TUBE_DEFS_H
#define TUBE_DEFS_H

#define RELEASENAME "black-dev"

//#define NDEBUG

#ifdef DEBUG
#define LOG_DEBUG(...) printf(__VA_ARGS__)
#else
#define LOG_DEBUG(...)
#endif

#define LOG_INFO(...) printf(__VA_ARGS__)

#define LOG_WARN(...) printf(__VA_ARGS__)

// Certain Co Pro numbers need to be pre-defined, as tube-client.c special cases these
// (define these as needed)
#define COPRO_65TUBE_0   0
#define COPRO_65TUBE_1   1
#define COPRO_65TUBE_2   2
#define COPRO_65TUBE_3   3

#define DEFAULT_COPRO COPRO_65TUBE_0

//
// tube_irq bit definitions
//
// bit 7 Selects if R7 is used to inform the copro of an interrupt event used for fast 6502
// bit 6 Selects if direct native arm irq are used
// bit 5 native arm irq lock
// bit 3 tube_enable
// bit 2 Reset event
// bit 1 NMI
// bit 0 IRQ

#define FAST6502_BIT 128
#define NATIVEARM_BIT 64
#define nativearmlock_bit 32
#define TUBE_ENABLE_BIT  8
#define RESET_BIT   4
#define NMI_BIT     2
#define IRQ_BIT     1

#define D7_PIN       (7)
#define D6_PIN       (6)
#define D5_PIN       (5)
#define D4_PIN       (4)
#define D3_PIN       (3)
#define D2_PIN       (2)
#define D1_PIN       (1)
#define D0_PIN       (0)

#define A2_PIN       (10)
#define A1_PIN       (9)
#define A0_PIN       (8)

#define PHI2_PIN     (14)
#define NTUBE_PIN    (13)
#define NRST_PIN     (12)
#define RNW_PIN      (11)

#define PHI2_MASK    (1 << PHI2_PIN)
#define NTUBE_MASK   (1 << NTUBE_PIN)
#define NRST_MASK    (1 << NRST_PIN)
#define RNW_MASK     (1 << RNW_PIN)

#define TEST_PIN     (21)
#define TEST_MASK    (1 << TEST_PIN)
#define TEST2_PIN    (20)
#define TEST2_MASK   (1 << TEST2_PIN)
#define TEST3_PIN    (16)
#define TEST3_MASK   (1 << TEST3_PIN)

#endif
