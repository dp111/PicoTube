#include <stdio.h>
#include <stdlib.h>
#include "tube-defs.h"

/*
 * If the elk_mode property is non-zero then patch
 * STA &FEE5 => STA &FCE5 (&8D)
 * LDA &FEE5 => LDA &FCE5 (&AD)
 */

void check_elk_mode_and_patch(unsigned char *rom, int start, int len, int expected) {
#ifdef ELK_MODE

      int actual = 0;
      for (int i = start; i < start + len - 2; i++) {
         if ((rom[i] == 0x8D) && (rom[i + 1] == 0xE5) && (rom[i + 2] == 0xFE)) {
            LOG_DEBUG("Patching STA &FEE5 to STA &FCE5 at %04x\r\n", i);
            rom[i + 2] = 0xFC;
            actual ++;
         }
         if ((rom[i] == 0xAD) && (rom[i + 1] == 0xE5) && (rom[i + 2] == 0xFE)) {
            LOG_DEBUG("Patching LDA &FEE5 to STA &FCE5 at %04x\r\n", i);
            rom[i + 2] = 0xFC;
            actual ++;
         }
      }
      if (actual == expected) {
         LOG_INFO("Elk client ROM patching successful\r\n");
      } else {
         LOG_WARN("Elk client ROM patching failed (expected = %d, actual = %d)\r\n", expected, actual);
      }
#endif
}
