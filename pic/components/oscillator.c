#include <xc.h>
#include "oscillator.h"


void init_oscillator() {
    // 7.37 MHz nominal
    // Dvi by 0.0737, 6 down from 11.25
    OSCTUNbits.TUN = 0b1100; // FRC +9%, FRC = 8.0333 MHz
    PLLFBD = 30; // PLL multiplier N+2 (128) M
    CLKDIVbits.PLLPOST = 0b00; // Output divider div_2
    // Fosc   = 64MHz
    // FcyMax = 40MHz = 40 MIPS
    // Fcy    = 32 MHz = 64 MHz / 2
}