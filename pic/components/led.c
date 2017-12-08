#include <xc.h>
#include "led.h"

void init_led() {
    // Set LED ports to outputs
    TRISBbits.TRISB9 = 0; // G
    TRISBbits.TRISB8 = 0; // R
    TRISBbits.TRISB7 = 0; // B
}
