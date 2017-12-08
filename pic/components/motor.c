#include <xc.h>
#include "motor.h"

void init_motors() {
    
    // Motor ports
    TRISBbits.TRISB14 = 0; // high to disable left
    TRISCbits.TRISC6  = 0; // high to disable right
    TRISCbits.TRISC8  = 0; // Left  direction
    TRISAbits.TRISA7  = 0; // Right direction
    TRISCbits.TRISC9  = 0; // Left brake
    TRISAbits.TRISA10 = 0; // Right brake
}