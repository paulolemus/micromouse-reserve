#include <xc.h>
#include "encoder.h"

void init_encoders() {
    // QEI ports
    TRISCbits.TRISC4 = 1; //Set QEI Channels to inputs
    TRISCbits.TRISC3 = 1;
    TRISBbits.TRISB6 = 1;
    TRISBbits.TRISB5 = 1;
}