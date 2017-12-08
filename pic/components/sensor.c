#include <xc.h>
#include "sensor.h"

void init_sensors() {
    // Emitter ports
    TRISAbits.TRISA4 = 0; // FLE
    TRISAbits.TRISA9 = 0; // SLE
    TRISBbits.TRISB4 = 0; // SRE
    TRISAbits.TRISA8 = 0; // FRE
}