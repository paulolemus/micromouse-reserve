#include <xc.h>
#include "startup_procedure.h"
#include "../adc.h"
#include "../components/led.h"
#include "../wait.h"
#include "../motor_control.h"


/**
 * Wait for the user to cover both the front and right sensors.
 * Then we flash the lights in a cirle three times before ending.
 */
extern volatile unsigned int fr_sensor;
extern volatile unsigned int sr_sensor;
void startup_procedure() {
    
    // Initial config
    brake();
    enable_adc();
    
    // Turn off lights.
    LED_OFF(LED_R);
    LED_OFF(LED_G);
    LED_OFF(LED_B);
    
    // wait for sensors to get close.
    while(fr_sensor < 900 && sr_sensor < 900);
    
    // Mini lightshow
    const unsigned ms = 75;
    int i;
    for(i = 0; i < 4; ++i) {
        LED_ON(LED_R);
        wait_ms(ms);
        
        LED_OFF(LED_R);
        LED_ON(LED_G);
        wait_ms(ms);
        
        LED_OFF(LED_G);
        LED_ON(LED_B);
        wait_ms(ms);
        
        LED_OFF(LED_B);
    }
}
