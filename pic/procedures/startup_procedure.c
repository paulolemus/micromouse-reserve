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
extern volatile unsigned int sl_sensor;

// Mode to run sensor in
unsigned int mode_selected;

/**
 * Wait for the user to flash hand before operating.
 * It also keeps track of what mode you want to use by toggling the button.
 */
void startup_procedure() {
    
    // Initial config
    mode_selected = 0;
    brake();
    enable_adc();
    
    // wait for sensors to get close.
    while(fr_sensor < 900 && sr_sensor < 900) {
        
        LED_OFF(LED_R);
        LED_OFF(LED_G);
        LED_OFF(LED_B);
        
        // Poll the QEI readings to select mode.
        if(L_QEI_CNT < L_QEI_MAX * 0.33) {
            mode_selected = 0;
            LED_ON(LED_R);
        } else if(L_QEI_CNT > L_QEI_MAX * 0.66) {
            mode_selected = 1;
            LED_ON(LED_G);
        } else {
            mode_selected = 2;
            LED_ON(LED_B);
        }
    }
    
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
