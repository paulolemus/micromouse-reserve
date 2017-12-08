/* 
 * File:   led.h
 * Author: paulo
 *
 * Created on December 7, 2017, 1:15 AM
 */

#ifndef LED_H
#define	LED_H

#ifdef	__cplusplus
extern "C" {
#endif
    
    
// LED PORTS
#define LED_R LATBbits.LATB8
#define LED_G LATBbits.LATB9
#define LED_B LATBbits.LATB7
    
#define LED_ON(led_lat) led_lat = 1
#define LED_OFF(led_lat) led_lat = 0
#define LED_TOGGLE(led_lat) led_lat = ~led_lat

    
    void init_led();



#ifdef	__cplusplus
}
#endif

#endif	/* LED_H */

