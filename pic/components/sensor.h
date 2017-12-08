/* 
 * File:   sensor.h
 * Author: paulo
 *
 * Created on December 7, 2017, 1:22 AM
 */

#ifndef SENSOR_H
#define	SENSOR_H

#ifdef	__cplusplus
extern "C" {
#endif


// Emitters
#define FLE LATAbits.LATA4
#define SLE LATAbits.LATA9
#define SRE LATBbits.LATB4
#define FRE LATAbits.LATA8
    
    
    
    void init_sensors();


#ifdef	__cplusplus
}
#endif

#endif	/* SENSOR_H */

