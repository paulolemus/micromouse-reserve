/* 
 * File:   pid.h
 * Author: paulo
 *
 * Created on December 7, 2017, 11:50 PM
 */

#ifndef PID_H
#define	PID_H

#ifdef	__cplusplus
extern "C" {
#endif

    // FINISHED
#define P(err, kp) (kp * err)
    
    // FINISHED - THIS SAVES NEW INTEGRAL TO integral AS WELL
#define I(err, integral, dt, ki) ((integral = integral + err * dt) * ki)
    
    // FINISHED - DONT FORGET TO SAVE ERR TO PREV_ERR AFTER
#define D(err, prev_err, dt, kd) (((err - prev_err) / dt) * kd)


#ifdef	__cplusplus
}
#endif

#endif	/* PID_H */

