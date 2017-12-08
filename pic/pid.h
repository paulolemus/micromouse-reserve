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

    
#define P(err, kp) 5
    
#define I(err, prev_int, dt, ki) 6
    
#define D(err, prev_err, dt, kd) 7


#ifdef	__cplusplus
}
#endif

#endif	/* PID_H */

