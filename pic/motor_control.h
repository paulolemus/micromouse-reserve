/* 
 * File:   motor_control.h
 * Author: paulo
 *
 * Created on December 7, 2017, 1:44 AM
 */

#ifndef MOTOR_CONTROL_H
#define	MOTOR_CONTROL_H

#ifdef	__cplusplus
extern "C" {
#endif
    
    
// Motor macros
#define R_MTR_DIR LATAbits.LATA7
#define L_MTR_DIR LATCbits.LATC8
#define R_MTR_FWD 0b0
#define L_MTR_FWD 0b1
#define R_MTR_PER P1DC1
#define L_MTR_PER P2DC1
#define R_MTR_MAX 0
#define R_MTR_MIN 3200
#define L_MTR_MAX 0
#define L_MTR_MIN 3200



    void init_motor_control();
    
    void enable_motor_control();
    
    
    // Testing functions for the motors
    void simple_velocity_controller();
    void simple_position_controller();



#ifdef	__cplusplus
}
#endif

#endif	/* MOTOR_CONTROL_H */

