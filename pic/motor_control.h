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
#define R_MTR_REV 0b1
#define L_MTR_REV 0b0
    
// PWM duty cycle and period macros
#define R_MTR_PER P1DC1
#define L_MTR_PER P2DC1
#define R_MTR_MAX 0
#define R_MTR_MIN 3200
#define L_MTR_MAX 0
#define L_MTR_MIN 3200
    
// QEI macros
#define L_QEI_CNT POS2CNT // Counter changed by qei module
#define R_QEI_CNT POS1CNT // Counter changed by qei module
#define L_QEI_MAX MAX2CNT // value to reach before reset
#define R_QEI_MAX MAX1CNT // Value to reach before reset
#define L_QEI_ROT 1024 * 5 // 1 rotation
#define R_QEI_ROT 1024 * 5 // 1 rotation



    void init_motor_control();
    
    void enable_motor_control();
    
    /**
     * Use this function to set the function to be called each loop.
     * @param motor_control_ptr a pointer to a function.
     */
    void set_motor_control_function(void (*motor_control_ptr)(void));
    
    // Motor control functions
    void init_straight_controller();
    void straight_controller();
    
    void init_position_controller();
    void position_controller();
    
    void turn_left_controller();
    void turn_right_controller();
    void turn_around_controller();

    // Testing functions for the motors
    void simple_velocity_tester();
    void simple_position_tester();



#ifdef	__cplusplus
}
#endif

#endif	/* MOTOR_CONTROL_H */

