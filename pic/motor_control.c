#include <xc.h>

#include "components/led.h"

#include "adc.h"
#include "motor_control.h"
#include "pid.h"


// Global variables
static volatile unsigned int tmr_2_ticks;

static volatile signed int l_qei_counter;
static volatile signed int r_qei_counter;
static volatile signed int l_qei_last;
static volatile signed int r_qei_last;

// Global position, velocity, and acceleration variables
// Position
static volatile signed int l_pos_last_err;
static volatile signed int l_pos_integral;
// Velocity
static volatile signed int l_vel;
static volatile signed int l_vel_last_err;
static volatile signed int l_vel_integral;
// Acceleration
static volatile signed int l_acc;
static volatile signed int l_acc_last_err;
static volatile signed int l_acc_integral;

// Position
static volatile signed int r_pos_last_err;
static volatile signed int r_pos_integral;
// Velocity
static volatile signed int r_vel;
static volatile signed int r_vel_last_err;
static volatile signed int r_vel_integral;
// Acceleration
static volatile signed int r_acc;
static volatile signed int r_acc_last_err;
static volatile signed int r_acc_integral;


// Indication that a controller has completed its task, or reached a steady state.
volatile unsigned int controller_finished;

// Sensor readings
extern volatile unsigned int sl_sensor;
extern volatile unsigned int sr_sensor;
extern volatile unsigned int fl_sensor;
extern volatile unsigned int fr_sensor;



// Function to execute to control the motors every scan period.
static void (*motor_control)(void);

/*
 * 16-bit res @ 40MIPS = 1220 Hz edge-aligned
 * 11-bit res @ 40MIPS  39.1 kHz edge-aligned
 * 
 * Used res: Fpwm = 20 kHz
 */
void init_motor_control() {
    
    /* Right Motor QEI */
    RPINR14bits.QEA1R = 20; // QEIA Input tied to RP20
    RPINR14bits.QEB1R = 19; // QEIB Input tied to RP19
    /* Left Motor QEI */
    RPINR16bits.QEA2R = 5;
    RPINR16bits.QEB2R = 6;
    
    
    // Motors
    LATCbits.LATC6  = 1; // High for off
    LATBbits.LATB14 = 1;
    R_MTR_DIR = R_MTR_FWD; // direction
    L_MTR_DIR = L_MTR_FWD;
    LATAbits.LATA10 = 1; // Brakes
    LATCbits.LATC9  = 1;
    
 
    // Init pwm software module
    P1TMRbits.PTMR = 0; // Reset count to 0
    P2TMRbits.PTMR = 0; // Reset count to 0
    P1TPER = 1600; // Fcy / Fpwm -1 => 32 MHz / 20 kHz -1
    P2TPER = 1600; // Fcy / Fpwm -1 => 32 MHz / 20 kHz -1 = 1599
    PWM1CON1bits.PMOD1 = 1; // Independent PWM mode
    PWM2CON1bits.PMOD1 = 1; // Independent PWM mode
    PWM1CON2bits.IUE = 1; // Immediately update Period reg
    PWM2CON2bits.IUE = 1; // Immediately update Period reg
    
    L_MTR_PER = L_MTR_MIN; // Duty cycle -> 3200 is 0%, 0 is 100%
    R_MTR_PER = R_MTR_MIN; // Duty cycle
    
    PWM1CON1bits.PEN1H = 1; // PWM1H1 enabled for PWM
    PWM2CON1bits.PEN1H = 1; // PWM2H1 enabled for PWM
    
    
    // Init QEI software module
    QEI1CONbits.QEIM = 0b101; // Use 2x mode without indx or updn pins
    QEI2CONbits.QEIM = 0b101; // Use 2x mode without indx or updn pins
    
    QEI1CONbits.UPDN = 1; // Position counter direction is +
    QEI2CONbits.UPDN = 1; // Position counter direction is +
    
    DFLT1CONbits.CEID = 1; // Disable count error interrupts
    DFLT2CONbits.CEID = 1; // Disable count error interrupts
    
    QEI1CONbits.SWPAB = 1; // QEI1 swap so forward counts +
    
    R_QEI_CNT = 0; // Reset position counter to 0
    L_QEI_CNT = 0; // Reset position counter to 0
    
    R_QEI_MAX = R_QEI_ROT; // Resets every 5120 slots - 1 rotation
    L_QEI_MAX = L_QEI_ROT; // Resets every 5120 slots - 1 rotation
    
    // QEI Interrupt setup
    IPC14bits.QEI1IP = 4; // Priority lvl4
    IPC18bits.QEI2IP = 4; // Priority lvl4
    IFS3bits.QEI1IF = 0; // Clear flag
    IFS4bits.QEI2IF = 0; // Clear flag
    IEC3bits.QEI1IE = 0; // Disable interrupts
    IEC4bits.QEI2IE = 0; // Disable interrupts
    
    
    // Init timer 2 used for PID updating
    T2CONbits.TCKPS = 0b10; // 1:64 prescaler, ticks at 500000 Hz
    TMR2 = 0; // Clear TMR2 counter
    PR2 = 500; // Overflows every 1 ms (500000 / 1000) == 500 == 1 ms
    tmr_2_ticks = 0;
    
    // Configure TIMER 2 interrupts
    IPC1bits.T2IP = 5; // lvl 6 priority (second highest)
    IFS0bits.T2IF = 0; // clear IF flag
    IEC0bits.T2IE = 0; // Enable TRM2 interrupts
    
    // Clear global variables
    l_pos_last_err = 0;
    l_pos_integral = 0;
    l_vel = 0;
    l_vel_last_err = 0;
    l_vel_integral = 0;
    l_acc = 0;
    l_acc_last_err = 0;
    l_acc_integral = 0;
    
    r_pos_last_err = 0;
    r_pos_integral = 0;
    r_vel = 0;
    r_vel_last_err = 0;
    r_vel_integral = 0;
    r_acc = 0;
    r_acc_last_err = 0;
    r_acc_integral = 0;
}

void set_motor_control_function(void (*motor_control_ptr)(void)) {
    motor_control = motor_control_ptr;
}

void enable_motor_control() {
    
    tmr_2_ticks = 0;
    l_qei_counter = 0;
    r_qei_counter = 0;
    l_qei_last = 0;
    r_qei_last = 0;
    controller_finished = 0;
    
    P1TCONbits.PTEN = 1; // Turn on PWM time base
    P2TCONbits.PTEN = 1; // Turn on PWM time base
    
    IFS0bits.T2IF = 0; // Turn off interrupt flag
    IEC0bits.T2IE = 1; // Enable TRM2 interrupts
    
    IFS3bits.QEI1IF = 0; // Clear flag
    IFS4bits.QEI2IF = 0; // Clear flag
    IEC3bits.QEI1IE = 1; // Enable interrupts
    IEC4bits.QEI2IE = 1; // Enable interrupts
    
    T2CONbits.TON = 1; // Turn on timer2 
}



/*
 * MOTOR CONTROL FUNCTIONS
 * 
 * Each controller is coupled with an init function to set initial state.
 * Each controller also has coupled global state.
 */

void init_straight_controller() {
    // Does nothing
}
void straight_controller() {
    
    L_MTR_PER = L_MTR_MIN / 4;
    R_MTR_PER = R_MTR_MIN / 4;
}

void adjust_left_pwm(signed int adj);
void adjust_right_pwm(signed int adj);

static signed int R_POS_SP;
static signed int L_POS_SP;
void init_position_controller() {
    L_MTR_PER = L_MTR_MIN;
    R_MTR_PER = R_MTR_MIN;
    l_qei_counter = 0;
    r_qei_counter = 0;
    R_POS_SP = R_QEI_CNT;
    L_POS_SP = L_QEI_CNT;
    
}
void position_controller() {
    
    const double kp = 3; 
    const double ki = 0.01;
    const double kd = 13;
    
    const signed int r_err = 
        -r_qei_counter * R_QEI_MAX - (signed)R_QEI_CNT + R_POS_SP;
    const signed int l_err =
        -l_qei_counter * L_QEI_MAX - (signed)L_QEI_CNT + L_POS_SP;
        
    const signed int r_out = 
        P(r_err, kp) + 
        I(r_err, r_pos_integral, CONTROL_DT, ki) +
        D(r_err, r_pos_last_err, CONTROL_DT, kd);
    
    const signed int l_out = 
        P(l_err, kp) +
        I(l_err, l_pos_integral, CONTROL_DT, ki) +
        D(l_err, l_pos_last_err, CONTROL_DT, kd);
    
    // Save last err
    r_pos_last_err = r_err;
    l_pos_last_err = l_err;
        
    adjust_right_pwm(r_out);
    adjust_left_pwm(l_out);
}

void turn_left_controller() {
    
}

void turn_right_controller() {
    
}

void turn_around_controller() {
    
}


// Goal of this function is to drive the left motor at a speed relative to 
// the value of a sensor.
void simple_velocity_tester() {
    
    const double P_TERM = 5.0;
    const unsigned int SENSOR_SETPOINT = SLD_CLOSE;
    const unsigned L_SPEED_SETPOINT = L_MTR_MIN / 2;
    const unsigned R_SPEED_SETPOINT = R_MTR_MIN / 2;
    
    while(1) {
        const signed int sensor_err = SENSOR_SETPOINT - sl_sensor;
        const signed int p_adj = sensor_err * P_TERM;
        
        signed int l_mtr_per = L_SPEED_SETPOINT + p_adj;
        signed int r_mtr_per = R_SPEED_SETPOINT + p_adj;
        
        if(l_mtr_per > L_MTR_MIN) l_mtr_per = L_MTR_MIN;
        else if(l_mtr_per < L_MTR_MAX) l_mtr_per = L_MTR_MAX;
        
        if(r_mtr_per > R_MTR_MIN) r_mtr_per = R_MTR_MIN;
        else if(r_mtr_per < R_MTR_MAX) r_mtr_per = R_MTR_MAX;
        
        L_MTR_PER = l_mtr_per;
        R_MTR_PER = r_mtr_per;
    }
}


/**
 * Used to adjust the speed of the PWM signal for left motor.
 * @param adj Positive increases velocity, negative decreases.
 */
void adjust_left_pwm(signed int adj) {
    
    
    if(adj > 0) {
        // Trying to drive forward
        L_MTR_DIR = L_MTR_FWD;
        adj = L_MTR_MIN - adj;
        if(adj < L_MTR_MAX) adj = L_MTR_MAX;
        if(adj > L_MTR_MIN) adj = L_MTR_MIN;
        L_MTR_PER = adj;
        
    } else {
        // Trying to drive backwards
        L_MTR_DIR = L_MTR_REV;
        adj = L_MTR_MIN + adj;
        if(adj > L_MTR_MIN) adj = L_MTR_MIN;
        if(adj < L_MTR_MAX) adj = L_MTR_MAX;
        L_MTR_PER = adj;
    }
}

/**
 * Used to adjust the speed of the PWM signal for left motor.
 * @param adj Positive increases velocity, negative decreases.
 */
void adjust_right_pwm(signed int adj) {
    
    if(adj > 0) {
        // Trying to drive forward
        R_MTR_DIR = R_MTR_FWD;
        adj = R_MTR_MIN - adj;
        if(adj < R_MTR_MAX) adj = R_MTR_MAX;
            if(adj > R_MTR_MIN) adj = R_MTR_MIN;
            R_MTR_PER = adj;
        
    } else {
        // Trying to drive backwards
        R_MTR_DIR = R_MTR_REV;
        adj = R_MTR_MIN + adj;
        if(adj > R_MTR_MIN) adj = R_MTR_MIN;
        if(adj < R_MTR_MAX) adj = R_MTR_MAX;
        R_MTR_PER = adj;
    }
}


void simple_position_tester() {
    
    const double p_term = 7; 
    const signed int SETPOINT = 1024 * 5 / 2;
    L_MTR_PER = L_MTR_MIN;
    R_MTR_PER = R_MTR_MIN;
    
    // Set current position to setpoint.
    POS1CNT = (unsigned)SETPOINT;
    POS2CNT = (unsigned)SETPOINT;
    
    /**
     * A simple PD controller.
     */
    while(1) {
        
        const signed int r_err = SETPOINT - (signed)POS1CNT;
        const signed int l_err = SETPOINT - (signed)POS2CNT;
        
        const signed r_adj = p_term * r_err;
        const signed l_adj = p_term * l_err;
        
        adjust_right_pwm(r_adj);
        adjust_left_pwm(l_adj);
    }
}


// Timer in charge of motor control - PID adjustments - every 2 ms
void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void) {
    IFS0bits.T2IF = 0; // Clear T2 interrupt flag
    
    tmr_2_ticks++;
    // Make pid adjustments
    if(tmr_2_ticks == CONTROL_DT) {
        tmr_2_ticks = 0;
        
        // Calculate velocity and acceleration
        
        // Call current motor controller function
        (*motor_control)();
    }
}


// QEI overflows for right motor
void __attribute__((__interrupt__, no_auto_psv)) _QEI1Interrupt(void) {
    IFS3bits.QEI1IF = 0; // Clear flag
    
    if(R_QEI_CNT > R_QEI_ROT / 2) {
        // If CNT is now max, it underflowed - moving backwards
        r_qei_counter--;
    } else {
        // If CNT is now 0, it overflowed - moving forward
        r_qei_counter++;
    }
}

// QEI overflows for left motor
void __attribute__((__interrupt__, no_auto_psv)) _QEI2Interrupt(void) {
    IFS4bits.QEI2IF = 0; // Clear flag
    
    if(L_QEI_CNT > L_QEI_ROT / 2) {
        // If CNT is now max, it underflowed - moving backwards
        l_qei_counter--;
    } else {
        // If CNT is now 0, it overflowed - moving forward
        l_qei_counter++;
    }
}