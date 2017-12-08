#include <xc.h>
#include "motor_control.h"
#include "adc.h"
#include "components/led.h"


// Global variables
static volatile unsigned int tmr_2_ticks;

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
    
    POS1CNT = 0; // Reset position counter to 0
    POS2CNT = 0; // Reset position counter to 0
    
    MAX1CNT = 1024 * 5; // Resets every 1024 slots
    MAX2CNT = 1024 * 5; // Resets every 1024 slots
    
    
    // Init timer 2 used for PID updating
    T2CONbits.TCKPS = 0b10; // 1:64 prescaler, ticks at 500000 Hz
    TMR2 = 0; // Clear TMR2 counter
    PR2 = 500; // Overflows every 1 ms (500000 / 1000) == 500 == 1 ms
    tmr_2_ticks = 0;
    
    // Configure TIMER 2 interrupts
    IPC1bits.T2IP = 5; // lvl 6 priority (second highest)
    IFS0bits.T2IF = 0; // clear IF flag
    IEC0bits.T2IE = 0; // Enable TRM2 interrupts
}

void enable_motor_control() {
    
    P1TCONbits.PTEN = 1; // Turn on PWM time base
    P2TCONbits.PTEN = 1; // Turn on PWM time base
    
    IEC0bits.T2IE = 1; // Enable TRM2 interrupts
    T2CONbits.TON = 1; // Turn on timer2 
}


extern volatile unsigned int sl_sensor;
// Goal of this function is to drive the left motor at a speed relative to 
// the value of a sensor.
void simple_velocity_controller() {
    
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
        L_MTR_DIR = ~L_MTR_FWD;
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
        R_MTR_DIR = ~R_MTR_FWD;
        adj = R_MTR_MIN + adj;
        if(adj > R_MTR_MIN) adj = R_MTR_MIN;
        if(adj < R_MTR_MAX) adj = R_MTR_MAX;
        R_MTR_PER = adj;
    }
}


void simple_position_controller() {
    
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


// Timer in charge of motor control - PID adjustments - every 1 ms
void __attribute__((__interrupt__, __shadow__, no_auto_psv)) _T2Interrupt(void) {
    IFS0bits.T2IF = 0; // Clear T2 interrupt flag
    
    tmr_2_ticks++;
    if(tmr_2_ticks == 1000) {
        LED_R = 1;
        tmr_2_ticks = 0;
    }
}