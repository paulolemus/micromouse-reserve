#include <xc.h>

#include "components/led.h"

#include "adc.h"
#include "motor_control.h"
#include "pid.h"


// Global variables
static volatile unsigned int tmr_2_ticks;

// QEI overflow trackers
static volatile signed int l_qei_curr;
static volatile signed int r_qei_curr;
static volatile signed int l_qei_last;
static volatile signed int r_qei_last;

// Global position, velocity, and acceleration variables
// Position
static volatile signed int l_pos;
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
static volatile signed int r_pos;
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
extern volatile unsigned int sl_last;
extern volatile unsigned int sr_last;
extern volatile unsigned int fl_last;
extern volatile unsigned int fr_last;

// Function to execute to control the motors every scan period.
static void (*volatile motor_control)(void);

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
    unbrake();
    
 
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
    l_pos = 0;
    l_pos_last_err = 0;
    l_pos_integral = 0;
    l_vel = 0;
    l_vel_last_err = 0;
    l_vel_integral = 0;
    l_acc = 0;
    l_acc_last_err = 0;
    l_acc_integral = 0;
    
    r_pos = 0;
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
    l_qei_curr = 0;
    r_qei_curr = 0;
    l_qei_last = 0;
    r_qei_last = 0;
    controller_finished = 0;
    
    unbrake();
    
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


void brake() {
    LATAbits.LATA10 = 0; // Brakes on
    LATCbits.LATC9 = 0;
}

void unbrake() {
    LATAbits.LATA10 = 1; // Brakes off
    LATCbits.LATC9  = 1;
}



/*
 * MOTOR CONTROL FUNCTIONS
 * 
 * Each controller is coupled with an init function to set initial state.
 * Each controller also has coupled global state.
 */

static signed int L_STR_VEL_SP;
static signed int R_STR_VEL_SP;
void init_straight_controller() {
    // Desired velocity is 10 ticks / ms, or 2 revolutions / sec
    L_STR_VEL_SP = 850; 
    R_STR_VEL_SP = 850; 
    
    // Will always drive forward
    L_MTR_PER = L_MTR_MIN;
    R_MTR_PER = R_MTR_MIN;
    L_MTR_DIR = L_MTR_FWD;
    R_MTR_DIR = R_MTR_FWD;
}
void straight_controller() {
    
    const double kp = 2;
    const double kd = 0.5;
    
    // Calculate error in velocity
    const signed int l_vel_err = L_STR_VEL_SP - l_vel;
    const signed int r_vel_err = R_STR_VEL_SP - r_vel;
    
    // Calculate PID correction value for straight driving
    const signed l_out =
        P(l_vel_err, kp) +
        D(l_vel_err, l_vel_last_err, CONTROL_DT, kd);
    
    const signed r_out =
        P(r_vel_err, kp) +
        D(r_vel_err, r_vel_last_err, CONTROL_DT, kd);
    
    // Save last error for derivative later
    l_vel_last_err = l_vel_err;
    r_vel_last_err = r_vel_err;
    
    // Make adjustments to the duty cycle of each motor
    
    // Left motor
    signed l_per = L_MTR_PER - l_out;
    if(l_per < L_MTR_MAX) l_per = L_MTR_MAX;
    else if(l_per > L_MTR_MIN) l_per = L_MTR_MIN;
    L_MTR_PER = l_per;
    
    // Right motor
    signed r_per = R_MTR_PER - r_out;
    if(r_per < R_MTR_MAX) r_per = R_MTR_MAX;
    else if(r_per > R_MTR_MIN) r_per = R_MTR_MIN;
    R_MTR_PER = r_per;
}

static signed int L_TRK_VEL_SP;
static signed int R_TRK_VEL_SP;
static unsigned int L_TRK_SNS_SP;
static unsigned int R_TRK_SNS_SP;

static signed int l_sns_prev_err;
static signed int r_sns_prev_err;
static signed int l_sns_integral;
static signed int r_sns_integral;
void init_track_controller() {
    // Desired velocity is 10 ticks / ms, or 2 revolutions / sec
    L_TRK_VEL_SP = 1400; 
    R_TRK_VEL_SP = 1400; 
    
    // Desired readings for sensors
    L_TRK_SNS_SP = SLD_CLOSE - 50;
    R_TRK_SNS_SP = SRD_CLOSE - 50;
    
    l_sns_prev_err = 0;
    r_sns_prev_err = 0;
    l_sns_integral = 0;
    r_sns_integral = 0;
    
    // Will always drive forward
    L_MTR_PER = L_MTR_MIN;
    R_MTR_PER = R_MTR_MIN;
    L_MTR_DIR = L_MTR_FWD;
    R_MTR_DIR = R_MTR_FWD;
}
void track_controller() {
    // PID values
    const double vel_kp = 2;
    const double vel_kd = 0.6;
    
    const double l_sns_kp = 1.1;
    const double l_sns_ki = 0.0;
    const double l_sns_kd = 0.5;
    
    const double r_sns_kp = 1.1;
    const double r_sns_ki = 0.005;
    const double r_sns_kd = 1.5;

    signed int l_sns_err = 0;
    signed int r_sns_err = 0;
    signed int sns_out = 0;
    
    LED_OFF(LED_R);
    LED_OFF(LED_G);
    
    // Select a motor to use
    if(sr_sensor > SRD_CLOSE - 310) {
        LED_ON(LED_G);
        // Use right motor
        r_sns_err = R_TRK_SNS_SP - sr_sensor;
        
        sns_out =
            P(r_sns_err, r_sns_kp) +
            I(r_sns_err, r_sns_integral, ADC_DT, r_sns_ki) +
            D(r_sns_err, r_sns_prev_err, ADC_DT, r_sns_kd);
    
        
    } else if(sl_sensor > SLD_CLOSE - 310) {
        LED_ON(LED_R);
        // Use left motor
        l_sns_err = sl_sensor - L_TRK_SNS_SP;
        
        sns_out =
            P(l_sns_err, l_sns_kp) +
            I(l_sns_err, l_sns_integral, ADC_DT, l_sns_ki) +
            D(l_sns_err, l_sns_prev_err, ADC_DT, l_sns_kd);
    } else {
        
    }
    
    
    // Calculate error in velocity
    const signed int l_vel_err = L_TRK_VEL_SP - l_vel;
    const signed int r_vel_err = R_TRK_VEL_SP - r_vel;
    
    // Calculate PD correction value for straight driving AND tracking
    const signed l_out =
        P(l_vel_err, vel_kp) +
        D(l_vel_err, l_vel_last_err, CONTROL_DT, vel_kd) +
        sns_out;
        
    const signed r_out =
        P(r_vel_err, vel_kp) +
        D(r_vel_err, r_vel_last_err, CONTROL_DT, vel_kd) -
        sns_out;
    
    
    // Save last velocity error for derivative later
    l_vel_last_err = l_vel_err;
    r_vel_last_err = r_vel_err;
    // Save last sensor error for derivative later
    l_sns_prev_err = l_sns_err;
    r_sns_prev_err = r_sns_err;
    
    
    // Make adjustments to the duty cycle of each motor
    // Left motor
    signed l_per = L_MTR_PER - l_out;
    if(l_per < L_MTR_MAX) l_per = L_MTR_MAX;
    else if(l_per > L_MTR_MIN) l_per = L_MTR_MIN;
    L_MTR_PER = l_per;
    
    // Right motor
    signed r_per = R_MTR_PER - r_out;
    if(r_per < R_MTR_MAX) r_per = R_MTR_MAX;
    else if(r_per > R_MTR_MIN) r_per = R_MTR_MIN;
    R_MTR_PER = r_per;
}


static signed int R_POS_SP;
static signed int L_POS_SP;
static signed int R_POS_QEI_SP;
static signed int L_POS_QEI_SP;
void init_position_controller() {
    
    L_MTR_PER = L_MTR_MIN;
    R_MTR_PER = R_MTR_MIN;

    R_POS_QEI_SP = r_qei_curr;
    L_POS_QEI_SP = l_qei_curr;
    
    R_POS_SP = R_QEI_CNT;
    L_POS_SP = L_QEI_CNT;
}
void position_controller() {
    
    const double kp = 3; 
    const double ki = 0.005;
    const double kd = 13;
    
    const signed int r_err = 
        (R_POS_QEI_SP - r_qei_curr) * R_QEI_MAX - (signed)R_QEI_CNT + R_POS_SP;
    const signed int l_err =
        (L_POS_QEI_SP - l_qei_curr) * L_QEI_MAX - (signed)L_QEI_CNT + L_POS_SP;
        
    signed int r_out = 
        P(r_err, kp) +
        I(r_err, r_pos_integral, CONTROL_DT, ki) +
        D(r_err, r_pos_last_err, CONTROL_DT, kd);
    
    signed int l_out = 
        P(l_err, kp) +
        I(l_err, l_pos_integral, CONTROL_DT, ki) +
        D(l_err, l_pos_last_err, CONTROL_DT, kd);
    
    // Save last err
    r_pos_last_err = r_err;
    l_pos_last_err = l_err;
      
    // Make motor adjustments for right and left motors separately
    
    
    if(l_out > 0) {
        // Trying to drive forward
        L_MTR_DIR = L_MTR_FWD;
        l_out = L_MTR_MIN - l_out;
        
    } else {
        // Trying to drive backwards
        L_MTR_DIR = L_MTR_REV;
        l_out = L_MTR_MIN + l_out;
    }
    
    // Guard overflow
    if(l_out < L_MTR_MAX) {
        l_out = L_MTR_MAX;
    } else if(l_out > L_MTR_MIN) {
        l_out = L_MTR_MIN;
    }
    L_MTR_PER = l_out;
    
    
    if(r_out > 0) {
        // Trying to drive forward
        R_MTR_DIR = R_MTR_FWD;
        r_out = R_MTR_MIN - r_out;
        
    } else {
        // Trying to drive backwards
        R_MTR_DIR = R_MTR_REV;
        r_out = R_MTR_MIN + r_out;
    }
    
    // Guard overflow
    if(r_out > R_MTR_MIN) {
        r_out = R_MTR_MIN;
    } else if(r_out < R_MTR_MAX) {
        r_out = R_MTR_MAX;
    }
    R_MTR_PER = r_out;
}


static signed int L_LEFT_POS_SP;
static signed int R_LEFT_POS_SP;
static signed int L_LEFT_POS_QEI_SP;
static signed int R_LEFT_POS_QEI_SP;
void init_left_controller() {
    L_MTR_PER = L_MTR_MIN;
    R_MTR_PER = R_MTR_MIN;

    L_LEFT_POS_QEI_SP = l_qei_curr;
    R_LEFT_POS_QEI_SP = r_qei_curr;
    
    L_LEFT_POS_SP = L_QEI_CNT - L_QEI_ROT / 1.6;
    R_LEFT_POS_SP = R_QEI_CNT + R_QEI_ROT / 1.6;
}
void turn_left_controller() {
    const double MAX_SPD_DIV = 1.3;
    const double kp = 2; 
    const double ki = 0.001;
    const double kd = 7;
    
    const signed int l_err =
        (L_LEFT_POS_QEI_SP - l_qei_curr) * L_QEI_MAX - (signed)L_QEI_CNT + L_LEFT_POS_SP;
    const signed int r_err = 
        (R_LEFT_POS_QEI_SP - r_qei_curr) * R_QEI_MAX - (signed)R_QEI_CNT + R_LEFT_POS_SP;
        
    
    signed int l_out = 
        P(l_err, kp) +
        I(l_err, l_pos_integral, CONTROL_DT, ki) +
        D(l_err, l_pos_last_err, CONTROL_DT, kd);
    
    signed int r_out = 
        P(r_err, kp) +
        I(r_err, r_pos_integral, CONTROL_DT, ki) +
        D(r_err, r_pos_last_err, CONTROL_DT, kd);
    
    // Save last err
    l_pos_last_err = l_err;
    r_pos_last_err = r_err;  
    
    // Make motor adjustments for right and left motors separately
    
    
    if(l_out > 0) {
        // Trying to drive forward
        L_MTR_DIR = L_MTR_FWD;
        l_out = L_MTR_MIN - l_out;
        
    } else {
        // Trying to drive backwards
        L_MTR_DIR = L_MTR_REV;
        l_out = L_MTR_MIN + l_out;
    }
    
    // Guard overflow
    if(l_out < L_MTR_MIN / MAX_SPD_DIV) {
        l_out = L_MTR_MIN / MAX_SPD_DIV;
    } else if(l_out > L_MTR_MIN) {
        l_out = L_MTR_MIN;
    }
    L_MTR_PER = l_out;
    
    
    if(r_out > 0) {
        // Trying to drive forward
        R_MTR_DIR = R_MTR_FWD;
        r_out = R_MTR_MIN - r_out;
        
    } else {
        // Trying to drive backwards
        R_MTR_DIR = R_MTR_REV;
        r_out = R_MTR_MIN + r_out;
    }
    
    // Guard overflow
    if(r_out > R_MTR_MIN) {
        r_out = R_MTR_MIN;
    } else if(r_out < R_MTR_MIN / MAX_SPD_DIV) {
        r_out = R_MTR_MIN / MAX_SPD_DIV;
    }
    R_MTR_PER = r_out;
}

static signed int L_RIGHT_POS_SP;
static signed int R_RIGHT_POS_SP;
static signed int L_RIGHT_POS_QEI_SP;
static signed int R_RIGHT_POS_QEI_SP;
void init_right_controller() {
    L_MTR_PER = L_MTR_MIN;
    R_MTR_PER = R_MTR_MIN;

    L_RIGHT_POS_QEI_SP = l_qei_curr;
    R_RIGHT_POS_QEI_SP = r_qei_curr;
    
    L_RIGHT_POS_SP = L_QEI_CNT + L_QEI_ROT / 1.6;
    R_RIGHT_POS_SP = R_QEI_CNT - R_QEI_ROT / 1.6;
}
void turn_right_controller() {
    const double MAX_SPD_DIV = 1.3;
    const double kp = 2; 
    const double ki = 0.001;
    const double kd = 7;
    
    const signed int l_err =
        (L_RIGHT_POS_QEI_SP - l_qei_curr) * L_QEI_MAX - (signed)L_QEI_CNT + L_RIGHT_POS_SP;
    const signed int r_err = 
        (R_RIGHT_POS_QEI_SP - r_qei_curr) * R_QEI_MAX - (signed)R_QEI_CNT + R_RIGHT_POS_SP;
        
    
    signed int l_out = 
        P(l_err, kp) +
        I(l_err, l_pos_integral, CONTROL_DT, ki) +
        D(l_err, l_pos_last_err, CONTROL_DT, kd);
    
    signed int r_out = 
        P(r_err, kp) +
        I(r_err, r_pos_integral, CONTROL_DT, ki) +
        D(r_err, r_pos_last_err, CONTROL_DT, kd);
    
    // Save last err
    l_pos_last_err = l_err;
    r_pos_last_err = r_err;  
    
    // Make motor adjustments for right and left motors separately
    
    
    if(l_out > 0) {
        // Trying to drive forward
        L_MTR_DIR = L_MTR_FWD;
        l_out = L_MTR_MIN - l_out;
        
    } else {
        // Trying to drive backwards
        L_MTR_DIR = L_MTR_REV;
        l_out = L_MTR_MIN + l_out;
    }
    
    // Guard overflow
    if(l_out < L_MTR_MIN / MAX_SPD_DIV) {
        l_out = L_MTR_MIN / MAX_SPD_DIV;
    } else if(l_out > L_MTR_MIN) {
        l_out = L_MTR_MIN;
    }
    L_MTR_PER = l_out;
    
    
    if(r_out > 0) {
        // Trying to drive forward
        R_MTR_DIR = R_MTR_FWD;
        r_out = R_MTR_MIN - r_out;
        
    } else {
        // Trying to drive backwards
        R_MTR_DIR = R_MTR_REV;
        r_out = R_MTR_MIN + r_out;
    }
    
    // Guard overflow
    if(r_out > R_MTR_MIN) {
        r_out = R_MTR_MIN;
    } else if(r_out < R_MTR_MIN / MAX_SPD_DIV) {
        r_out = R_MTR_MIN / MAX_SPD_DIV;
    }
    R_MTR_PER = r_out;
}

static signed int L_ROUND_POS_SP;
static signed int R_ROUND_POS_SP;
static signed int L_ROUND_POS_QEI_SP;
static signed int R_ROUND_POS_QEI_SP;
void init_around_controller() {
    L_MTR_PER = L_MTR_MIN;
    R_MTR_PER = R_MTR_MIN;

    L_ROUND_POS_QEI_SP = l_qei_curr;
    R_ROUND_POS_QEI_SP = r_qei_curr;
    
    L_ROUND_POS_SP = L_QEI_CNT + (L_QEI_ROT / 1.6) * 2;
    R_ROUND_POS_SP = R_QEI_CNT - (R_QEI_ROT / 1.6) * 2;
}
void turn_around_controller() {
    const double MAX_SPD_DIV = 1.2;
    const double kp = 1.9; 
    const double ki = 0.001;
    const double kd = 20;
    
    const signed int l_err =
        (L_ROUND_POS_QEI_SP - l_qei_curr) * L_QEI_MAX - (signed)L_QEI_CNT + L_ROUND_POS_SP;
    const signed int r_err = 
        (R_ROUND_POS_QEI_SP - r_qei_curr) * R_QEI_MAX - (signed)R_QEI_CNT + R_ROUND_POS_SP;
        
    
    signed int l_out = 
        P(l_err, kp) +
        I(l_err, l_pos_integral, CONTROL_DT, ki) +
        D(l_err, l_pos_last_err, CONTROL_DT, kd);
    
    signed int r_out = 
        P(r_err, kp) +
        I(r_err, r_pos_integral, CONTROL_DT, ki) +
        D(r_err, r_pos_last_err, CONTROL_DT, kd);
    
    // Save last err
    l_pos_last_err = l_err;
    r_pos_last_err = r_err;  
    
    // Make motor adjustments for right and left motors separately
    
    
    if(l_out > 0) {
        // Trying to drive forward
        L_MTR_DIR = L_MTR_FWD;
        l_out = L_MTR_MIN - l_out;
        
    } else {
        // Trying to drive backwards
        L_MTR_DIR = L_MTR_REV;
        l_out = L_MTR_MIN + l_out;
    }
    
    // Guard overflow
    if(l_out < L_MTR_MIN / MAX_SPD_DIV) {
        l_out = L_MTR_MIN / MAX_SPD_DIV;
    } else if(l_out > L_MTR_MIN) {
        l_out = L_MTR_MIN;
    }
    L_MTR_PER = l_out;
    
    
    if(r_out > 0) {
        // Trying to drive forward
        R_MTR_DIR = R_MTR_FWD;
        r_out = R_MTR_MIN - r_out;
        
    } else {
        // Trying to drive backwards
        R_MTR_DIR = R_MTR_REV;
        r_out = R_MTR_MIN + r_out;
    }
    
    // Guard overflow
    if(r_out > R_MTR_MIN) {
        r_out = R_MTR_MIN;
    } else if(r_out < R_MTR_MIN / MAX_SPD_DIV) {
        r_out = R_MTR_MIN / MAX_SPD_DIV;
    }
    R_MTR_PER = r_out;
}
/**
 * Simply does nothing after setting motor speed to MIN.
 */
void init_off_controller() {
    L_MTR_PER = L_MTR_MIN;
    R_MTR_PER = R_MTR_MIN;
}
void off_controller() {
    // Does nothing.
}


void simple_position_tester() {
    
    const double p_term = 7; 
    const signed int SETPOINT = 1024 * 5 / 2.0;
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
        
        signed r_adj = p_term * r_err;
        signed l_adj = p_term * l_err;
        
        // Make motor adjustments for right and left motors separately
        if(l_adj > 0) {
            // Trying to drive forward
            L_MTR_DIR = L_MTR_FWD;
            l_adj = L_MTR_MIN - l_adj;
            if(l_adj < L_MTR_MAX) l_adj = L_MTR_MAX;
            if(l_adj > L_MTR_MIN) l_adj = L_MTR_MIN;
            L_MTR_PER = l_adj;

        } else {
            // Trying to drive backwards
            L_MTR_DIR = L_MTR_REV;
            l_adj = L_MTR_MIN + l_adj;
            if(l_adj > L_MTR_MIN) l_adj = L_MTR_MIN;
            if(l_adj < L_MTR_MAX) l_adj = L_MTR_MAX;
            L_MTR_PER = l_adj;
        }

        if(r_adj > 0) {
            // Trying to drive forward
            R_MTR_DIR = R_MTR_FWD;
            r_adj = R_MTR_MIN - r_adj;
            if(r_adj < R_MTR_MAX) r_adj = R_MTR_MAX;
                if(r_adj > R_MTR_MIN) r_adj = R_MTR_MIN;
                R_MTR_PER = r_adj;

        } else {
            // Trying to drive backwards
            R_MTR_DIR = R_MTR_REV;
            r_adj = R_MTR_MIN + r_adj;
            if(r_adj > R_MTR_MIN) r_adj = R_MTR_MIN;
            if(r_adj < R_MTR_MAX) r_adj = R_MTR_MAX;
            R_MTR_PER = r_adj;
        }
    }
}


// Timer in charge of motor control - PID adjustments - every 2 ms
void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void) {
    IFS0bits.T2IF = 0; // Clear T2 interrupt flag
    
    tmr_2_ticks++;
    // Make pid adjustments
    if(tmr_2_ticks == CONTROL_DT) {
        tmr_2_ticks = 0;
        
        // Calculate position velocity and acceleration
        const signed int l_pos_fin = L_QEI_CNT;
        const signed int r_pos_fin = R_QEI_CNT;
        
        const double l_pos_del 
            = (l_qei_curr - l_qei_last) * L_QEI_ROT + l_pos_fin - l_pos;
        const double r_pos_del 
            = (r_qei_curr - r_qei_last) * R_QEI_ROT + r_pos_fin - r_pos;
        
        const signed int l_vel_fin = l_pos_del * 100 / (double)CONTROL_DT;
        const signed int r_vel_fin = r_pos_del * 100 / (double)CONTROL_DT;
        
        l_acc = (l_vel_fin - l_vel) / (double)CONTROL_DT;
        r_acc = (r_vel_fin - r_vel) / (double)CONTROL_DT;
        
        l_vel = l_vel_fin;
        r_vel = r_vel_fin;
        
        l_pos = l_pos_fin;
        r_pos = r_pos_fin;
        
        
        // Call current motor controller function
        (*motor_control)();
        
        // Update last qei counter
        l_qei_last = l_qei_curr;
        r_qei_last = r_qei_curr;
    }
}


// QEI overflows for right motor
void __attribute__((__interrupt__, no_auto_psv)) _QEI1Interrupt(void) {
    IFS3bits.QEI1IF = 0; // Clear flag
    
    if(R_QEI_CNT > R_QEI_ROT / 2.0) {
        // If CNT is now max, it underflowed - moving backwards
        r_qei_curr--;
        
        // Check for counter underflows
        if(r_qei_curr < -32000) {
            r_qei_last = r_qei_last - r_qei_curr;
            r_qei_curr = 0;
        }
        
    } else {
        // If CNT is now 0, it overflowed - moving forward
        r_qei_curr++;
        
        // Check for counter overflows
        if(r_qei_curr > 32000) {
            r_qei_last = r_qei_last - r_qei_curr;
            r_qei_curr = 0;
        }
    }
}

// QEI overflows for left motor
void __attribute__((__interrupt__, no_auto_psv)) _QEI2Interrupt(void) {
    IFS4bits.QEI2IF = 0; // Clear flag
    
    if(L_QEI_CNT > L_QEI_ROT / 2.0) {
        // If CNT is now max, it underflowed - moving backwards
        l_qei_curr--;
        
        // Check for counter underflows - fix if less than -30000
        if(l_qei_curr < -32000) {
            l_qei_last = l_qei_last - l_qei_curr;
            l_qei_curr = 0;
        }
        
    } else {
        // If CNT is now 0, it overflowed - moving forward
        l_qei_curr++;
        
        // Check for counter overflows - fix if approaching signed 16 bit max
        if(l_qei_curr > 32000) {
            l_qei_last = l_qei_last - l_qei_curr;
            l_qei_curr = 0;
        }
    }
}
