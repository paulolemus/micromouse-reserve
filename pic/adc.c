#include <xc.h>
#include "adc.h"
#include "components/led.h"
#include "components/sensor.h"


// Number of MS to wait before starting a new scanning sequence.
static volatile unsigned int tmr_4_ticks;
// ADC scan rotation
static volatile unsigned int adc_rotation;

// Global variables
// Storage for sensor readings
volatile unsigned int sl_sensor;
volatile unsigned int sr_sensor;
volatile unsigned int fl_sensor;
volatile unsigned int fr_sensor;

#define CAPTURE_SCAN(sensor_var) (sensor_var = (sensor_var + ADC1BUF0) / 2)


void init_adc() {
    
    // Set AN4/5/6/7 to analog control, rest to digital control
    AD1PCFGL = 0b100001111;
    
    // Configure ADC software module
    AD1CON1bits.ADON = 0; // Disable to configure
    AD1CON1bits.SSRC = 0b111; // End sample after internal counter time
    
    AD1CON2bits.BUFM = 1; // Fill buffer from 0x0
    
    AD1CON3bits.SAMC = 2; // 2 TAD for auto sample time
    AD1CON3bits.ADCS = 2; // 3 * Tcy = TAD, used for auto conversion
    
    // Configure finish of conversion interrupts
    IPC3bits.AD1IP = 6; // lvl 6 priority
    IEC0bits.AD1IE = 0; // Disable interrupts
    
    
    // Configure Timer for sampling and converting ADC
    T4CONbits.TCKPS = 0b10; // 1:64 prescaler, ticks at 500000 Hz
    TMR4 = 0; // Clear TMR4 counter
    PR4 = 500; // Overflows every 1 ms (500000 / 1000) == 500 == 1 ms
    tmr_4_ticks = 0;
    
    // Configure TIMER 4 interrupts
    IPC6bits.T4IP = 5; // lvl 6 priority (second highest)
    IFS1bits.T4IF = 0; // Clear IF
    IEC1bits.T4IE = 0; // Enable Tmr4 interrupts
    T4CONbits.TON = 0; // Turn off timer4  
}

void enable_adc() {
    // Clear interrupt flag and enable interrupts.
    IFS0bits.AD1IF = 0; // Clear flag bit
    IEC0bits.AD1IE = 1; // Enable conversion interrupts
    
    // Clear interrupt flags and enable interrupts for timer.
    IFS1bits.T4IF = 0; // Clear IF
    IEC1bits.T4IE = 1; // Enable Tmr4 interrupts
    
    // Set state of global variables
    sl_sensor = SLD_CLOSE;
    sr_sensor = SRD_CLOSE;
    fl_sensor = FLD_CLOSE;
    fr_sensor = FRD_CLOSE;
    tmr_4_ticks = 0;
    adc_rotation = 0;
    
    // Turn on emitters
    // TODO: Remove this portion
    FLE = 1;
    FRE = 1;
    SLE = 1;
    SRE = 1;
    
    // Select first sensor in rotation
    // Rotation goes SLD, FLD, FRD, SRD
    SELECT_DETECTOR(SLD);
    
    
    T4CONbits.TON = 1; // Turn on timer4  
    AD1CON1bits.ADON = 1; // Turn on
}

void disable_adc() {
    // Disable modules
    T4CONbits.TON = 0; // Turn on timer4  
    AD1CON1bits.ADON = 0; // Turn on
    
    // Disable interrupts
    IEC0bits.AD1IE = 0; // Enable conversion interrupts
    IEC1bits.T4IE = 0;  // Enable Tmr4 interrupts
    
    // Turn off emitters
    FLE = 0;
    FRE = 0;
    SLE = 0;
    SRE = 0;
}



// Timer used to start ADC scans of all detectors - operates at 2x PID speed
void __attribute__((__interrupt__, __shadow__, no_auto_psv)) _T4Interrupt(void) {
    IFS1bits.T4IF = 0; // clear T4 interrupt flag

    tmr_4_ticks++;
    
    if(tmr_4_ticks == 1) {
        // Convert here
        AD1CON1bits.SAMP = 1;
        tmr_4_ticks = 0;
    }
}



// Interrupt to save values from the four detectors once conversions finished.
// Order of scan goes: SLD, FLD, FRD, SRD
void __attribute__((__interrupt__, __shadow__, no_auto_psv)) _ADC1Interrupt(void) {
    IFS0bits.AD1IF = 0; // Clear conversion flag

    // Capture value from scan depending on adc rotation
    switch(adc_rotation) {
        case 0: CAPTURE_SCAN(sl_sensor); break;
        case 1: CAPTURE_SCAN(fl_sensor); break;
        case 2: CAPTURE_SCAN(fr_sensor); break;
        case 3: CAPTURE_SCAN(sr_sensor); break;
    }
    
    // Rotate to next scan
    adc_rotation = (adc_rotation + 1) % NUM_DETECTORS;
    
    // Move to next scan
    switch(adc_rotation) {
        case 0: SELECT_DETECTOR(SLD); break;
        case 1: SELECT_DETECTOR(FLD); AD1CON1bits.SAMP = 1; break;
        case 2: SELECT_DETECTOR(FRD); AD1CON1bits.SAMP = 1; break;
        case 3: SELECT_DETECTOR(SRD); AD1CON1bits.SAMP = 1; break;
    }
    
    // Check if sl is working
    if(sl_sensor > SLD_CLOSE) {
        LED_ON(LED_R);
    }
    else {
        LED_OFF(LED_R);
    }
    // Check if sr is working - GOOD
    if(sr_sensor > SRD_CLOSE) {
        //LED_ON(LED_G);
    } else {
        //LED_OFF(LED_G);
    }
    // Check if fr is working - GOOD
    if(fr_sensor > FRD_CLOSE) {
        //LED_ON(LED_B);
    } else {
        //LED_OFF(LED_B);
    }
}