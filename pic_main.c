/* 
 * File:   pic_main.c
 * Author: paulo
 *
 * Created on November 21, 2017, 9:10 PM
 */

// DSPIC33FJ128MC804 Configuration Bit Settings

// 'C' source line config statements

// FBS
#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)
#pragma config RBS = NO_RAM             // Boot Segment RAM Protection (No Boot RAM)

// FSS
#pragma config SWRP = WRPROTECT_OFF     // Secure Segment Program Write Protect (Secure segment may be written)
#pragma config SSS = NO_FLASH           // Secure Segment Program Flash Code Protection (No Secure Segment)
#pragma config RSS = NO_RAM             // Secure Segment Data RAM Protection (No Secure RAM)

// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)

// FOSCSEL
#pragma config FNOSC = FRCPLL           // Oscillator Mode (Internal Fast RC (FRC) w/ PLL)
#pragma config IESO = ON                // Internal External Switch Over Mode (Start-up device with FRC, then automatically switch to user-selected oscillator source when ready)

// FOSC
#pragma config POSCMD = NONE            // Primary Oscillator Source (Primary Oscillator Disabled)
#pragma config OSCIOFNC = ON            // OSC2 Pin Function (OSC2 pin has digital I/O function)
#pragma config IOL1WAY = OFF            // Peripheral Pin Select Configuration (Allow Multiple Re-configurations)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor (Both Clock Switching and Fail-Safe Clock Monitor are disabled)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler (1:32,768)
#pragma config WDTPRE = PR128           // WDT Prescaler (1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog timer enabled/disabled by user software)

// FPOR
#pragma config FPWRT = PWR128           // POR Timer Value (128ms)
#pragma config ALTI2C = OFF             // Alternate I2C  pins (I2C mapped to SDA1/SCL1 pins)
#pragma config LPOL = ON                // Motor Control PWM Low Side Polarity bit (PWM module low side output pins have active-high output polarity)
#pragma config HPOL = ON                // Motor Control PWM High Side Polarity bit (PWM module high side output pins have active-high output polarity)
#pragma config PWMPIN = ON              // Motor Control PWM Module Pin Mode bit (PWM module pins controlled by PORT register at device Reset)

// FICD
#pragma config ICS = PGD1               // Comm Channel Select (Communicate on PGC1/EMUC1 and PGD1/EMUD1)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG is Disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>


#include "pic/components/oscillator.h"
#include "pic/components/led.h"
#include "pic/components/sensor.h"
#include "pic/components/encoder.h"
#include "pic/components/motor.h"

#include "pic/adc.h"
#include "pic/motor_control.h"

#include "pic/procedures/straights_procedure.h"
#include "pic/procedures/startup_procedure.h"
#include "pic/procedures//explore_procedure.h"
#include "pic/procedures//hug_procedure.h"


extern unsigned int mode_selected;

/*
 * MIPS = million instructions per second
 * MAX = 40 MIPS = 40 Million instructions per second
 * 
 * Output of PLL is Fosc, however the instruction clock Fcy = Fp = Fosc / 2
 * Fcy MAX = 40MHz
 * 
 */
int main(int argc, char** argv) {
    
    // Initialize hardware components
    init_oscillator();
    init_led();
    init_motors();
    init_encoders();
    init_sensors();
    
    // Initialize software modules
    init_adc();
    init_motor_control();
    
    /*
     * Mode to run the mouse in.
     * Pression the button will toggle the mode.
     * 0 == random search
     * 1 == map
     * 2 == self defined 
     */
    
    // Run primary procedures in this while loop.
    // The procedures are long lived programs that complete a specific task,
    // such as mapping a maze, speedrunning, or waiting for commands.
    while(1) {
        
        
        startup_procedure();
        
        // RED MODE
        if(mode_selected == 0) {
            
            rand_explore_procedure();
            
            // GREEN MODE
        } else if(mode_selected == 1) {
            
            right_hugger_procedure();
            
            // BLUE MODE
        } else if(mode_selected == 2) {
            
            left_hugger_procedure();
            
        }
    }
    
    return (EXIT_SUCCESS);
}
