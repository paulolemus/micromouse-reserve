#include <xc.h>

#include "hug_procedure.h"

#include "../adc.h"
#include "../motor_control.h"
#include "../wait.h"

// Sensor variables
extern volatile unsigned int sl_sensor;
extern volatile unsigned int sr_sensor;
extern volatile unsigned int fl_sensor;
extern volatile unsigned int fr_sensor;
extern volatile unsigned int controller_finished;

//extern volatile unsigned int nonblock_wait_finished;

// For calibrating maneuvers
#define CENTER_MS 400
#define POS_MS 240
#define TURN_MS 650
#define AROUND_MS 1000

// Sensor adjustments
#define SL_ADJ 50
#define SR_ADJ 10

/**
 * Follow the wall on the left hand side. 
 * The mouse mouses forward as if it can't take a hand off the left wall.
 */
void left_hugger_procedure() {
    
    init_track_controller();
    set_motor_control_function(&track_controller);
    
    enable_adc();
    enable_motor_control();
    
    volatile unsigned int left_open  = 0;
    unsigned int front_open = 0;
    
    
    /*
     * Primary loop to track against a left wall.
     * There are a couple cases to consider for tracking against a left wall:
     * 
     * 1. front open and left closed.
     * 2. front closed and left closed.
     * 3. front closed and left open.
     */
    while(1) {
        
        // Poll sensors
        if(sl_sensor < SLD_FAR + SL_ADJ) left_open = 1;
        else                             left_open = 0;
        if(fr_sensor < FRD_CLOSE) front_open = 1;
        else                      front_open = 0;
        
        
        // Repoll 3 times to ensure it wasn't a ghost value
        if(left_open || !front_open) {
            
            wait_ms(3);
            if(sl_sensor < SLD_FAR + SL_ADJ) {
                left_open = 1;
            } else {
                left_open = 0;
            }
        }
        
        
        if(!front_open) {
            // Stop
            init_position_controller();
            set_motor_control_function(&position_controller);
            wait_ms(POS_MS);
            
            // Repoll one more time
            if(sl_sensor < SLD_FAR - SL_ADJ) left_open |= 1;
            else                             left_open |= 0;
            
            if(left_open) {
                // Turn left
                init_left_controller();
                set_motor_control_function(&turn_left_controller);
                wait_ms(TURN_MS);
                
                // Continue driving forward
                init_track_controller();
                set_motor_control_function(&track_controller);
                wait_ms(CENTER_MS / 2);
            } else {
                // Turn right
                init_right_controller();
                set_motor_control_function(&turn_right_controller);
                wait_ms(TURN_MS);
                
                // Continue driving forward
                init_track_controller();
                set_motor_control_function(&track_controller);
            }
            
            
        } else if(left_open == 1) {
            
            // Drive to center of block
            wait_ms(CENTER_MS);
            
            
            // Stop in middle of block
            init_position_controller();
            set_motor_control_function(&position_controller);
            wait_ms(POS_MS);
            
            // Turn left
            init_left_controller();
            set_motor_control_function(&turn_left_controller);
            wait_ms(TURN_MS);
            
            // Continue driving forward
            init_track_controller();
            set_motor_control_function(&track_controller);
            
            // Clear the edge
            wait_ms(CENTER_MS / 1.8);
        }
    }
}




/**
 * Follow the wall on the right hand side. 
 * The mouse mouses forward as if it can't take a hand off the right wall.
 */
void right_hugger_procedure() {
    
    init_track_controller();
    set_motor_control_function(&track_controller);
    
    enable_adc();
    enable_motor_control();
    
    volatile unsigned int right_open  = 0;
    unsigned int front_open = 0;
    
    
    /*
     * Primary loop to track against a right wall.
     * There are a couple cases to consider for tracking against a right wall:
     * 
     * 1. front open and right closed.
     * 2. front closed and right closed.
     * 3. front closed and right open.
     */
    while(1) {
        
        // Poll sensors
        if(sr_sensor < SRD_FAR - SR_ADJ) right_open = 1;
        else                              right_open = 0;
        if(fr_sensor < FRD_CLOSE) front_open = 1;
        else                      front_open = 0;
        
        
        // Repoll 3 times to ensure it wasn't a ghost value
        if(right_open || !front_open) {
            
            wait_ms(3);
            if(sr_sensor < SRD_FAR - SR_ADJ) {
                right_open = 1;
            } else {
                right_open = 0;
            }
        }
        
        
        if(!front_open) {
            // Stop
            init_position_controller();
            set_motor_control_function(&position_controller);
            wait_ms(POS_MS);
            
            // Repoll one more time
            if(sr_sensor < SRD_FAR - SR_ADJ) right_open |= 1;
            else                             right_open |= 0;
            
            if(right_open) {
                // Turn right
                init_right_controller();
                set_motor_control_function(&turn_right_controller);
                wait_ms(TURN_MS);
                
                // Continue driving forward
                init_track_controller();
                set_motor_control_function(&track_controller);
                wait_ms(CENTER_MS / 2);
                
            } else {
                // Turn left
                init_left_controller();
                set_motor_control_function(&turn_left_controller);
                wait_ms(TURN_MS);
                
                // Continue driving forward
                init_track_controller();
                set_motor_control_function(&track_controller);
            }
            
            
        } else if(right_open == 1) {
            
            // Drive to center of block
            wait_ms(CENTER_MS);
            
            
            // Stop in middle of block
            init_position_controller();
            set_motor_control_function(&position_controller);
            wait_ms(POS_MS);
            
            // Turn right
            init_right_controller();
            set_motor_control_function(&turn_right_controller);
            wait_ms(TURN_MS);
            
            // Continue driving forward
            init_track_controller();
            set_motor_control_function(&track_controller);
            
            // Clear the edge
            wait_ms(CENTER_MS / 2);
        }
    }
}