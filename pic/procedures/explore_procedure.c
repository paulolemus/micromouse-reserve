#include <xc.h>

#include "explore_procedure.h"

#include "../adc.h"
#include "../motor_control.h"
#include "../wait.h"


// Global variables
extern volatile unsigned int sl_sensor;
extern volatile unsigned int sr_sensor;
extern volatile unsigned int fl_sensor;
extern volatile unsigned int fr_sensor;
extern volatile unsigned int controller_finished;


/**
 * The goal of this procedure is to randomly explore the maze. In an intersection,
 * it will randomly choose a direction to move in.
 * In a dead end, it will turn around and continue exploring.
 * It makes decisions each time it encounters a front wall.
 */
void rand_explore_procedure() {
    
    init_track_controller();
    set_motor_control_function(&track_controller);
    
    enable_adc();
    enable_motor_control();
    
    unsigned int left_open  = 0;
    unsigned int right_open = 0;
    
    /*
     * Poll sensors in a loop. There are a couple cases that need to be considered:
     * Open left
     * Open right
     * Open front
     * All closed
     */
    while(1) {
        
        // Encountered a wall in front, make decision on direction to turn.
        if(fr_sensor > FRD_CLOSE) {
            left_open = 0;
            right_open = 0;
            
            if(sl_sensor < SLD_CLOSE - 200) left_open = 1;
            if(sr_sensor < SRD_CLOSE - 200) right_open = 1;
            
            // Stop
            init_position_controller();
            set_motor_control_function(&position_controller);
            wait_ms(400);
            
            
            // If both are close turn around
            if(!left_open && !right_open) {
                
                // turn around
                init_around_controller();
                set_motor_control_function(&turn_around_controller);
                wait_ms(1100);
                
           
                // Otherwise if both are open
            } else if(left_open && right_open) {
                
                unsigned direction = sl_sensor % 2;
                // Turn left
                if(direction == 0) {
                    init_left_controller();
                    set_motor_control_function(&turn_left_controller);
                    
                    // Turn right
                } else {
                    init_right_controller();
                    set_motor_control_function(&turn_right_controller);
                }
                
                wait_ms(700);
                
                // Otherwise turn left if its the only one open
            } else if(left_open) {
                init_left_controller();
                set_motor_control_function(&turn_left_controller);
                wait_ms(700);
                
                // Otherwise turn right if its the only one open
            } else {
                init_right_controller();
                set_motor_control_function(&turn_right_controller);
                wait_ms(700);
            }
            
            // continue driving forward
            init_track_controller();
            set_motor_control_function(&track_controller);
        }
    }
}