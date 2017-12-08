#include <xc.h>

#include "straights_procedure.h"

#include "../adc.h"
#include "../motor_control.h"


// Global variables
extern volatile unsigned int sl_sensor;
extern volatile unsigned int sr_sensor;
extern volatile unsigned int fl_sensor;
extern volatile unsigned int fr_sensor;
extern volatile unsigned int controller_finished;


/**
 * The goal of this procedure is to drive straight down a straight corridor.
 * It stops once it gets to a wall. It continues if there is no wall.
 */
void straights_stop_procedure() {
    
    // Set controller to drive straight, then enable everything.
    init_straight_controller();
    set_motor_control_function(&straight_controller);
    
    enable_adc();
    enable_motor_control();
    
    unsigned int curr_mode = 0; // 0 == straight, 1 == position (in-place)
    
    // Procedures runs indefinitely
    while(1) {
            
        if(curr_mode == 0) {
            // Currently driving straight
            

            // Check for a wall
            if(fr_sensor > FRD_CLOSE) {
                // Set to position control
                curr_mode = 1;
                init_position_controller();
                set_motor_control_function(&position_controller);
            }

        } else {
            // Currently held in-place

            // Check if wall is cleared
            if(fr_sensor < FRD_CLOSE) {
                // Continue driving forward
                curr_mode = 0;
                init_straight_controller();
                set_motor_control_function(&straight_controller);
            }
        }
    }
}

/**
 * This procedure does the same as above, however instead of stopping it turns 
 * around 180* so it can continue down to the other side infinitely.
 */
void straights_rotate_procedure() {
    
}