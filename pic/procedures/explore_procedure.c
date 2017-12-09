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

#define CENTER_MS 400
#define POS_MS 250
#define TURN_MS 675
#define AROUND_MS 1000


// Special breakout loop if stuck in a cycle. Turn at next opening.
void rand_explore_breakout() {
    unsigned int u_turn_count = 2;
    unsigned int left_open;
    unsigned int right_open;
    
    while(u_turn_count > 1) {
        // Poll sensors
        if(sl_sensor < SLD_FAR) left_open  = 1;
        else                    left_open  = 0;
        if(sr_sensor < SRD_FAR) right_open = 1;
        else                    right_open = 0;


        // Drive to center if left is open then turn.
        if(left_open) {

            u_turn_count = 0;

            wait_ms(CENTER_MS);
            init_position_controller();
            set_motor_control_function(&position_controller);
            wait_ms(POS_MS);
            init_left_controller();
            set_motor_control_function(&turn_left_controller);
            wait_ms(TURN_MS);
            init_track_controller();
            set_motor_control_function(&track_controller);

        } else if(right_open) {

            u_turn_count = 0;

            wait_ms(CENTER_MS);
            init_position_controller();
            set_motor_control_function(&position_controller);
            wait_ms(POS_MS);
            init_right_controller();
            set_motor_control_function(&turn_right_controller);
            wait_ms(TURN_MS);
            init_track_controller();
            set_motor_control_function(&track_controller);

        } else if(fr_sensor > FRD_CLOSE) {
            u_turn_count++;

            init_position_controller();
            set_motor_control_function(&position_controller);
            wait_ms(POS_MS);
            init_around_controller();
            set_motor_control_function(&turn_around_controller);
            wait_ms(AROUND_MS);
            init_track_controller();
            set_motor_control_function(&track_controller);
        }
    }
}

/**
 * The goal of this procedure is to randomly explore the maze. In an intersection,
 * it will randomly choose a direction to move in.
 * In a dead end, it will turn around and continue exploring.
 * It makes decisions each time it encounters a front wall.
 * It also keeps track of 
 */
void rand_explore_procedure() {
    
    init_track_controller();
    set_motor_control_function(&track_controller);
    
    enable_adc();
    enable_motor_control();
    
    unsigned int left_open  = 0;
    unsigned int right_open = 0;
    
    unsigned u_turn_count = 0;
    
    /*
     * Poll sensors in a loop. There are a couple cases that need to be considered:
     * Open left
     * Open right
     * Open front
     * All closed
     */
    while(1) {
        
        // Poll sensors
        if(sl_sensor < SLD_FAR) left_open  = 1;
        else                    left_open  = 0;
        if(sr_sensor < SRD_FAR) right_open = 1;
        else                    right_open = 0;
        
        
        if(u_turn_count > 1) {
            u_turn_count = 0;
            rand_explore_breakout();
        }
        
        
        // Encountered a wall in front, make decision on direction to turn.
        // This is the primary loop to handle turns.
        if(fr_sensor > FRD_CLOSE) {
            
            // Stop
            init_position_controller();
            set_motor_control_function(&position_controller);
            wait_ms(POS_MS);
            
            
            // If both are close turn around
            if(!left_open && !right_open) {
                u_turn_count++;
                
                // turn around
                init_around_controller();
                set_motor_control_function(&turn_around_controller);
                wait_ms(AROUND_MS);
                
                
           
                // Otherwise if both are open
            } else if(left_open && right_open) {
                u_turn_count = 0;
                
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
                
                wait_ms(TURN_MS);
                
                // Otherwise turn left if its the only one open
            } else if(left_open) {
                u_turn_count = 0;
                                
                init_left_controller();
                set_motor_control_function(&turn_left_controller);
                wait_ms(TURN_MS);
                
                // Otherwise turn right if its the only one open
            } else {
                u_turn_count = 0;
                
                init_right_controller();
                set_motor_control_function(&turn_right_controller);
                wait_ms(TURN_MS);
            }
            
            // continue driving forward
            init_track_controller();
            set_motor_control_function(&track_controller);
        }
    }
}