#include "libs_drivers.h"
#include "tests.h"

#include "turbine.h"

#include "control_loop.h"

#include "line_follower.h"


int main()
{

    LibsDriversInit();

    terminal << "machine ready\n";

    
    
    LineFollower line_follower;
    line_follower.init(0);  
    key_wait();        

    line_follower.run();



    //motor_foc_test();

    //sensors_test();
    //encoder_test();   
    //motor_foc_test();

    //motor_identification();
    //motor_controll_test();    

    //robot_identification();    
    
    // init position control loop

    /*
    ControlLoop control_loop;
    control_loop.init();

    while (1) 
    {
        control_loop.set_position(0.0f, 0.0f*PI/180.0f);

        uint32_t steps_prev = control_loop.steps;
        timer.delay_ms(800);    
        uint32_t steps_curr = control_loop.steps;   

        uint32_t sps = (float)(steps_curr - steps_prev)*(1000.0f/800.0f);

        terminal << "sps "<< sps << "\n";   

        terminal << "distance "<< motor_control.state.x_dist_est*1000.0f << "\n";
        terminal << "angle    "<< motor_control.state.x_theta_est*180.0f/PI << "\n";    

        control_loop.set_position(0.0f, 90.0f*PI/180.0f);

        timer.delay_ms(800);  
        terminal << "distance "<< motor_control.state.x_dist_est*1000.0f << "\n";
        terminal << "angle "<< motor_control.state.x_theta_est*180.0f/PI << "\n"; 

        terminal << "\n\n"; 
    }
    */


    ControlLoop control_loop;
    control_loop.init();

    while (1)   
    {
        //control_loop.set_circle_motion(0.2f, 0.7f);
        control_loop.set_turn_motion(1.0f, 0.7f);
        timer.delay_ms(1000);      
        
        float turn_radius;

        turn_radius = motor_control.state.x_vel_est/motor_control.state.x_omega_est;
        
        terminal << "velocity " << motor_control.state.x_vel_est << "\n";
        terminal << "radius "   << turn_radius << "\n";
        terminal << "\n\n"; 
    }
    
    return 0;
}