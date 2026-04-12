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
        

    //sensors_test();
    //encoder_test();   
    //motor_foc_test();

    //motor_identification();
    //motor_controll_test();    

    //robot_identification();    
    
    // init position control loop
    
    
    //ControlLoop control_loop;
    //control_loop.init();

    /*
    while (1) 
    {
        control_loop.set_position(0.0f, 0.0f*PI/180.0f, false);

        uint32_t steps_prev = control_loop.steps;
        timer.delay_ms(800);    
        uint32_t steps_curr = control_loop.steps;   

        uint32_t sps = (float)(steps_curr - steps_prev)*(1000.0f/800.0f);

        terminal << "sps "<< sps << "\n";   

        terminal << "distance "<< motor_control.state.x_dist_est*1000.0f << "\n";
        terminal << "angle    "<< motor_control.state.x_theta_est*180.0f/PI << "\n";    

        control_loop.set_position(0.1f, 0.0f*90.0f*PI/180.0f, false);

        timer.delay_ms(800);  
        terminal << "distance "<< motor_control.state.x_dist_est*1000.0f << "\n";
        terminal << "angle "<< motor_control.state.x_theta_est*180.0f/PI << "\n"; 

        terminal << "\n\n";
    }
    */

    /*
    while (1)
    {
        control_loop.set_circle_motion(0.18f, 0.5f);

        uint32_t steps_prev = control_loop.steps;   
        timer.delay_ms(800);    
        uint32_t steps_curr = control_loop.steps;   

        uint32_t sps = (float)(steps_curr - steps_prev)*(1000.0f/800.0f);
        
        float w     = motor_control.state.x_omega_est;
        float radius = sgn(w)*motor_control.state.x_vel_est /(abs(w) + 0.00001f); 

        terminal << "sps "<< sps << "\n";   

        terminal << "distance "<< motor_control.state.x_dist_est*1000.0f << "\n";
        terminal << "velocity "<< motor_control.state.x_vel_est*1000.0f  << "\n";
        terminal << "angle    "<< motor_control.state.x_theta_est*180.0f/PI << "\n";
        terminal << "omega    "<< motor_control.state.x_omega_est*180.0f/PI << "\n";  
        terminal << "radius   "<< radius*1000.0f << "\n";     

        terminal << "\n\n"; 
    }
    */
    
    /*
    turbine_on();

    ControlLoop control_loop;
    control_loop.init(sensors, motor_control);

    
    control_loop.set_xr(0.8f, 0.0);
    timer.delay_ms(1000);


    turbine_off();

    while (1)
    {
        led.on(LED::RIGHT_BLUE);
        timer.delay_ms(100);

        led.off(LED::RIGHT_BLUE);
        timer.delay_ms(200);
    }
    */

    
    /*
    //turbine_on();

    float u_max = 1.0f;     

    while (1)
    {
        while ((motor_control.state.x_theta_est*180.0f/PI) < 90.0f)
        {
            motor_control.set(0.0f, u_max); 

            timer.delay_ms(4);
        }   

        motor_control.set(0.0f, 0.0f);
        
        timer.delay_ms(500);     
        terminal << "angle "<< motor_control.state.x_theta_est*180.0f/PI << "\n";


        while ((motor_control.state.x_theta_est*180.0f/PI) > 0.0f)
        {
            motor_control.set(0.0f, -u_max);

            timer.delay_ms(4);   
        }

        motor_control.set(0.0f, 0.0f);
        
        timer.delay_ms(500); 
        terminal << "angle "<< motor_control.state.x_theta_est*180.0f/PI << "\n";
    }
    */

    return 0;
}