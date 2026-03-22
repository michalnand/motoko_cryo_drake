#include "libs_drivers.h"
#include "tests.h"

#include "turbine.h"

#include "control_loop.h"

int main()
{

    LibsDriversInit();

    terminal << "machine ready\n";

    // sensors_test();
    //encoder_test();   
    //motor_foc_test();

    //motor_identification();
    //motor_controll_test();    

    //robot_identification();    
    
    // init position control loop
    
    ControlLoop control_loop;
    control_loop.init(sensors, motor_control);

    while (1) 
    {
        control_loop.set_xr(0.0f, 0.0f*PI/180.0f);

        uint32_t steps_prev = control_loop.steps;
        timer.delay_ms(800);    
        uint32_t steps_curr = control_loop.steps;   

        uint32_t sps = (float)(steps_curr - steps_prev)*(1000.0f/800.0f);

        terminal << "sps "<< sps << "\n";   

        terminal << "angle "<< motor_control.state.x_theta_est*180.0f/PI << "\n";

        control_loop.set_xr(0.0f, 90.0f*PI/180.0f);

        timer.delay_ms(800);   
        terminal << "angle "<< motor_control.state.x_theta_est*180.0f/PI << "\n"; 

        terminal << "\n\n";
    }
    

    /*
    float k = 0.95f;
    float u_max = 0.2f;
    float u_val = 0.0f;

    while (1)
    {
        while ((motor_control.state.x_theta_est*180.0f/PI) < 90.0f)
        {
            u_val = k*u_val + (1.0-k)*u_max;

            motor_control.set_right_torque(u_val);
            motor_control.set_left_torque(-u_val);

            timer.delay_ms(4);
        }

        motor_control.set_right_torque(0);
        motor_control.set_left_torque(0);

        timer.delay_ms(500);    
        terminal << "angle "<< motor_control.state.x_theta_est*180.0f/PI << "\n";


        while ((motor_control.state.x_theta_est*180.0f/PI) > 0.0f)
        {
            u_val = k*u_val + (1.0-k)*(-u_max);
            
            motor_control.set_right_torque(u_val);
            motor_control.set_left_torque(-u_val);

            timer.delay_ms(4);  
        }

        motor_control.set_right_torque(0);
        motor_control.set_left_torque(0);

        timer.delay_ms(500); 
        terminal << "angle "<< motor_control.state.x_theta_est*180.0f/PI << "\n";
    }
    */

    float x_r = 0.0f;

    PID pid_angle;
    pid_angle.init(3.0f, 0.4f, 6.0f, 1.0f);  

    ShaperFilter shaper_angle;   
    shaper_angle.init(0.95f);    

    uint32_t steps = 0; 

    while (1)
    {
        if (((steps/300)%2) == 0)
        {
            x_r = 0.0f;
        }   
        else      
        {
            x_r = 90.0f*PI/180.0f;
        }

        //float x     = motor_control.state.x_theta_est;  

        float right_position = motor_control.get_right_position();
        float left_position  = motor_control.get_left_position();
        float x = (WHEEL_RADIUS_MM)*(right_position - left_position)/(WHEEL_BRACE_MM);

   
        float xr_s  = shaper_angle.step(x_r);

        float u_turn = pid_angle.step(xr_s - x);

        motor_control.set_right_torque(u_turn); 
        motor_control.set_left_torque(-u_turn);

        timer.delay_ms(4);

        if ((steps%50) == 0)    
        {   
            terminal << "angle "<< x_r*180.0f/PI << " " << xr_s*180.0f/PI << " " << x*180.0f/PI << "\n";
        }   

        steps++;
    }

    return 0;
}