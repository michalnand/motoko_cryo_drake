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
        timer.delay_ms(500);
        terminal << "angle "<< motor_control.state.x_theta_est*180.0f/PI << "\n";

        control_loop.set_xr(0.0f, 90.0f*PI/180.0f);
        timer.delay_ms(500);   
        terminal << "angle "<< motor_control.state.x_theta_est*180.0f/PI << "\n"; 
    }

    /*
    float k = 0.95f;
    float u_val = 0.0f;

    while (1)
    {
        u_val = 0.0f;
        while ((motor_control.state.x_theta_est*180.0f/PI) < 90.0f)
        {
            u_val = k*u_val + (1.0-k)*0.2f*MOTOR_CONTROL_MAX_VELOCITY;

            motor_control.set_right_velocity(u_val);
            motor_control.set_left_velocity(-u_val);

            timer.delay_ms(4);
        }

        motor_control.set_right_velocity(0);
        motor_control.set_left_velocity(0);

        timer.delay_ms(500);    
        terminal << "angle "<< motor_control.state.x_theta_est*180.0f/PI << "\n";

        u_val = 0.0f;
        while ((motor_control.state.x_theta_est*180.0f/PI) > 0.0f)
        {
            u_val = k*u_val + (1.0-k)*(-0.2f)*MOTOR_CONTROL_MAX_VELOCITY;
            
            motor_control.set_right_velocity(u_val);
            motor_control.set_left_velocity(-u_val);

            timer.delay_ms(4);  
        }

        motor_control.set_right_velocity(0);
        motor_control.set_left_velocity(0);

        timer.delay_ms(500); 
        terminal << "angle "<< motor_control.state.x_theta_est*180.0f/PI << "\n";
    }
    */

    return 0;
}