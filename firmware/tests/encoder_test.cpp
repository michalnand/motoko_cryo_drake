#include <libs_drivers.h>
#include <tmath.h>

void encoder_test()
{
    Gpio<'B', 0, GPIO_MODE_OUT> led_0;

    /*
    AS5600T<12, 13, 5, 'B', 'B'> left_encoder;
    AS5600T<10, 11, 5,  'C', 'C'> right_encoder;

    left_encoder.init();
    right_encoder.init();

    while (1)
    {
        led_0 = 0;
        uint32_t m_prev = motor_control.steps;
        timer.delay_ms(100);
        uint32_t m_curr = motor_control.steps;
        led_0 = 1;

        left_encoder.update();
        right_encoder.update();
            

        float left_position         = -(2.0f*PI*left_encoder.position)/ENCODER_RESOLUTION;
        float right_position        = (2.0f*PI*right_encoder.position)/ENCODER_RESOLUTION;
        
        terminal << "left_position_degrees: " << left_position*360/(2*PI) << "\n";
        terminal << "right_position_degrees: " << right_position*360/(2*PI) << "\n";
        terminal << "-----------------------------\n\n\n";  
      
        timer.delay_ms(200);
    }
    */

    uint32_t steps = 0;

    float alpha         = 0.9f;
    
    float distance_mean = 0.0f;
    float distance_var  = 0.0f;
    float theta_mean    = 0.0f;
    float theta_var     = 0.0f;

    float velocity_mean = 0.0f;
    float velocity_var  = 0.0f;
    float omega_mean    = 0.0f;
    float omega_var     = 0.0f;

    while (1)
    {
        
        float distance  = motor_control.state.x_dist_est;
        float theta     = motor_control.state.x_theta_est;

        float velocity  = motor_control.state.x_vel_est;
        float omega     = motor_control.state.x_omega_est;

        // EMA stats
        distance_mean = alpha*distance_mean + (1-alpha)*distance;
        distance_var  = alpha*distance_var  + (1-alpha)*(distance - distance_mean)*(distance - distance_mean);

        theta_mean = alpha*theta_mean + (1-alpha)*theta;
        theta_var  = alpha*theta_var  + (1-alpha)*(theta - theta_mean)*(theta - theta_mean);

        velocity_mean = alpha*velocity_mean + (1-alpha)*velocity;
        velocity_var  = alpha*velocity_var  + (1-alpha)*1000*(velocity - velocity_mean)*(velocity - velocity_mean);

        omega_mean = alpha*omega_mean + (1-alpha)*omega;
        omega_var  = alpha*omega_var  + (1-alpha)*1000*(omega - omega_mean)*(omega - omega_mean);

        if ((steps%100) == 0)   
        {   
            led_0 = 0;  
            //terminal << "steps : " << (m_curr - m_prev) * 10 << "\n";
            terminal << "left_position_degrees: " << motor_control.get_left_position()*180/PI << "\n";
            //terminal << "left_velocity_rpm: " << motor_control.get_left_velocity()*60/(2*PI) << "\n";
            terminal << "right_position_degrees: " << motor_control.get_right_position()*180/PI << "\n";
            //terminal << "right_velocity_rpm: " << motor_control.get_right_velocity()*60/(2*PI) << "\n";
            terminal << "distance [m]:    "  << distance     << " (mean: " << distance_mean     << ", var: " << distance_var << ")\n";  
            terminal << "theta [deg]  :   "  << theta*180/PI << " (mean: " << theta_mean*180/PI << ", var: " << theta_var*180/PI << ")\n";
            terminal << "velocity [m/s]:  "  << velocity     << " (mean: " << velocity_mean     << ", var: " << velocity_var << ")\n";
            terminal << "omega [deg/s]:   "  << omega*180/PI << " (mean: " << omega_mean*180/PI << ", var: " << omega_var*180/PI << ")\n";
            terminal << "-----------------------------\n\n\n";  
            led_0 = 1;

            //motor_control.set_right_velocity(0.0f);
            //motor_control.set_left_velocity(0.0f);
        }

        steps++;

        timer.delay_ms(2);

    }
}
