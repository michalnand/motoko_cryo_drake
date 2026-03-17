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

    while (1)
    {
        led_0 = 0;
        uint32_t m_prev = motor_control.steps;
        timer.delay_ms(100);
        uint32_t m_curr = motor_control.steps;
        led_0 = 1;

        float distance  = motor_control.state.x_dist_est;
        float theta_ref = motor_control.state.x_theta_est;

        
        terminal << "steps : " << (m_curr - m_prev) * 10 << "\n";
        terminal << "left_position_degrees: " << motor_control.get_left_position()*180/PI << "\n";
        terminal << "left_velocity_rpm: " << motor_control.get_left_velocity()*60/(2*PI) << "\n";
        terminal << "right_position_degrees: " << motor_control.get_right_position()*180/PI << "\n";
        terminal << "right_velocity_rpm: " << motor_control.get_right_velocity()*60/(2*PI) << "\n";
        terminal << "distance [mm]: "  << distance << "\n";  
        terminal << "theta_ref [deg]: " << theta_ref*180/PI << "\n";
        terminal << "-----------------------------\n\n\n";

        timer.delay_ms(200);
    }
}
