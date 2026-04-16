#include <libs_drivers.h>
#include <tmath.h>

void motor_foc_test()
{
        Gpio<'B', 0, GPIO_MODE_OUT> led_0;

        uint32_t state = 0;
        float torque_max = 0.7;
        float torque   = 0.0;
        float ramp_up  = 0.01;
        float ramp_down =  0.1;

        
        while (1)
        {
            led_0 = 0;
            uint32_t m_prev = motor_control.steps;
            timer.delay_ms(100);
            uint32_t m_curr = motor_control.steps;
            led_0 = 1;

            if (state == 0)
            {
                torque += ramp_up;
                if (torque > torque_max)
                {
                    torque = torque_max;
                    state = 1;
                }
            }
            else
            {
                torque -= ramp_down;
                if (torque < 0.0)
                {
                    torque = 0.0;
                    state = 0;
                }
            }   
            
            motor_control.set_left_torque(-torque);  
            motor_control.set_right_torque(-torque);

            terminal << "steps : " << (m_curr - m_prev) * 10 << "\n";
            terminal << "left_position_degrees: " << motor_control.get_left_position()*180/PI << "\n";
            //terminal << "left_velocity_rpm: " << motor_control.get_left_velocity()*60/(2*PI) << "\n";
            terminal << "right_position_degrees: " << motor_control.get_right_position()*180/PI << "\n";
            //terminal << "right_velocity_rpm: " << motor_control.get_right_velocity()*60/(2*PI) << "\n";
            terminal << "-----------------------------\n\n\n";
        }
}
