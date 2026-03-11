#include <libs_drivers.h>
#include <tmath.h>

void motor_controll_test()
{
        Gpio<'B', 0, GPIO_MODE_OUT> led_0;

        uint32_t steps = 0;
        float req_rpm_list[] = {0.0, 100.0, 200.0, 500.0, 1000.0};
        
        while (1)
        {
            led_0 = 0;
            uint32_t m_prev = motor_control.steps;
            timer.delay_ms(10);
            uint32_t m_curr = motor_control.steps;
            led_0 = 1;  

            float req_rpm;  

            req_rpm = req_rpm_list[(steps/300)%5];   

            motor_control.set_left_velocity(req_rpm * (2.0*PI/60.0));
            motor_control.set_right_velocity(req_rpm * (2.0*PI/60.0));

            float left_rpm     = motor_control.get_left_velocity()*60/(2*PI);
            float left_rpm_hat = motor_control.get_left_velocity_hat()*60/(2*PI);

            float right_rpm     = motor_control.get_right_velocity()*60/(2*PI);
            float right_rpm_hat = motor_control.get_right_velocity_hat()*60/(2*PI);

            terminal << req_rpm << " " << left_rpm << " " << left_rpm_hat << " " << right_rpm << " " << right_rpm_hat << "\n";

            steps++;
        }
}
