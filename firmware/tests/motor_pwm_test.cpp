#include <libs_drivers.h>

// run both BLDC motors forward, no feedback loop, just simple three phase sine waves
void motor_pwm_test()
{
    PWMLeftThreePhase  left_pwm;
    PWMRightThreePhase right_pwm;

    left_pwm.init();
    right_pwm.init();

    uint32_t phase = 0;

    Gpio<'C', 4, GPIO_MODE_OUT> led_1;

    while (1)       
    {
        uint32_t phase_a    = (phase + (0*SINE_TABLE_SIZE)/3)%SINE_TABLE_SIZE;
        uint32_t phase_b    = (phase + (1*SINE_TABLE_SIZE)/3)%SINE_TABLE_SIZE;
        uint32_t phase_c    = (phase + (2*SINE_TABLE_SIZE)/3)%SINE_TABLE_SIZE;

        uint32_t pwm_a      = (PWM_VALUE_MAX*sine_table[phase_a])/SINE_VALUE_MAX;
        uint32_t pwm_b      = (PWM_VALUE_MAX*sine_table[phase_b])/SINE_VALUE_MAX;
        uint32_t pwm_c      = (PWM_VALUE_MAX*sine_table[phase_c])/SINE_VALUE_MAX;

        left_pwm.set(0.2*pwm_a, 0.2*pwm_b, 0.2*pwm_c);  
        right_pwm.set(0.2*pwm_a, 0.2*pwm_b, 0.2*pwm_c);   

        phase+= 7;           

        timer.delay_ms(2);

        if ((phase%10) == 0)
        {
            led_1 = 0;
        }
        else
        {
            led_1 = 1;
        }
    }
}
