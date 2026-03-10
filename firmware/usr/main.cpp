#include "libs_drivers.h"
#include "tests.h"

#include "turbine.h"

int main()
{

    LibsDriversInit();

    terminal << "machine ready\n";

    // sensors_test();
    // encoder_test();
    motor_foc_test();

    /*
    Gpio<'B', 0, GPIO_MODE_OUT> led_0;
    Gpio<'C', 4, GPIO_MODE_OUT> led_1;
    Gpio<'A', 7, GPIO_MODE_OUT> led_2;

    Gpio<'A', 0, GPIO_MODE_OUT> led_3;
    Gpio<'A', 1, GPIO_MODE_OUT> led_4;
    Gpio<'A', 2, GPIO_MODE_OUT> led_5;

    led_0 = 1;
    led_1 = 1;
    led_2 = 1;

    led_3 = 1;
    led_4 = 1;  
    led_5 = 1;


    Gpio<'B', 2, GPIO_MODE_IN_PULLUP> key_0;

    turbine_init();

    uint32_t time_out = 0;

    while (1)
    {
        if (key_0 == 0)
        {
            led_0 = 0;  
            time_out = timer.get_time() + 10000;
        }
        
        if (timer.get_time() > time_out)
        {
            led_0 = 1;
            turbine_set(0);
        }
        else
        {
            led_0 = 0;
            turbine_set(30);
        }

        led_4 = 0;
        timer.delay_ms(100);
        led_4 = 1;
        timer.delay_ms(100);
    }
    */

    /*
    while (1)
    {
        led_0 = 0;
        timer.delay_ms(100);
        led_0 = 1;
        timer.delay_ms(900);

        line_sensor_hw_test();

        led_1 = 0;
        terminal << "time = " << timer.get_time() << " ms\n";
        //motor_pwm_test(); 
    }
    
    */

    return 0;
}