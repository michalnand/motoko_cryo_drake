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

   // robot_identification();   

    // init position control loop
    ControlLoop control_loop;
    control_loop.init(sensors, motor_control);

    Gpio<'B', 0, GPIO_MODE_OUT> led_0;

    /*
    while (1)
    {
        uint32_t steps_before = control_loop.steps;
        timer.delay_ms(100);
        uint32_t steps_curr   = control_loop.steps;
        
        led_0 = 0;
        terminal << "steps = " << steps_curr << ", steps/s = " << (steps_curr - steps_before)*10 << "\n";   
        led_0 = 1;
    }
    */

    while (1)
    {
        control_loop.set_xr(0.0f, 0.0f);
        timer.delay_ms(500);
        
        control_loop.set_xr(0.0f, 90.0*PI/180.0f);
        timer.delay_ms(500);
        
        control_loop.set_xr(0.0f, 0.0f);
        timer.delay_ms(500);    

        control_loop.set_xr(0.0f, -90.0*PI/180.0f);
        timer.delay_ms(500);
    }


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