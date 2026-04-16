#include "libs_drivers.h"


// global drivers are instantianed here
Terminal    terminal;
Timer       timer;   
Sensors     sensors; 

MotorControl motor_control;

LED          led;

void LibsDriversInit()
{
    int init_res;
    // low level init, cache, clock
    drivers_init();  

    // uart initialisation
    uart_init();

    led.init();

    // terminal init
    terminal.init();

    // timer
    timer.init();

    // line sensor and proximity sensor init
    init_res = sensors.init();

    if (init_res != 0)
    {
        terminal << "sensors init failed\n";

        while (1)
        {
            led.on(LED::LEFT_RED);
            led.on(LED::RIGHT_RED);
            timer.delay_ms(200);
            led.off(LED::LEFT_RED);
            led.off(LED::RIGHT_RED);
            timer.delay_ms(200);
        }   
    }

    
    // motor control init, FOC control, encoders
    init_res = motor_control.init(0.8f);    

    if (init_res == -1)
    {
        terminal << "left encoder init failed\n";

        while (1)
        {
            led.on(LED::LEFT_RED);
            timer.delay_ms(200);
            led.off(LED::LEFT_RED);
            timer.delay_ms(200);
        }
    }
    else if (init_res == -2)
    {
        terminal << "right encoder init failed\n";

        while (1)
        {
            led.on(LED::RIGHT_RED);
            timer.delay_ms(200);
            led.off(LED::RIGHT_RED);
            timer.delay_ms(200);
        }
    }
    else
    {
        led.on(LED::LEFT_GREEN);    
        led.on(LED::RIGHT_GREEN);
    }

    turbine_init();
    

    timer.delay_ms(500);

    led.all_off();
    led.on(LED::RIGHT_BLUE);
    led.on(LED::LEFT_BLUE);
}