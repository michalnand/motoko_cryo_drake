#include "key.h"

#include <libs_drivers.h>

void key_wait()
{
    Gpio<'B', 2, GPIO_MODE_IN_PULLUP> key_0;
    Gpio<'B', 0, GPIO_MODE_OUT> led;

    // wait for key press to start experiment
    while (key_0 == 1)
    {
        led = 0;  
        timer.delay_ms(100);
        led = 1;  
        timer.delay_ms(800);
    }

    while (key_0 == 0)
    {
        led = 0;  
        timer.delay_ms(100);
        led = 1;  
        timer.delay_ms(100);
    }
}