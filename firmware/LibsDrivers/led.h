#pragma once

#include "drivers.h"
#include "gpio.h"

class LED
{
public:
    enum LedId
    {
        LEFT_RED   = 0,
        LEFT_GREEN = 1,
        LEFT_BLUE  = 2,
        RIGHT_RED  = 3,
        RIGHT_GREEN= 4,
        RIGHT_BLUE = 5,
        LED_COUNT  = 6
    };

    void init();
    void on(LedId id);
    void off(LedId id);

    void all_off();

    void led_blink(LedId id);
    
private:
    void set(LedId id, int value);

    Gpio<'B', 0, GPIO_MODE_OUT> led_0; // left red
    Gpio<'C', 4, GPIO_MODE_OUT> led_1; // left green
    Gpio<'A', 7, GPIO_MODE_OUT> led_2; // left blue
    Gpio<'A', 0, GPIO_MODE_OUT> led_3; // right red
    Gpio<'A', 1, GPIO_MODE_OUT> led_4; // right green
    Gpio<'A', 2, GPIO_MODE_OUT> led_5; // right blue

    uint32_t counter;
};