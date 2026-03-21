#include "led.h"

void LED::init()
{
    // 1 = off, 0 = on (NPN transistor circuit)
    led_0 = 1;
    led_1 = 1;
    led_2 = 1;
    led_3 = 1;
    led_4 = 1;
    led_5 = 1;
}

void LED::on(LedId id)
{
   set(id, 0); // active low
}   

void LED::off(LedId id)
{
   set(id, 1); // active low
}


void LED::all_off()
{
    set(LEFT_RED, 1);
    set(LEFT_GREEN, 1);
    set(LEFT_BLUE, 1);
    set(RIGHT_RED, 1);
    set(RIGHT_GREEN, 1);
    set(RIGHT_BLUE, 1);
}

void LED::set(LedId id, int value)
{
    switch (id)
    {
        case LEFT_RED:    led_0 = value; break;
        case LEFT_GREEN:  led_1 = value; break;
        case LEFT_BLUE:   led_2 = value; break;
        case RIGHT_RED:   led_3 = value; break;
        case RIGHT_GREEN: led_4 = value; break;
        case RIGHT_BLUE:  led_5 = value; break;
        default: break;
    }
}
