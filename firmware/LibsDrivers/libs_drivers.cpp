#include "libs_drivers.h"


// global drivers are instantianed here
Terminal    terminal;
Timer       timer;   
Sensors     sensors; 

MotorControl motor_control;

void LibsDriversInit()
{
    // low level init, cache, clock
    drivers_init();  

    // uart initialisation
    uart_init();

    // terminal init
    terminal.init();

    // timer
    timer.init();

    // line sensor and proximity sensor init
    sensors.init();

    // motor control init, FOC control, encoders
    motor_control.init();
}