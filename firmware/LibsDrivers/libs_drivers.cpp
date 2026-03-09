#include "libs_drivers.h"


// global drivers are instantianed here
Terminal    terminal;
Timer       timer;   
Sensors     sensors; 

//ADC_driver  adc;

//LineSensor   line_sensor;
//IRSensor     ir_sensor;
    

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

    sensors.init();

    // motor control init
    //motor_control.init();
}