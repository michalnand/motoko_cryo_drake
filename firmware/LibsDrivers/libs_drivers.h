#ifndef _LIBS_DRIVERS_H_
#define _LIBS_DRIVERS_H_

#include <drivers.h>
#include <common.h>


#include "motor_pwm.h"  
#include "motor_control_bldc.h"

#include "sensors.h"

#include "turbine.h"


// availible from all files
extern Timer        timer;
extern Terminal     terminal;
extern Sensors      sensors;
extern MotorControl motor_control;

void LibsDriversInit();

#endif

