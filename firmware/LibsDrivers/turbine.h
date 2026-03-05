#ifndef _TURBINE_H_
#define _TURBINE_H_

#include "libs_drivers.h"

#define TURBINE_MAX_PWM         ((uint32_t)100)

void turbine_init();
void turbine_set(uint32_t pwm);


#endif //_TURBINE_H_