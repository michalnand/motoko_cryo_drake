#ifndef _MOTOR_PWM_H_
#define _MOTOR_PWM_H_

#include <stdint.h>

//20kHz PWM, 216MHz/2 is timer clock source
#define PWM_FREQUENCY        ((uint32_t)20000)
#define PWM_VALUE_MAX        ((uint32_t)(216000000/2)/PWM_FREQUENCY - 1)


/*
    PWM for two DC motors,
    uses tim3 and tim4
*/

// left DC motor PWM, TIM3
//PWMA      : PC6, TIM3_CH1
//PWMB      : PC7, TIM3_CH2
class PWMLeft 
{   
    public:
        void init();
        void set(int32_t pwm);
};

// right DC motor, TIM4
//PWMA      : PB6, TIM4_CH1
//PWMB      : PB7, TIM4_CH2
class PWMRight
{   
    public:
        void init();
        void set(int32_t pwm);
};




/*
    PWM for two BLDC three phase motors,
    uses tim3 and tim4
*/

// left BLDC motor, TIM3
//  PWMA      : PC6, TIM3_CH1
//  PWMB      : PC7, TIM3_CH2
//  PWMC      : PC8, TIM3_CH3
//  enable    : PC9
class PWMLeftThreePhase
{   
    public:
        void init();
        void set(int32_t pwm_a, int32_t pwm_b, int32_t pwm_c);
};


// right BLDC motor, TIM4
//  PWMA      : PB6, TIM4_CH1
//  PWMB      : PB7, TIM4_CH2
//  PWMC      : PB8, TIM4_CH3
//  enable    : PB9
class PWMRightThreePhase
{
    public:
        void init();
        void set(int32_t pwm_a, int32_t pwm_b, int32_t pwm_c);
};

#endif