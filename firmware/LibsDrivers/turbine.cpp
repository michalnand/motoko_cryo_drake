#include "turbine.h"

void turbine_init()
{
    uint32_t prescaler = 200; // 1MHz timer clock
    uint32_t period    = TURBINE_MAX_PWM;

    /* PB10 -> AF1 (TIM2_CH3) */
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_10, LL_GPIO_AF_1);
    LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_10, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_10, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_10, LL_GPIO_PULL_NO);

    /* Timer base configuration */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

    LL_TIM_SetPrescaler(TIM2, prescaler);
    LL_TIM_SetAutoReload(TIM2, period);
    LL_TIM_EnableARRPreload(TIM2);

    /* PWM mode CH3 */
    LL_TIM_OC_SetMode(TIM2, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH3);
    LL_TIM_OC_SetCompareCH3(TIM2, 0);

    LL_TIM_OC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH3, LL_TIM_OCPOLARITY_HIGH);

    /* Enable output */
    LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH3);

    /* Start timer */
    LL_TIM_EnableCounter(TIM2);
}

void turbine_set(uint32_t pwm)
{
    if (pwm > TURBINE_MAX_PWM)
    {
        pwm = TURBINE_MAX_PWM;  
    }

    LL_TIM_OC_SetCompareCH3(TIM2, pwm);
} 


void turbine_on()   
{
    uint32_t turbine_power = 10;
    for (uint32_t n = 0; n < 10; n++)    
    {
        uint32_t pwm = ((n+1)*turbine_power)/10;
        turbine_set(pwm);
        timer.delay_ms(100);  
    }  

    timer.delay_ms(500);          
}

void turbine_off()
{
    turbine_set(0);
    timer.delay_ms(500);
}