#include "motor_pwm.h"
#include <drivers.h>

void PWMLeft::init()
{
    //left motor PWM controll pins 
    //PWMA      : PC6, TIM3_CH1
    //PWMB      : PC7, TIM3_CH2

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);


    // GPIO CONFIG for TIM3: PB6, PB7 (AF2)
    LL_GPIO_InitTypeDef gpio_init;
    gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;
    gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    gpio_init.Pull = LL_GPIO_PULL_NO;
    gpio_init.Alternate = LL_GPIO_AF_2;

    gpio_init.Pin = LL_GPIO_PIN_6 | LL_GPIO_PIN_7;
    LL_GPIO_Init(GPIOC, &gpio_init);


    // TIM3 CONFIG  
    LL_TIM_SetPrescaler(TIM3, 0);
    LL_TIM_SetAutoReload(TIM3, PWM_VALUE_MAX - 1);
    LL_TIM_SetCounterMode(TIM3, LL_TIM_COUNTERMODE_UP);
    LL_TIM_EnableARRPreload(TIM3);

    LL_TIM_OC_SetMode(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_SetMode(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1);

    LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH2);

    LL_TIM_OC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_OCPOLARITY_HIGH);

    LL_TIM_OC_SetCompareCH1(TIM3, 0);
    LL_TIM_OC_SetCompareCH2(TIM3, 0);

    LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH2);

    LL_TIM_EnableCounter(TIM3); 
}

void PWMLeft::set(int32_t pwm)
{
    if (pwm > ((int32_t)PWM_VALUE_MAX-1))
    {
        pwm = ((int32_t)PWM_VALUE_MAX-1);
    }

    if (pwm < -((int32_t)PWM_VALUE_MAX-1))
    {
        pwm = -((int32_t)PWM_VALUE_MAX-1);
    }  

    if (pwm > 0)
    {
        TIM3->CCR1 = PWM_VALUE_MAX - pwm;
        TIM3->CCR2 = PWM_VALUE_MAX;      
    }   
    else if (pwm < 0)     
    {
        pwm = -pwm;     
        
        TIM3->CCR1 = PWM_VALUE_MAX;      
        TIM3->CCR2 = PWM_VALUE_MAX - pwm;
    }
    else
    {
        //brake mode
        TIM3->CCR1 = PWM_VALUE_MAX;
        TIM3->CCR2 = PWM_VALUE_MAX;
    }
}







void PWMRight::init()
{
    //right motor PWM controll pins 
    //PWMA      : PB6, TIM4_CH1
    //PWMB      : PB7, TIM4_CH2

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);


    // GPIO CONFIG for TIM4: PB6, PB7 (AF2)
    LL_GPIO_InitTypeDef gpio_init;
    gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;
    gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    gpio_init.Pull = LL_GPIO_PULL_NO;
    gpio_init.Alternate = LL_GPIO_AF_2;

    gpio_init.Pin = LL_GPIO_PIN_6 | LL_GPIO_PIN_7;
    LL_GPIO_Init(GPIOB, &gpio_init);


    // TIM4 CONFIG  
    LL_TIM_SetPrescaler(TIM4, 0);
    LL_TIM_SetAutoReload(TIM4, PWM_VALUE_MAX - 1);
    LL_TIM_SetCounterMode(TIM4, LL_TIM_COUNTERMODE_UP);
    LL_TIM_EnableARRPreload(TIM4);

    LL_TIM_OC_SetMode(TIM4, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_SetMode(TIM4, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1);

    LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH2);

    LL_TIM_OC_SetPolarity(TIM4, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_SetPolarity(TIM4, LL_TIM_CHANNEL_CH2, LL_TIM_OCPOLARITY_HIGH);

    LL_TIM_OC_SetCompareCH1(TIM4, 0);
    LL_TIM_OC_SetCompareCH2(TIM4, 0);

    LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH2);

    LL_TIM_EnableCounter(TIM4); 
}

void PWMRight::set(int32_t pwm)
{
    if (pwm > ((int32_t)PWM_VALUE_MAX-1))
    {
        pwm = ((int32_t)PWM_VALUE_MAX-1);
    }

    if (pwm < -((int32_t)PWM_VALUE_MAX-1))
    {
        pwm = -((int32_t)PWM_VALUE_MAX-1);
    }  

    if (pwm > 0)
    {
        TIM4->CCR1 = PWM_VALUE_MAX;
        TIM4->CCR2 = PWM_VALUE_MAX - pwm;
    }   
    else if (pwm < 0)     
    {
        pwm = -pwm;     
        
        TIM4->CCR1 = PWM_VALUE_MAX - pwm;      
        TIM4->CCR2 = PWM_VALUE_MAX;
    }
    else
    {
        //brake mode
        TIM4->CCR1 = PWM_VALUE_MAX;
        TIM4->CCR2 = PWM_VALUE_MAX;
    }
}



void PWMLeftThreePhase::init()
{
    //left motor PWM controll pins 
    //PWMA      : PC6, TIM3_CH1
    //PWMB      : PC7, TIM3_CH2
    //PWMC      : PC8, TIM3_CH3
    //enable    : PC9 

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);


    // GPIO CONFIG for TIM3: PB6, PB7, PB8 (AF2)
    LL_GPIO_InitTypeDef gpio_init;
    gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;
    gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    gpio_init.Pull = LL_GPIO_PULL_NO;
    gpio_init.Alternate = LL_GPIO_AF_2;

    gpio_init.Pin = LL_GPIO_PIN_6 | LL_GPIO_PIN_7 | LL_GPIO_PIN_8;
    LL_GPIO_Init(GPIOC, &gpio_init);


    // TIM3 CONFIG  
    LL_TIM_SetPrescaler(TIM3, 0);
    LL_TIM_SetAutoReload(TIM3, PWM_VALUE_MAX - 1);
    LL_TIM_SetCounterMode(TIM3, LL_TIM_COUNTERMODE_UP);
    LL_TIM_EnableARRPreload(TIM3);

    LL_TIM_OC_SetMode(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_SetMode(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_SetMode(TIM3, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1);

    LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH3);

    LL_TIM_OC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH3, LL_TIM_OCPOLARITY_HIGH);

    LL_TIM_OC_SetCompareCH1(TIM3, 0);
    LL_TIM_OC_SetCompareCH2(TIM3, 0);
    LL_TIM_OC_SetCompareCH3(TIM3, 0);

    LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH2);
    LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH3);

    LL_TIM_EnableCounter(TIM3); 

    //stop motor    
    this->set(0, 0, 0);

    // driver enable
    Gpio<'C', 9, GPIO_MODE_OUT> enable;
    enable = 1;
}


void PWMLeftThreePhase::set(int32_t pwm_a, int32_t pwm_b, int32_t pwm_c)
{
    TIM3->CCR1 = pwm_a;
    TIM3->CCR2 = pwm_b;
    TIM3->CCR3 = pwm_c;
}



void PWMRightThreePhase::init()
{
    //right motor PWM controll pins 
    //PWMA      : PB6, TIM4_CH1
    //PWMB      : PB7, TIM4_CH2
    //PWMC      : PB8, TIM4_CH3
    //enable    : PB9

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);


    // GPIO CONFIG for TIM4: PB6, PB7, PB8 (AF2)
    LL_GPIO_InitTypeDef gpio_init;
    gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;
    gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    gpio_init.Pull = LL_GPIO_PULL_NO;
    gpio_init.Alternate = LL_GPIO_AF_2;

    gpio_init.Pin = LL_GPIO_PIN_6 | LL_GPIO_PIN_7 | LL_GPIO_PIN_8;
    LL_GPIO_Init(GPIOB, &gpio_init);


    // TIM4 CONFIG  
    LL_TIM_SetPrescaler(TIM4, 0);
    LL_TIM_SetAutoReload(TIM4, PWM_VALUE_MAX - 1);
    LL_TIM_SetCounterMode(TIM4, LL_TIM_COUNTERMODE_UP);
    LL_TIM_EnableARRPreload(TIM4);

    LL_TIM_OC_SetMode(TIM4, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_SetMode(TIM4, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_SetMode(TIM4, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1);

    LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH3);

    LL_TIM_OC_SetPolarity(TIM4, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_SetPolarity(TIM4, LL_TIM_CHANNEL_CH2, LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_SetPolarity(TIM4, LL_TIM_CHANNEL_CH3, LL_TIM_OCPOLARITY_HIGH);

    LL_TIM_OC_SetCompareCH1(TIM4, 0);
    LL_TIM_OC_SetCompareCH2(TIM4, 0);
    LL_TIM_OC_SetCompareCH3(TIM4, 0);

    LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH2);
    LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH3);

    LL_TIM_EnableCounter(TIM4); 

    //stop motor    
    this->set(0, 0, 0); 

    // driver enable
    Gpio<'B', 9, GPIO_MODE_OUT> enable;
    enable = 1;
}

void PWMRightThreePhase::set(int32_t pwm_a, int32_t pwm_b, int32_t pwm_c)
{
    TIM4->CCR1 = pwm_a;
    TIM4->CCR2 = pwm_b;
    TIM4->CCR3 = pwm_c;
}
