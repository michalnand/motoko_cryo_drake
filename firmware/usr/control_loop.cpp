#include "control_loop.h"

#define CONTROLLER_UPDATE_RATE_HZ  ((uint32_t)250)

#ifdef __cplusplus  
extern "C" {    
#endif


ControlLoop *g_control_loop_ptr;

// timer 8 interrupt handler, running controller update
void TIM8_UP_TIM13_IRQHandler(void)
{ 
    if (LL_TIM_IsActiveFlag_UPDATE(TIM8))
    {
        LL_TIM_ClearFlag_UPDATE(TIM8);
        g_control_loop_ptr->callback();
    }
} 


#ifdef __cplusplus
}
#endif

int ControlLoop::init()
{
    // instance init, variables set
    g_control_loop_ptr = this;

    
    float dt = 1.0f/(float)CONTROLLER_UPDATE_RATE_HZ;


    // shapers init, smooth run

    
    {
        float acc_max   = 4*9.81;
        float acc_min   = -30*9.81; 

        float tau       = 0.1;  

        shaper_distance.init(acc_min, acc_max, tau, dt);
    }

    {   
        float acc_max   = 50*9.81;

        float tau       = 0.2;

        shaper_angle.init(-acc_max, acc_max, tau, dt);
    }

    
    //agressive turn
    {
        float acc_max   = 500*9.81;

        float tau       = 0.02; 

        shaper_angle.init(-acc_max, acc_max, tau, dt);
    }
    
    position_controller.init();
    
    this->x_distance_req = 0.0f;                  
    this->x_theta_req    = 0.0f;    

    this->steps = 0;    

    this->timer_init();

        
    return 0;
}

void ControlLoop::set_xr(float x_distance_req, float x_theta_req)
{
    this->x_distance_req = x_distance_req;
    this->x_theta_req    = x_theta_req;
}


void ControlLoop::callback()
{
    // udpate sensors data (line sensor, proximity sensor, incl filtering, processing)
    sensors.callback();

    // controller computation, obtain control outputs
    float dist_req_s  = shaper_distance.step(this->x_distance_req, motor_control.state.x_dist_est, motor_control.state.x_vel_est);
    float theta_req_s = shaper_angle.step(this->x_theta_req, motor_control.state.x_theta_est, motor_control.state.x_omega_est);


    // required state, fill constant trajectory for MPC   
    position_controller.set_xr_constant(dist_req_s, theta_req_s);

    position_controller.callback(); 

    this->steps++;
}



void ControlLoop::timer_init()
{
     /* Enable TIM8 clock (on APB2) */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM8);

    /* Disable TIM8 during configuration */
    LL_TIM_DisableCounter(TIM8);

    /* Timer clock = 216 MHz (APB2) */
    /* Prescaler = 215 → 216 MHz / 216 = 1 MHz timer tick */
    LL_TIM_SetPrescaler(TIM8, 215);

    /* Auto-reload for desired frequency : 1 MHz / CONTROLLER_UPDATE_RATE_HZ */
    /* CONTROLLER_UPDATE_RATE_HZ = 200 → ARR = 5000, 200Hz */   
    /* CONTROLLER_UPDATE_RATE_HZ = 400 → ARR = 2500, 400Hz */
    LL_TIM_SetAutoReload(TIM8, 1000000/CONTROLLER_UPDATE_RATE_HZ - 1);

    /* Count direction up */
    LL_TIM_SetCounterMode(TIM8, LL_TIM_COUNTERMODE_UP);

    /* No repetition */
    LL_TIM_SetRepetitionCounter(TIM8, 0);

    /* Enable update interrupt (UIE) */
    LL_TIM_EnableIT_UPDATE(TIM8);       

    /* TIM8 update interrupt, preemption priority = 2 (lower than motor FOC, higher than SysTick) */
    NVIC_SetPriority(TIM8_UP_TIM13_IRQn, NVIC_EncodePriority(0, 2, 0));
    NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);

    /* Clear update flag */
    LL_TIM_ClearFlag_UPDATE(TIM8);

    /* Start counter */
    LL_TIM_EnableCounter(TIM8);  
}
    