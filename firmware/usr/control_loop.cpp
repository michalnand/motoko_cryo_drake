#include "control_loop.h"

#define CONTROLLER_UPDATE_RATE_HZ  ((uint32_t)250)

#ifdef __cplusplus
extern "C" {
#endif

ControlLoop *g_control_loop_ptr;

// timer 1 interrupt handler, running controller update
void TIM1_UP_TIM10_IRQHandler(void)
{ 
    if (LL_TIM_IsActiveFlag_UPDATE(TIM1))
    {
        LL_TIM_ClearFlag_UPDATE(TIM1);
        g_control_loop_ptr->callback();
    }
} 


#ifdef __cplusplus
}
#endif

int ControlLoop::init(Sensors &sensors, MotorControl &motor_control)
{
    // instance init, variables set
    g_control_loop_ptr = this;

    this->sensors       = &sensors;
    this->motor_control = &motor_control;
    
    // hardware init 
    this->sensors->init();
    this->motor_control->init();


    float dt = 1.0f/(float)CONTROLLER_UPDATE_RATE_HZ;

    // input shapers, for acceleration and velocity limits
    ShaperVelAcc shaper_distance;
    ShaperVelAcc shaper_angle;

    // linear motion constrains
    {
        float v_max            = 1000.0f; // mm/s
        float acc_positive_max = 2000.0f; // mm/s^2
        float acc_negative_max = -2.0f*acc_positive_max; // mm/s^2, faster decceleration for braking

        shaper_distance.init(-v_max, v_max, acc_negative_max, acc_positive_max, 0.3, dt);
    }

    // turn motion constrains
    {
        float v_max            = 1000.0f; // mm/s
        float acc_positive_max = 10.0f; // mm/s^2   
        float acc_negative_max = -acc_positive_max; // mm/s^2, faster decceleration for braking

        shaper_angle.init(-v_max, v_max, acc_negative_max, acc_positive_max, 0.3, dt);
    }


    // init controller  
    //lqr_controller.init((float*)lqr_k,  1.0f);
    lqr_controller.init((float*)lqr_k, (float*)lqr_ku, 100.0f, 1.0f);
    


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
    this->sensors->callback();

    // shapers for acceleration and velocity limits
    float x_distance_req_s = shaper_distance.step(this->x_distance_req);
    float x_theta_req_s    = shaper_angle.step(this->x_theta_req);
        
    // controller computation, obtain control outputs

    // required state
    lqr_controller.xr[0] = this->x_distance_req;
    lqr_controller.xr[1] = 0.0f;
    lqr_controller.xr[2] = this->x_theta_req;
    lqr_controller.xr[3] = 0.0f;
    
    // get current state (already filtered in fast 2kHz motor control loop)
    lqr_controller.x[0] = motor_control->state.x_dist_est;
    lqr_controller.x[1] = motor_control->state.x_vel_est;
    lqr_controller.x[2] = motor_control->state.x_theta_est;
    lqr_controller.x[3] = motor_control->state.x_omega_est;
    
    // controller step
    lqr_controller.step();  

    // control outputs  
    float u_forward = lqr_controller.u[0];
    float u_turn    = lqr_controller.u[1];

    float left_u  =  u_forward + u_turn;
    float right_u =  u_forward - u_turn;

    // motor loop handles LQR velocity control
    motor_control->set_left_velocity(left_u   * MOTOR_CONTROL_MAX_VELOCITY);
    motor_control->set_right_velocity(right_u * MOTOR_CONTROL_MAX_VELOCITY);

    this->steps++;


        
}



void ControlLoop::timer_init()
{
    /* Enable TIM1 clock (on APB2) */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

    /* Disable TIM1 during configuration */
    LL_TIM_DisableCounter(TIM1);

    /* Timer clock = 216 MHz (APB2 timer clock on STM32F722)
       Prescaler = 215 → 216 MHz / 216 = 1 MHz timer tick */
    LL_TIM_SetPrescaler(TIM1, 215);

    /* Auto-reload for 250 Hz: 1 MHz / 250 = 4000 ticks */
    LL_TIM_SetAutoReload(TIM1, (1000000 / CONTROLLER_UPDATE_RATE_HZ) - 1);

    /* Count direction up */
    LL_TIM_SetCounterMode(TIM1, LL_TIM_COUNTERMODE_UP);

    /* No repetition */
    LL_TIM_SetRepetitionCounter(TIM1, 0);

    /* Enable update interrupt (UIE) */
    LL_TIM_EnableIT_UPDATE(TIM1);

    /* TIM1 update interrupt, preemption priority = 3 (lower than TIM8 at 2) */
    NVIC_SetPriority(TIM1_UP_TIM10_IRQn, NVIC_EncodePriority(0, 3, 0));
    NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

    /* Clear update flag */
    LL_TIM_ClearFlag_UPDATE(TIM1);  

    /* Start counter */
    LL_TIM_EnableCounter(TIM1);
}
