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
    /*
    // linear motion constrains
    {
        float v_max            = 5.0f; // m/s
        float acc_positive_max = 100.0f; // m/s^2
        float acc_negative_max = -2.0f*acc_positive_max; // m/s^2, faster decceleration for braking

        shaper_distance.init(-v_max, v_max, acc_negative_max, acc_positive_max, 0.3, dt);
    }

    // turn motion constrains       
    {
        float v_max            = 20.0f; // rad/s  
        float acc_positive_max = 1000.0f; // rad/s^2   
        float acc_negative_max = -acc_positive_max; // rad/s^2, faster decceleration for braking

        shaper_angle.init(-v_max, v_max, acc_negative_max, acc_positive_max, 0.3, dt);
    }
    */

    shaper_distance.init(0.6f);
    shaper_angle.init(0.6f);         


    // init controller  
    controller.init((float*)mpc_phi, (float*)mpc_sigma);    
    
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


    // get current state (already filtered in fast 2kHz motor control loop)
    controller.x[0] = motor_control->state.x_dist_est;
    controller.x[1] = motor_control->state.x_vel_est;
    controller.x[2] = motor_control->state.x_theta_est;
    controller.x[3] = motor_control->state.x_omega_est; 

    // required state, fill constant trajectory for MPC   
    controller.set_constant_xr(0, x_distance_req_s);    
    controller.set_constant_xr(2, x_theta_req_s);   


    // controller step 
    controller.step();       
        
    // control outputs convert to robot control inputs
    float u_forward = controller.u[0];
    float u_turn    = controller.u[1];     

    float right_u =  u_forward + u_turn;      
    float left_u  =  u_forward - u_turn;
    
    motor_control->set_right_torque(right_u);
    motor_control->set_left_torque(left_u);

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
    