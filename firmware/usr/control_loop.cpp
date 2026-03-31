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
    
    this->steps = 0;    

    // shapers init, smooth run
    this->distance_target = 0.0f;
    this->angle_target    = 0.0f;
    this->radius_target   = 0.0f;
    this->velocity_target = 0.0f;

    
    this->position_mode = true;

    

    position_controller.init();

    
    this->timer_init();

    return 0;
}


void ControlLoop::set_position(float distance_target, float angle_target)
{
    this->distance_target = distance_target;
    this->angle_target    = angle_target;

    this->position_mode = true;
}

void ControlLoop::set_circle_motion(float radius_target, float velocity_target)
{
    this->velocity_target   = velocity_target;
    this->radius_target     = radius_target;

    this->position_mode = false;
}



void ControlLoop::callback()
{
    // udpate sensors data (line sensor, proximity sensor, incl filtering, processing)
    sensors.callback();

  
    // position control mode
    float acc_min   = -10.0f;    
    float acc_max   = 10.0f;    
    float acc_w_max = 100.0f;
    
    if (this->position_mode)
    {
        this->planner_set_position(this->distance_target, this->angle_target, acc_min, acc_max, acc_w_max);
    }
    else
    {
        this->planner_set_circle_motion(this->radius_target, this->velocity_target, acc_min, acc_max);
    }

    position_controller.callback(); 

    this->steps++;
}


void ControlLoop::planner_set_position(float x_req, float a_req, float acc_min, float acc_max, float acc_w_max)
{
    // obtain time interval
    float dt = 1.0f/(float)CONTROLLER_UPDATE_RATE_HZ;

    // obtain state
    float x     = motor_control.state.x_dist_est;
    float v     = motor_control.state.x_vel_est;
    float a     = motor_control.state.x_theta_est;
    float w     = motor_control.state.x_omega_est;

    // required position change
    float v_req = (x_req - x)/dt;
    v_req       = v + clip((v_req - v), acc_min, acc_max);  

    // velocity braking limit 
    //float v_brake = sqrtf(2.0f * abs(acc_min) * abs(x_req - x)) * sgn(x_req - x);
    //v_req = sgn(v_req) * fminf(abs(v_req), abs(v_brake));
    
    float x_new = x + v_req*dt; 


    // required angle change
    float w_req = (a_req - a)/dt;   
    w_req       = w + clip((w_req - w), -acc_w_max, acc_w_max);
    float a_new = a + w_req*dt;

    position_controller.set_xr_constant(x_new, a_new);
}



void ControlLoop::planner_set_circle_motion(float r_req, float v_req, float acc_min, float acc_max)
{
    // obtain time interval
    float dt = 1.0f/(float)CONTROLLER_UPDATE_RATE_HZ;

    // obtain state
    float x     = motor_control.state.x_dist_est;
    float v     = motor_control.state.x_vel_est;
    float a     = motor_control.state.x_theta_est;
    float w     = motor_control.state.x_omega_est;

    // estimate required velocity change (acceleration)
    float acc_req = (v_req - v)/dt;

    // acceleration limit
    acc_req = clip(acc_req, acc_min, acc_max);

    float v_curr    = v;
    float req_x     = x;
    float req_angle = a;

    for (unsigned int n = 0; n < MPC_PREDCTION_HORIZON; n++)
    {
        // update velocity with acceleration, and constrains
        v_curr+= acc_req*dt;
        v_curr = clip(v_curr, -abs(v_req), abs(v_req)); 

        //split velocity to tangential and angular components
        float vc = v_curr;  
        float va = v_curr/r_req;    
        
        // calculate motion change
        req_x+= vc*dt;
        req_angle+= va*dt;      

        // fill controller predictive trajectory with calculated motion change
        this->position_controller.set_xr_trajectory(n, req_x, req_angle);
    }
}

float ControlLoop::get_distance()
{
    return motor_control.state.x_dist_est;
}

float ControlLoop::get_angle()
{
    return motor_control.state.x_theta_est;
}

float ControlLoop::get_velocity()
{
    return motor_control.state.x_vel_est;
}

float ControlLoop::get_angular_velocity()
{
    return motor_control.state.x_omega_est;
}


void ControlLoop::timer_init()
{
    // Enable TIM8 clock (on APB2) 
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM8);

    // Disable TIM8 during configuration 
    LL_TIM_DisableCounter(TIM8);

    // Timer clock = 216 MHz (APB2) 
    // Prescaler = 215 → 216 MHz / 216 = 1 MHz timer tick 
    LL_TIM_SetPrescaler(TIM8, 215);

    // Auto-reload for desired frequency : 1 MHz / CONTROLLER_UPDATE_RATE_HZ 
    // CONTROLLER_UPDATE_RATE_HZ = 200 → ARR = 5000, 200Hz  
    // CONTROLLER_UPDATE_RATE_HZ = 400 → ARR = 2500, 400Hz 
    LL_TIM_SetAutoReload(TIM8, 1000000/CONTROLLER_UPDATE_RATE_HZ - 1);

    // Count direction up 
    LL_TIM_SetCounterMode(TIM8, LL_TIM_COUNTERMODE_UP);

    // No repetition 
    LL_TIM_SetRepetitionCounter(TIM8, 0);

    // Enable update interrupt (UIE) 
    LL_TIM_EnableIT_UPDATE(TIM8);       

    // TIM8 update interrupt, preemption priority = 2 (lower than motor FOC, higher than SysTick) 
    NVIC_SetPriority(TIM8_UP_TIM13_IRQn, NVIC_EncodePriority(0, 2, 0));
    NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);

    // Clear update flag 
    LL_TIM_ClearFlag_UPDATE(TIM8);

    // Start counter 
    LL_TIM_EnableCounter(TIM8);  
}
    