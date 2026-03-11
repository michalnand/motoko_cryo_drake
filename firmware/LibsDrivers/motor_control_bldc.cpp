#include <libs_drivers.h>
#include <motor_control_bldc.h>

#include <tmath.h>
#include <sine_table.h>
    
#define MOTOR_TIMER_FREQ             ((uint32_t)2000)
#define MOTOR_CONTROL_MAX_TORQUE     ((float)1.0)

    
#define MOTOR_POLES                  ((int32_t)14)       




#ifdef __cplusplus
extern "C" {
#endif

MotorControl *g_motor_control_ptr;

// timer 7 interrupt handler, running velocity control
void TIM7_IRQHandler(void)
{ 
    if (LL_TIM_IsActiveFlag_UPDATE(TIM7))
    {
        LL_TIM_ClearFlag_UPDATE(TIM7);
        g_motor_control_ptr->callback();
    }
}   


#ifdef __cplusplus
}
#endif


//init motor control process
void MotorControl::init()
{
    g_motor_control_ptr = this;

    this->steps = 0;

    this->left_torque         = 0;
    this->right_torque        = 0;

    // encoder reading
    this->left_position_prev    = 0.0f;
    this->right_position_prev   = 0.0f;
    this->left_position         = 0.0f;
    this->right_position        = 0.0f;

    this->left_req_velocity     = 0.0f;
    this->right_req_velocity     = 0.0f;

    this->left_cl_mode  = false;
    this->right_cl_mode = false;    


    
    left_pwm.init();    
    right_pwm.init();

    // set motors to zero position      
    set_torque_from_rotation(PWM_VALUE_MAX/2, 0, true, 0);
    set_torque_from_rotation(PWM_VALUE_MAX/2, 0, true, 1);
    

    timer.delay_ms(200);    

    // calibrate encoders   
    left_encoder.init(); 
    right_encoder.init();   

    
    // release motors   
    set_torque_from_rotation(0, 0, true, 0);
    set_torque_from_rotation(0, 0, true, 1);

    timer.delay_ms(100);

    //optimal control init 
    float a =  0.92059711;
    float b =  16.43346038;

    float k  =  0.00170242;
    float ki =  0.00015592;
    float f  =  0.05386474; 
    

    left_controller.init(a, b, k, ki, f, 1.0);
    right_controller.init(a, b, k, ki, f, 1.0);

    //init timer        
    timer_init();
}


// turn OFF closed loop control, and set torque directly
// range : -1, 1, for min and max torque
void MotorControl::set_left_torque(float left_torque)
{
    this->left_cl_mode  = false;
    this->left_torque   = left_torque;
}

// turn OFF closed loop control, and set torque directly
// range : -1, 1, for min and max torque
void  MotorControl::set_right_torque(float right_torque)
{
    this->right_cl_mode = false;
    this->right_torque  = right_torque;
}


// turn ON closed loop control, and set required velocity in rad/s
// max velocity is arround 1000RPM (2PI*1000/60 [rad/s])
void MotorControl::set_left_velocity(float left_velocity)
{
    this->left_cl_mode       = true; 
    this->left_req_velocity  = left_velocity;
}

// turn ON closed loop control, and set required velocity in rad/s
// max velocity is arround 1000RPM (2PI*1000/60 [rad/s])
void MotorControl::set_right_velocity(float right_velocity)
{
    this->right_cl_mode       = true;
    this->right_req_velocity  = right_velocity;
}


// force break to both motors
void MotorControl::halt()
{
    set_torque_from_rotation(0, 0, false, 0);
    set_torque_from_rotation(0, 0, false, 1);
}


// wheel position (angle), 2PI is equal to one full forward rotation, -2PI for backward
float MotorControl::get_left_position()
{
    return this->left_position;
}

// wheel position (angle), 2PI is equal to one full forward rotation, -2PI for backward
float MotorControl::get_right_position()
{
    return this->right_position;
}

float MotorControl::get_left_velocity()
{
    return (this->left_position - this->left_position_prev)/(1.0f/MOTOR_TIMER_FREQ);
}

float MotorControl::get_left_velocity_hat()
{   
    return left_controller.get_x_hat();
}

float MotorControl::get_right_velocity()    
{
    return (this->right_position - this->right_position_prev)/(1.0f/MOTOR_TIMER_FREQ);
}   

float MotorControl::get_right_velocity_hat()
{   
    return right_controller.get_x_hat();
}   


void MotorControl::callback()
{
    // refresh encoders
    left_encoder.update();          
    right_encoder.update();      

    // update state
    this->left_position_prev    = this->left_position;
    this->right_position_prev   = this->right_position;
    this->left_position         = -(2.0f*PI*left_encoder.position)/ENCODER_RESOLUTION;
    this->right_position        = (2.0f*PI*right_encoder.position)/ENCODER_RESOLUTION;

    
    // LQG controller   
    if (this->left_cl_mode)
    {
        this->left_torque = left_controller.step(this->left_req_velocity, this->get_left_velocity());
    }
    else
    {
        left_controller.reset();
        left_controller.kalman_step(this->get_left_velocity(), this->left_torque);
    }

    if (this->right_cl_mode)
    {
        this->right_torque = right_controller.step(this->right_req_velocity, this->get_right_velocity());
    }
    else
    {
        right_controller.reset();
        right_controller.kalman_step(this->get_right_velocity(), this->right_torque);
    }
    

    // scale -1...1 range into -MOTOR_CONTROL_MAX ... MOTOR_CONTROL_MAX
    // send torques to motors   
    set_torque_from_rotation(-this->left_torque*PWM_VALUE_MAX, left_encoder.angle,  false, 0);
    set_torque_from_rotation(this->right_torque*PWM_VALUE_MAX, right_encoder.angle, false, 1);
    

    this->steps++;  
}









void MotorControl::timer_init()
{
    /* Enable TIM7 clock (on APB1) */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM7);

    /* Disable TIM7 during configuration */
    LL_TIM_DisableCounter(TIM7);

    /* Timer clock = 216 MHz (APB1 doubled) */
    /* Prescaler = 215 → 216 MHz / 216 = 1 MHz timer tick */
    LL_TIM_SetPrescaler(TIM7, 215);

    /* Auto-reload for 4 kHz: 1 MHz / 250 = 4000 Hz */
    LL_TIM_SetAutoReload(TIM7, 1000000/MOTOR_TIMER_FREQ);


    /* Enable update interrupt (UIE) */
    LL_TIM_EnableIT_UPDATE(TIM7);       

    // TIM7 highest priority, preemption = 0
    NVIC_SetPriority(TIM7_IRQn, NVIC_EncodePriority(0, 0, 0));
    NVIC_EnableIRQ(TIM7_IRQn);

    /* Clear update flag */
    LL_TIM_ClearFlag_UPDATE(TIM7);

    /* Start counter */
    LL_TIM_EnableCounter(TIM7);
}




void MotorControl::set_torque_from_rotation(int32_t torque, uint32_t rotor_angle, bool brake, int motor_id)
{
    torque = clip(torque, -(int32_t)PWM_VALUE_MAX, (int32_t)PWM_VALUE_MAX);

    // convert mechanical angle to electrical angle and index into sine table
    uint32_t table_angle = (rotor_angle*MOTOR_POLES*SINE_TABLE_SIZE)/(2*ENCODER_RESOLUTION);

    if (brake)
    {
        rotor_angle = 0;
        table_angle = 0;
    }  
    else if (torque >= 0)
    {   
        table_angle = (table_angle - (3*SINE_TABLE_SIZE)/4) % SINE_TABLE_SIZE;
    }
    else if (torque < 0)          
    {   
        torque = -torque;
        table_angle = (table_angle + (3*SINE_TABLE_SIZE)/4) % SINE_TABLE_SIZE;
    }    

    

    // produce 3-phase shifted angles
    uint32_t angle_a = table_angle % SINE_TABLE_SIZE;
    uint32_t angle_b = (table_angle + SINE_TABLE_SIZE / 3) % SINE_TABLE_SIZE;
    uint32_t angle_c = (table_angle + 2 * (SINE_TABLE_SIZE / 3)) % SINE_TABLE_SIZE;

    // read sinusoidal values
    int32_t sa = sine_table[angle_a];
    int32_t sb = sine_table[angle_b];   
    int32_t sc = sine_table[angle_c];

   

    int32_t pwm_a = (sa*torque)/SINE_VALUE_MAX;
    int32_t pwm_b = (sb*torque)/SINE_VALUE_MAX; 
    int32_t pwm_c = (sc*torque)/SINE_VALUE_MAX; 


    //pwm values    
    pwm_a = clip(pwm_a, (int32_t)(0), ((int32_t)PWM_VALUE_MAX)-1);
    pwm_b = clip(pwm_b, (int32_t)(0), ((int32_t)PWM_VALUE_MAX)-1);
    pwm_c = clip(pwm_c, (int32_t)(0), ((int32_t)PWM_VALUE_MAX)-1);

    if (motor_id == 0)      
    {
        left_pwm.set(pwm_b, pwm_a, pwm_c);
    }   
    else         
    {
        right_pwm.set(pwm_a, pwm_b, pwm_c);
    }
}   
