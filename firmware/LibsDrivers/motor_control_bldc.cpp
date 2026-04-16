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
int MotorControl::init(float k)
{
    g_motor_control_ptr = this;

    this->steps = 0;

    this->right_torque        = 0;
    this->left_torque         = 0;

    this->right_torque_s = 0;
    this->left_torque_s  = 0;

    this->k = k;
    
    
    right_pwm.init();
    left_pwm.init();    

    // set motors to zero position 
    for (unsigned int i = 0; i < 10; i++)     
    {
        int32_t torque = (PWM_VALUE_MAX*i)/10;
        set_torque_from_rotation(torque, (90*(int32_t)ENCODER_RESOLUTION)/360, 0);
        set_torque_from_rotation(torque, (90*(int32_t)ENCODER_RESOLUTION)/360, 1);
        timer.delay_ms(4);
    }

    
    timer.delay_ms(500);       

    // calibrate encoders   
    int right_init_res = right_encoder.init();   
    int left_init_res  = left_encoder.init(); 

    
    // release motors   
    set_torque_from_rotation(0, 0, 0);
    set_torque_from_rotation(0, 0, 1);

    timer.delay_ms(100);

    // velocity filters
    right_velocity_median_filter.init();
    left_velocity_median_filter.init();

    // state estimator, holds smooth robot state
    float pos_cutoff_freq = 200.0f;
    float vel_cutoff_freq = 200.0f;     
    state.init(pos_cutoff_freq, vel_cutoff_freq, 1.0f/MOTOR_TIMER_FREQ);


    //init timer  
    timer_init();

    if (left_init_res != 0)
    {
        return -1;
    }

    if (right_init_res != 0)
    {
        return -2;
    }

    return 0;
}


// range : -1, 1, for min and max torque
void  MotorControl::set_right_torque(float right_torque)
{
    this->right_torque  = right_torque;
}

// range : -1, 1, for min and max torque
void MotorControl::set_left_torque(float left_torque)
{
    this->left_torque   = left_torque;
}

void MotorControl::set(float forward, float turn)
{
    float right_torque = forward + turn;
    float left_torque  = forward - turn;

    set_right_torque(right_torque);
    set_left_torque(left_torque);
}   

// force break to both motors
void MotorControl::halt()
{
    set_torque_from_rotation(0, 0, 0);
    set_torque_from_rotation(0, 0, 1);
}

float MotorControl::get_right_position()
{
    return (2.0f*PI*right_encoder.position)/ENCODER_RESOLUTION;
}

float MotorControl::get_left_position()
{
    return -(2.0f*PI*left_encoder.position)/ENCODER_RESOLUTION;
}

void MotorControl::callback()
{
    // refresh encoders
    right_encoder.update();      
    left_encoder.update();          
    
    // update wheels state, position to radians
    float right_position        = (2.0f*PI*right_encoder.position)/ENCODER_RESOLUTION;
    float left_position         = -(2.0f*PI*left_encoder.position)/ENCODER_RESOLUTION;
    
      
    // linear distance : average wheels traveled distance, convert radians to distance in meters
    float distance = 0.001*(WHEEL_RADIUS_MM)*(left_position + right_position)/(2.0f);

    // angle : difference of wheels traveled distance ratio to wheels brace, convert radians to angle
    float theta    = (WHEEL_RADIUS_MM)*(right_position - left_position)/(WHEEL_BRACE_MM);

    // update state estimator, holds smooth robot state
    state.step(distance, theta);     

    this->right_torque_s = this->k*this->right_torque_s + (1.0f - this->k)*this->right_torque;
    this->left_torque_s  = this->k*this->left_torque_s  + (1.0f - this->k)*this->left_torque;

    // scale -1...1 range into -PWM_VALUE_MAX .. PWM_VALUE_MAX
    // send torques to motors   
    set_torque_from_rotation(this->right_torque_s*PWM_VALUE_MAX, right_encoder.angle, 1);
    set_torque_from_rotation(-this->left_torque_s*PWM_VALUE_MAX, left_encoder.angle, 0);
    
        
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

    // TIM7  highest priority, preemption = 0   
    NVIC_SetPriority(TIM7_IRQn, NVIC_EncodePriority(0, 0, 0));
    NVIC_EnableIRQ(TIM7_IRQn);

    /* Clear update flag */
    LL_TIM_ClearFlag_UPDATE(TIM7);

    /* Start counter */
    LL_TIM_EnableCounter(TIM7);
}




void MotorControl::set_torque_from_rotation(int32_t torque, uint32_t rotor_angle, int motor_id)
{
    torque = clip(torque, -(int32_t)PWM_VALUE_MAX, (int32_t)PWM_VALUE_MAX);

    // convert mechanical angle to electrical angle and index into sine table
    uint32_t table_angle = (rotor_angle*MOTOR_POLES*SINE_TABLE_SIZE)/(2*ENCODER_RESOLUTION);

    
    if (torque >= 0)
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
        left_pwm.set(pwm_c, pwm_b, pwm_a);
    }   
    else         
    {
        right_pwm.set(pwm_a, pwm_b, pwm_c);
    }
}   
