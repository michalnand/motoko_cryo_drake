#include "sensors.h"
#include "libs_drivers.h"
#include <tmath.h>



#ifdef __cplusplus
extern "C" {
#endif

Sensors *g_sensors_ptr;

// timer 8 interrupt handler, running sensors reading
/*
void TIM8_UP_TIM13_IRQHandler(void)
{ 
    if (LL_TIM_IsActiveFlag_UPDATE(TIM8))
    {
        LL_TIM_ClearFlag_UPDATE(TIM8);
        g_sensors_ptr->callback();
    }
} 
*/


#ifdef __cplusplus
}
#endif

int Sensors::init()
{
    // instance init
    g_sensors_ptr = this;

    // measurement incremental counter for debug
    this->measurement_id = 0;   

    // line sensor vars init
    weights[0] = -5*LINE_SENSOR_STEP;
    weights[1] = -4*LINE_SENSOR_STEP;
    weights[2] = -3*LINE_SENSOR_STEP; 
    weights[3] = -2*LINE_SENSOR_STEP;
    weights[4] = -1*LINE_SENSOR_STEP;
    weights[5] =  1*LINE_SENSOR_STEP;
    weights[6] =  2*LINE_SENSOR_STEP;
    weights[7] =  3*LINE_SENSOR_STEP;
    weights[8] =  4*LINE_SENSOR_STEP;
    weights[9] =  5*LINE_SENSOR_STEP;

    this->sensor_status = 0;
    this->line_lost_type = LINE_LOST_CENTER;
    this->on_line_count  = 0;

    this->left_position = 0.0f;
    this->right_position = 0.0f;
    this->center_position = 0.0f;

    this->minimal_position = 0.0f;
    this->extremal_position = 0.0f; 

    this->left_angle = 0.0f;
    this->right_angle = 0.0f;

    // proximity sensor vars init
    this->front_left_proximity = 0.0f;
    this->left_proximity = 0.0f;
    this->right_proximity = 0.0f;
    this->front_right_proximity = 0.0f;


    // hardware init 
    i2c.init();
    ls_driver.init(i2c);

    int resp = ls_driver.init(i2c);

    //timer_init();

    return resp;
}

void Sensors::callback()
{
    uint8_t result = ls_driver.read_who_am_i();
    if (result == LS_WHO_AM_I_VALUE)
    {
        this->sensor_status = 1;
    }
    else
    {
        this->sensor_status = 0;
    }   
        
    uint16_t sensor_reading[LS_DATA_SIZE];

    // read all sensors, line and proximity
    ls_driver.read_data(sensor_reading, LS_DIF_FIL_REG);

    // copy line and proximity sensor readings to class members
    for (int i = 0; i < (int)line_reading_result.size(); i++)
    {
        line_reading_result[i] = 4096 - sensor_reading[i];
    }

    for (int i = 0; i < (int)proximity_reading_result.size(); i++)
    {
        proximity_reading_result[i] = sensor_reading[i + LINE_SENSOR_COUNT];
    }   
    
    // data processing
    line_sensor_process();

    proximity_sensor_process();

    this->measurement_id++;
}   







void Sensors::line_sensor_process()
{    

    //find most left sensor on line
    unsigned int left_idx = 0;
    bool left_valid = false;
    for (int i = (line_reading_result.size()-1); i >= 0; i--)
        if (line_reading_result[i] > LINE_SENSOR_THRESHOLD)
        {
            left_idx = i;
            left_valid = true; 
            break;
        }
        
    //find most right sensor on line 
    unsigned int right_idx = 0;
    bool right_valid = false;
    for (int i = 0; i < (int)line_reading_result.size(); i++)
        if (line_reading_result[i] > LINE_SENSOR_THRESHOLD)
        {
            right_idx = i;
            right_valid = true;
            break;
        }


    //compute line position arround strongest sensors
    float k = 1.0/((LINE_SENSOR_COUNT/2)*LINE_SENSOR_STEP);

    float left_position_tmp  = left_position;
    float right_position_tmp = right_position;

   
    if (left_valid)
    {
        left_position_tmp  = k*integrate(left_idx);
        right_position_tmp = left_position_tmp;

    }

    if (right_valid)
    {
        right_position_tmp  = k*integrate(right_idx);
    }   

  
    //solve if line lost
    if ((left_valid == false) && (right_valid == false))
    {
        if (left_position_tmp < -0.4)   
        {
            line_lost_type = LINE_LOST_RIGHT;   
        }
        else if (left_position_tmp > 0.4) 
        {
            line_lost_type = LINE_LOST_LEFT;
        }
        else
        {
            line_lost_type = LINE_LOST_CENTER;
        }
    }
    else
    {
        line_lost_type  = LINE_LOST_NONE;
    }
    

    left_position  = left_position_tmp;
    right_position = right_position_tmp;

    center_position = (left_position + right_position)/2.0;

    if (abs(left_position) < abs(right_position))
    { 
        minimal_position = left_position;
        extremal_position= right_position;
    }
    else
    {
        minimal_position = right_position;
        extremal_position= left_position;
    }
    

    //compute to robot angle in radians
    left_angle  = fatan(left_position * (SENSORS_BRACE/2.0) / SENSORS_DISTANCE);
    right_angle = fatan(right_position * (SENSORS_BRACE/2.0) / SENSORS_DISTANCE);
}


void Sensors::proximity_sensor_process()
{
    float k = 0.125;

   
    
    {
        float tmp = (4096 - proximity_reading_result[0])/4096.0;
        this->front_left_proximity = k*tmp + (1.0 - k)*this->front_left_proximity;
    }

    {
        float tmp = (4096 - proximity_reading_result[1])/4096.0;
        this->left_proximity = k*tmp + (1.0 - k)*this->left_proximity;
    }
    
    {
        float tmp = (4096 - proximity_reading_result[2])/4096.0;
        this->right_proximity = k*tmp + (1.0 - k)*this->right_proximity;
    }

    {
        float tmp = (4096 - proximity_reading_result[3])/4096.0;
        this->front_right_proximity = k*tmp + (1.0 - k)*this->front_right_proximity;
    }
}
   



int Sensors::integrate(int center_idx)
{
    if (center_idx < 0)
        center_idx = 0;

    if (center_idx > (int)(LINE_SENSOR_COUNT-1))
        center_idx = (int)(LINE_SENSOR_COUNT-1);

    int center  = weights[center_idx]*line_reading_result[center_idx];
    int sum     = line_reading_result[center_idx];

    int int_result = center;

    if (center_idx > 0)
    {
        int_result+= weights[center_idx - 1]*line_reading_result[center_idx - 1];
        sum+= line_reading_result[center_idx - 1];
    }
    else
    {
        int_result+= center;
        sum+= line_reading_result[center_idx];
    }

    if (center_idx < (int)(LINE_SENSOR_COUNT-1))
    {
        int_result+= weights[center_idx+1]*line_reading_result[center_idx+1];
        sum+= line_reading_result[center_idx+1];
    }
    else
    {
        int_result+= center;
        sum+= line_reading_result[center_idx];
    }

    int_result = int_result/sum;

    return int_result;
}




void Sensors::timer_init()
{
    /* Enable TIM8 clock (on APB2) */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM8);

    /* Disable TIM8 during configuration */
    LL_TIM_DisableCounter(TIM8);

    /* Timer clock = 216 MHz (APB2) */
    /* Prescaler = 215 → 216 MHz / 216 = 1 MHz timer tick */
    LL_TIM_SetPrescaler(TIM8, 215);

    /* Auto-reload for desired frequency : 1 MHz / SENSORS_TIMER_FREQ */
    /* SENSORS_TIMER_FREQ = 200 → ARR = 5000, 200Hz */
    /* SENSORS_TIMER_FREQ = 400 → ARR = 2500, 400Hz */
    LL_TIM_SetAutoReload(TIM8, 1000000/SENSORS_TIMER_FREQ - 1);

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
