#include <libs_drivers.h>
#include <tmath.h>

#define FORWARD_RPM_MAX     ((uint32_t)300)
#define TURN_RPM_MAX        ((uint32_t)700) 

#define NUM_SAMPLES ((uint32_t)1250)
#define DT_MS       ((uint32_t)4)

// distance and angle thresholds for bang-bang switching
// these sweep from max to min over the course of the experiment
// to produce frequency-sweeping square wave excitation
#define DIST_THRESHOLD_MAX  ((float)100.0)   // mm
#define DIST_THRESHOLD_MIN  ((float)5.0)     // mm
#define THETA_THRESHOLD_MAX ((float)1.0)     // rad
#define THETA_THRESHOLD_MIN ((float)0.05)    // rad


uint32_t g_random_var = 0;

uint8_t random()
{
    g_random_var = (g_random_var * 1664525) + 1013904223;
    return (int32_t)g_random_var >> 3;
}


void robot_idenditification()
{
    Gpio<'B', 2, GPIO_MODE_IN_PULLUP> key_0;
    Gpio<'B', 0, GPIO_MODE_OUT> led;
    
    // wait for key press to start experiment
    while (key_0 == 1)
    {
        led = 0;  
        timer.delay_ms(100);
        led = 1;  
        timer.delay_ms(800);
    }

    while (key_0 == 0)
    {
        led = 0;  
        timer.delay_ms(100);
        led = 1;  
        timer.delay_ms(100);
    }

    led = 0;  

    timer.delay_ms(500); 

    //stop motor
    motor_control.halt();
    timer.delay_ms(200); 


    float u_forward[NUM_SAMPLES];
    float u_turn[NUM_SAMPLES];
    
    float x_distance[NUM_SAMPLES];
    float x_theta[NUM_SAMPLES];
    float x_velocity[NUM_SAMPLES];
    float x_omega[NUM_SAMPLES];

    // bang-bang state: +1 or -1
    float forward_sign = 0.0f;
    float turn_sign    = 0.0f;

    // reference points for measuring distance/angle travelled since last switch
    float dist_ref  = motor_control.state.x_dist_est;
    float theta_ref = motor_control.state.x_theta_est;

    
    for (unsigned int n = 0; n < NUM_SAMPLES; n++)
    {
        uint32_t time_start = timer.get_time();


        if ((n%15) == 0)
        {
            switch (random()%9)
            {
                case 0:
                    forward_sign = 0.0f;
                    turn_sign    = 0.0f;
                    break;

                    
                case 1:
                    forward_sign = 0.5f;
                    turn_sign    = 0.0f;
                    break;

                case 2:
                    forward_sign = -0.5f;
                    turn_sign    = 0.0f;
                    break;

                case 3:
                    forward_sign = 1.0f;
                    turn_sign    = 0.0f;
                    break;

                case 4:
                    forward_sign = -1.0f;
                    turn_sign    = 0.0f;
                    break;


                case 5:
                    forward_sign = 0.0f;
                    turn_sign    = 0.5f;
                    break;

                case 6:
                    forward_sign = 0.0f;
                    turn_sign    = 0.5f;
                    break;

                case 7:
                    forward_sign = 0.0f;
                    turn_sign    = 1.0f;
                    break;

                case 8:
                    forward_sign = 0.0f;
                    turn_sign    = -1.0f;
                    break;
              
            }

            /*
            switch (random()%5)
            {
                case 0:
                    forward_sign = 0.0f;
                    break;
                case 1:
                    forward_sign = 1.0f;
                    break;
                case 2:
                    forward_sign = -1.0f;
                    break;
                case 3:
                    forward_sign = 0.5f;
                    break;
                case 4:
                    forward_sign = -0.5f;
                    break;
            }
            

            switch (random()%5)
            {
                case 0:
                    turn_sign = 0.0f;
                    break;
                case 1:
                    turn_sign = 1.0f;
                    break;
                case 2:
                    turn_sign = -1.0f;
                    break;
                case 3:
                    turn_sign = 0.5f;
                    break;
                case 4:
                    turn_sign = -0.5f;
                    break;
            }
            */
        }
        

        // generate two kind of motions, separated for forward only and turn only
        // the both motions are square wave like pattern with sweeping frequency
        // the switch of direction is when given distance travelled, or angle rotated
        // motions have zero mean value - basically it is simple bang bang control with sweeping frequency
        float u_forward_ = forward_sign * (float)FORWARD_RPM_MAX;
        float u_turn_    = turn_sign    * (float)TURN_RPM_MAX;

        float left_rpm  = u_forward_ + u_turn_;
        float right_rpm = u_forward_ - u_turn_;

        motor_control.set_left_velocity(left_rpm   * (2.0*PI/60.0));
        motor_control.set_right_velocity(right_rpm * (2.0*PI/60.0));

        u_forward[n] = u_forward_;
        u_turn[n]    = u_turn_; 

        x_distance[n] = motor_control.state.x_dist_est;
        x_theta[n]    = motor_control.state.x_theta_est;
        x_velocity[n] = motor_control.state.x_vel_est;
        x_omega[n]    = motor_control.state.x_omega_est;

        if ((n%10) == 0)
        {
            led = 0;
        }
        else
        {
            led = 1;
        }





        uint32_t time_stop = timer.get_time();

        int32_t time_wait = DT_MS - (time_stop - time_start);
        if (time_wait > 0)        
        {
            timer.delay_ms(time_wait);
        }
    }

    motor_control.set_left_torque(0);   
    motor_control.set_right_torque(0);
    
    timer.delay_ms(200);     
    
    while (true)
    {
        while (key_0 == 1)
        {
            led = 0;  
            timer.delay_ms(100);
            led = 1;  
            timer.delay_ms(800);
        }

        terminal << "\n\n\n";
        
        for (unsigned int n = 0; n < NUM_SAMPLES; n++)
        {
            terminal << n << " " << u_forward[n] << " " << u_turn[n] << " " << x_distance[n] << " " << x_velocity[n] << " "  << x_theta[n] << " " << x_omega[n] << "\n";
        }

        terminal << "\n\n\n";
    }
}
