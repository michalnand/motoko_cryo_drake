#include <libs_drivers.h>
#include <tmath.h>

#define RPM_MAX     ((uint32_t)1000)
#define NUM_SAMPLES ((uint32_t)2000)
#define DT_MS       ((uint32_t)4)

// distance and angle thresholds for bang-bang switching
// these sweep from max to min over the course of the experiment
// to produce frequency-sweeping square wave excitation
#define DIST_THRESHOLD_MAX  ((float)100.0)   // mm
#define DIST_THRESHOLD_MIN  ((float)5.0)     // mm
#define THETA_THRESHOLD_MAX ((float)1.0)     // rad
#define THETA_THRESHOLD_MIN ((float)0.05)    // rad


void robot_idenditification()
{
    Gpio<'B', 0, GPIO_MODE_OUT> led;
   
    led = 0;  

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
    float forward_sign = 1.0f;
    float turn_sign    = 1.0f;

    // reference points for measuring distance/angle travelled since last switch
    float dist_ref  = motor_control.state.x_dist_est;
    float theta_ref = motor_control.state.x_theta_est;

    
    for (unsigned int n = 0; n < NUM_SAMPLES; n++)
    {
        uint32_t time_start = timer.get_time();

        // progress ratio 0..1 over the experiment
        float progress = (float)n / (float)NUM_SAMPLES;

        // sweep thresholds from max to min (frequency increases over time)
        float dist_threshold  = DIST_THRESHOLD_MAX  + (DIST_THRESHOLD_MIN  - DIST_THRESHOLD_MAX)  * progress;
        float theta_threshold = THETA_THRESHOLD_MAX + (THETA_THRESHOLD_MIN - THETA_THRESHOLD_MAX) * progress;

        // current state
        float dist_now  = motor_control.state.x_dist_est;
        float theta_now = motor_control.state.x_theta_est;

        // check if distance travelled since last forward switch exceeds threshold
        float dist_delta = dist_now - dist_ref;
        if (dist_delta < 0.0f) dist_delta = -dist_delta;

        if (dist_delta >= dist_threshold)
        {
            forward_sign = -forward_sign;
            dist_ref = dist_now;
        }

        // check if angle rotated since last turn switch exceeds threshold
        float theta_delta = theta_now - theta_ref;
        if (theta_delta < 0.0f) theta_delta = -theta_delta;

        if (theta_delta >= theta_threshold)
        {
            turn_sign = -turn_sign;
            theta_ref = theta_now;
        }

        // generate two kind of motions, separated for forward only and turn only
        // the both motions are square wave like pattern with sweeping frequency
        // the switch of direction is when given distance travelled, or angle rotated
        // motions have zero mean value - basically it is simple bang bang control with sweeping frequency
        float u_forward_ = forward_sign * (float)RPM_MAX;
        float u_turn_    = turn_sign    * (float)RPM_MAX;

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
    

    // TODO terminal print results
    
    for (unsigned int n = 0; n < NUM_SAMPLES; n++)
    {
        terminal << "u_forward = " << u_forward[n] << "\t";
        terminal << "u_turn    = " << u_turn[n]    << "\t";
        terminal << "x_dist    = " << x_distance[n] << "\t";
        terminal << "x_theta   = " << x_theta[n]    << "\t";
        terminal << "x_vel     = " << x_velocity[n] << "\t";
        terminal << "x_omega   = " << x_omega[n]    << "\n";
    }
   
    while(1)
    {
        led = 0;  
        timer.delay_ms(100);
        led = 1;     
        timer.delay_ms(900);
    }
}
