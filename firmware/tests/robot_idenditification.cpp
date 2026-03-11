#include <libs_drivers.h>
#include <tmath.h>

#define RPM_MAX     ((uint32_t)1000)
#define NUM_SAMPLES ((uint32_t)2000)



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

    
    for (unsigned int n = 0; n < NUM_SAMPLES; n++)
    {
        // generate two kind of motions, separated for forward only and turn only
        // the both motions are square wave like pattern with sweeping frequency
        // the switch of direction is when given distance travelled, or angle rotated
        float u_forward_ = RPM_MAX*0; // TODO
        float u_turn_    = RPM_MAX*0; // TODO

        float left_rpm  = u_forward_ + u_turn_;
        float right_rpm = u_forward_ - u_turn_;

        motor_control.set_left_velocity(left_rpm   * (2.0*PI/60.0));
        motor_control.set_right_velocity(right_rpm * (2.0*PI/60.0));

        timer.delay_ms(10);

        u_forward[n] = u_forward_;
        u_turn[n]    = u_turn_; 

        x_distance[n] = motor_control.state.x_dist_est;
        x_theta[n]    = motor_control.state.x_theta_est;
        x_velocity[n] = motor_control.state.x_vel_est;
        x_omega[n]    = motor_control.state.x_omega_est;
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
