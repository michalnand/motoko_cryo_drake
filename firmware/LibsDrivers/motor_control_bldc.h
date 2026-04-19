#ifndef _MOTOR_CONTROL_BLDC_H_
#define _MOTOR_CONTROL_BLDC_H_

#include <as5600_t.h>
#include <state_estimator.h>
#include <median_filter.h>
#include <lqg_single.h>


// 2000rpm max speed, convert to rad/s
#define MOTOR_CONTROL_MAX_VELOCITY  ((float)2000.0*2.0*PI/60.0)

#define WHEEL_RADIUS_MM             (float(30.8/2.0))   
#define WHEEL_BRACE_MM              (float(83.0))        


#define SG_WINDOW_SIZE 11



class MotorControl
{

    public:
        int init(float k);

        void set_right_torque(float right_torque);
        void set_left_torque(float left_torque);

        void set_left_velocity(float left_velocity);
        void set_right_velocity(float right_velocity);

        void set(float forward, float turn);

        
        void halt();

        float get_right_position();
        float get_left_position();

        

    public:
        void callback();
        void set_torque_from_rotation(int32_t torque, uint32_t rotor_angle,  int motor_id);

    private:
        void timer_init();

    private:    
        AS5600T<10, 11, 12,  'C', 'C'> right_encoder;    
        AS5600T<12, 13, 12, 'B', 'B'> left_encoder;
        

        PWMRightThreePhase right_pwm;
        PWMLeftThreePhase  left_pwm;

        LQGSingle left_controller, right_controller;

        bool  left_cl_mode, right_cl_mode;

    public:
        uint32_t steps;
        
        float k;

        float right_torque, left_torque;
        float right_velocity, left_velocity;

        float right_torque_s, left_torque_s;
        float right_position, left_position;

    private:
        MedianFilter<float, MEDIAN_5> right_velocity_median_filter;
        MedianFilter<float, MEDIAN_5> left_velocity_median_filter;

    public:
        StateEstimator state;
};


#endif

