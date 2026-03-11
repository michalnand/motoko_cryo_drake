#ifndef _MOTOR_CONTROL_BLDC_H_
#define _MOTOR_CONTROL_BLDC_H_

#include <as5600_t.h>
#include <lqg_single.h>

#define MOTOR_CONTROL_MAX_VELOCITY   ((float)2000.0*2.0*PI/60.0)

class MotorControl
{

    public:
        void init();

        void set_left_torque(float left_torque);
        void set_right_torque(float right_torque);

        void set_left_velocity(float left_velocity);
        void set_right_velocity(float right_velocity);

        void halt();

    public:
        float get_left_position();
        float get_left_velocity();
        float get_left_velocity_hat();

    public:
        float get_right_position();
        float get_right_velocity();
        float get_right_velocity_hat();
        

    public:
        void callback();
        void set_torque_from_rotation(int32_t torque, uint32_t rotor_angle, bool brake, int motor_id);

    private:
        void timer_init();

    private: 
        AS5600T<12, 13, 5, 'B', 'B'> left_encoder;
        AS5600T<10, 11, 5,  'C', 'C'> right_encoder;
        

        PWMLeftThreePhase  left_pwm;
        PWMRightThreePhase right_pwm;

        LQGSingle left_controller, right_controller;

    public:
        uint32_t steps;

    private:
        float left_position, right_position;
        float left_position_prev, right_position_prev;

        float left_torque, right_torque;
        float left_req_velocity, right_req_velocity;

        bool  left_cl_mode, right_cl_mode;
};


#endif

