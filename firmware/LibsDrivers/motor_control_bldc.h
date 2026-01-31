#ifndef _MOTOR_CONTROL_BLDC_H_
#define _MOTOR_CONTROL_BLDC_H_

#include <as5600_t.h>
#include <lqg_single.h>


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
        
        //float get_left_position_smooth();
        //float get_left_velocity_smooth();

    public:
        float get_right_position();
        float get_right_velocity();
        float get_right_velocity_hat();
        
        //float get_right_position_smooth();
        //float get_right_velocity_smooth();

    public:
        void callback();
        void set_torque_from_rotation(int32_t torque, uint32_t rotor_angle, bool brake, int motor_id);

    private:
        void timer_init();

    private: 
        AS5600T<11, 10, 5, 'C', 'C'> left_encoder;
        AS5600T<5, 12,  5,  'B', 'C'> right_encoder;

            
        PWMLeftThreePhase  left_pwm;
        PWMRightThreePhase right_pwm;

        LQGSingle left_controller, right_controller;

    public:
        uint32_t steps;

    private:
        float left_position, right_position;
        float left_position_prev, right_position_prev;

        //bool left_ol_mode, right_ol_mode;
        float left_torque, right_torque;
        float left_req_velocity, right_req_velocity;

        bool  left_cl_mode, right_cl_mode;
};


#endif

