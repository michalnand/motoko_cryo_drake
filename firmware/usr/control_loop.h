#ifndef CONTROL_LOOP_H_
#define CONTROL_LOOP_H_

#include <libs_drivers.h>


#include <tmath.h>
#include <position_controller.h>
#include <shaper_state.h>






class ControlLoop
{
    public:
        int init();

        void callback();

        void set_position(float distance_target, float angle_target, bool fast_mode);
        void set_circle_motion(float radius_target, float velocity_target, bool fast_mode);

    private:
        void planner_set_position(float x_req, float a_req, float acc_min, float acc_max, float acc_w_max);   
        void planner_set_circle_motion(float r_req, float v_req, float acc_min, float acc_max);


        void timer_init();

        

    private:    
        PositionController position_controller;
    
    private:
        float   distance_target, angle_target;
        float   radius_target, velocity_target;
        bool    fast_mode, position_mode;


    public:
        uint32_t steps;
};

#endif
