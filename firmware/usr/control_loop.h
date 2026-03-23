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

        void set_xr(float x_distance_req, float x_theta_req);

    private:
        void timer_init();

    private:    
        ShaperState shaper_distance, shaper_angle;   
        PositionController position_controller;
    
       
    private:
        float x_distance_req;
        float x_theta_req;

    public:
        uint32_t steps;
};

#endif
