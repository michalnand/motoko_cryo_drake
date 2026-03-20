#ifndef CONTROL_LOOP_H_
#define CONTROL_LOOP_H_

#include <libs_drivers.h>


#include <tmath.h>
#include <shaper.h>

#include <mpc.h>
#include <mpc_config.h>

//#include <lqr.h>
//#include <lqr_config.h>

class ControlLoop
{
    public:
        int init(Sensors &sensors, MotorControl &motor_control);

        void callback();

        void set_xr(float x_distance_req, float x_theta_req);

    private:
        void timer_init();

    private:    
        Sensors *sensors;
        MotorControl *motor_control;    
    
    private:
        ShaperVelAcc shaper_distance;
        ShaperVelAcc shaper_angle;
        
        
        //LQR<LQR_SYSTEM_ORDER, LQR_SYSTEM_INPUTS> lqr_controller;
        //LQRI<LQR_SYSTEM_ORDER, LQR_SYSTEM_INPUTS> lqr_controller;
        MPC<MPC_SYSTEM_ORDER, MPC_SYSTEM_INPUTS, MPC_PREDCTION_HORIZON> mpc_controller;

    private:
        float x_distance_req;
        float x_theta_req;

    public:
        uint32_t steps;
};

#endif
