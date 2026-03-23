#ifndef _POSITION_CONTROLLER_H_
#define _POSITION_CONTROLLER_H_

#include <tmath.h>
#include <mpc.h>
#include <mpc_config.h>


class PositionController
{
    public:
        int init();
        void set_xr_constant(float x_distance_req, float x_theta_req);
        void set_xr_trajectory(uint32_t step, float x_distance_req, float x_theta_req);
        void callback();  
        
    private:
        MPC<MPC_SYSTEM_ORDER, MPC_SYSTEM_INPUTS, MPC_PREDCTION_HORIZON> controller;

};

#endif
