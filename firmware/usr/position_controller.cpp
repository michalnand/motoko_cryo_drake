#include "position_controller.h"

#include <libs_drivers.h>

int PositionController::init()
{
    // init controller      
    controller.init((float*)mpc_phi, (float*)mpc_omega, (float*)mpc_sigma, 1.0f, 1.0f);    
    
    return 0;
}

void PositionController::set_xr_constant(float x_distance_req, float x_theta_req)
{
    // required state, fill constant trajectory for MPC   
    controller.set_constant_xr(0, x_distance_req);    
    controller.set_constant_xr(2, x_theta_req);       
}


void PositionController::set_xr_trajectory(uint32_t step, float x_distance_req, float x_theta_req)
{
    controller.set_xr(step, 0, x_distance_req);
    controller.set_xr(step, 2, x_theta_req);
}


void PositionController::callback()
{
    // get current state (already filtered in fast 2kHz motor control loop)
    controller.x[0] = motor_control.state.x_dist_est;
    controller.x[1] = motor_control.state.x_vel_est;
    controller.x[2] = motor_control.state.x_theta_est;
    controller.x[3] = motor_control.state.x_omega_est; 


    // controller step 
    controller.step();          
        
    // control outputs convert to robot control inputs
    float u_forward = controller.u[0];
    float u_turn    = controller.u[1];   
    
    /*
    // TODO constrains
    // constrains, turning have priority over forward
    u_turn = clip(u_turn, -1.0f, 1.0f);
            
    // clamp u forward
    float max_u_forward = min( 1.0f - u_turn,  1.0f + u_turn);
    float min_u_forward = max(-1.0f - u_turn, -1.0f + u_turn);
    
    u_forward = clip(u_forward, min_u_forward, max_u_forward);
    */

    float right_u =  u_forward + u_turn;         
    float left_u  =  u_forward - u_turn;
    
    motor_control.set_right_torque(right_u);
    motor_control.set_left_torque(left_u);
}
