#include "shaper_state.h"

#include <tmath.h>

void ShaperState::init(float acc_min, float acc_max, float tau, float dt)
{
    this->acc_min   = acc_min;
    this->acc_max   = acc_max;
    this->alpha     = dt / (tau + dt);
    this->dt        = dt;

    this->v_smooth  = 0.0;
}

float ShaperState::step(float x_req, float x_curr, float v_curr)
{
    // required position change
    float dx    = x_req - x_curr;
    float v_req = dx/this->dt;      
    
    float v_new    = v_curr + clip(v_req - v_curr, this->acc_min, this->acc_max);
    this->v_smooth = (1.0 - this->alpha)*this->v_smooth + this->alpha*v_new;

    float v_tmp    = min(abs(v_req), abs(this->v_smooth))*sgn(v_req);

    float x_new = x_curr + v_tmp*this->dt;

    return x_new;
}

   

