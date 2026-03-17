#include "shaper.h"

ShaperVelAcc::ShaperVelAcc()
{
}

ShaperVelAcc::~ShaperVelAcc()
{
}

void ShaperVelAcc::init(float vn_max, float vp_max,
                         float an_max, float ap_max,
                         float alpha,  float dt)
{
    this->vn_max = vn_max;
    this->vp_max = vp_max;
    this->an_max = an_max;
    this->ap_max = ap_max;
    this->alpha  = alpha;
    this->dt     = dt;

    this->x     = 0.0f;
    this->v     = 0.0f;
    this->a     = 0.0f;
    this->x_fil = 0.0f;
}

float ShaperVelAcc::step(float x_target)
{
    float dx = x_target - x;

    float v_brake_pos;
    float v_brake_neg;

    if (dx > 0.0f)
    {
        // target is ahead: limit positive vel so we can brake with |an_max|
        v_brake_pos = sqrtf(2.0f * _abs(an_max) * dx);
        v_brake_neg = vn_max;
    }
    else if (dx < 0.0f)
    {
        // target is behind: limit negative vel so we can brake with ap_max
        v_brake_pos = vp_max;
        v_brake_neg = -sqrtf(2.0f * ap_max * _abs(dx));
    }
    else
    {
        v_brake_pos = 0.0f;
        v_brake_neg = 0.0f;
    }

    // effective velocity bounds
    float v_upper = _min(vp_max, v_brake_pos);
    float v_lower = _max(vn_max, v_brake_neg);

    // requested velocity
    float v_req = dx / dt;
    v_req = _clip(v_req, v_lower, v_upper);

    // requested acceleration
    float a_req = (v_req - v) / dt;
    a = _clip(a_req, an_max, ap_max);

    // integrate acceleration
    v += a * dt;
    v = _clip(v, v_lower, v_upper);

    // integrate velocity
    x += v * dt;

    // EMA output filter
    x_fil = (1.0f - alpha) * x_fil + alpha * x;

    return x_fil;
}

void ShaperVelAcc::reset()
{
    x     = 0.0f;
    v     = 0.0f;
    a     = 0.0f;
    x_fil = 0.0f;
}

float ShaperVelAcc::_clip(float v, float min_v, float max_v)
{
    if (v < min_v)
    {
        v = min_v;
    }
    else if (v > max_v)
    {
        v = max_v;
    }
    return v;
}

float ShaperVelAcc::_min(float a, float b)
{
    return (a < b) ? a : b;
}

float ShaperVelAcc::_max(float a, float b)
{
    return (a > b) ? a : b;
}

float ShaperVelAcc::_abs(float v)
{
    return (v < 0.0f) ? -v : v;
}
