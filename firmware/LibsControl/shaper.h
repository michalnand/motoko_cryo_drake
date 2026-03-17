#ifndef _SHAPER_H_
#define _SHAPER_H_

#include <cmath>

class ShaperVelAcc
{
    public:
        ShaperVelAcc();
        virtual ~ShaperVelAcc();

        void init(float vn_max, float vp_max,
                  float an_max, float ap_max,
                  float alpha,  float dt);

        // x_target : desired position
        // returns  : shaped (filtered) position
        float step(float x_target);

        void reset();

    public:
        float vn_max;
        float vp_max;
        float an_max;
        float ap_max;
        float alpha;
        float dt;

        float x;
        float v;
        float a;
        float x_fil;

    private:
        float _clip(float v, float min_v, float max_v);
        float _min(float a, float b);
        float _max(float a, float b);
        float _abs(float v);
};

#endif
