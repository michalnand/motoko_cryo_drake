#ifndef _SHAPER_FILTER_H_
#define _SHAPER_FILTER_H_

class ShaperState
{
    public:
        void init(float acc_min, float acc_max, float tau, float dt);
        float step(float x_req, float x_curr, float v_curr);

    private:

        float acc_min;
        float acc_max;
        float alpha;
        float dt;

        float v_smooth;
};

#endif
