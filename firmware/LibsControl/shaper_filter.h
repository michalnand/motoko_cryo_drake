#ifndef _SHAPER_FILTER_H_
#define _SHAPER_FILTER_H_

#include <cmath>

class ShaperFilter
{
    public:
        ShaperFilter();
        virtual ~ShaperFilter();

        void init(float k);

        // x_target : desired position
        // returns  : shaped (filtered) position
        float step(float x);

    private:
        float k;
        float xa;
        float xb;
       

};

#endif
