#ifndef LINE_FOLLOWER_H_
#define LINE_FOLLOWER_H_


#include "sensors.h"
#include "control_loop.h"
#include "QEstimator.h"

class LineFollower
{
    public:
        void init();
        void run();


    private:
        void line_follow();
        void line_follow_basic();
        void line_search(uint32_t line_lost_type, float curvature);
        //void obstacle_avoid();
        //void curtain_avoid();
        float estimate_turn_radius(float sensor_reading);


    private:
        ControlLoop control_loop;
        QEstimator<20> q_estimator;

        float speed_min, speed_max;
        float r_min,  r_max;
        float qr_max, qr_min;

        float position_prev;
};

#endif