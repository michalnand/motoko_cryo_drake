#ifndef LINE_FOLLOWER_H_
#define LINE_FOLLOWER_H_


#include "sensors.h"
#include "control_loop.h"
#include "QEstimator.h"

class LineFollower
{
    public:
        void init(uint32_t mode);
        void run();


    private:
        void line_follow();
        void line_follow_basic();
        
        void line_search(uint32_t line_lost_type, float curvature);
        
        void obstacle_avoid();
        void curtain_avoid();

    private:
        ControlLoop control_loop;
        QEstimator<16> q_estimator;

        float speed_min, speed_max;
        
        float kp_min, kp_max;
        float kd_min, kd_max;

        float position_prev;

        bool turbine_enabled;

        uint32_t obstacle_idx;
        Array<bool, 4> obstacle_map;
};

#endif