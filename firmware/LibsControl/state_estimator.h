#ifndef _STATE_ESTIMATOR_H_
#define _STATE_ESTIMATOR_H_

#include <median_filter.h>

// Hybrid state estimator — header-only, fully templated.
//
//   Positions  : EMA low-pass   (cutoff frequency parameter)
//   Velocities : EMA low-pass + median filter (cutoff frequency parameter + window size)
//
// Template parameter WINDOW_SIZE sets the SG sliding-window length.
// All memory is statically allocated — no heap, no STL.
//
// Usage:
//   static const float sg_coeffs[31] = { ... };   // from Python
//   StateEstimator<31> est;
//   est.init(sg_coeffs, 1.0f/2000.0f, 50.0f);
//   est.step(dist, theta);

class StateEstimator
{
    public:
        void init(float pos_cutoff_freq, float vel_cutoff_freq, float dt)
        {
            float rc;
            this->dt = dt;
            // EMA coefficient from cutoff frequency
            // alpha = dt / (RC + dt),  RC = 1 / (2 * pi * f_c)
            rc = 1.0f / (2.0f * 3.14159265358979f * pos_cutoff_freq);
            pos_alpha    = dt / (rc + dt);

            rc = 1.0f / (2.0f * 3.14159265358979f * vel_cutoff_freq);
            vel_alpha    = dt / (rc + dt);

            x_dist_est  = 0.0f;
            x_vel_est   = 0.0f;
            x_theta_est = 0.0f;
            x_omega_est = 0.0f;

            x_dist_prev  = 0.0f;
            x_theta_prev = 0.0f;

            velocity_median_filter.init();
            omega_median_filter.init(); 
        }

        // push new distance and angle measurements, updates all public outputs
        void step(float x_dist, float x_theta)
        {
            // EMA for positions
            x_dist_est  =  (1.0f - pos_alpha) * x_dist_est  + pos_alpha * x_dist;
            x_theta_est =  (1.0f - pos_alpha) * x_theta_est + pos_alpha * x_theta;
            
            // EMA + median filter for velocities
            x_vel_est   = (1.0f - vel_alpha) * x_vel_est   + vel_alpha * velocity_median_filter.step((x_dist  - x_dist_prev)/dt);
            x_omega_est = (1.0f - vel_alpha) * x_omega_est + vel_alpha * omega_median_filter.step((x_theta - x_theta_prev)/dt);   

            x_dist_prev  = x_dist;  
            x_theta_prev = x_theta;
        }

    public:
        // filtered outputs
        float x_dist_est;
        float x_vel_est;
        float x_theta_est;
        float x_omega_est;

    private:
        float pos_alpha;
        float vel_alpha;
        float dt;

        float x_dist_prev;
        float x_theta_prev;

        MedianFilter<float, MEDIAN_5> velocity_median_filter;
        MedianFilter<float, MEDIAN_5> omega_median_filter;

};

#endif
