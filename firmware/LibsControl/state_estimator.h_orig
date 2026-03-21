#ifndef _STATE_ESTIMATOR_H_
#define _STATE_ESTIMATOR_H_


// Hybrid state estimator — header-only, fully templated.
//
//   Positions  : EMA low-pass   (cutoff frequency parameter)
//   Velocities : Savitzky-Golay 1st-derivative (precomputed coefficients)
//
// Template parameter WINDOW_SIZE sets the SG sliding-window length.
// All memory is statically allocated — no heap, no STL.
//
// Usage:
//   static const float sg_coeffs[31] = { ... };   // from Python
//   StateEstimator<31> est;
//   est.init(sg_coeffs, 1.0f/2000.0f, 50.0f);
//   est.step(dist, theta);

template<int WINDOW_SIZE>
class StateEstimator
{
    public:
        StateEstimator()
        {
            x_dist_est  = 0.0f;
            x_vel_est   = 0.0f;
            x_theta_est = 0.0f;
            x_omega_est = 0.0f;

            alpha     = 1.0f;
            buf_count = 0;
        }

        // sg_coeffs  : precomputed Savitzky-Golay 1st-derivative coefficients
        //              (length = WINDOW_SIZE, causal / right-aligned)
        // dt         : sampling period [s]
        // cutoff_freq: EMA cutoff frequency [Hz] for position smoothing
        void init(const float *sg_coeffs, float dt, float cutoff_freq)
        {
            buf_count = 0;

            for (int i = 0; i < WINDOW_SIZE; i++)
            {
                this->sg_coeffs[i] = sg_coeffs[i];
                buf_dist[i]  = 0.0f;
                buf_theta[i] = 0.0f;
            }

            // EMA coefficient from cutoff frequency
            // alpha = dt / (RC + dt),  RC = 1 / (2 * pi * f_c)
            float rc = 1.0f / (2.0f * 3.14159265358979f * cutoff_freq);
            alpha    = dt / (rc + dt);

            x_dist_est  = 0.0f;
            x_vel_est   = 0.0f;
            x_theta_est = 0.0f;
            x_omega_est = 0.0f;
        }

        // push new distance and angle measurements, updates all public outputs
        void step(float x_dist, float x_theta)
        {
            // EMA for positions
            x_dist_est  += alpha * (x_dist  - x_dist_est);
            x_theta_est += alpha * (x_theta - x_theta_est);

            // shift ring buffers left, append EMA-smoothed values
            for (int i = 0; i < WINDOW_SIZE - 1; i++)
            {
                buf_dist[i]  = buf_dist[i + 1];
                buf_theta[i] = buf_theta[i + 1];
            }
            buf_dist[WINDOW_SIZE - 1]  = x_dist_est;
            buf_theta[WINDOW_SIZE - 1] = x_theta_est;

            if (buf_count < WINDOW_SIZE)
            {
                buf_count++;
            }

            if (buf_count < WINDOW_SIZE)
            {
                // not enough samples for SG — zero velocity
                x_vel_est   = 0.0f;
                x_omega_est = 0.0f;
            }
            else
            {
                // velocity estimation via SG 1st-derivative dot product
                float vel   = 0.0f;
                float omega = 0.0f;

                for (int i = 0; i < WINDOW_SIZE; i++)
                {
                    vel   += sg_coeffs[i] * buf_dist[i];
                    omega += sg_coeffs[i] * buf_theta[i];
                }

                x_vel_est   = vel;
                x_omega_est = omega;
            }
        }

    public:
        // filtered outputs
        float x_dist_est;
        float x_vel_est;
        float x_theta_est;
        float x_omega_est;

    private:
        float alpha;

        float sg_coeffs[WINDOW_SIZE];
        float buf_dist[WINDOW_SIZE];
        float buf_theta[WINDOW_SIZE];

        int   buf_count;
};

#endif
