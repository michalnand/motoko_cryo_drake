#ifndef _Q_ESTIAMTOR_H_
#define _Q_ESTIAMTOR_H_

#include <tmath.h>
#include <fifo_buffer.h>

template<unsigned int window_size>
class QEstimator
{
    public:
        QEstimator()
        {
            init(1.0, 0.0, 0.0);
        }

        void init(float k0, float k1, float k2)
        {
            this->distance.init(0);
            this->line_sensor.init(1);
            this->d_curr = 0.0f;

            this->q0 = 0.0f;
            this->q1 = 0.0f;
            this->q2 = 0.0f;

            this->k0 = k0;
            this->k1 = k1;
            this->k2 = k2;
        }
        
        void reset()
        {
            this->distance.init(0);
            this->line_sensor.init(1);
            this->radius.init(1000);    
        }

        void add(float distance, float line_sensor, float radius, float ds = 5.0)
        {
            if (distance >= (this->d_curr + ds))
            {
                // add into buffer
                this->distance.add(distance);
                this->line_sensor.add(line_sensor);
                this->radius.add(radius);

                this->d_curr = distance;
            }
        }

        float process() 
        {   
            this->q0 = this->_get_max_pos();
            this->q1 = this->_get_max_der();
            this->q2 = this->_get_max_der2();

            float q0 = clip(this->k0*this->q0, 0.0f, 1.0f);
            float q1 = clip(this->k1*this->q1, 0.0f, 1.0f);
            float q2 = clip(this->k2*this->q2, 0.0f, 1.0f);

            float q = 1.0 - max(max(q0, q1), q2);

            return q;
        }

        float _get_max_pos()
        {
            float result = 0.0;

            for (unsigned int i = 0; i < window_size; i++)
            {
                float tmp = abs(this->line_sensor[i]);
                if (tmp > result)
                {
                    result = tmp;
                }
            }

            return result;
        }



        float _get_max_der()
        {
            float result = 0.0;

            for (unsigned int i = 0; i < window_size-1; i++)
            {
                float dx = abs(this->distance[i]    - this->distance[i+1]);
                float dy = abs(this->line_sensor[i] - this->line_sensor[i+1]);

                float tmp = dy/(dx + 0.000001f);

                if (tmp > result)
                {
                    result = tmp;
                }
            }

            return result;
        }


        float _get_max_der2()
        {
            float result = 0.0;

            for (unsigned int i = 0; i < window_size-2; i++)
            {
                float dx = abs(this->distance[i]    - 2.0*this->distance[i+1] + this->distance[i+2]);
                float dy = abs(this->line_sensor[i] - 2.0*this->line_sensor[i+1] + this->line_sensor[i+2]);

                float tmp = dy/(dx + 0.000001f);   
                if (tmp > result)
                {
                    result = tmp;
                }
            }

            return result;
        }

        float get_curvature()
        {
            float curvature = 0.0;
            float dist_sum  = 0.0;  

            for (unsigned int i = 0; i < window_size; i++)
            {
                curvature+= this->line_sensor[i];
                dist_sum+= this->distance[i];
            }   

            float result = curvature/(abs(dist_sum) + 0.0001);

            return result;
        }


        float get_radius()
        {
            float r_sum = 0.0;

            for (unsigned int i = 0; i < radius.size(); i++)
            {
                r_sum+= this->radius[i];
            }   

            float result = r_sum/radius.size();

            return result;
        }

    
    public:
        float q0, q1, q2;

    private:
        float d_curr;
        float k0, k1, k2;
        
        FifoBuffer<float, window_size> distance;
        FifoBuffer<float, window_size> line_sensor;
        FifoBuffer<float, window_size/4> radius;
};


#endif