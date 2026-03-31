#include "line_follower.h"

void LineFollower::init()
{
    this->speed_min = 400.0;     

    //this->speed_max = 400.0;
    this->speed_max = 600.0;
    //this->speed_max = 800.0;
    //this->speed_max = 1000.0;
    //this->speed_max = 1200.0;   
        
    this->r_min   = 80.0;
    this->r_max   = 10000.0;
    this->qr_max  = 10.0;         
    this->qr_min  = 2.0;  


    // init main position control loop
    control_loop.init();

    
}

void LineFollower::run()
{
    q_estimator.reset();

    while (1)   
    {
        // lost line search
        while (sensors.line_lost_type != LINE_LOST_NONE)   
        {
          float curvature = q_estimator.get_curvature();

          line_search(sensors.line_lost_type, curvature);

          q_estimator.reset(); 
        }   


        // main line following
        line_follow();
    }
}


void LineFollower::line_follow()
{
    // main line following  
    
    //float position = 0.4*sensors.center_position;    
    float position = 0.4*sensors.right_position;    
    //float position = sensors.center_position;    

    float radius  = estimate_turn_radius(position, 1.0f/r_max);
    radius = -sgn(position)*clip(radius, r_min*0.1f, r_max);    

    float d = control_loop.get_distance(); 
    q_estimator.add(d, sensors.center_position, radius);

    // estimate line straightness
    float q = q_estimator.process();

    // obstacle warning, slow down TODO
    /*
    if (obstacle != 0)
    {
        q = 0.0;  
    }
    */

    //if quality is high (close to 1), increase radius - allows faster speed
    float kr = q*this->qr_max + (1.0f - q)*this->qr_min;  
    radius = kr*radius; 
    
    //if quality is high (close to 1), use higher speed
    float speed = (1.0f - q)*this->speed_min + q*this->speed_max;  

    control_loop.set_circle_motion(radius, speed);
    timer.delay_ms(4); 
}


void LineFollower::line_search(uint32_t line_lost_type, float curvature)
{
    float turn_search_distance      = 60.0f;
    float forward_search_distance   = 70.0f;

    float r_search  = 90.0f; 
    float r_max     = 10000.0f;

    float speed     = 500.0f;

    int state       = 2;
    int way         = 1; 

    /*
    if (line_lost_type == LINE_LOST_LEFT)
    {
      way   = 1;
      state = 0;
    }
    else if (line_lost_type == LINE_LOST_RIGHT)
    {
      way   = -1;
      state = 0;  
    }   
    else
    {   
      if (curvature > 0)
      {
        way = 1;
      }
      else
      {
        way = -1;
      }
      
      state = 2;
    } 
    */

    if (line_lost_type == LINE_LOST_LEFT)
    {
      way   = 1;
      //state = 3;
      state = 2;
    }
    else if (line_lost_type == LINE_LOST_RIGHT)
    {
      way   = -1;
      //state = 3;  
      state = 2;
    }   
    else
    {   
      if (curvature > 0)
      {
        way = 1;
      }
      else
      {
        way = -1;
      }
      
      state = 2;
    } 


  
    while (true)
    {
        // left or right line searching
        if (state == 0 || state == 1)
        {
            // stop motors
            float d_target  = control_loop.get_distance();
            float a_target  = control_loop.get_angle();

            while (control_loop.get_velocity() > 10.0f)
            { 
                control_loop.set_position(d_target, a_target);
                timer.delay_ms(4);      

                led.led_blink(LED::LEFT_RED);
            }   

            //turn until line found, or distance trehold
            float start_distance      = control_loop.get_distance();
            float target_distance     = start_distance + turn_search_distance;

            while (control_loop.get_distance() < target_distance)
            { 
                control_loop.set_circle_motion(way*r_search, 0.25f*speed);
                timer.delay_ms(4);         

                if (sensors.line_lost_type == LINE_LOST_NONE)
                {
                  return; 
                } 

                led.led_blink(LED::LEFT_RED);
            }       

            while (control_loop.get_distance() > start_distance)
            { 
                control_loop.set_circle_motion(way*r_search, -0.25f*speed);
                timer.delay_ms(4);    
                
                
                if (sensors.line_lost_type == LINE_LOST_NONE)
                {
                  return;
                }
                

                led.led_blink(LED::LEFT_RED);
            }     

            way*= -1;

            state++;
        }

        // line lost in midle, center
        // go forward, until line found or maximal distance reached
        else if (state == 2)  
        {          
            float start_distance      = control_loop.get_distance();
            float target_distance     = start_distance + forward_search_distance;

            while (control_loop.get_distance() < target_distance)
            {   
                control_loop.set_circle_motion(r_max, speed);
                timer.delay_ms(4);  

                if (sensors.line_lost_type == LINE_LOST_NONE)
                {
                  return; 
                }

                led.led_blink(LED::LEFT_RED);
            }       
            
            state = 0;  
        }
        // wave motion to find line
        else
        {
          float search_distance = 0.5f*turn_search_distance;

          
          while (1)
          {
            float start_distance      = control_loop.get_distance();
            float target_distance     = start_distance + search_distance;

            while (control_loop.get_distance() < target_distance)
            {   
                control_loop.set_circle_motion(way*r_search, 0.5f*speed_min);
                timer.delay_ms(4);  

                if (sensors.line_lost_type == LINE_LOST_NONE)
                {
                  return; 
                }   
 
                led.led_blink(LED::LEFT_RED);
            }  

            way = -way;

            search_distance = turn_search_distance;
        
          }
        }
    }
}


float LineFollower::estimate_turn_radius(float sensor_reading, float eps)
{
  float x = SENSORS_DISTANCE;
  float y = 0.5f*SENSORS_BRACE*abs(sensor_reading);

  float r = (y*y + x*x)/(2.0f*y + eps);

  return r;
}
