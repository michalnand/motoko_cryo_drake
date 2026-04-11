#include "line_follower.h"

void LineFollower::init(uint32_t mode)
{
  this->turbine_enabled = false;

  if (mode == 0)
  {
    // baseline, no turbine
    this->speed_min = 0.5f;          
    this->speed_max = 1.0f;     

    this->kp_max    = 2.0f;   
    this->kp_min    = 4.0f; 

    this->kd_max    = 4.0f;
    this->kd_min    = 4.0f;  
  }
  else if (mode == 1)
  { 
    // fast run, no turbine
    this->speed_min = 0.5f;          
    this->speed_max = 1.2f;        

    this->kp_max    = 2.0f;     
    this->kp_min    = 4.0f; 

    this->kd_max    = 4.0f;
    this->kd_min    = 4.0f;
  }

  else if (mode == 2)   
  { 
    // fast run, turbine
    this->speed_min = 0.6f;          
    this->speed_max = 1.5f;        

    this->kp_max    = 1.0f;        
    this->kp_min    = 3.0f;         

    this->kd_max    = 4.0f;
    this->kd_min    = 4.0f;

    this->turbine_enabled = true;
  }

    /*  
    // fast run, turbine turbo
    this->speed_min = 0.6f;          
    this->speed_max = 2.8f;          

    this->kp_max    = 1.0f;     
    this->kp_min    = 2.0f;   

    this->kd_max    = 4.0f;
    this->kd_min    = 4.0f;
    */

    this->position_prev = 0;




    this->obstacle_idx = 0;
    obstacle_map.set(true);

    
    obstacle_map[0] = true;
    obstacle_map[1] = false;
    obstacle_map[2] = false;
    obstacle_map[3] = true;


    // init main position control loop
    control_loop.init();
}

void LineFollower::run()
{
    if (this->turbine_enabled)
    {
        turbine_on();
    }

      
    q_estimator.reset();

   
    while (1)   
    {

      // obstacle avoiding
      int obstacle = sensors.obstacle_detected;

      if (obstacle == 2)
      { 
        led.all_off();
        led.on(LED::RIGHT_RED); 
        led.on(LED::LEFT_RED);  

        if (obstacle_map[this->obstacle_idx] == true)
        {
          //TODO
          //obstacle_avoid(); 
        }
        else
        {
          //TODO
          //curtain_avoid();
        } 

        q_estimator.reset();

        this->obstacle_idx = (this->obstacle_idx+1)%obstacle_map.size();
      }


      // lost line search
      while (sensors.line_lost_type != LINE_LOST_NONE)   
      {
        led.all_off();
        led.on(LED::RIGHT_GREEN); 
        led.on(LED::LEFT_GREEN);

        float curvature = q_estimator.get_curvature();

        line_search(sensors.line_lost_type, curvature);

        q_estimator.reset(); 
      }   

        
      // main line following
      led.all_off();
      led.on(LED::RIGHT_BLUE);
      led.on(LED::LEFT_BLUE);

      //control_loop.set_circle_motion(1.0, 0.0f);
      //timer.delay_ms(4);

      line_follow();
    }
}


void LineFollower::line_follow()
{
    //main line following with quality estimation
    float position = sensors.center_position;

    // line quality estimation
    float d = control_loop.get_distance(); 
    q_estimator.add(d, sensors.center_position, 0.01f);

    // estimate line straightness
    float q = q_estimator.process();

    // lifted state vector z, koopman lifted state, with nonlinear features
    Vector<float, 8> z;

    z[0] = q;
    z[1] = 1.0f - q;
    z[2] = position;
    z[3] = position - position_prev;
    z[4] = q*position;  
    z[5] = q*(position - position_prev);
    z[6] = (1.0f - q)*position;
    z[7] = (1.0f - q)*(position - position_prev);

    position_prev = position;   

    // apply koopman operator

    // speed estimating is simple, proportional to line quality, with min and max limits
    float speed    = z[0]*speed_max + z[1]*speed_min;

    // steering is gain scheduled PD control, with gains depending on line quality
    float steering = z[4]*kp_max + z[6]*kp_min + z[5]*kd_max + z[7]*kd_min;

    //terminal << position << " " << q << " " << speed << " " << steering << "\n";  

    // apply control to path planner and MPC, 
    // internally converted to turn radius and speed, motion on circle    
    control_loop.set_turn_motion(steering, speed);
    timer.delay_ms(4); 

}

void LineFollower::line_follow_basic()
{
    //float position = 0.4*sensors.center_position;    
    //float position = 0.4*sensors.right_position;    
    float position = sensors.center_position;   
    
    
    float speed = this->speed_min;       

    float turn  = 1.5f*position;  

  
    terminal << position << " " << turn << "\n";

    control_loop.set_turn_motion(turn, speed);
    timer.delay_ms(4); 
}


void LineFollower::line_search(uint32_t line_lost_type, float curvature)
{
    float turn_search_distance      = 0.15f;  
    float forward_search_distance   = 0.07f;

    float r_search  = 0.04f; 
    float r_max     = 1.0f;

    float speed     = 0.4f;
  
    int state       = 2;
    int way         = 1; 

    terminal << line_lost_type << " "  << curvature << " " << "\n";


    if (line_lost_type == LINE_LOST_LEFT)
    {
      way   = 1;
      //state = 3;
      state = 0;  
    }
    else if (line_lost_type == LINE_LOST_RIGHT)
    {
      way   = -1;
      //state = 3;  
      state = 0;
    }   
    else
    {   
      // middle lost line, state = 2, search forward
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
            /*
            // stop motors
            float d_target  = control_loop.get_distance();
            float a_target  = control_loop.get_angle();

            while (control_loop.get_velocity() > 0.01f)
            { 
                control_loop.set_position(d_target, a_target);
                timer.delay_ms(4);      
            }   
            */  

            //turn until line found, or distance trehold
            float start_distance      = control_loop.get_distance();
            float target_distance     = start_distance + turn_search_distance;

            while (control_loop.get_distance() < target_distance)
            { 
                control_loop.set_turn_motion(way, speed);
                timer.delay_ms(4);         

                if (sensors.line_lost_type == LINE_LOST_NONE)
                {
                  return; 
                }   
            }         

            while (control_loop.get_distance() > start_distance)
            { 
                control_loop.set_turn_motion(way, -speed);
                timer.delay_ms(4);    
                
                
                if (sensors.line_lost_type == LINE_LOST_NONE)
                {
                  return;
                }                
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
            }  

            way = -way;

            search_distance = turn_search_distance;
        
          }
        }
    }
}
