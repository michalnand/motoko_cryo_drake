#include "line_follower.h"


#define DT_MS       ((uint32_t)4)

void LineFollower::init(uint32_t mode)
{
  this->turbine_enabled = false;

  if (mode == 0)
  {
    this->speed_min = 0.5f;          
    this->speed_max = 1.0f;      

    this->kp_max    = 0.7f;   
    this->kp_min    = 1.3f;  

    this->kd_max    = 15.0f; 
    this->kd_min    = 15.0f;   
  } 
  else if (mode == 1)
  { 
    // fast run, no turbine
    this->speed_min = 0.5f;          
    this->speed_max = 1.5f;       

    this->kp_max    = 0.7f;   
    this->kp_min    = 1.3f;  

    this->kd_max    = 15.0f; 
    this->kd_min    = 15.0f; 
  }   
  else if (mode == 2)   
  { 
    // fastest run, without turbine
    this->speed_min = 0.5f;          
    this->speed_max = 2.0f;       

    this->kp_max    = 0.5f;    
    this->kp_min    = 1.3f;    

    this->kd_max    = 25.0f;   
    this->kd_min    = 25.0f;  
  } 

  else if (mode == 3)   
  { 
    // fast run, turbine
    this->speed_min = 0.7f;          
    this->speed_max = 2.5f;       

    this->kp_max    = 0.5f;      
    this->kp_min    = 1.5f;         

    this->kd_max    = 50.0f;    
    this->kd_min    = 50.0f;   

    this->turbine_enabled = true;
  }


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
          obstacle_avoid(); 
        }
        else
        {
          curtain_avoid();
        } 
        
        // start with lowest speed after obstacle avoiding
        q_estimator.reset();
        this->obstacle_idx = (this->obstacle_idx+1)%obstacle_map.size();
      }
      
      // obstacle warning, proximity alert, slow down to minimal speed
      if (obstacle == 1)
      {
        led.all_off();
        led.on(LED::RIGHT_RED); 
        led.on(LED::LEFT_RED);  

        q_estimator.reset();
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
      

      line_follow();  
      
      //control_loop.set_circle_motion(1.0, 0.0f);
      //timer.delay_ms(DT_MS);
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
    timer.delay_ms(DT_MS); 

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
    timer.delay_ms(DT_MS); 
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

    // always lost in middle the line
    // state = 2;
  

    
    while (true)
    {
        // left or right line searching
        if (state == 0 || state == 1)
        {
            //turn until line found, or distance trehold
            float start_distance      = control_loop.get_distance();
            float target_distance     = start_distance + turn_search_distance;

            while (control_loop.get_distance() < target_distance)
            { 
                control_loop.set_turn_motion(way, speed);
                timer.delay_ms(DT_MS);         

                if (sensors.line_lost_type == LINE_LOST_NONE)
                {
                  return; 
                }   
            }         

            while (control_loop.get_distance() > start_distance)
            { 
                control_loop.set_turn_motion(way, -speed);
                timer.delay_ms(DT_MS);    
                
                
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
                timer.delay_ms(DT_MS);  

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
                timer.delay_ms(DT_MS);  

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


void LineFollower::obstacle_avoid()
{
    float speed = speed_min;  
    float d_req = 0.5f;     
    
    //float s_min = -0.8f;
    //float s_max = 0.95f;   

    float s_min = -0.85f;
    float s_max = 0.95f;   


    float kp = 5.0f;    

    terminal << "obstacle detected "<< sensors.obstacle_detected << "\n";

    
    terminal << "moving back\n";
    // move back until minimal distance from obstalce reached 
    {
      float target_distance = control_loop.get_distance() - 0.01;  
      
      while (sensors.obstacle_distance < 0.05 || control_loop.get_distance() > target_distance) 
      {
        control_loop.set_turn_motion(0.0f, -speed_min);
        timer.delay_ms(DT_MS);       
      }   
    }

    terminal << "turning left, 90 degrees\n";
    //turn left, 90degrees 
    {
      float distance_target = control_loop.get_distance();
      float angle_target    = control_loop.get_angle() + 90.0*PI/180.0;

      while (abs(control_loop.get_angle() - angle_target) > 0.02*PI)
      { 
        control_loop.set_position(distance_target, angle_target);
        timer.delay_ms(DT_MS);   
      } 
    } 
    

    terminal << "avoiding obstacle\n";
    
    
    //turn around obstacle, circular motion to right
    {

      uint32_t state    = 0;   
      float angle_start = control_loop.get_angle();
      float angle_turn  = -90.0f*PI/180.0f;
      
      while (1)       
      {
        
        if (state == 0 && (control_loop.get_angle() - angle_start) < angle_turn)
        {
          state = 1;    
        }
        else if (state == 1 && sensors.line_lost_type == LINE_LOST_NONE)
        {
          break; 
        }  
        

        float diff = d_req - sensors.right_proximity;     
        
        float steering = kp*diff;    
        
        if (steering < s_min)
        {
          steering = s_min;
        }
        else if (steering > s_max)
        {
          steering = s_max;
        }

        
        control_loop.set_turn_motion(steering, speed);
        timer.delay_ms(DT_MS);      
      }   
    }
    

    //final turn left back to line, 90 degrees 
    {
      float distance_target = control_loop.get_distance();
      float angle_target    = control_loop.get_angle() + 90.0*PI/180.0;

      while (angle_target > control_loop.get_angle())
      {
        control_loop.set_position(distance_target, angle_target);
        timer.delay_ms(DT_MS);   
      }   
    }
}


// curtain avoid - slow line following for 25cm, ignore obstacle reading
void LineFollower::curtain_avoid()
{
  float curtain_distance  = 0.25f; 
  float target_distance   = curtain_distance + control_loop.get_distance();

  float position_prev = sensors.center_position;

  while (control_loop.get_distance() < target_distance)
  {
    float position = sensors.center_position;

    // steering with PD control
    float steering = position*kp_min + (position - position_prev)*kd_min;
    position_prev = position;
   
    control_loop.set_turn_motion(steering, speed_min);
    timer.delay_ms(DT_MS); 
  }
}