#include <libs_drivers.h>
#include <tmath.h>

void motor_identification()
{
    Gpio<'B', 0, GPIO_MODE_OUT> led;
   
    led = 0;  

    //stop motor
    motor_control.halt();
    timer.delay_ms(200); 

    //1, estimate motor constant k, on different input values
    uint32_t n_steps = 500;

    float u_values[9] = {0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
    
    float k_mean     = 0.0;
    float x_var_mean = 0.0;

    for (unsigned int j = 0; j < 9; j++)
    {
        //run motor with desired input and wait for steady state
        float u_in = u_values[j];

        motor_control.set_right_torque(u_in); 
        timer.delay_ms(1000);       

        float a      = 0.95;    
        float x_mean = 0.0;
        float x_var  = 0.0; 

        for (unsigned int i = 0; i < n_steps; i++)
        {
            float x = motor_control.get_right_velocity();
            
            x_mean = a*x_mean + (1.0 - a)*x; 
            
            if (i > n_steps/2)
            {
                x_var  = a*x_var  + (1.0 - a)*(x - x_mean)*(x - x_mean);
            }       

            timer.delay_ms(2); 
        }   


        float k = x_mean/u_in;
        k_mean+= x_mean/u_in;    
        x_var_mean+= x_var; 

        int32_t rpm  = (x_mean*60.0)/(2.0*PI);

        terminal << "u_in   = " << u_in << "\n";
        terminal << "k      = " << k << "\n";
        terminal << "rpm    = " << rpm << "\n";
        terminal << "x_mean = " << x_mean << "\n";
        terminal << "x_var  = " << x_var << "\n";
        terminal << "\n\n";
    }   

    //print summary results
    k_mean     = k_mean/10.0;
    x_var_mean = x_var_mean/10.0;

    terminal << "k          = " << k_mean << "\n";
    terminal << "x_var_mean = " << x_var_mean << "\n";
    
    terminal << "\n\n";


    //2, estimate time constant by oscilating motor

    motor_control.halt();
    timer.delay_ms(200); 

    float u_in = 0.25;
    uint32_t periods = 0; 

    n_steps = 3000;

    for (unsigned int i = 0; i < n_steps; i++)
    {
        motor_control.set_right_torque(u_in);
        float x = motor_control.get_right_velocity(); //*60.0/(2.0*PI);

        if (u_in > 0.0) 
        {
            if (x > 0.632*k_mean*u_in)
            {
                u_in = -u_in;
                periods++;
            }
        }
        else
        {
            if (x < 0.632*k_mean*u_in)
            {
                u_in = -u_in;
                periods++;  
            }
        }

        timer.delay_ms(1);
    }
    
    motor_control.set_right_torque(0);
    timer.delay_ms(200);     
    
    float t_period = (2*n_steps/periods)/PI;
    terminal << "periods = " << periods << "\n";
    terminal << "tau     = " << t_period << "[ms]\n";
    terminal << "\n\n";
    
    led = 1; 

    while(1)
    {
        led = 0;  
        timer.delay_ms(100);
        led = 1;     
        timer.delay_ms(900);
    }
}
