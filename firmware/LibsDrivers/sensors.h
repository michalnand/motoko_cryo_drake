#ifndef _SENSORS_H_
#define _SENSORS_H_

#include "ls_driver.h"

#include <array.h>


// sensors sampling frequency, 200Hz or 400Hz
#define SENSORS_TIMER_FREQ      ((uint32_t)250)



#define LINE_SENSOR_STEP                ((int32_t)128)

//sensitivity, higher value less sensitive
#define LINE_SENSOR_THRESHOLD           ((int32_t)2800)   

    

//brace from first to last sesor in mm
#define SENSORS_BRACE                  ((float)90.0)

//sensors distance from wheel axis in mm
#define SENSORS_DISTANCE                  ((float)93.5)
    

#define LINE_SENSOR_COUNT   ((unsigned int)10)

//state where line was lost
#define LINE_LOST_NONE      ((unsigned char)0)
#define LINE_LOST_CENTER    ((unsigned char)1)
#define LINE_LOST_RIGHT     ((unsigned char)2)
#define LINE_LOST_LEFT      ((unsigned char)3)


#define PROXIMITY_SENSOR_COUNT   ((unsigned int)4)



class Sensors
{
    public:
        int init();

        void callback();

    private:
        void line_sensor_process();
        void proximity_sensor_process();
        int integrate(int center_idx);

        void timer_init();

    private:
        // init i2c for line sensor, SDA = PA5, SCL = PA4
        TI2C<'A', 5, 4, 50> i2c;
        LSDriver ls_driver;

    public:
        uint32_t measurement_id;    

    // line sensor vars
    public:
        uint32_t sensor_status;
        uint32_t line_lost_type;
        uint32_t on_line_count;

        //this stores last valid line position <-1, 1>
        float left_position, right_position, center_position;

        float minimal_position, extremal_position;

        //line position into angle (radians)
        float left_angle, right_angle;

    // proximity sensor vars
    public:
        float front_left_proximity, left_proximity, right_proximity, front_right_proximity;


    public:
        Array<int, LINE_SENSOR_COUNT> line_reading_result;
        Array<int, LINE_SENSOR_COUNT> weights;

        Array<int, PROXIMITY_SENSOR_COUNT> proximity_reading_result;
};
         

#endif