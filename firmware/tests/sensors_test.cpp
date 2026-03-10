#include <libs_drivers.h>



void sensors_test()
{
  Gpio<'B', 0, GPIO_MODE_OUT> led_0;

  while (1)
  {
    led_0 = 0;
    uint32_t m_prev = sensors.measurement_id;
    timer.delay_ms(100);
    uint32_t m_curr = sensors.measurement_id;
    led_0 = 1;

    terminal << "measurements per second: " << (m_curr - m_prev) * 10 << "\n";
    terminal << "sensor_status " << sensors.sensor_status << "\n";
    terminal << "line_lost_type " << sensors.line_lost_type << "\n";
    terminal << "on_line_count " << sensors.on_line_count << "\n";
    terminal << "left_position " << sensors.left_position << "\n";
    terminal << "right_position " << sensors.right_position << "\n";
    terminal << "center_position " << sensors.center_position << "\n";
    terminal << "minimal_position " << sensors.minimal_position << "\n";
    terminal << "extremal_position " << sensors.extremal_position << "\n";
    terminal << "left_angle " << sensors.left_angle << "\n";
    terminal << "right_angle " << sensors.right_angle << "\n";
    terminal << "-----------------------------\n\n\n";

    for (int i = 0; i < (int)sensors.line_reading_result.size(); i++)
    {
      terminal << sensors.line_reading_result[i] << " ";
    }
    terminal << "\n";
    terminal << "-----------------------------\n\n\n";


    terminal << "front_left_proximity " << sensors.front_left_proximity << "\n";
    terminal << "left_proximity " << sensors.left_proximity << "\n";
    terminal << "right_proximity " << sensors.right_proximity << "\n";
    terminal << "front_right_proximity " << sensors.front_right_proximity << "\n";
    terminal << "-----------------------------\n\n\n";
    
    timer.delay_ms(500);


  }
    
}
