#include <libs_drivers.h>


// debug data print
void print_data(uint16_t *result_buffer)
{
  for (uint8_t i = 0; i < LS_DATA_SIZE; i++)
  {
    uint16_t tmp = result_buffer[i];
    terminal << tmp << " ";
  }

  terminal << "\n\n";
}


void line_sensor_hw_test()
{
    Gpio<'C', 4, GPIO_MODE_OUT> led_1;
    Gpio<'A', 2, GPIO_MODE_OUT> led_5;

    led_1 = 1;
    led_5 = 1;

    // init i2c for line sensor, SDA = PA5, SCL = PA4
    TI2C<'A', 5, 4, 150> ls_i2c;
    ls_i2c.init();  

    terminal << "i2c init done\n";

    // init line sensor driver
    LSDriver ls_driver;
    int resp = ls_driver.init(ls_i2c);


    terminal << "line sensor init done with " << resp << "\n";

    if (resp == 0)
    {
        led_5 = 0;
    }

     
    uint16_t sensor_reading[LS_DATA_SIZE];

    while (1)
    {
        led_1 = 0;
        uint8_t result = ls_driver.read_who_am_i();
        terminal << "who am i reg : " << (int)result << "\n";
    
        // perform 20 measurements, and estimate sensor reading speed
        uint32_t time_start = timer.get_time();
        for (unsigned int i = 0; i < 20; i++)
        {
            ls_driver.read_data(sensor_reading, LS_DIF_FIL_REG);
        }

        uint32_t time_stop = timer.get_time();

        uint32_t rps = ((uint32_t)20*(uint32_t)1000)/(time_stop - time_start);

        terminal << "readings per second " << rps << "\n";
    
    
        terminal << "readed data\n";

        ls_driver.read_data(sensor_reading, LS_RAW_OFF_REG);
        print_data(sensor_reading);

        ls_driver.read_data(sensor_reading, LS_RAW_ON_REG);
        print_data(sensor_reading);

        ls_driver.read_data(sensor_reading, LS_FIL_OFF_REG);
        print_data(sensor_reading);

        ls_driver.read_data(sensor_reading, LS_FIL_ON_REG);
        print_data(sensor_reading);

        ls_driver.read_data(sensor_reading, LS_DIF_RAW_REG);
        print_data(sensor_reading);

        ls_driver.read_data(sensor_reading, LS_DIF_FIL_REG);
        print_data(sensor_reading);

        ls_driver.read_data(sensor_reading, LS_TEST_DAT_REG);
        print_data(sensor_reading);

        terminal << "\n\n\n\n";

        led_1 = 1;


        timer.delay_ms(800);
    }
}
