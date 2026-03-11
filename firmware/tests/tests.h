#ifndef _TESTS_H_
#define _TESTS_H_

// raw read adc channels
// print raw readings and how many measurements per second
//void adc_test();

// init magnetic encoders
// print readed raw reading (0 .. 4096) and accumulated angle
//void encoder_test();


//void gyro_test();

// run both BLDC motors forward, no feedback loop, just simple three phase sine waves
void motor_pwm_test();
//void motor_driver_test();


void line_sensor_hw_test();

void sensors_test();
void encoder_test();

void motor_foc_test();


void motor_identification();

void motor_controll_test();

void robot_idenditification();

#endif