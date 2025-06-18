#include <stdio.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>

#define TIME_STEP 64
#define TRESHOLD 100
#define MAX_SPEED 10.0

#define Kp 10.0
#define Ki 0.01
#define Kd 5

double previous_error = 0;
double integral = 0;

int main(int argc, char **argv) {
  wb_robot_init();

  WbDeviceTag motor_kanan = wb_robot_get_device("motorkanan");
  WbDeviceTag motor_kiri = wb_robot_get_device("motorkiri");
  
  wb_motor_set_position(motor_kanan, INFINITY);
  wb_motor_set_position(motor_kiri, INFINITY);
  
  WbDeviceTag sensor0 = wb_robot_get_device("sensor0");
  WbDeviceTag sensor1 = wb_robot_get_device("sensor1");
  WbDeviceTag sensor2 = wb_robot_get_device("sensor2");
  WbDeviceTag sensor3 = wb_robot_get_device("sensor3");
  WbDeviceTag sensor4 = wb_robot_get_device("sensor4");
  WbDeviceTag sensor5 = wb_robot_get_device("sensor5");
  WbDeviceTag sensor6 = wb_robot_get_device("sensor6");
  WbDeviceTag sensor7 = wb_robot_get_device("sensor7");
  
  wb_distance_sensor_enable(sensor0, TIME_STEP);
  wb_distance_sensor_enable(sensor1, TIME_STEP);
  wb_distance_sensor_enable(sensor2, TIME_STEP);
  wb_distance_sensor_enable(sensor3, TIME_STEP);
  wb_distance_sensor_enable(sensor4, TIME_STEP);
  wb_distance_sensor_enable(sensor5, TIME_STEP);
  wb_distance_sensor_enable(sensor6, TIME_STEP);
  wb_distance_sensor_enable(sensor7, TIME_STEP);
  
  while (wb_robot_step(TIME_STEP) != -1) {
    double val0 = wb_distance_sensor_get_value(sensor0);
    double val1 = wb_distance_sensor_get_value(sensor1);
    double val2 = wb_distance_sensor_get_value(sensor2);
    double val3 = wb_distance_sensor_get_value(sensor3);
    double val4 = wb_distance_sensor_get_value(sensor4);
    double val5 = wb_distance_sensor_get_value(sensor5);
    double val6 = wb_distance_sensor_get_value(sensor6);
    double val7 = wb_distance_sensor_get_value(sensor7);
    
    int v0 = (val0 < TRESHOLD) ? 0 : 1;
    int v1 = (val1 < TRESHOLD) ? 0 : 1;
    int v2 = (val2 < TRESHOLD) ? 0 : 1;
    int v3 = (val3 < TRESHOLD) ? 0 : 1;
    int v4 = (val4 < TRESHOLD) ? 0 : 1;
    int v5 = (val5 < TRESHOLD) ? 0 : 1;
    int v6 = (val6 < TRESHOLD) ? 0 : 1;
    int v7 = (val7 < TRESHOLD) ? 0 : 1;
    
    int sensor_values[8] = {v0, v1, v2, v3, v4, v5, v6, v7};

    int sum_side = v0 + v1 + v2 + v5 + v6 + v7;
    int background = (sum_side >= 4) ? 1 : 0;
    
    if (background == 1) {
      v0 = 1 - v0;
      v1 = 1 - v1;
      v2 = 1 - v2;
      v3 = 1 - v3;
      v4 = 1 - v4;
      v5 = 1 - v5;
      v6 = 1 - v6;
      v7 = 1 - v7;
    }
    
    double error = (v0*-10 + v1*-5 + v2*-3 + v3*-1 + v4*1 + v5*3 + v6*5 + v7*10) / 
                   (v0 + v1 + v2 + v3 + v4 + v5 + v6 + v7 + 1);
    
    double derivative = error - previous_error;
    integral += error;
    double adjustment = Kp * error + Ki * integral + Kd * derivative;
    previous_error = error;
    
    double left_speed = MAX_SPEED + adjustment;
    double right_speed = MAX_SPEED - adjustment;
    
    if (left_speed > MAX_SPEED) left_speed = MAX_SPEED;
    if (left_speed < -MAX_SPEED) left_speed = -MAX_SPEED;
    if (right_speed > MAX_SPEED) right_speed = MAX_SPEED;
    if (right_speed < -MAX_SPEED) right_speed = -MAX_SPEED;
    
    wb_motor_set_velocity(motor_kiri, left_speed);
    wb_motor_set_velocity(motor_kanan, right_speed);
    
    for(int i = 0; i < 8; i++) {
      printf("%d ", sensor_values[i]);
    }
    
    printf("\nBG: %s | Sensor: %d %d %d %d %d %d %d %d\n", 
      (background == 1) ? "Hitam" : "Putih",
      v0, v1, v2, v3, v4, v5, v6, v7);

    printf("Error: %f, Adjustment: %f\n", error, adjustment);
    printf("Left Speed: %f, Right Speed: %f\n", left_speed, right_speed);
  };
  wb_robot_cleanup();
  return 0;
}
 