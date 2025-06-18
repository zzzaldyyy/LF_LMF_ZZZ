#include <stdio.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>

#define TIME_STEP 64
#define THRESHOLD 100 
// Lebih dari Threshold = PUTIH
// Kurang dari Threshold = HITAM
#define MAX_SPEED 10.0

#define Kp 4.0
#define Ki 0.01
#define Kd 2.5

double previous_error = 0;
double integral = 0;

void check_sensor(WbDeviceTag motor_kiri, WbDeviceTag motor_kanan, WbDeviceTag* sensors){
  double val2 = wb_distance_sensor_get_value(sensors[2]);
  double val3 = wb_distance_sensor_get_value(sensors[3]);
  double val4 = wb_distance_sensor_get_value(sensors[4]);
  double val5 = wb_distance_sensor_get_value(sensors[5]);
  
  if (val3 > THRESHOLD && val4 < THRESHOLD){
    wb_motor_set_velocity(motor_kiri, -0.05 * MAX_SPEED);
    wb_motor_set_velocity(motor_kanan, 0.05 * MAX_SPEED);
    wb_robot_step(500);
  } else if (val3 < THRESHOLD && val4 > THRESHOLD){
    wb_motor_set_velocity(motor_kiri, 0.05 * MAX_SPEED);
    wb_motor_set_velocity(motor_kanan, -0.05 * MAX_SPEED);
    wb_robot_step(500);
  } else if (val4 < THRESHOLD && val5 > THRESHOLD){
    wb_motor_set_velocity(motor_kiri, 0.1 * MAX_SPEED);
    wb_motor_set_velocity(motor_kanan, -0.1 * MAX_SPEED);
    wb_robot_step(500);
  } else if (val3 < THRESHOLD && val2 > THRESHOLD){
    wb_motor_set_velocity(motor_kiri, -0.1 * MAX_SPEED);
    wb_motor_set_velocity(motor_kanan, 0.1 * MAX_SPEED);
    wb_robot_step(500);
  }
}

void belok_kiri(WbDeviceTag motor_kiri, WbDeviceTag motor_kanan, WbDeviceTag* sensors) {
  wb_motor_set_velocity(motor_kiri, 0.32 * MAX_SPEED);
  wb_motor_set_velocity(motor_kanan, 0.32 * MAX_SPEED);
  wb_robot_step(1000);
  wb_motor_set_velocity(motor_kiri, 0);
  wb_motor_set_velocity(motor_kanan, 0);
  wb_robot_step(2000);
  wb_motor_set_velocity(motor_kiri, -0.38 * MAX_SPEED);
  wb_motor_set_velocity(motor_kanan, 0.38 * MAX_SPEED);
  wb_robot_step(1000);
  wb_motor_set_velocity(motor_kiri, 0);
  wb_motor_set_velocity(motor_kanan, 0);
  wb_robot_step(2000);
  check_sensor(motor_kiri, motor_kanan, sensors);
}

void belok_kanan(WbDeviceTag motor_kiri, WbDeviceTag motor_kanan, WbDeviceTag* sensors){
  wb_motor_set_velocity(motor_kiri, 0.32 * MAX_SPEED);
  wb_motor_set_velocity(motor_kanan, 0.32 * MAX_SPEED);
  wb_robot_step(1000);
  wb_motor_set_velocity(motor_kiri, 0);
  wb_motor_set_velocity(motor_kanan, 0);
  wb_robot_step(2000);
  wb_motor_set_velocity(motor_kiri, 0.36 * MAX_SPEED);
  wb_motor_set_velocity(motor_kanan, -0.36 * MAX_SPEED);
  wb_robot_step(1000);
  wb_motor_set_velocity(motor_kiri, 0);
  wb_motor_set_velocity(motor_kanan, 0);
  wb_robot_step(2000);
  check_sensor(motor_kiri, motor_kanan, sensors);
}

void putar_balik(WbDeviceTag motor_kiri, WbDeviceTag motor_kanan, WbDeviceTag* sensors){
  wb_motor_set_velocity(motor_kiri, 0.32 * MAX_SPEED);
  wb_motor_set_velocity(motor_kanan, 0.32 * MAX_SPEED);
  wb_robot_step(1000);
  wb_motor_set_velocity(motor_kiri, 0);
  wb_motor_set_velocity(motor_kanan, 0);
  wb_robot_step(2000);
  wb_motor_set_velocity(motor_kiri, -0.365 * MAX_SPEED);
  wb_motor_set_velocity(motor_kanan, 0.365 * MAX_SPEED);
  wb_robot_step(2000);
  wb_motor_set_velocity(motor_kiri, 0);
  wb_motor_set_velocity(motor_kanan, 0);
  wb_robot_step(2000);
  check_sensor(motor_kiri, motor_kanan, sensors);
}

void baca_sensor(WbDeviceTag* sensors, int* sensor_values) {
  for (int i = 0; i < 8; i++) {
    double val = wb_distance_sensor_get_value(sensors[i]);
    sensor_values[i] = (val < THRESHOLD) ? 0 : 1;
  }
}

int main() {
  wb_robot_init();

  WbDeviceTag motor_kanan = wb_robot_get_device("motorkanan");
  WbDeviceTag motor_kiri = wb_robot_get_device("motorkiri");

  wb_motor_set_position(motor_kanan, INFINITY);
  wb_motor_set_position(motor_kiri, INFINITY);

  WbDeviceTag sensors[8];
  const char sensor_names[8][10] = {
    "sensor0", "sensor1", "sensor2", "sensor3",
    "sensor4", "sensor5", "sensor6", "sensor7"
  };

  for (int i = 0; i < 8; i++) {
    sensors[i] = wb_robot_get_device(sensor_names[i]);
    wb_distance_sensor_enable(sensors[i], TIME_STEP);
  }

  while (wb_robot_step(TIME_STEP) != -1) {
    int sensor_values[8];
    int total_on_line = 0;

    baca_sensor(sensors, sensor_values);

    for (int i = 0; i < 8; i++)
      total_on_line += sensor_values[i];

    int L = sensor_values[0] || sensor_values[1];
    int LF = sensor_values[2];
    int F = sensor_values[3] || sensor_values[4];
    int RF = sensor_values[5];
    int R = sensor_values[6] || sensor_values[7];

if ((L || LF) && F && (RF || R)) {
    wb_motor_set_velocity(motor_kiri, 0.32 * MAX_SPEED);
    wb_motor_set_velocity(motor_kanan, 0.32 * MAX_SPEED);
    wb_robot_step(1000);
    wb_motor_set_velocity(motor_kiri, 0);
    wb_motor_set_velocity(motor_kanan, 0);
    wb_robot_step(2000);
    
    baca_sensor(sensors, sensor_values);

    total_on_line = 0;
    for (int i = 0; i < 8; i++) total_on_line += sensor_values[i];
    printf("Sensor Saat ini: ");
    for (int i = 0; i < 8; i++) printf("%d ", sensor_values[i]);
    printf("\n");

    wb_motor_set_velocity(motor_kiri, 0);
    wb_motor_set_velocity(motor_kanan, 0);
    wb_robot_step(2000);

      if (total_on_line == 8) {
        // End Maze
        printf("End of maze confirmed\n");
        break;
      } else if (total_on_line == 0) {
        // T-Junction
        printf("Detected: T-Junction\n");
        wb_motor_set_velocity(motor_kiri, 0.38 * MAX_SPEED);
        wb_motor_set_velocity(motor_kanan, -0.38 * MAX_SPEED);
        wb_robot_step(1000);
        wb_motor_set_velocity(motor_kiri, 0);
        wb_motor_set_velocity(motor_kanan, 0);
        wb_robot_step(2000);
        check_sensor(motor_kiri, motor_kanan, sensors);
      } else {
        // Cross-Junction (semua sisi terbuka, tapi bukan end maze)
        printf("Detected: Cross-Junction\n");
        wb_motor_set_velocity(motor_kiri, 0.38 * MAX_SPEED);
        wb_motor_set_velocity(motor_kanan, -0.38 * MAX_SPEED);
        wb_robot_step(1000);
        wb_motor_set_velocity(motor_kiri, 0);
        wb_motor_set_velocity(motor_kanan, 0);
        wb_robot_step(2000);
        check_sensor(motor_kiri, motor_kanan, sensors);
      }
    } else if (!L && !F && !R) {
      // Dead End
      printf("Detected: Dead End\n");
      putar_balik(motor_kiri, motor_kanan, sensors);
    } else if (!L && F && R) {
      // Right or Straight
      printf("Detected: Right or Straight\n");
      belok_kanan(motor_kiri, motor_kanan, sensors);
    } else if (L && F && !R) {
      // Left or Straight
      printf("Detected: Left or Straight\n");
      // Straiht PID
    } else if (!L && !F && R) {
      // Right Only
      printf("Detected: Right Only\n");
      belok_kanan(motor_kiri, motor_kanan, sensors);
    } else if (L && !F && !R) {
      // Left Only
      printf("Detected: Left Only\n");
      belok_kiri(motor_kiri, motor_kanan, sensors);
    } else if (!L && F && !R) {
      // Straight with PID
      double error = (sensor_values[0]*-10 + sensor_values[1]*-5 + sensor_values[2]*-3 +
                      sensor_values[3]*-2 + sensor_values[4]*2 + sensor_values[5]*3 +
                      sensor_values[6]*5 + sensor_values[7]*10) /
                      (sensor_values[0] + sensor_values[1] + sensor_values[2] +
                       sensor_values[3] + sensor_values[4] + sensor_values[5] +
                       sensor_values[6] + sensor_values[7] + 1);
      
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
      
      wb_motor_set_velocity(motor_kiri, 0.5 * left_speed);
      wb_motor_set_velocity(motor_kanan, 0.5 * right_speed);
    }

    for (int i = 0; i < 8; i++) printf("%d ", sensor_values[i]);
    printf("\n");
  }

  wb_robot_cleanup();
  return 0;
}
