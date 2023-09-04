/*****************************************************************************
 *                                                                           *
 *  Copyright 2018 Simon M. Werner                                           *
 *                                                                           *
 *  Licensed under the Apache License, Version 2.0 (the "License");          *
 *  you may not use this file except in compliance with the License.         *
 *  You may obtain a copy of the License at                                  *
 *                                                                           *
 *      http://www.apache.org/licenses/LICENSE-2.0                           *
 *                                                                           *
 *  Unless required by applicable law or agreed to in writing, software      *
 *  distributed under the License is distributed on an "AS IS" BASIS,        *
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. *
 *  See the License for the specific language governing permissions and      *
 *  limitations under the License.                                           *
 *                                                                           *
 *****************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_task_wdt.h"

#include "driver/i2c.h"

#include "../lib/ahrs/MadgwickAHRS.h"
#include "../lib/mpu9250/mpu9250.h"
#include "../lib/mpu9250/calibrate.h"
#include "../lib/mpu9250/common.h"
#include "../lib/testing/data_spawner.h"

static const char *TAG = "main";

#define IMU_1_I2C_MASTER_SCL_IO (int)22 /*!< gpio number for I2C master clock */
#define IMU_1_I2C_MASTER_SDA_IO 21      /*!< gpio number for I2C master data  */

#define IMU_2_I2C_MASTER_SCL_IO 19 /*!< gpio number for I2C master clock */
#define IMU_2_I2C_MASTER_SDA_IO 18 /*!< gpio number for I2C master data  */
// #define CONFIG_CALIBRATION_MODE 1

typedef struct
{
  Madgwick_state *madgwick_state;
  const int imu_number;
  const int i2c_port_num;
  const int i2c_addr;
  const int imu_i2c_sda;
  const int imu_i2c_scl;
  state_container *mpu_state;
} IMU_CONF;

// Problem: axis x: imu 0 can't determine rotation > 180, it flips the direction back.
// Problem: axis y: imu 0 has a "spring" effect, returns to the initial position, but can determine the direction of rotation, if the angle <120*, then flips the direction, either speeds up and go through to initial position..
// Problem: axis z: imu 0 has a "spring" effect, returns to the initial position, but can determine the direction of rotation.

// Problem: axis x: imu 1 can't determine rotation > 180, it flips the direction back, can remember position.
// Problem: axis y: imu 1 can't determine rotation > 180, it flips the direction back, can remember position.
// Problem: axis z: imu 1 has a "spring" effect, the direction always "up". returns to the initial position, but can determine the direction of rotation, likes to speedup and return to init position, otherwise almost doesn't move

calibration_t cal_1 = {
    .mag_offset = {.x = -38.114082, .y = -89.604561, .z = 89.840302},
    .mag_scale = {.x = 1.00, .y = 1.00, .z = 1.00},
    .accel_offset = {.x = 0.002456, .y = 0.005308, .z = -0.048982},
    .accel_scale_lo = {.x = 0.999159, .y = 1.005793, .z = 0.983132},
    .accel_scale_hi = {.x = -1.001248, .y = -0.998091, .z = -1.035969},
    .gyro_bias_offset = {.x = -3.995865, .y = 0.341634, .z = 0.174921}
};

calibration_t cal_2 = {
    .mag_offset = {.x = -38.114082, .y = -89.604561, .z = 89.840302},
    .mag_scale = {.x = 1.00, .y = 1.00, .z = 1.00},
    .accel_offset = {.x = 0.008634, .y = -0.012420, .z = -0.121289},
    .accel_scale_lo = {.x = 1.000932, .y = 1.004860, .z = 0.962099},
    .accel_scale_hi = {.x = 1.001018, .y = 0.999670, .z = 1.070351},
    .gyro_bias_offset = {.x = 0.924213, .y = -0.589062, .z = 0.735103}};

/**
 * Transformation:
 *  - Rotate around Z axis 180 degrees
 *  - Rotate around X axis -90 degrees
 * @param  {object} s {x,y,z} sensor
 * @return {object}   {x,y,z} transformed
 */

static void transform_accel_gyro(vector_t *v)
{
  float x = v->x;
  float y = v->y;
  float z = v->z;

  v->x = -x;
  v->y = -z;
  v->z = -y;
}

/**
 * Transformation: to get magnetometer aligned
 * @param  {object} s {x,y,z} sensor
 * @return {object}   {x,y,z} transformed
 */
static void transform_mag(vector_t *v)
{
  float x = v->x;
  float y = v->y;
  float z = v->z;

  v->x = -y;
  v->y = z;
  v->z = -x;
}
void run_imu(int i2c_port_num, int i2c_addr, int imu_i2c_sda, int imu_i2c_scl, state_container *mpu_state, int imu_number, Madgwick_state *madgwick_state)
{
  int64_t prev_time = esp_timer_get_time();
  int64_t time = 0;

  int j =0;
  uint64_t i = 0;
  while (true)
  {
    i2c_mpu9250_init(&cal_1, i2c_port_num, imu_i2c_sda, imu_i2c_scl, mpu_state, i2c_addr);
    // madgwick_state = MadgwickAHRSinit(madgwick_state);

    while (true)
    {
      vector_t va, vg, vm;

      // Get the Accelerometer, Gyroscope and Magnetometer values.
      if (get_accel_gyro_mag(&va, &vg, &vm, mpu_state, i2c_addr) != ESP_OK)
      {
        i2c_reset_rx_fifo(i2c_port_num);
        i2c_reset_tx_fifo(i2c_port_num);
        break;
      }
      // Transform these values to the orientation of our device.
      transform_accel_gyro(&va);
      transform_accel_gyro(&vg);
      transform_mag(&vm);
      if (j==1000){
        time = esp_timer_get_time();
        madgwick_state->sampleFreq = 1000000000.0/(time-prev_time);
        ESP_LOGI(TAG, "Freq updated %f",madgwick_state->sampleFreq);
        j = 0;
      }

      // Apply the AHRS algorithm
      madgwick_state = MadgwickAHRSupdate(DEG2RAD(vg.x), DEG2RAD(vg.y), DEG2RAD(vg.z),
                                          va.x, va.y, va.z,
                                          vm.x, vm.y, vm.z, madgwick_state);

      // Print the data out every 10 items
      if (i++ % 10 == 0)
      {
        float temp;
        if (get_temperature_celsius(&temp, i2c_port_num, i2c_addr) != ESP_OK)
        {
          break;
        }

        // ESP_LOGI(TAG, "{\"imu_number\": %d, \"q0\": %2.3f, \"q1\": %2.3f, \"q2\": %2.3f, \"q3\": %2.3f}", imu_number, madgwick_state->q->q0, madgwick_state->q->q1, madgwick_state->q->q2, madgwick_state->q->q3);
        float yaw, pitch, roll;
        MadgwickGetEulerAnglesDegrees(&yaw, &pitch, &roll, madgwick_state);
        ESP_LOGI(TAG, "{\"imu_number\": %d, \"yaw\": %2.3f, \"pitch\": %2.3f, \"roll\": %2.3f, \"Temp\": %2.3f}", imu_number, yaw, pitch, roll, temp);

        // Make the WDT happy
        esp_task_wdt_reset();
      }
      j++;
      pause();
    }
    ESP_LOGI(TAG, "i2c failed, restarting");
    i2c_reset_rx_fifo(i2c_port_num);
    i2c_reset_tx_fifo(i2c_port_num);
    i2c_driver_delete(i2c_port_num);
  }
}

static void imu_task(void *imu_conf)
{
  IMU_CONF *conf = (IMU_CONF *)imu_conf;
#ifdef CONFIG_CALIBRATION_MODE
  calibrate_gyro(conf->i2c_port_num, conf->imu_i2c_sda, conf->imu_i2c_scl, conf->mpu_state, conf->i2c_addr);
  calibrate_accel(conf->i2c_port_num, conf->imu_i2c_sda, conf->imu_i2c_scl, conf->mpu_state, conf->i2c_addr);
  calibrate_mag(conf->i2c_port_num, conf->imu_i2c_sda, conf->imu_i2c_scl, conf->mpu_state, conf->i2c_addr);
#else
  run_imu(conf->i2c_port_num, conf->i2c_addr, conf->imu_i2c_sda, conf->imu_i2c_scl, conf->mpu_state, conf->imu_number, conf->madgwick_state);
#endif

  // Exit
  vTaskDelay(100 / portTICK_RATE_MS);
  i2c_driver_delete(conf->i2c_port_num);

  vTaskDelete(NULL);
}

Quaternion q1 = {.q0 = 1, .q1 = 0, .q2 = 0, .q3 = 0};
Quaternion q2 = {.q0 = 1, .q1 = 0, .q2 = 0, .q3 = 0};

Madgwick_state madgwick_state_1 = {.beta = 0.866, .sampleFreq = 50, .q = &q1};
Madgwick_state madgwick_state_2 = {.beta = 0.866, .sampleFreq = 50, .q = &q2};

state_container state_1_magn = {.i2c_num = I2C_NUM_0, .mag_offset = &cal_1.mag_offset, .mag_scale = &cal_1.mag_scale};
state_container state_2_magn = {.i2c_num = I2C_NUM_1, .mag_offset = &cal_2.mag_offset, .mag_scale = &cal_2.mag_scale};

IMU_CONF imu_1_conf = {.imu_number = 0, .i2c_port_num = I2C_NUM_0, .i2c_addr = 0x68, .imu_i2c_sda = IMU_1_I2C_MASTER_SDA_IO, .imu_i2c_scl = IMU_1_I2C_MASTER_SCL_IO, .mpu_state = &state_1_magn, .madgwick_state = &madgwick_state_1};
IMU_CONF imu_2_conf = {.imu_number = 1, .i2c_port_num = I2C_NUM_1, .i2c_addr = 0x69, .imu_i2c_sda = IMU_2_I2C_MASTER_SDA_IO, .imu_i2c_scl = IMU_2_I2C_MASTER_SCL_IO, .mpu_state = &state_2_magn, .madgwick_state = &madgwick_state_2};

void app_main(void)
{

  // start i2c task
  xTaskCreatePinnedToCore(imu_task, "imu_1_task", 16384, &imu_1_conf, 1, NULL, 0);
  xTaskCreatePinnedToCore(imu_task, "imu_2_task", 16384, &imu_2_conf, 1, NULL, 1);
}