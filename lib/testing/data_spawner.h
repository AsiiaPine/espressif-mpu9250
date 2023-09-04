#include "freertos/FreeRTOS.h"
#include "esp_random.h"
#include "../mpu9250/ak8963.h"
#include "../mpu9250/mpu9250.h"


int spawn_get_accel_gyro_mag(vector_t *va, vector_t *vg, vector_t *vm, state_container *state, int mpu9250_i2c_addr);