#include "data_spawner.h"

int spawn_get_accel_gyro_mag(vector_t *va, vector_t *vg, vector_t *vm, state_container *state, int mpu9250_i2c_addr){
    // va - accel, vg - gyro, vm - magn
    vm->x=0.0f;
    vm->y=0.0f;
    vm->z=0.0f;

    va->x=esp_random()/UINT32_MAX;
    va->y=esp_random()/UINT32_MAX;
    va->z=1-esp_random()/UINT32_MAX;

    vg->x = esp_random()/(UINT32_MAX *10);
    vg->y = esp_random()/(UINT32_MAX * 10);
    vg->z = 1 + esp_random()/UINT32_MAX;
    return 0;
};