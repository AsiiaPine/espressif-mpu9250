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

#ifndef __CALIBRATE_H
#define __CALIBRATE_H
#include "ak8963.h"

void calibrate_gyro(int i2c_port, int i2c_master_sda, int i2c_master_scl, state_container* state, int i2c_addr);
void calibrate_accel(int i2c_port, int i2c_master_sda, int i2c_master_scl, state_container* mag_state, int i2c_addr);
void calibrate_mag(int i2c_port, int i2c_master_sda, int i2c_master_scl, state_container* state, int i2c_addr);

#endif