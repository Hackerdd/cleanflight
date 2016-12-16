/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "common/axis.h"
#include "drivers/exti.h"
#include "drivers/sensor.h"

#ifndef MPU_I2C_INSTANCE
#define MPU_I2C_INSTANCE I2C_DEVICE
#endif

#define GYRO_LPF_256HZ      0
#define GYRO_LPF_188HZ      1
#define GYRO_LPF_98HZ       2
#define GYRO_LPF_42HZ       3
#define GYRO_LPF_20HZ       4
#define GYRO_LPF_10HZ       5
#define GYRO_LPF_5HZ        6
#define GYRO_LPF_NONE       7

typedef struct gyroDev_s {
    sensorGyroInitFuncPtr init;                             // initialize function
    sensorGyroReadFuncPtr read;                             // read 3 axis data function
    sensorReadFuncPtr temperature;                          // read temperature if available
    sensorGyroInterruptStatusFuncPtr intStatus;
    extiCallbackRec_t exti;
    float scale;                                            // scalefactor
    volatile bool dataReady;
    uint16_t lpf;
    int16_t gyroADCRaw[XYZ_AXIS_COUNT];
    sensor_align_e gyroAlign;
} gyroDev_t;

typedef struct accDev_s {
    sensorAccInitFuncPtr init;                              // initialize function
    sensorReadFuncPtr read;                                 // read 3 axis data function
    uint16_t acc_1G;
    char revisionCode;                                      // a revision code for the sensor, if known
    sensor_align_e accAlign;
} accDev_t;
