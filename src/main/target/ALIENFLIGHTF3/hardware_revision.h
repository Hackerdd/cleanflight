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

typedef enum awf3HardwareRevision_t {
    AFF3_UNKNOWN = 0,
    AFF3_REV_1, // MPU6050 (I2C)
    AFF3_REV_2  // MPU6500 / MPU9250 (SPI)
} awf3HardwareRevision_e;

extern uint8_t hardwareRevision;

void updateHardwareRevision(void);
void detectHardwareRevision(void);

struct extiConfig_s;
const struct extiConfig_s *selectMPUIntExtiConfigByHardwareRevision(void);
