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

#include "io/serial.h"
#include "drivers/serial.h"

enum {
    GROUP_SYS = 0x00,
        SYS_HELLO           = 0x0000,
        SYS_LOOPTIME        = 0x0001,
        SYS_EMF_AVOIDANCE   = 0x0002,
        
    GROUP_SENSOR = 0x01,
        SEN_ALIGN_GYRO          = 0x0000,
        SEN_ALIGN_ACC           = 0x0001,
        SEN_ALIGN_MAG           = 0x0002,
        SEN_GYRO_LPF            = 0x0003,
        SEN_ACCZERO             = 0x0004,
        SEN_ACCGAIN             = 0x0005,
        SEN_MAGZERO             = 0x0006,
    
    GROUP_RC = 0x02,
        RC_MID                  = 0x0000,
        RC_MIN_CHECK            = 0x0001,
        RC_MAX_CHECK            = 0x0002,
        RC_RSSI_CHANNEL         = 0x0003,
        RC_RSSI_SCALE           = 0x0004,
        RC_RSSI_PPM_INVERT      = 0x0005,
        RC_RC_SMOOTHING         = 0x0006,
        RC_INPUT_FILTERING_MODE = 0x0007,

    GROUP_DRIVE = 0x03,
        DRV_MIN_THROTTLE        = 0x0001,
        DRV_MAX_THROTTLE        = 0x0002,
        DRV_MIN_COMMAND         = 0x0003,

    GROUP_ERROR = 0xFF,
        ERR_PARAM               = 0xFFFF,
} paramGroupAndId_e;

typedef union {
    uint8_t  uint8_value;
    int8_t   int8_value;
    uint16_t uint16_value;
    int16_t  int16_value;
    uint32_t uint32_value;
    float    float_value;
} packedParamValue_t;

typedef struct __attribute__((packed)) {
    uint16_t            param_count;   // allow to get all params by index one by one
    uint8_t             group_id;      // param group
    uint16_t            param_id;      // param index within group
    uint8_t             data_type;     // data type
    uint8_t             array_index;   // current param index for array parameters (profile, aux, servo, etc)
    uint8_t             array_count;   // total item count for array parameters (profile, aux, servo, etc)
    packedParamValue_t  value;         // raw packed value
    int32_t             value_min;     // min value (int)
    int32_t             value_max;     // max value (int)
} paramProtocolData_t;

bool mspGetParamByIndex(uint16_t tableIndex, uint8_t arrayIndex, paramProtocolData_t * data);
bool mspSetParamByIndex(uint16_t tableIndex, paramProtocolData_t * data);
bool mspGetParamByGroupAndId(uint8_t group_id, uint16_t param_id, uint8_t arrayIndex, paramProtocolData_t * data);
bool mspSetParamByGroupAndId(paramProtocolData_t * data);