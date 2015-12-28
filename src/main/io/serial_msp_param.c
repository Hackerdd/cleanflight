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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "build_config.h"
#include "debug.h"

#include "platform.h"

#include "common/axis.h"
#include "common/color.h"
#include "common/maths.h"

#include "drivers/system.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"

#include "drivers/serial.h"
#include "drivers/bus_i2c.h"
#include "drivers/gpio.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"

#include "rx/rx.h"
#include "rx/msp.h"

#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/gps.h"
#include "io/gimbal.h"
#include "io/serial.h"
#include "io/ledstrip.h"
#include "io/flashfs.h"

#include "telemetry/telemetry.h"

#include "sensors/boardalignment.h"
#include "sensors/sensors.h"
#include "sensors/battery.h"
#include "sensors/sonar.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/navigation_rewrite.h"

#include "mw.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

#include "serial_msp_param.h"

typedef enum {
    /* 5 bits, at most 32 data types */
    VAR_UINT8       = 0,
    VAR_INT8        = 1,
    VAR_UINT16      = 2,
    VAR_INT16       = 3,
    VAR_UINT32      = 4,
    VAR_FLOAT       = 5,
    VAR_INT16_XYZ   = 6,
    VAR_MMIX        = 7,
} paramTypeAndFlags_e;

typedef struct {
    const uint8_t  group_id;
    const uint16_t param_id;
    const uint16_t type;
    void *         ptr;
    const int16_t  min;
    const int16_t  max;
} paramValue_t;

static uint32_t mspHelloValue = 0xAA0055FF;

static const paramValue_t valueTable[] = {
    { GROUP_SYS,    SYS_HELLO,                  VAR_UINT32,     &mspHelloValue, 0, 0 },

    { GROUP_SYS,    SYS_LOOPTIME,               VAR_UINT16,     &masterConfig.looptime, 0, 9000 },
    { GROUP_SYS,    SYS_EMF_AVOIDANCE,          VAR_UINT8,      &masterConfig.emf_avoidance, 0, 1 },
    
    { GROUP_SENSOR, SEN_ALIGN_GYRO,             VAR_UINT8,      &masterConfig.sensorAlignmentConfig.gyro_align, 0, 8 },
    { GROUP_SENSOR, SEN_ALIGN_ACC,              VAR_UINT8,      &masterConfig.sensorAlignmentConfig.acc_align, 0, 8 },
    { GROUP_SENSOR, SEN_ALIGN_MAG,              VAR_UINT8,      &masterConfig.sensorAlignmentConfig.mag_align, 0, 8 },
    { GROUP_SENSOR, SEN_GYRO_LPF,               VAR_UINT16,     &masterConfig.gyro_lpf, 0, 256 },
    { GROUP_SENSOR, SEN_ACCZERO,                VAR_INT16,      &masterConfig.accZero.raw[0], -32767, 32767 },
    { GROUP_SENSOR, SEN_ACCGAIN,                VAR_INT16_XYZ,  &masterConfig.accGain.raw[0], 1, 8192 },
    { GROUP_SENSOR, SEN_MAGZERO,                VAR_INT16_XYZ,  &masterConfig.magZero.raw[0], -32767, 32767 },
    
    { GROUP_RC,     RC_MID,                     VAR_UINT16,     &masterConfig.rxConfig.midrc, 1200, 1700 },
    { GROUP_RC,     RC_MIN_CHECK,               VAR_UINT16,     &masterConfig.rxConfig.mincheck, PWM_RANGE_ZERO, PWM_RANGE_MAX },
    { GROUP_RC,     RC_MAX_CHECK,               VAR_UINT16,     &masterConfig.rxConfig.maxcheck, PWM_RANGE_ZERO, PWM_RANGE_MAX },
    { GROUP_RC,     RC_RSSI_CHANNEL,            VAR_INT8,       &masterConfig.rxConfig.rssi_channel, 0, MAX_SUPPORTED_RC_CHANNEL_COUNT },
    { GROUP_RC,     RC_RSSI_SCALE,              VAR_UINT8,      &masterConfig.rxConfig.rssi_scale, RSSI_SCALE_MIN, RSSI_SCALE_MAX },
    { GROUP_RC,     RC_RSSI_PPM_INVERT,         VAR_INT8,       &masterConfig.rxConfig.rssi_ppm_invert, 0, 1 },
    { GROUP_RC,     RC_RC_SMOOTHING,            VAR_INT8,       &masterConfig.rxConfig.rcSmoothing, 0, 1 },
    { GROUP_RC,     RC_INPUT_FILTERING_MODE,    VAR_INT8,       &masterConfig.inputFilteringMode, 0, 1 },

    { GROUP_DRIVE,  DRV_MIN_THROTTLE,           VAR_UINT16,     &masterConfig.escAndServoConfig.minthrottle, PWM_RANGE_ZERO, PWM_RANGE_MAX },
    { GROUP_DRIVE,  DRV_MAX_THROTTLE,           VAR_UINT16,     &masterConfig.escAndServoConfig.maxthrottle, PWM_RANGE_ZERO, PWM_RANGE_MAX },
    { GROUP_DRIVE,  DRV_MIN_COMMAND,            VAR_UINT16,     &masterConfig.escAndServoConfig.mincommand, PWM_RANGE_ZERO, PWM_RANGE_MAX },

    { GROUP_DRIVE,  MMIX_MOTOR_0,               VAR_MMIX,       &masterConfig.customMotorMixer[0], -1, 1},
    { GROUP_DRIVE,  MMIX_MOTOR_1,               VAR_MMIX,       &masterConfig.customMotorMixer[1], -1, 1},
    { GROUP_DRIVE,  MMIX_MOTOR_2,               VAR_MMIX,       &masterConfig.customMotorMixer[2], -1, 1},
    { GROUP_DRIVE,  MMIX_MOTOR_3,               VAR_MMIX,       &masterConfig.customMotorMixer[3], -1, 1},
    { GROUP_DRIVE,  MMIX_MOTOR_4,               VAR_MMIX,       &masterConfig.customMotorMixer[4], -1, 1},
    { GROUP_DRIVE,  MMIX_MOTOR_5,               VAR_MMIX,       &masterConfig.customMotorMixer[5], -1, 1},
    { GROUP_DRIVE,  MMIX_MOTOR_6,               VAR_MMIX,       &masterConfig.customMotorMixer[6], -1, 1},
    { GROUP_DRIVE,  MMIX_MOTOR_7,               VAR_MMIX,       &masterConfig.customMotorMixer[7], -1, 1},
    { GROUP_DRIVE,  MMIX_MOTOR_8,               VAR_MMIX,       &masterConfig.customMotorMixer[8], -1, 1},
    { GROUP_DRIVE,  MMIX_MOTOR_9,               VAR_MMIX,       &masterConfig.customMotorMixer[9], -1, 1},
    { GROUP_DRIVE,  MMIX_MOTOR_10,              VAR_MMIX,       &masterConfig.customMotorMixer[10], -1, 1},
    { GROUP_DRIVE,  MMIX_MOTOR_11,              VAR_MMIX,       &masterConfig.customMotorMixer[11], -1, 1},
};

#define PARAM_COUNT (sizeof(valueTable) / sizeof(valueTable[0]))

static int lookupParamTableIndex(uint8_t group_id, uint16_t param_id)
{
    uint16_t index;
    for (index = 0; index < PARAM_COUNT; index++) {
        if (valueTable[index].group_id == group_id && valueTable[index].param_id == param_id)
            return index;
    }
    return -1;
}

static bool paramRead_MMIX(paramProtocolData_t * data, motorMixer_t * ptr)
{
    data->value[0].float_value = ptr->throttle;
    data->value[1].float_value = ptr->roll;
    data->value[2].float_value = ptr->pitch;
    data->value[3].float_value = ptr->yaw;
    return true;
}

static bool paramWrite_MMIX(motorMixer_t * ptr, paramProtocolData_t * data)
{
    ptr->throttle = data->value[0].float_value;
    ptr->roll = data->value[1].float_value;
    ptr->pitch = data->value[2].float_value;
    ptr->yaw = data->value[3].float_value;
    return true;
}

bool mspGetParamByIndex(uint16_t tableIndex, paramProtocolData_t * data)
{
    memset(data, 0, sizeof(paramProtocolData_t));

    data->group_id    = GROUP_ERROR;
    data->param_id    = ERR_PARAM;
    
    // Sanity check 
    if (tableIndex >= PARAM_COUNT)
        return false;

    data->group_id    = valueTable[tableIndex].group_id;
    data->param_id    = valueTable[tableIndex].param_id;
    data->data_type   = valueTable[tableIndex].type;
    data->value_min   = valueTable[tableIndex].min;
    data->value_max   = valueTable[tableIndex].max;

    void * ptr = valueTable[tableIndex].ptr;

    switch (valueTable[tableIndex].type) {
        case VAR_UINT8:
            data->value[0].uint8_value = *(uint8_t *)ptr;
            break;
        case VAR_INT8:
            data->value[0].int8_value = *(int8_t *)ptr;
            break;
        case VAR_UINT16:
            data->value[0].uint16_value = *(uint16_t *)ptr;
            break;
        case VAR_INT16:
            data->value[0].int16_value = *(int16_t *)ptr;
            break;
        case VAR_UINT32:
            data->value[0].uint32_value = *(uint32_t *)ptr;
            break;
        case VAR_FLOAT:
            data->value[0].float_value = *(float *)ptr;
            break;
        case VAR_INT16_XYZ:
            data->value[0].int16_value = ((int16_t *)ptr)[0];
            data->value[1].int16_value = ((int16_t *)ptr)[1];
            data->value[2].int16_value = ((int16_t *)ptr)[2];
            break;
        case VAR_MMIX:
            return paramRead_MMIX(data, (motorMixer_t *)ptr);
    }
    
    return true;
}

bool mspSetParamByIndex(uint16_t tableIndex, paramProtocolData_t * data)
{
    // Sanity check 
    if (tableIndex >= PARAM_COUNT)
        return false;

    if (data->group_id != valueTable[tableIndex].group_id || data->param_id != valueTable[tableIndex].param_id)
        return false;

    if (data->data_type != valueTable[tableIndex].type)
        return false;

    int32_t minValue = valueTable[tableIndex].min;
    int32_t maxValue = valueTable[tableIndex].max;

    void * ptr = valueTable[tableIndex].ptr;

    switch (data->data_type) {
        case VAR_UINT8:
            if (data->value[0].uint8_value >= minValue && data->value[0].uint8_value <= maxValue) {
                *(uint8_t *)ptr = data->value[0].uint8_value;
                return true;
            }
            break;
        case VAR_INT8:
            if (data->value[0].int8_value >= minValue && data->value[0].int8_value <= maxValue) {
                *(int8_t *)ptr = data->value[0].int8_value;
                return true;
            }
            break;
        case VAR_UINT16:
            if (data->value[0].uint16_value >= minValue && data->value[0].uint16_value <= maxValue) {
                *(uint16_t *)ptr = data->value[0].uint16_value;
                return true;
            }
            break;
        case VAR_INT16:
            if (data->value[0].int16_value >= minValue && data->value[0].int16_value <= maxValue) {
                *(int16_t *)ptr = data->value[0].int16_value;
                return true;
            }
            break;
        case VAR_UINT32:
            if (data->value[0].uint32_value >= (uint32_t)minValue && data->value[0].uint32_value <= (uint32_t)maxValue) {
                *(uint32_t *)ptr = data->value[0].uint32_value;
                return true;
            }
            break;
        case VAR_FLOAT:
            if (data->value[0].float_value >= minValue && data->value[0].float_value <= maxValue) {
                *(float *)ptr = data->value[0].float_value;
                return true;
            }
            break;
        case VAR_INT16_XYZ:
            if (data->value[0].int16_value >= minValue && data->value[0].int16_value <= maxValue && 
                data->value[1].int16_value >= minValue && data->value[1].int16_value <= maxValue && 
                data->value[2].int16_value >= minValue && data->value[2].int16_value <= maxValue) {
                ((int16_t *)ptr)[0] = data->value[0].int16_value;
                ((int16_t *)ptr)[1] = data->value[1].int16_value;
                ((int16_t *)ptr)[2] = data->value[2].int16_value;
                return true;
            }
            break;
        case VAR_MMIX:
            return paramWrite_MMIX((motorMixer_t *)ptr, data);
    }

    return false;
}

bool mspGetParamByGroupAndId(uint8_t group_id, uint16_t param_id, paramProtocolData_t * data)
{
    int tableIndex = lookupParamTableIndex(group_id, param_id);
    
    if (tableIndex >= 0) {
        return mspGetParamByIndex(tableIndex, data);
    }
    else {
        return false;
    }
}

bool mspSetParamByGroupAndId(paramProtocolData_t * data)
{
    int tableIndex = lookupParamTableIndex(data->group_id, data->param_id);
    
    if (tableIndex >= 0) {
        return mspSetParamByIndex(tableIndex, data);
    }
    else {
        return false;
    }
}

bool mspGetParamDescriptorByIndex(uint16_t tableIndex, paramProtocolDataDescriptor_t * data)
{
    data->param_index = tableIndex;
    data->param_count = PARAM_COUNT;
    data->group_id    = GROUP_ERROR;
    data->param_id    = ERR_PARAM;
    
    // Sanity check 
    if (tableIndex >= PARAM_COUNT)
        return false;

    data->group_id    = valueTable[tableIndex].group_id;
    data->param_id    = valueTable[tableIndex].param_id;
    data->data_type   = valueTable[tableIndex].type;

    return true;
}
