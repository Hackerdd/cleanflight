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

#include <platform.h>

#include "fc/rc_controls.h"

#include "flight/failsafe.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "io/motors.h"

#include "rx/rx.h"

#include "config/config_profile.h"
#include "config/config_master.h"
#include "config/feature.h"

void targetConfiguration(master_t *config) {
    config->motorConfig.motorPwmRate = 4000;
    
    config->serialConfig.portConfigs[2].functionMask = FUNCTION_TELEMETRY_FRSKY;    
    config->rxConfig.sbus_inversion = 0;
    config->serialConfig.portConfigs[3].functionMask = FUNCTION_RX_SERIAL;
    featureSet(FEATURE_RX_SERIAL);
    config->rxConfig.serialrx_provider = SERIALRX_SBUS;
    
    config->profile[0].pidProfile.P8[ROLL] = 80;
    config->profile[0].pidProfile.I8[ROLL] = 37;
    config->profile[0].pidProfile.D8[ROLL] = 35;
    config->profile[0].pidProfile.P8[PITCH] = 100;
    config->profile[0].pidProfile.I8[PITCH] = 37;
    config->profile[0].pidProfile.D8[PITCH] = 35;
    config->profile[0].pidProfile.P8[YAW] = 180;
    config->profile[0].pidProfile.I8[YAW] = 45;
    
    config->profile[0].pidProfile.P8[PIDLEVEL] = 30;    
}