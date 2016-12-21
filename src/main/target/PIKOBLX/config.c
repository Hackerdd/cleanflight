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

#include <stdint.h>

#include <platform.h>

#include "config/config_master.h"
#include "config/feature.h"

#if defined(KOMBINI)
void targetConfiguration(master_t *config) {
	featureSet(FEATURE_CURRENT_METER);
    config->batteryConfig.currentMeterScale = 125;
}

#elif defined(RACEWHOOP)
void targetConfiguration(master_t *config) {
	config->serialConfig.portConfigs[2].functionMask = FUNCTION_TELEMETRY_FRSKY;
	config->rxConfig.sbus_inversion = 0;
	config->serialConfig.portConfigs[3].functionMask = FUNCTION_RX_SERIAL;
	featureSet(FEATURE_RX_SERIAL);
	featureSet(FEATURE_TELEMETRY);
	config->rxConfig.serialrx_provider = SERIALRX_SBUS;
	featureSet(FEATURE_CURRENT_METER);
	config->batteryConfig.currentMeterScale = 1000;
	config->batteryConfig.batteryCapacity = 300;
}

#elif defined(ACROWHOOP) || defined(NUKE)
void targetConfiguration(master_t *config) {
	config->motorConfig.motorPwmRate = 32000;
	config->motorConfig.minthrottle = 1100;

#if defined(ACROWHOOP)
	//config for onboard FrSky RX
	config->serialConfig.portConfigs[2].functionMask = FUNCTION_TELEMETRY_FRSKY;
	config->rxConfig.sbus_inversion = 0;
	config->serialConfig.portConfigs[3].functionMask = FUNCTION_RX_SERIAL;
	featureSet(FEATURE_RX_SERIAL);
	featureSet(FEATURE_TELEMETRY);
	config->rxConfig.serialrx_provider = SERIALRX_SBUS;
#endif //end ACROWHOOP
}

#endif
