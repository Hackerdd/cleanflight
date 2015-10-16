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
#include <math.h>

#include "platform.h"

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/sensor.h"
#include "drivers/compass.h"
#include "drivers/compass_hmc5883l.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"

#include "sensors/boardalignment.h"
#include "config/runtime_config.h"
#include "config/config.h"

#include "sensors/sensors.h"
#include "sensors/compass.h"

#ifdef NAZE
#include "hardware_revision.h"
#endif

mag_t mag;                   // mag access functions

extern uint32_t currentTime; // FIXME dependency on global variable, pass it in instead.

int16_t magADC[XYZ_AXIS_COUNT];
sensor_align_e magAlign = 0;
#ifdef MAG
static uint8_t magInit = 0;

void compassInit(void)
{
    // initialize and calibration. turn on led during mag calibration (calibration routine blinks it)
    LED1_ON;
    mag.init();
    LED1_OFF;
    magInit = 1;
}

/*
 *  Offset learning algorithm is inspired by this paper from Bill Premerlani
 *  http://gentlenav.googlecode.com/files/MagnetometerOffsetNullingRevisited.pdf
 */

#define COMPASS_UPDATE_FREQUENCY_10HZ   (1000 * 100)
#define MAG_CALIBRATION_GAIN            0.05f
#define MAG_OFFSET_LIMIT                2000
#define MAG_DIFF_THRESHOLD              25.0f

void updateCompass(flightDynamicsTrims_t *magZero)
{
    static uint32_t nextUpdateAt, calStartedAt = 0;
    static float magZerof[XYZ_AXIS_COUNT];      // Used only for calibration purposes
    static int16_t magADCPrev[XYZ_AXIS_COUNT];         // Previous measurement
    uint32_t axis;

    if ((int32_t)(currentTime - nextUpdateAt) < 0)
        return;

    nextUpdateAt = currentTime + COMPASS_UPDATE_FREQUENCY_10HZ;

    mag.read(magADC);

    if (STATE(CALIBRATE_MAG)) {
        calStartedAt = nextUpdateAt;
        for (axis = 0; axis < 3; axis++) {
            magADCPrev[axis] = magADC[axis];
            magZerof[axis] = magZero->raw[axis];
        }
        DISABLE_STATE(CALIBRATE_MAG);
    }

    if (magInit) {              // we apply offset only once mag calibration is done
        magADC[X] -= magZero->raw[X];
        magADC[Y] -= magZero->raw[Y];
        magADC[Z] -= magZero->raw[Z];
    }

    if (calStartedAt != 0) {
        if ((nextUpdateAt - calStartedAt) < 30000000) {    // 30s: you have 30s to turn the multi in all directions
            // Still calibrating - update magZero
            float newMagnitude = 0;
            float prevMagnitude = 0;
            float diffMagnitude = 0;

            for (axis = 0; axis < 3; axis++) {
                newMagnitude += (float)magADC[axis] * magADC[axis];
                prevMagnitude += (float)magADCPrev[axis] * magADCPrev[axis];
                diffMagnitude += ((float)magADC[axis] - magADCPrev[axis]) * ((float)magADC[axis] - magADCPrev[axis]);
            }

            diffMagnitude = sqrtf(diffMagnitude);

            if (diffMagnitude > MAG_DIFF_THRESHOLD) {
                newMagnitude = sqrtf(newMagnitude);
                prevMagnitude = sqrtf(prevMagnitude);

                for (axis = 0; axis < 3; axis++) {
                    magZerof[axis] += MAG_CALIBRATION_GAIN * ((float)magADC[axis] - (int32_t)magADCPrev[axis]) * (newMagnitude - prevMagnitude) / diffMagnitude;
                    magZerof[axis] = constrainf(magZerof[axis], -MAG_OFFSET_LIMIT, MAG_OFFSET_LIMIT);
                    magZero->raw[axis] = lrintf(magZerof[axis]);
                    magADCPrev[axis] = magADC[axis];
                }
            }
        } else {
            calStartedAt = 0;
            saveConfigAndNotify();
        }
    }

    alignSensors(magADC, magADC, magAlign);
}
#endif
