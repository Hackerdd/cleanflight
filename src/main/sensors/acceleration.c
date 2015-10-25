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
#include "debug.h"

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"

#include "sensors/battery.h"
#include "sensors/sensors.h"
#include "io/beeper.h"
#include "sensors/boardalignment.h"
#include "config/runtime_config.h"
#include "config/config.h"

#include "sensors/acceleration.h"

int16_t accADC[XYZ_AXIS_COUNT];

acc_t acc;                       // acc access functions
sensor_align_e accAlign = 0;
uint16_t acc_1G = 256;          // this is the 1G measured acceleration.

uint16_t calibratingA = 0;      // the calibration is done is the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.

static flightDynamicsTrims_t * accZero;

void accSetCalibrationCycles(uint16_t calibrationCyclesRequired)
{
    calibratingA = calibrationCyclesRequired;
}

bool isAccelerationCalibrationComplete(void)
{
    return calibratingA == 0;
}

bool isOnFinalAccelerationCalibrationCycle(void)
{
    return calibratingA == 1;
}

bool isOnFirstAccelerationCalibrationCycle(void)
{
    return calibratingA == CALIBRATING_ACC_CYCLES;
}

static offsetCalibrationState_t calState;
static bool calibratedAxis[6];
static int  calibratedAxisCount = 0;

void performAcclerationCalibration(void)
{
    int axisIndex = 0;
    uint8_t axis;

    if (ABS(accADC[Z]) > ABS(accADC[X]) && ABS(accADC[Z]) > ABS(accADC[Y])) {
        //Z-axis
        axisIndex = (accADC[Z] > 0) ? 0 : 1;
    }
    else if (ABS(accADC[X]) > ABS(accADC[Y]) && ABS(accADC[X]) > ABS(accADC[Z])) {
        //X-axis
        axisIndex = (accADC[X] > 0) ? 2 : 3;
    }
    else if (ABS(accADC[Y]) > ABS(accADC[X]) && ABS(accADC[Y]) > ABS(accADC[Z])) {
        //Y-axis
        axisIndex = (accADC[Y] > 0) ? 4 : 5;
    }

    // Top-up and first calibration cycle, reset everything
    if (axisIndex == 0 && isOnFirstAccelerationCalibrationCycle()) {
        for (axis = 0; axis < 6; axis++) {
            calibratedAxis[axis] = false;
        }

        calibratedAxisCount = 0;
        offsetCalibrationResetState(&calState);
    }

    if (!calibratedAxis[axisIndex]) {
        offsetCalibrationPushSample(&calState, accADC);

        if (isOnFinalAccelerationCalibrationCycle()) {
            calibratedAxis[axisIndex] = true;
            calibratedAxisCount++;

            beeperConfirmationBeeps(2);
        }
    }

    if (calibratedAxisCount == 6) {
        float accZerof[3];
        offsetCalibrationCalculateOffset(&calState, accZerof);

        for (axis = 0; axis < 3; axis++) {
            accZero->raw[axis] = lrintf(accZerof[axis]);
        }

        saveConfigAndNotify();
    }

    calibratingA--;
}

void applyAccelerationZero(flightDynamicsTrims_t * accZero)
{
    accADC[X] -= accZero->raw[X];
    accADC[Y] -= accZero->raw[Y];
    accADC[Z] -= accZero->raw[Z];
}

void updateAccelerationReadings(void)
{
    if (!acc.read(accADC)) {
        return;
    }

    if (!isAccelerationCalibrationComplete()) {
        performAcclerationCalibration();
    }

    alignSensors(accADC, accADC, accAlign);
    applyAccelerationZero(accZero);
}

void setAccelerationZero(flightDynamicsTrims_t * accZeroToUse)
{
    accZero = accZeroToUse;
}
