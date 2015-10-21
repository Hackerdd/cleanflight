/*
 * filter.c
 *
 *  Created on: 24 jun. 2015
 *      Author: borisb
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "common/axis.h"

#include "flight/filter.h"

extern uint16_t cycleTime;

// PT1 Low Pass filter
float filterApplyPt1(float input, filterStatePt1_t* state, uint8_t f_cut) {
   float dT = (float)cycleTime * 0.000001f;
   float RC= 1.0f / ( 2.0f * (float)M_PI * f_cut );

   *state = *state + dT / (RC + dT) * (input - *state);

   return *state;
}

// 7 Tap FIR filter as described here:
// Thanks to Qcopter & BorisB
void filterApply9TapFIR(int16_t data[3], int16_t state[3][9], int16_t coeff[9])
{
    int32_t FIRsum;
    int axis, i;

    for (axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        FIRsum = 0;
        for (i = 0; i <= 7; i++) {
            state[axis][i] = state[axis][i + 1];
            FIRsum += state[axis][i] * coeff[i];
        }
        state[axis][8] = data[axis];
        FIRsum += state[axis][8] * coeff[8];
        data[axis] = FIRsum / 256;
    }
}