/*
 * filter.h
 *
 *  Created on: 24 jun. 2015
 *      Author: borisb
 */


typedef float filterStatePt1_t;

float filterApplyPt1(float input, filterStatePt1_t* state, uint8_t f_cut);
void filterApply9TapFIR(int16_t data[3], int16_t state[3][9], int8_t coeff[9]);