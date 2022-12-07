/*
 * FIRFilter.h
 *
 *  Created on: Nov 11, 2022
 *      Author: nwbad
 */

#ifndef INC_FIRFILTER_H_
#define INC_FIRFILTER_H_

#include "stdint.h"

#define FIR_FILTER_LENGTH 10

typedef struct {
	float buf[FIR_FILTER_LENGTH];
	uint8_t bufIndex;

	float out;
} FIRFilter;

void FIRFilter_Init(FIRFilter *fir);
float FIRFilter_Update(FIRFilter *fir, float inp);

#endif /* INC_FIRFILTER_H_ */
