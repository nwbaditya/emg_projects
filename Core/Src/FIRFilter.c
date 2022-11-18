/*
 * FIRFilter.c
 *
 *  Created on: Nov 11, 2022
 *      Author: nwbad
 */

#include "FIRFilter.h"

static float FIR_IMPULSE_RESPONSE[FIR_FILTER_LENGTH] = {-0.027313375519213704,-0.02397030968726964,-0.07322923821766152,-0.17458688342254686,-0.23710222197190461,-0.11556723292677823,0.1941226814572781,0.4717067279432907,0.4717067279432907,0.1941226814572781,-0.11556723292677823,-0.23710222197190461,-0.17458688342254686,-0.07322923821766152,-0.02397030968726964,-0.027313375519213704};

void FIRFilter_Init(FIRFilter *fir){
	//Clear Filter Buffer
	for(uint8_t n = 0; n < FIR_FILTER_LENGTH; n++){
		fir->buf[n] = 0.0f;
	}

	//Clear Buf Index
	fir->bufIndex = 0;

	//Clear Filter Output
	fir->out = 0.0f;

}

float FIRFilter_Update(FIRFilter *fir, float inp){
	/*Store Latest Sample in buffer */
	fir->buf[fir->bufIndex] = inp;

	/*increment buffer index and wrap around if necessary*/
	fir->bufIndex++;

	if(fir->bufIndex == FIR_FILTER_LENGTH){
		fir->bufIndex = 0;
	}

	/*Compute New Output Sample (Via Convolution)*/
	fir->out = 0.0f;

	uint8_t sumIndex = fir->bufIndex;

	for(uint8_t n = 0; n < FIR_FILTER_LENGTH; n++){
		/*Decrement Index and Wrap if Necessary*/
		if(sumIndex > 0){
			sumIndex--;
		}else{
			sumIndex = FIR_FILTER_LENGTH - 1;
		}

		/*Multiply Impulse Response with Shifted input sample and add to output*/
		fir->out += FIR_IMPULSE_RESPONSE[n] * fir->buf[sumIndex];
	}

	/*return filtered output*/
	return fir->out;
}
