/*
 * pid.h
 *
 *  Created on: Dec 9, 2022
 *      Author: Nawab
 */

#ifndef INC_PID_H_
#define INC_PID_H_


typedef struct{
	float kp;
	float ti;
	float td;

	float p;
	float i;
	float d;

	float e_now;
	float e_prev;
	float de;
	float dt;
}PID_t;

void PID_Init(PID_t *pid, float _kp, float _ti, float _td, float _dt);
float PID_Update(PID_t *pid, int reference, int input);

#endif /* INC_PID_H_ */
