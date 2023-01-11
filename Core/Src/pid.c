/*
 * pid.c
 *
 *  Created on: Dec 9, 2022
 *      Author: Nawab
 */

#include "pid.h"

void PID_Init(PID_t *pid, float _kp, float _ti, float _td, float _dt){
	pid->kp = _kp;
	pid->ti = _ti;
	pid->td = _td;

	pid->p = 0;
	pid->i = 0;
	pid->d = 0;

	pid->e_now = 0;
	pid->e_prev = 0;
	pid->de = 0;
	pid->dt = _dt;
}

float PID_Update(PID_t *pid, float reference, float input){
	float out;
	pid->e_now = reference - input;
	pid->de = pid->e_now - pid->e_prev;

	pid->p = pid->kp * pid->e_now;
	pid->i = pid->i + (pid->e_now * pid->dt);
	if(pid->i > 500){
		pid->i = 500;
	}else if(pid->i < -500){
		pid->i = -500;
	}
	pid->d = pid->de / pid->dt;

	out = pid->kp * (pid->p + (1 / pid->ti) * pid->i + pid->td * pid->d);
	if(out < -500){
		out = -500;
	}else if(out > 500){
		out = 500;
	}
	pid->e_prev = pid->e_now;
	return out;
}
