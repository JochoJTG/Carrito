/*
 * PID.h
 *
 *  Created on: Nov 12, 2023
 *      Author: eduar
 */

#ifndef INC_PIDPWM_H_
#define INC_PIDPWM_H_

typedef struct {
	//Coeficientes
	float Kp;
	float Ki;
	float Kd;

	//cambios de setpoint pid.setpoit =
	float setpoint;

	//variables
	float integral; //Integral acumulada
	float errorAnterior;
	float medicionSensor_Anterior;

	//limites del PID
	float min_output;
	float max_output;
}PIDpwm_Controller;

void PIDpwm_Init(PIDpwm_Controller *pid, float Kp, float Ki, float Kd, float min_output, float max_output);
float PIDpwm_Compute(PIDpwm_Controller *pid, float measurement);


#endif /* INC_PIDPWM_H_ */
