#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

typedef struct {

	/* Controller gains */
	float Kp;
	float Ki;

	/* Output limits */
	int limMin;
	int limMax;

	/* Sample time (in seconds) */
	float T;

	/* Controller "memory" */
	int integrator;
	float prevError;			/* Required for integrator */
	float prevMeasurement;		/* Required for differentiator */

	/* Controller output */
	int out;

} PIDController;

void  PIDController_Init(PIDController *pid);
int PIDController_Update(PIDController *pid, float setpoint, float measurement, int currentpwm);

#endif
