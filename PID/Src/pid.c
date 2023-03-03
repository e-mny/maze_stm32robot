#include "pid.h"
#include "math.h"

void PIDController_Init(PIDController *pid) {

	/* Clear controller variables */
	pid->integrator = 0;
	pid->prevError  = 0.0f;
	pid->prevMeasurement = 0.0f;
	pid->out = 0;

}

int PIDController_Update(PIDController *pid, float setpoint, float measurement, int currentpwm) {

	/*
	* Error signal
	*/
    float error = setpoint - measurement;
    if (isnan(error) == 1) error = 0;

	// Proportional
    //int proportional = pid->Kp * error * currentpwm;

    // velocity implementation (instead of distance)
    int proportional;
    if (error >= 0) // positive error, need to increase pwm
    {
    	 proportional = (int)((1 + pid->Kp * (error / setpoint)) * currentpwm); // kP * (1 + percentage of error based on setpoint) * currentpwm
    }
    else // negative error, need to decrease pwm
    {
    	proportional = (int)((1 + pid->Kp * (error / measurement)) * currentpwm); // kP * (1 + percentage of error based on measurement) * currentpwm
    }

	// Integral
    pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError) * currentpwm;

	// Anti-wind-up via dynamic integrator clamping
	int limMinInt, limMaxInt;

	// Compute integrator limits
	if (pid->limMax > proportional) {

		limMaxInt = pid->limMax - proportional;

	} else {

		limMaxInt = 0;

	}

	if (pid->limMin < proportional) {

		limMinInt = pid->limMin - proportional;

	} else {

		limMinInt = 0;

	}

	// Clamp integrator
    if (pid->integrator > limMaxInt) {

        pid->integrator = limMaxInt;

    } else if (pid->integrator < limMinInt) {

        pid->integrator = limMinInt;

    }

	/*
	* Compute output and apply limits
	*/

    pid->out = proportional + pid->integrator;

    if (pid->out > pid->limMax) {

        pid->out = pid->limMax;

    } else if (pid->out < pid->limMin) {

        pid->out = pid->limMin;

    }

	/* Store error and measurement for later use */
    pid->prevError       = error;
    pid->prevMeasurement = measurement;

	/* Return controller output */
    return pid->out;

}
