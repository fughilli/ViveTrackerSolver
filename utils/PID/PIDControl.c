#include "PIDControl.h"


void PID_PIDController(PIDController_t* controller, float pCoeff, float iCoeff, float dCoeff, float iDecayFactor)
{
    controller->coefficients.p = pCoeff;
    controller->coefficients.i = iCoeff;
    controller->coefficients.d = dCoeff;
    controller->coefficients.idf = iDecayFactor;
    controller->state.integral = 0.0f;
    controller->state.prev_error_x = 0.0f;
}

inline float my_fabs(float arg)
{
	if(arg < 0.0f)
		return -arg;
	return arg;
}

float PID_calculate(PIDController_t* controller, float x, float target_x, float dt)
{
    float error_x = (target_x - x);
    float P = (error_x * controller->coefficients.p);

    controller->state.integral += (error_x * dt);
    float I = (controller->state.integral * controller->coefficients.i);
    controller->state.integral /= controller->coefficients.idf;
    if(my_fabs(controller->state.integral) < I_ZERO_THRESHOLD)
    {
    	controller->state.integral = 0.0f;
    }

    float D = ((error_x - controller->state.prev_error_x) / dt * controller->coefficients.d);

    controller->state.prev_error_x = error_x;

    return P + I + D;
}
