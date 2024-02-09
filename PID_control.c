// 
//	Author: Yusuf
//

#include "PID_control.h"


void initialize_PID(PID_Controller_t* controller, double _Kp, double _Ki, double _Kd, double _ref, double _upper_lim, double _lower_lim) {
    // initialize constants
    controller->Kp = _Kp;
    controller->Ki = _Ki;
    controller->Kd = _Kd;

    // initialize reference
    controller->ref = _ref;

    // initialize integral sum to 0, so we can sum up values in the step
    controller->integral_sum = 0;

    // indicates if it's the first time the controller is run, so there won't be integral or derivative action for the first iteration
    controller->initial = true;
}


// clamps the output value to the limits of the controller.
double clamp(PID_Controller_t* controller, double val) {
    if(val > controller->upper_lim) {
        val = controller->upper_lim;
    }
    else if(val < controller->lower_lim) {
        val = controller->lower_lim;
    }

    return val;
}


double step(PID_Controller_t* controller, double cur) {
    time_t current_time = time(NULL);

    double error = controller->ref - cur;

    // proportional control
    double proportional = controller->Kp * error;

    // delta error and delta t
    double de = error - controller->prev_error;
    time_t dt = current_time - controller->prev_time;

    // derivative action and integral action if it's not the first iteration
    double derivative;

    if(!controller->initial)
    {
        derivative = controller->Kd * (de/dt);
        controller->integral_sum += controller->Ki * error * dt;
    }
    else
    {
        derivative = 0;
        controller->initial = false;
    }

    // manipulated variable is combination of all 3 controllers
    double output = proportional + controller->integral_sum + derivative;

    // update persistent variables
    controller->prev_error = error;
    controller->prev_time = current_time;

    // clamp output
    output = clamp(controller, output);

    return output;
}

