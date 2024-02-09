// 
//	Author: Yusuf
//

#ifndef PID_CONTROL_PID_CONTROL_H
#define PID_CONTROL_PID_CONTROL_H

#include <time.h>
#include <stddef.h>
#include <stdbool.h>


typedef struct {
    double Kp;
    double Ki;
    double Kd;
    double ref;

    double integral_sum;
    time_t prev_time;
    double prev_error;
    bool initial;
    double upper_lim;
    double lower_lim;
} PID_Controller_t;

/** Initialize a PID controller with the desired parameters.
 * \param controller A reference to the controller that will be initialized by this function.
 * \param _Kp The proportional constant Kp of the controller.
 * \param _Ki The integral constant Ki of the controller.
 * \param _Kd The derivative constant Kd of the controller.
 * \param _ref The desired reference value that the controller will attempt to minimize the error towards.
 * \param _upper_lim The maximum value the output (manipulated variable) can be set to. Output is clamped to this value if the controller attempts to output a higher value.
 * \param _lower_lim The minimum value the output (manipulated variable) can be set to. Output is clamped to this value if the controller attempts to output a lower value.
 * \return returns nothing.
 */
void initialize_PID(PID_Controller_t* controller, double _Kp, double _Ki, double _Kd, double _ref, double _upper_lim, double _lower_lim);


/** Calculate a single step with a PID controller.
 * \param controller A reference to the controller that this function will calculate a step for.
 * \param cur The current reading being input into the controller. It will be used to calculate the error.
 * \return The output (manipulated variable) of the PID controller that will be passed into the actuator.
 */
double step(PID_Controller_t* controller, double cur);

#endif //PID_CONTROL_PID_CONTROL_H
