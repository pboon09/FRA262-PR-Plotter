/*
 * Controller.c
 *
 *  Created on: Apr 14, 2024
 *      Author: beamk
 */
#include "Controller.h"

int32_t PWM_Satuation(float _u, int32_t _upper_limit, int32_t _lower_limit) {
  if (_u > _upper_limit) return _upper_limit;
  else if (_u < _lower_limit) return _lower_limit;
  return (int32_t)_u;
}

void PID_CONTROLLER_Init(PID_CONTROLLER *controller, float _Kp, float _Ki, float _Kd, float _u_max) {
  controller->Kp = _Kp;
  controller->Ki = _Ki;
  controller->Kd = _Kd;
  controller->u_max = _u_max;
  controller->ek_1 = 0;
  controller->ek_2 = 0;
  controller->u = 0;

}

float PID_CONTROLLER_Compute(PID_CONTROLLER *controller, float ek) {
    if (!((controller->u >= controller->u_max && ek > 0) || (controller->u <= -controller->u_max && ek < 0))) {
        controller->u += ((controller->Kp + controller->Ki + controller->Kd) * ek)
                         - ((controller->Kp + (2 * controller->Kd)) * controller->ek_1)
                         + (controller->Kd * controller->ek_2);
    }
    controller->ek_2 = controller->ek_1;
    controller->ek_1 = ek;
    return controller->u;
}
