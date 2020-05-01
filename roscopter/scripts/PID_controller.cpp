//
// Created by lars on 3/24/20.
//

#include "PID_controller.h"
#include <iostream>

double PID_controller::run_pid(double desired, double current, double dt) {
  double error = desired - current;

  double d_value = error - this->prev_error;
  this->prev_error = error;

  double i_value = this->prev_integrator + dt*error;

  double summed_gain = (this->pGain*error + this->dGain*d_value + this->iGain*i_value)/this->value_scaler;
  std::cout << "error = " << error << ", gain = " << summed_gain;

  summed_gain = summed_gain > max_limit ? max_limit : summed_gain;
  summed_gain = summed_gain < -max_limit ? -max_limit : summed_gain;

  return summed_gain;
}