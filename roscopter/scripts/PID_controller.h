//
// Created by lars on 3/24/20.
//

#ifndef SRC_PID_CONTROLLER_H
#define SRC_PID_CONTROLLER_H


class PID_controller {
  double prev_integrator = 0;
  double prev_error = 0;
  double value_scaler = 1;
  double max_limit = 0.15;

  public:
  float pGain = 1;
  double dGain = 0.0;
  double iGain = 0.0;

  double run_pid(double desired, double current, double dt);
};

#endif //SRC_PID_CONTROLLER_H
