//
// Created by lars on 4/20/20.
//

#ifndef ROSCOPTER_DRONE_H
#define ROSCOPTER_DRONE_H

#include <Eigen/Core>
#include <iostream>
#include "drake/common/symbolic.h"
#include "constants.h"

//using namespace drake;
using namespace drake::symbolic;
using namespace Eigen;

using Eigen::VectorXf;
//using Eigen::Vector;

typedef Matrix<double, 12, 1> DroneState;

class Drone {  // TODO: Remove all these refs to "Eigen"
  private:
  DroneState state;
  double mass;
  double iyy;
  double ixx;
  double izz;
  public:
  DroneState getState();
  Drone(DroneState state, double m, double ixx, double iyy, double izz);
  Vector<Expression, 12> getDrakeDelta(Vector<Variable, 12> state, Variable t_phi, Variable  t_theta, Variable t_gamma, Variable f);
  Vector<Expression, 12> discrete_dynamics(Vector<Variable, 12> state, Vector<Variable, 12> next_state, Vector4<Variable> thrust, double time_step);
  Matrix3<Expression> drake_rotation_matrix(Variable phi, Variable theta, Variable gamma);
  Matrix3<Expression> drake_transformation_matrix(Variable phi,Variable theta);
};


#endif //ROSCOPTER_DRONE_H
