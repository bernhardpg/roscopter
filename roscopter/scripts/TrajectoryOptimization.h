
// Created by lars on 4/19/20.
//

#ifndef SRC_TRAJECTORYOPTIMIZATION_H
#define SRC_TRAJECTORYOPTIMIZATION_H
#include "Drone.h"
#include <iostream>
#include "constants.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"
#include "configParser.h"
#include <time.h>

using namespace NoroboConstants;
using namespace drake::solvers;
using namespace drake::symbolic;

typedef Matrix<double, Dynamic, droneNumberOfStates> StateDynamicMatrix;
typedef Matrix<double, Dynamic, droneNumberOfControl> ControlDynamicMatrix;

class TrajectoryOptimization {

  int timesteps; // CONST
  double dt;
  Config config;
  DroneState initialState;
  DroneState finalState;
  Drone drone;
  MathematicalProgram prog;

  public:
  Eigen::Matrix<Variable, Eigen::Dynamic, droneNumberOfStates> state_var;
  Eigen::Matrix<Variable, Eigen::Dynamic, droneNumberOfControl> thrust_var;
  Binding<LinearConstraint> initalConstraint;
  Binding<LinearConstraint> finalConstraint;
  void setNewProgVar(int timesteps);
  TrajectoryOptimization(int timesteps, double dt, Drone inputDrone, DroneState initialState, DroneState finalState);
  void addConstraintsToProg();
  std::tuple<StateDynamicMatrix, ControlDynamicMatrix> getPath();
  void setInitialGuess(MatrixXd stateGuess, MatrixXd thrustGuess);
  };


#endif //SRC_TRAJECTORYOPTIMIZATION_H
