#include "TrajectoryOptimization.h"


TrajectoryOptimization::TrajectoryOptimization(int timesteps, double dt, Drone inputDrone, DroneState initialState, DroneState finalState):
                                              prog{},
                                              state_var(prog.NewContinuousVariables(timesteps + 1, NoroboConstants::droneNumberOfStates,"state")),
                                              thrust_var(prog.NewContinuousVariables(timesteps + 1, NoroboConstants::droneNumberOfControl, "thrust")),
                                              initalConstraint(prog.AddLinearConstraint(state_var.row(0) == initialState.transpose())),
                                              finalConstraint(prog.AddLinearConstraint(state_var.row(timesteps-1) == finalState.transpose())),
                                              drone(inputDrone),
                                              timesteps(timesteps){
  this->dt = dt;
  this->finalState = finalState;
  this->initialState = initialState;
}

void TrajectoryOptimization::setNewProgVar(int timesteps){
  this->state_var = this->prog.NewContinuousVariables(timesteps + 1, NoroboConstants::droneNumberOfStates,"state");
  this->thrust_var = this->prog.NewContinuousVariables(timesteps + 1, NoroboConstants::droneNumberOfControl, "thrust");
}

void TrajectoryOptimization::addConstraintsToProg() {


  for (int i = 0; i < this->timesteps; i++){
    Vector<Variable, 12> stateRow = this->state_var.row(i);
    Vector<Variable, 4> thrustRow = this->thrust_var.row(i);
    this->prog.AddQuadraticCost((stateRow-finalState).dot(stateRow-finalState));
  }

  for (int i = 0; i < this->timesteps + 1; i++) {
    this->prog.AddLinearConstraint(this->thrust_var(i, 0) <= 20);
    this->prog.AddLinearConstraint(this->thrust_var(i, 1) <= 20);
    this->prog.AddLinearConstraint(this->thrust_var(i, 2) <= 20);
    this->prog.AddLinearConstraint(this->thrust_var(i, 3) <= NoroboConstants::droneMass * NoroboConstants::g * 2);
  }

  int counter = 0;
  for (int i = 0; i < this->timesteps; i++) {
    Vector<Variable, 12> stateRow = this->state_var.row(i);
    Vector<Variable, 12> nextStateRow = this->state_var.row(i+1);
    Vector<Variable, 4> thrustRow = this->thrust_var.row(i);

    Vector<Expression, 12> dynamics_expression = this->drone.discrete_dynamics(stateRow, nextStateRow, thrustRow, this->dt);

    for (int d = 0; d < NoroboConstants::droneNumberOfStates; d++) {
        this->prog.AddLinearConstraint(dynamics_expression(d) == 0);
    }

    counter += 1;
  }

  std::cout << "dt = " << this->dt << "\n";
}

void TrajectoryOptimization::setInitialGuess(MatrixXd stateGuess, MatrixXd thrustGuess){
  std::cout << "this.state_var.rows = " << this->state_var.rows() << ", this.state guess = " << stateGuess.rows();
  this->prog.SetInitialGuess(this->state_var, stateGuess);
  this->prog.SetInitialGuess(this->thrust_var, thrustGuess);
}

std::tuple<StateDynamicMatrix, ControlDynamicMatrix> TrajectoryOptimization::getPath(){

  const drake::solvers::MathematicalProgramResult result = drake::solvers::Solve(this->prog);

  auto sol_state = result.GetSolution(this->state_var);
  auto sol_thrust = result.GetSolution(this->thrust_var);

  return std::make_tuple(sol_state, sol_thrust);
}
