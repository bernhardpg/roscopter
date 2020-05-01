//
// Created by lars on 4/20/20.
//

#include "Drone.h"

Drone::Drone(DroneState state, double m, double ixx, double iyy, double izz) {
  this->state = state;
  this->mass = m;
  this->ixx = ixx;
  this->iyy = iyy;
  this->izz = izz;
}

Vector<Expression, 12> Drone::getDrakeDelta(Vector<Variable, 12> state, Variable t_phi, Variable  t_theta, Variable t_gamma, Variable f) {
  double m = this->mass;
  double g = NoroboConstants::g;
  double ixx = this->ixx;
  double izz = this->izz;
  double iyy = this->iyy;

  Variable phi = state(3);
  Variable theta = state(4);
  Variable gamma = state(5);

  Variable u = state(6);
  Variable v = state(7);
  Variable w = state(8);

  Variable p = state(9);
  Variable q = state(10);
  Variable r = state(11);

//  Vector3<Expression> dPos = this->drake_rotation_matrix(phi, theta, gamma) * Vector3<Variable>(u,v,w);
//  Vector3<Expression> dVel = Vector3<Expression>(r*v-q*w, p*w-r*u, p*u-p*v) + Vector3<Expression>(-g*sin(theta), g*cos(theta)*sin(gamma), g*cos(theta)*cos(gamma)) + (1/m)*Vector3<Expression>(0,0,-f);
//
//  Vector3<Expression> dAngle = this->drake_transformation_matrix(phi, theta) * Vector3<Variable>(p, q, r);
//  Vector3<Expression> dAngleVel = Vector3<Expression>(((iyy-izz)/ixx)*q*r, ((izz-ixx)/iyy)*p*r, ((ixx-iyy)/izz)*p*q) + Vector3<Expression>((1/ixx)*t_phi, (1/iyy)*t_theta, (1/izz)*t_gamma);

//  Vector3<Expression> dPos = this->drake_rotation_matrix(phi, theta, gamma) * Vector3<Variable>(u,v,w);
//  Vector3<Expression> dVel = Vector3<Expression>(r*v-q*w, p*w-r*u, p*u-p*v) + Vector3<Expression>(-g*sin(theta), g*cos(theta)*sin(gamma), g*cos(theta)*cos(gamma)) + (1/m)*Vector3<Expression>(0,0,-f);
//
//  Vector3<Expression> dAngle = Vector3<Variable>(p, q, r);
//  Vector3<Expression> dAngleVel = Vector3<Expression>((1/ixx)*t_phi, (1/iyy)*t_theta, (1/izz)*t_gamma);

  Vector3<Expression> dPos = Vector3<Variable>(u,v,w);
  Vector3<Expression> dVel = (1/m)*Vector3<Expression>(0,0,-f);

  Vector3<Expression> dAngle = Vector3<Variable>(p, q, r);
  Vector3<Expression> dAngleVel = Vector3<Expression>((1/ixx)*t_phi, (1/iyy)*t_theta, (1/izz)*t_gamma);


  Vector<Expression, 12> vec_joined(12);
  vec_joined << dPos, dAngle, dVel, dAngleVel;

  return vec_joined;
}

Vector<Expression, 12> Drone::discrete_dynamics(Vector<Variable, 12> state, Vector<Variable, 12> next_state, Vector4<Variable> thrust, double time_step){
//  Vector<Expression, 12> stateDelta = this->getDrakeDelta(next_state, thrust(0), thrust(1), thrust(2), thrust(3));
//  Vector<Expression, 12> residual = next_state - state - time_step*stateDelta;
  Vector<Expression, 12> stateDelta = this->getDrakeDelta(state, thrust(0), thrust(1), thrust(2), thrust(3));
  Vector<Expression, 12> residual =  state + time_step*stateDelta - next_state;
  return residual;
}

Matrix3<Expression> Drone::drake_rotation_matrix(Variable phi, Variable theta, Variable gamma) {
  Matrix3<Expression> m;
//  Variable a = cos(gamma).Evaluate();
  m(0,0) = cos(gamma)*cos(theta);
  m(0,1) = cos(gamma)*sin(theta)*sin(phi) - sin(gamma)*cos(phi);
  m(0,2) = cos(gamma)*sin(theta)*cos(phi) + sin(gamma)*sin(phi);

  m(1,0) = sin(gamma)*cos(theta);
  m(1,1) = sin(gamma)*sin(theta)*sin(phi)+cos(gamma)*cos(phi);
  m(1,2) = sin(gamma)*sin(theta)*cos(phi) - cos(gamma)*sin(phi);

  m(2,0) = -sin(theta);
  m(2,1) = cos(theta)*sin(phi);
  m(2,2) = cos(theta)*cos(phi);

  return m;
}

DroneState Drone::getState() {
  return this->state;
}

Matrix3<Expression> Drone::drake_transformation_matrix(Variable phi, Variable theta) {

  Matrix3<Expression> m;
  m(0,0) = 1;
  m(0,1) = sin(phi)*tan(theta);
  m(0,2) = cos(phi)*tan(theta);

  m(1,0) = 0;
  m(1,1) = cos(phi);
  m(1,2) = -sin(phi);

  m(2,0) = 0;
  m(2,1) = sin(phi)/cos(theta);
  m(2,2) = cos(phi)/cos(theta);

  return m;
}