//
// Created by lars on 4/20/20.
//

#ifndef NOROBO_CONSTANTS_H
#define NOROBO_CONSTANTS_H

#include <iostream>

using namespace std;

namespace NoroboConstants{
  const double g = 9.80;
  const int droneNumberOfStates = 12;
  const int droneNumberOfControl = 4;
  const double droneMass = 2.86;
  const double droneIX = 0.07;
  const double droneIY = 0.08;
  const double droneIZ = 0.12;
//  const int planningTimesteps = 50;
//  const double planningDt = 0.05;
  const string configName = "/home/lars/catkin_ws/src/roscopter/roscopter/scripts/config.txt";  // TODO: Move config.txt to a better location.
}

#endif //NOROBO_CONSTANTS_H
