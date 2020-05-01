//
// Created by lars on 4/23/20.
//

#ifndef NOROBO_CONFIGPARSER_H
#define NOROBO_CONFIGPARSER_H
#include "Drone.h"

class Config{
  public:
  int windowSize;
  int stateDebug;
  int planningTimesteps;
  double planningDt;
  int enableRollingWindow;
  DroneState initialState;
};

class ConfigParser  // TODO: Make the config in a better way.
{
  public:
  static Config getConfig();
};


#endif //NOROBO_CONFIGPARSER_H
