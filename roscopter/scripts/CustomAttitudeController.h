//
// Created by lars on 3/28/20.
//

#ifndef SRC_CUSTOMATTITUDECONTROLLER_H
#define SRC_CUSTOMATTITUDECONTROLLER_H

#include <rosflight_msgs/NoroboCommand.h>  // TODO: Clean up these headers.
#include <rosflight_msgs/Attitude.h>
#include <rosflight_msgs/Command.h>
#include <rosflight_msgs/GNSSRaw.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <iostream>
#include <string>

#include "ros/ros.h"
#include "rosflight_utils/Quaternion.h"
#include "rosflight_utils/QuaternionMath.h"
#include "TrajectoryOptimization.h"

#include "Drone.h"

#include <fstream>
#include "quadmath.h"
#include "roscopter/quaternionMath.h"
#include <time.h>
#include "configParser.h"


class CustomAttitudeController {
  private:
  int defaultQueueSize = 49;
  int currentProgress = 0;
  Config config;
  Drone drone;
  TrajectoryOptimization trajectoryOptimization;
  DroneState finalState;
  std::tuple<StateDynamicMatrix, ControlDynamicMatrix> controlPlan;
  ros::Publisher command_pub;
  boost::shared_ptr<const nav_msgs::Odometry> mostRecentAlttitude;
  MathematicalProgram prog;
  ros::Timer heartbeat_timer_;
  ros::NodeHandle nh;

  public:
  CustomAttitudeController(DroneState initialState, DroneState finalState, Config conf, Drone drone);
  void arm();
  void send_command(float x, float y, float z, float f);
  void groundTruthAttitudeCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void runControl(const ros::TimerEvent &e);
  DroneState getCurrentState();
};


#endif //SRC_CUSTOMATTITUDECONTROLLER_H
