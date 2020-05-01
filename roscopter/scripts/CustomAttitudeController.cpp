//
// Created by lars on 3/28/20.
//

#include "CustomAttitudeController.h"
#include "configParser.h"
#include "Eigen/Dense" // TODO: Update Eigen.
using namespace Eigen;
using namespace std;
using namespace NoroboConstants;
using Eigen::last;


double translateForceToThrust(double force){
  double a = 0.000015;
  double b = -0.024451;
  double c = 9.002250 - force;
  double sol1 = (-b+sqrt(b*b-4*a*c))/(2*a);
  return (sol1-1000)/1000;
}

void CustomAttitudeController::arm(){
  ROS_INFO("Sending ARM");
  ros::Publisher arm_pub = nh.advertise<rosflight_msgs::NoroboCommand>("norobo_command", 999);
  rosflight_msgs::NoroboCommand msg;
  msg.arm = true;

  ros::Rate default_loop_rate(9);
  // This is to wait for subscriber to be ready. If obmitted the message could get lost.
  while(arm_pub.getNumSubscribers() == 0)
    default_loop_rate.sleep();

  arm_pub.publish(msg); // TODO: make it more reliable.
}
void CustomAttitudeController::send_command(float x, float y, float z, float f){
  rosflight_msgs::Command msg;
  msg.header.stamp = ros::Time::now();
  msg.mode = rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
  msg.ignore = rosflight_msgs::Command::IGNORE_NONE;
  msg.x = x;
  msg.y = y;
  msg.z = z;
  msg.F = f;
  command_pub.publish(msg);
}

void CustomAttitudeController::groundTruthAttitudeCallback(const nav_msgs::Odometry::ConstPtr& msg){
  this->mostRecentAlttitude = msg;
}

static std::string matrixToString(const Eigen::MatrixXd& mat){
  std::stringstream ss;
  ss << mat;
  return ss.str();
}


DroneState CustomAttitudeController::getCurrentState(){

   Vector3d euler_angle = QuaternionMath::quaternion_to_euler(this->mostRecentAlttitude->pose.pose.orientation.x, this->mostRecentAlttitude->pose.pose.orientation.y, this->mostRecentAlttitude->pose.pose.orientation.z, this->mostRecentAlttitude->pose.pose.orientation.w);
   DroneState currentState;

   currentState << this->mostRecentAlttitude->pose.pose.position.x, this->mostRecentAlttitude->pose.pose.position.y, this->mostRecentAlttitude->pose.pose.position.z,
                       euler_angle(0), euler_angle(1), euler_angle(2),
                       this->mostRecentAlttitude->twist.twist.linear.x, this->mostRecentAlttitude->twist.twist.linear.y, this->mostRecentAlttitude->twist.twist.linear.z,
                       this->mostRecentAlttitude->twist.twist.angular.x, this->mostRecentAlttitude->twist.twist.angular.y, this->mostRecentAlttitude->twist.twist.angular.z;

   return currentState;
}



void CustomAttitudeController::runControl(const ros::TimerEvent &e){
//  this->trajectoryController.getPath()
  if (!this->mostRecentAlttitude){
    ROS_INFO("Waiting for state");
    return ;
  }
  std::cout << "before";
  int slidingWindowSize = 0;
  if (this->config.windowSize + this->currentProgress >= get<0>(this->controlPlan).rows()){  // TODO: Make this prettier.
    slidingWindowSize = get<0>(this->controlPlan).rows() - this->currentProgress - 1;
  }
  else {
    slidingWindowSize = this->config.windowSize;
  }

  DroneState currentState = this->getCurrentState();

  std::cout << "window size = " << slidingWindowSize << ", planningTimesteps = " << this->config.planningTimesteps << "planning dt = " << this->config.planningDt;
  clock_t tStart = clock();

  int windowEnd = this->currentProgress + slidingWindowSize;
  MatrixXd stateWindow = std::get<0>(this->controlPlan)(Eigen::seq(this->currentProgress,windowEnd), Eigen::all);
  MatrixXd thrustWindow = std::get<1>(this->controlPlan)(Eigen::seq(this->currentProgress,windowEnd), Eigen::all);

  auto goalState = stateWindow(last, Eigen::all);
  printf("Time taken1: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
  tStart = clock();

//  this->trajectoryOptimization.setInitialGuess(stateWindow, thrustWindow);
  tuple<StateDynamicMatrix, ControlDynamicMatrix> currentPlan;
  if (config.enableRollingWindow) {
    this->trajectoryOptimization.initalConstraint.evaluator()->set_bounds(currentState, currentState);
    this->trajectoryOptimization.finalConstraint.evaluator()->set_bounds(goalState, goalState);
    currentPlan = this->trajectoryOptimization.getPath();
  }
  else{
    currentPlan = pair(get<0>(this->controlPlan)(seq(this->currentProgress, last), all),get<1>(this->controlPlan)(seq(this->currentProgress, last), all));
  }
  Vector4d nextControl = get<1>(currentPlan).row(0);
  double thrust = translateForceToThrust(nextControl(3)/4);

  printf("Time taken2: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);

  this->send_command(nextControl(0), nextControl(1), nextControl(2), thrust);
  this->currentProgress += 1;

  if (this->config.stateDebug >= 1){
    printf("-------%d-------", this->currentProgress);
    printf("Current state = \n %s \n", matrixToString(currentState).c_str());
    printf("Goal state = %s\n", matrixToString(goalState).c_str());
    printf("Thrust = \n%s, \nforce = %e\n", matrixToString(nextControl).c_str(), nextControl(3));
  }

  if (this->config.stateDebug >= 2){
    for (int i = 0; i < get<1>(currentPlan).rows(); i++){
      printf("StatePlan = %s \n", matrixToString(std::get<0>(currentPlan).row(i)).c_str());
    }
    for (int i = 0; i < get<1>(currentPlan).rows(); i++){
      printf("thrustPlan = %s \n", matrixToString(std::get<1>(currentPlan).row(i)).c_str());
    }
  }

  if (this->config.stateDebug >= 1){
    printf("---------------\n\n");
  }
}

CustomAttitudeController::CustomAttitudeController(DroneState initialState, DroneState finalState, Config conf, Drone drone)
                                                  :trajectoryOptimization{conf.windowSize, conf.planningDt, drone, initialState, finalState},
                                                   drone(drone), finalState(finalState)
{
  this->config = ConfigParser::getConfig();
  command_pub = nh.advertise<rosflight_msgs::Command>("command", defaultQueueSize);
  ros::Subscriber sub_attitude = nh.subscribe("multirotor/truth/NED", defaultQueueSize, &CustomAttitudeController::groundTruthAttitudeCallback, this);
  arm();
  heartbeat_timer_ = nh.createTimer(ros::Duration(this->config.planningDt), &CustomAttitudeController::runControl, this);


  TrajectoryOptimization initialTrajectory(conf.planningTimesteps, conf.planningDt, drone, initialState, finalState);

  ROS_INFO("Running trajectory optimization..");
  initialTrajectory.addConstraintsToProg();
  this->controlPlan = initialTrajectory.getPath();

  trajectoryOptimization.addConstraintsToProg();
  this->trajectoryOptimization.addConstraintsToProg();

  if (this->config.stateDebug >= 2){
    printf("--originalPlan--");
    for (int i = 0; i < get<1>(this->controlPlan).rows(); i++){
      printf("StatePlan = %s \n", matrixToString(std::get<0>(this->controlPlan).row(i)).c_str());
    }
    for (int i = 0; i < get<1>(this->controlPlan).rows(); i++){
      printf("thrustPlan = %s \n", matrixToString(std::get<1>(this->controlPlan).row(i)).c_str());
    }
  }
  ROS_INFO("Found config!");
  ros::spin();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pid_controller");
  DroneState finalState;
  finalState << 0, 0, -3, 0, 0, 0, 0, 0, 0, 0, 0, 0;

  Config config = ConfigParser::getConfig();
  Drone drone(config.initialState, droneMass, droneIX, droneIY, droneIZ); // TODO: Find better name for droneIY, IZ and iX
  CustomAttitudeController controller(config.initialState, finalState, config, drone);

  ros::spin();

  return 0;
}