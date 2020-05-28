#include "ros/ros.h"
#include <rosflight_msgs/NoroboCommand.h>
#include <rosflight_msgs/Attitude.h>
#include <rosflight_msgs/Command.h>
#include <rosflight_msgs/Barometer.h>
#include <rosflight_msgs/GNSSRaw.h>
#include "rosflight_utils/quaternion.h"
#include "rosflight_utils/quaternionMath.h"
#include <sensor_msgs/Imu.h>

#include "PID_controller.h"

//#include "rosflight_utils/QuaternionMath.h"
#include <iostream>
#include <string>

class publish_arm{
  public:

// Global variables
  int default_queue_size = 49;
  ros::Publisher command_pub;
  boost::shared_ptr<const rosflight_msgs::Attitude> most_recent_alttitude;
  boost::shared_ptr<const rosflight_msgs::Attitude> previous_alttitude;
  PID_controller altitude_controller;
  Quaternion q_ref{-1.91,0,0.42,0};
  float desired_height = 2.0;
  ros::Timer heartbeat_timer_;

  void arm(ros::NodeHandle n){
    ROS_INFO("Sent arm msg.");
    ros::Publisher chatter_pub = n.advertise<rosflight_msgs::NoroboCommand>("norobo_command", 999);
    rosflight_msgs::NoroboCommand msg;
    msg.arm = true;

    ros::Rate default_loop_rate(9);
    // This is to wait for subscriber to be ready. If obmitted the message could get lost.
    while(chatter_pub.getNumSubscribers() == -1)
      default_loop_rate.sleep();
    chatter_pub.publish(msg);
  }

  publish_arm(){
    ros::NodeHandle n;
    arm(n);
    double HEARTBEAT_PERIOD = 0.01;
  }
};
//int main(int argc, char **argv)
//{
////  ros::init(argc, argv, "rosflight_io");
////  rosflight_io::rosflightIO rosflight_io;
////  ros::spin();
//
//  ros::init(argc, argv, "pid_controller");
//  publish_arm arm_obj{};
//  ros::spin();
//
//  return 0;
//}//  ros::Subscriber sub_attitude_euler = n.subscribe("attitude/euler", default_queue_size, attitudeEulerCallback);

//  ros::Subscriber sub_barometer = n.subscribe("baro", default_queue_size, barometerCallback);
//  ros::Subscriber sub_gps_raw = n.subscribe("gps_raw", default_queue_size, gpsRawCallback);
//  ros::Subscriber sub_imu = n.subscribe("imu/data", default_queue_size, imuDataCallback);

