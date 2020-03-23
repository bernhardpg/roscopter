#include "ros/ros.h"
#include "rosflight_msgs/NoroboCommand.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
 
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<rosflight_msgs::NoroboCommand>("norobo_command", 1000);

  ros::Rate loop_rate(10);
  

  // This is to wait for subscriber to be ready. If obmitted the message could get lost. 
  while(chatter_pub.getNumSubscribers() == 0)
    loop_rate.sleep();
  
  if (ros::ok())
  {
    
    rosflight_msgs::NoroboCommand msg;
    msg.arm = true;

    ROS_INFO("Message sent.");

    chatter_pub.publish(msg);
  }


  return 0;
}
