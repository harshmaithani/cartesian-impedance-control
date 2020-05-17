#include <ros/ros.h>
#include "std_msgs/String.h"
#include <ros/message_operations.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <string>

// FTSensor class definition
#include "FTSensor/FTSensor.h"


int my_count = 0;
ros::Time previous;

void datacallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  //ROS_INFO("Force : x=[%f] y=[%f] z=[%f] Torque: x=[%f] y=[%f] z=[%f]", msg->wrench.force.x, msg->wrench.force.y , msg->wrench.force.z, msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z);
   ++my_count;
   ros::Time actual = msg->header.stamp;
   
   if ((my_count%2000) == 0)
   {
	   ros::Duration diff = actual - previous;
	   ROS_INFO("nsec = %f", (diff.toNSec()/1e6));
   }
   
   previous = actual;
}

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "FT_sensor_listener");
	ros::NodeHandle n;
	previous = ros::Time::now();
	ros::Subscriber sub = n.subscribe("sensor_readings", 1000, datacallback);

   ros::spin();

  return 0;
}


