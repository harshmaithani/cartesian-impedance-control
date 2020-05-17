/*
* Harsh Maithani
* Insitut Pascal
* SIGMA Clermont 
*/

#include <group_command_controller_fri.h>

// For plugin
#include <pluginlib/class_list_macros.h>

#include <algorithm>

#include "ros/ros.h"

#include "std_msgs/String.h"

#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Vector3.h"

#include <sstream>

#include "std_msgs/Float64MultiArray.h"

#include <sensor_msgs/JointState.h>  //For reading the Joint States
#include <std_msgs/Float64.h>

// Ros messages generated
#include <kuka_lwr_controllers/PoseRPY.h>

#include "geometry_msgs/WrenchStamped.h"

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <vector>
#include <iostream>
#include <fstream>

float Fx,Fy,Fz,Tx,Ty,Tz;  // Reading forces from Force Sensor

float Fx_filtered,Fy_filtered,Fz_filtered,Tx_filtered,Ty_filtered,Tz_filtered;

double A1,A2,A3,E1,A4,A5,A6;
double temp_A1,temp_A2,temp_A3,temp_E1,temp_A4,temp_A5,temp_A6;

float F=1.0;
float M=5;
float B=5;
float K=5;
float x=0;float x0=0;
float v=0;float v0=0;
float a=0;
double acceleration[]={0,0,0,0,0,0};
double velocity[]={0,0,0,0,0,0};
double q_init[6];
float dt=0.001;
int tracker=1;

kuka_lwr_controllers::PoseRPY current_joint_positions;

void jointCallback(const kuka_lwr_controllers::PoseRPY::ConstPtr &msg)
{
	if (tracker==1)
	{
	current_joint_positions.position.x=msg->position.x;
	current_joint_positions.position.y=msg->position.y;
	current_joint_positions.position.z=msg->position.z;
	current_joint_positions.orientation.pitch=msg->orientation.pitch;
	current_joint_positions.orientation.roll=msg->orientation.roll;
	current_joint_positions.orientation.yaw=msg->orientation.yaw;
	tracker++;
	}
}

void wrenchCallback(const std_msgs::Float64MultiArray::ConstPtr& wrench1)
//void wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& wrench1)
{
ROS_INFO("I heard the force as %f %f %f",wrench1->data[0],wrench1->data[1],wrench1->data[2]);
ROS_INFO("I heard the torque as %f %f %f",wrench1->data[3],wrench1->data[4],wrench1->data[5]);
	Fx=wrench1->data[0];
	Fy=wrench1->data[1];
	Fz=wrench1->data[2];
	Tx=wrench1->data[3];
	Ty=wrench1->data[4];
	Tz=wrench1->data[5];
}


 std::vector<float> meanFilterFT(std::vector<std::vector<float> > vectoravgfx)
 {
	 std::vector<float> force;
	 force.resize(6);
	 for(int i=0; i < force.size(); i++ )
	 {
		force[i]= 0.0;
		for(int j=0; j < vectoravgfx.size(); j++ )
		{
			force[i]+= vectoravgfx[j][i];
		}
		force[i]/= vectoravgfx.size();
	 }
	 return force;
 }

int main(int argc, char **argv)
{
	
  ros::init(argc, argv, "admittance_control_global_frame");

  ros::NodeHandle n;
  
  // Read the current Joint Positions of the robot from outside
  ros::Subscriber current_joint_position_sub = n.subscribe("/kuka_lwr_right/harsh_admittance_controller/current_joint_position",1000, jointCallback);  

  // Read the Force sensor values 
  ros::Subscriber wrench_sub = n.subscribe("/sensor_readings_global_frame/",1000,wrenchCallback); 	

  // Publish the desired joint position values 
  ros::Publisher joint_position_pub = n.advertise<kuka_lwr_controllers::PoseRPY>("/kuka_lwr_right/harsh_admittance_controller/command",1000);	
 
  ros::Rate loop_rate(1000);

  // Initialization of Force vector for mean filter
	float avgfx=0.0;
	int timesteps=10; // Default window size of filter
		
	std::vector<std::vector<float> > vectoravgfx;
	std::vector<float>filtered_forces;
	for (int i=0; i < timesteps; i++)
	{	
		std::vector<float> force;
		force.push_back(Fx); force.push_back(Fy);force.push_back(Fz); force.push_back(Tx); force.push_back(Ty); force.push_back(Tz);
		vectoravgfx.push_back(force);
	} 

  int count = 0;
  int counterForceVector= 0;


  kuka_lwr_controllers::PoseRPY commanded_pose;

  while (ros::ok())
  {
   
   //!Reading the sensor values from FT
	
	vectoravgfx[counterForceVector][0]=Fx;
	vectoravgfx[counterForceVector][1]=Fy;
	vectoravgfx[counterForceVector][2]=Fz;
	vectoravgfx[counterForceVector][3]=Tx;
	vectoravgfx[counterForceVector][4]=Ty;
	vectoravgfx[counterForceVector][5]=Tz;
	
	counterForceVector=(counterForceVector++)%timesteps;
	filtered_forces= meanFilterFT(vectoravgfx);
   
   Fx_filtered=filtered_forces[0];
   Fy_filtered=filtered_forces[1];
   Fz_filtered=filtered_forces[2];
   Tx_filtered=filtered_forces[3];
   Ty_filtered=filtered_forces[4];
   Tz_filtered=filtered_forces[5];
        
   F=Fz_filtered;
        
   //Mxdd+Bxd+Kx=F	
   
   a=(F-B*v-K*x)/M;
   v=v+a*dt;
   x=x+v*dt;
   
   commanded_pose.position.x=current_joint_positions.position.x;
   commanded_pose.position.y=current_joint_positions.position.y;
   commanded_pose.position.z=current_joint_positions.position.z+x;
   commanded_pose.orientation.roll=current_joint_positions.orientation.roll;
   commanded_pose.orientation.pitch=current_joint_positions.orientation.pitch;
   commanded_pose.orientation.yaw=current_joint_positions.orientation.yaw;
   
	ROS_INFO("Force applied: %f",F);
	ROS_INFO("Commanded z: %f",commanded_pose.position.z);
	
	joint_position_pub.publish(commanded_pose);
	
    ros::spinOnce();

    loop_rate.sleep();

    //++count;
  }

  return 0;
}

