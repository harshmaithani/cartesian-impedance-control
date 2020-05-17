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

float F_x=1.0; float F_y=1.0; float F_z=1.0; 
float M_x=1;   float M_y=1;   float M_z=1;   
float B_x=1;   float B_y=1;   float B_z=10;
float K_x=1;   float K_y=1;   float K_z=100;

float x=0;	float x_0=0;
float y=0;	float y_0=0;
float z=0;	float z_0=0;

float v_x=0;	float v_x_0=0;
float v_y=0;	float v_y_0=0;
float v_z=0;	float v_z_0=0;

float a_x=0;
float a_y=0;
float a_z=0;

float upper_lim=0.001;
float lower_lim=-0.001;

float acc_upper_limit=2;
float acc_lower_limit=-2;

float vel_upper_limit=1;
float vel_lower_limit=-1;

float displacement_upper_limit=10;
float displacement_lower_limit=-10;


double acceleration[]={0,0,0,0,0,0};
double velocity[]={0,0,0,0,0,0};
double q_init[6];
float dt=0.001;
int tracker=1;

kuka_lwr_controllers::PoseRPY current_pose_rpy;
kuka_lwr_controllers::PoseRPY commanded_pose;
std_msgs::Float64MultiArray acc_;
std_msgs::Float64MultiArray vel_;
std_msgs::Float64MultiArray displacement_;
std_msgs::Float64MultiArray current_vel_;

void jointCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
	//if (tracker==1)
	//{
	current_pose_rpy.position.x         = msg->data[0];
	current_pose_rpy.position.y         = msg->data[1];
	current_pose_rpy.position.z         = msg->data[2];
	current_pose_rpy.orientation.pitch  = msg->data[3];
	current_pose_rpy.orientation.roll   = msg->data[4];
	current_pose_rpy.orientation.yaw    = msg->data[5];
	tracker++;
	//}

}

void wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& wrench1)
{
ROS_INFO("I heard the force as %f %f %f",wrench1->wrench.force.x,wrench1->wrench.force.y,wrench1->wrench.force.z);
ROS_INFO("I heard the torque as %f %f %f",wrench1->wrench.torque.x,wrench1->wrench.torque.y,wrench1->wrench.torque.z);
	Fx=wrench1->wrench.force.x;
	Fy=wrench1->wrench.force.y;
	Fz=wrench1->wrench.force.z;
	Tx=wrench1->wrench.torque.x;
	Ty=wrench1->wrench.torque.y;
	Tz=wrench1->wrench.torque.z;

}

void velocityCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{	
	current_vel_.data[0]                = msg->data[0];
	current_vel_.data[1]                = msg->data[1];
	current_vel_.data[2]                = msg->data[2];
	current_vel_.data[3]                = msg->data[3];
	current_vel_.data[4]                = msg->data[4];
	current_vel_.data[5]                = msg->data[5];
	tracker++;
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
	
  ros::init(argc, argv, "admittance_node_fri");

  ros::NodeHandle n;

  // Subscribers
  ros::Subscriber current_joint_position_sub = n.subscribe("/cart_adm_kdl_pose_meters_rpy_rad",1000, jointCallback);    // Current Pose 
  ros::Subscriber wrench_sub = n.subscribe("/sensor_readings",1000,wrenchCallback); 	                                // Current Forces
  ros::Subscriber vel_sub    = n.subscribe("/cart_adm_kdl_cart_velocity_m_rad",1000,velocityCallback); 	                // Current Velocity
  
  // Publishers 
  ros::Publisher joint_position_pub = n.advertise<kuka_lwr_controllers::PoseRPY>("/kuka_lwr_right/harsh_admittance_controller_fri/command",1000);	
  ros::Publisher acc_pub = n.advertise<std_msgs::Float64MultiArray>("/admittance_acceleration",1000);
  ros::Publisher vel_pub = n.advertise<std_msgs::Float64MultiArray>("/admittance_velocity",1000);
  ros::Publisher displacement_pub = n.advertise<std_msgs::Float64MultiArray>("/admittance_displacement",1000);

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

  acc_.data.resize(1);
  vel_.data.resize(1);
  displacement_.data.resize(1);
  current_vel_.data.resize(6);

  commanded_pose.position.x=0.078;
  commanded_pose.position.y=0.007;
  commanded_pose.position.z=0.868;
   
  commanded_pose.orientation.roll   =  2.1096;
  commanded_pose.orientation.pitch  =  1.5706;
  commanded_pose.orientation.yaw    =  2.1097;

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
        
   F_z=-Fx_filtered;
   F_y=Fy_filtered;
   F_x=Fz_filtered;


   //Mxdd+Bxd+Kx=F	
     
   //z=(F_z/K_z);
   /*
   if (z > displacement_upper_limit)
   {
   z=displacement_upper_limit;
   }

   if (z < displacement_lower_limit)
   {
   z=displacement_lower_limit;
   }
   */

  
   v_z = (F_z / B_z);
  // z = v_z*dt;
  // z = v_z*2.5;    
  z=v_z;

   /*
   a_z = (F_z / M_z);
   v_z = v_z + a_z;
   z   = v_z*dt; 
   */


   if ((F_z > 0) && (F_z < upper_lim))
    {
    a_z=0;
    v_z=0;
    z=0;
    }
	   
    if ((F_z < 0) && (F_z > lower_lim))
    {
    a_z=0;
    v_z=0;
    z=0;
    }	
    
   acc_.data[0]=a_z;
   vel_.data[0]=v_z;
   displacement_.data[0]=current_pose_rpy.position.z + z;

   commanded_pose.position.x=0.078;
   commanded_pose.position.y=0.007;
   commanded_pose.position.z=current_pose_rpy.position.z + z*0.01;  //Active Following
//   commanded_pose.position.z=0.868 + z;     

   commanded_pose.orientation.roll   =  2.1096;
   commanded_pose.orientation.pitch  =  1.5706;
   commanded_pose.orientation.yaw    =  2.1097;  

   //commanded_pose.orientation.roll=current_joint_positions.orientation.roll;
   //commanded_pose.orientation.pitch=current_joint_positions.orientation.pitch;
   //commanded_pose.orientation.yaw=current_joint_positions.orientation.yaw;
   
   ROS_INFO("Force applied: %f %f %f",F_x,F_y,F_z);
   ROS_INFO("Commanded x: %f y: % f z: %f",commanded_pose.position.x,commanded_pose.position.y,commanded_pose.position.z);
   ROS_INFO("Acceleration: %f, Velocity : %f, Displacement: %f",acc_.data[0],vel_.data[0],displacement_.data[0]);	
	
   joint_position_pub.publish(commanded_pose);
   acc_pub.publish(acc_);
   vel_pub.publish(vel_);
   displacement_pub.publish(displacement_);
	
    ros::spinOnce();

    loop_rate.sleep();

    //++count;
  }

  return 0;
}

