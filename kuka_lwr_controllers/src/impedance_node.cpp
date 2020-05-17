/*
* Harsh Maithani
* Insitut Pascal
* SIGMA Clermont 
*/

#include <harsh_trust_impedance_controller.h>

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

float F_x_commanded = 0, F_y_commanded = 0, F_z_commanded=0;
float Tau_x_commanded = 0, Tau_y_commanded = 0, Tau_z_commanded = 0;

float Fx,Fy,Fz,Tx,Ty,Tz;  // Reading forces from Force Sensor

float Fx_filtered,Fy_filtered,Fz_filtered,Tx_filtered,Ty_filtered,Tz_filtered;

double A1,A2,A3,E1,A4,A5,A6;
double temp_A1,temp_A2,temp_A3,temp_E1,temp_A4,temp_A5,temp_A6;

float F_x=1.0; float F_y=1.0; float F_z=1.0; 
float M_x=2;   float M_y=1;   float M_z=0.1;   
float B_x=100;   float B_y=1;   float B_z=0.1;
float K_x=100;   float K_y=1;   float K_z=0;

float x=0;	float x_0=0;
float y=0;	float y_0=0;
float z=0;	float z_0=0;

float x_init =0;

float v_x=0;	float v_x_0=0;
float v_y=0;	float v_y_0=0;
float v_z=0;	float v_z_0=0;

float a_x=0;
float a_y=0;
float a_z=0;

float newx=0;
float newvx=0;
float newax=0;

float upper_lim=0.001;
float lower_lim=-0.001;

float acc_upper_limit=2;
float acc_lower_limit=-2;

float vel_upper_limit=1;
float vel_lower_limit=-1;

float displacement_upper_limit=1;
float displacement_lower_limit=-1;


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

std_msgs::Float64MultiArray wrench;

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
	
  ros::init(argc, argv, "impedance_node");

  ros::NodeHandle n;

  // Subscribers
  ros::Subscriber current_joint_position_sub = n.subscribe("/cart_imp_pose_meters_rpy_rad",1000, jointCallback);   // Current Pose 
  ros::Subscriber wrench_sub = n.subscribe("/sensor_readings",1000,wrenchCallback); 	                           // Force Sensor Readings
  ros::Subscriber vel_sub    = n.subscribe("/cart_imp_kdl_cart_velocity_m_rad",1000,velocityCallback); 	           // Current Velocity
  
  // Publishers
  ros::Publisher wrench_pub = n.advertise<std_msgs::Float64MultiArray>("/kuka_lwr_right/harsh_trust_impedance_controller/setCartesianWrench",1000);	
  ros::Publisher acc_pub = n.advertise<std_msgs::Float64MultiArray>("/impedance_acceleration",1000);
  ros::Publisher vel_pub = n.advertise<std_msgs::Float64MultiArray>("/impedance_velocity",1000);
  ros::Publisher displacement_pub = n.advertise<std_msgs::Float64MultiArray>("/impedance_displacement",1000);

  ros::Rate loop_rate(100);

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

  // Resize Local Variables

  acc_.data.resize(1);
  vel_.data.resize(1);
  displacement_.data.resize(1);
  wrench.data.resize(6); 
  current_vel_.data.resize(6);

  x = current_pose_rpy.position.z;
  x_init = current_pose_rpy.position.z;
  v_x=0;
  a_x=0;	

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

   //F=Mxdd+Bxd+Kx
   
   newx = current_pose_rpy.position.z;
   
   newvx = current_vel_.data[2]; // Velocity in z direction
   
   newax = (newvx - v_x);
   
   //F_x_commanded = K_x*(current_pose_rpy.position.z - x);

   //F_x_commanded = B_x*(current_vel_.data[2]);

   F_x_commanded = M_x*newax;
   
   x = newx;
   v_x=newvx;
   a_x=newax;
   
   if (F_x_commanded > 5)
   { 
   F_x_commanded = 5;
   }
   if (F_x_commanded < -5)
   {
   F_x_commanded = -5;
   }
   
   //F=Mxdd+Bxd+Kx
   
   wrench.data[0]=F_x_commanded; // x direction of TCP 
   wrench.data[1]=0;
   wrench.data[2]=0;
   wrench.data[3]=0;
   wrench.data[4]=0;
   wrench.data[5]=0;
   
   acc_.data[0]=a_x;
   vel_.data[0]=v_x;
   displacement_.data[0]=x;
	
   acc_pub.publish(acc_);
   vel_pub.publish(vel_);
   displacement_pub.publish(displacement_);
 
   ROS_INFO("x is %f, v is %f, Fx is %f",x,v_x,F_x_commanded);

   wrench_pub.publish(wrench); 
	
    ros::spinOnce();

    loop_rate.sleep();

    //++count;
  }

  return 0;
}

