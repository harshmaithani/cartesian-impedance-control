/*
*Harsh Maithani
*Universite Clermont Auvergne
 * 
*/

#ifndef LWR_CONTROLLERS_HARSH_ADMITTANCE_CONTROLLER_ROS_H
#define LWR_CONTROLLERS_HARSH_ADMITTANCE_CONTROLLER_ROS_H

// Controller base
#include "kinematic_chain_controller_base.h"

// Hardware_interface 
#include <kuka_lwr_hw/lwr_kuka_cartesian_interface.h> // contains definition of KUKACartesianInterface

// hardware_interface 
#include <kuka_lwr_hw/lwr_kuka_interface.h> // contains definition of KUKAJointInterface

// KDL JntArray
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>

// KDL
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>

// msgs 
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/WrenchStamped.h>

// msgs Float64MultiArray and PoseStamped
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>

// KDL added
#include <kdl/stiffness.hpp>
#include <kdl/trajectory.hpp>
#include <kdl_conversions/kdl_msg.h>

// ROS Realtime Publisher Tools
#include <realtime_tools/realtime_publisher.h>

// Boost
#include <boost/scoped_ptr.hpp>

// hardware_interface 
#include <hardware_interface/joint_command_interface.h> // contains definition of PositionJointInterface

// Ros messages generated
#include <kuka_lwr_controllers/PoseRPY.h>

namespace kuka_lwr_controllers
{
	class HarshAdmittanceControllerROS: 
	public controller_interface::KinematicChainControllerBase<hardware_interface::PositionJointInterface>
	{
		public:
			HarshAdmittanceControllerROS();
			~HarshAdmittanceControllerROS();

			bool init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n);
			void starting(const ros::Time& time);
			void update(const ros::Time& time, const ros::Duration& period);
			void command(const std_msgs::Float64MultiArrayConstPtr& msg);

		private:
			ros::Subscriber sub_command_;
			ros::Subscriber sub_gains_;
			ros::Publisher pub_current_joint_position_;
			ros::Publisher pub_cart_adm_data_;
			ros::Publisher pub_cart_adm_kdl_pose_meters_;
			ros::Publisher pub_cart_adm_kdl_pose_mm_;
			ros::Publisher pub_cart_adm_kdl_pose_meters_rpy_rad_;
			ros::Publisher pub_cart_adm_kdl_pose_mm_rpy_deg_;
			ros::Publisher pub_cart_adm_joint_states_rad_;
			ros::Publisher pub_cart_adm_joint_states_deg_;
			ros::Publisher pub_cart_adm_joint_velocity_rads_;
			ros::Publisher pub_cart_adm_joint_velocity_degs_;
			ros::Publisher pub_cart_adm_kdl_cart_velocity_m_rad_;
			ros::Publisher pub_cart_adm_kdl_cart_velocity_mm_deg_;
			ros::Publisher pub_cart_adm_joint_motor_torques_;
			ros::Publisher pub_cart_adm_kdl_jacobian_;
			
			kuka_lwr_controllers::PoseRPY current_joint_position;

			KDL::Frame    			    pose_current_;			// Current end-effector pose
			KDL::Frame    			    pose_initial_;			// Current end-effector pose
			KDL::FrameVel 			    cartesian_velocity_current_; 		// Current end-effector velocity
			KDL::Jacobian 			    jacobian_;          // Current Jacobian 
			KDL::JntArray				joint_positions_;   // Current Joint Positions for computing Jacobian	
			KDL::JntArrayVel			joint_velocities_;  // Current Joint Velocities for computing Cartesian Velocities
			KDL::JntArray				joint_positions_test_;   // Current Joint Positions for computing Jacobian	

			KDL::Frame x_;		//current pose
			KDL::Frame x_des_;	//desired pose

			KDL::Twist x_err_;

			KDL::JntArray q_cmd_; // computed set points

			KDL::Jacobian J_;	//Jacobian

			Eigen::MatrixXd J_pinv_;
			Eigen::Matrix<double,3,3> skew_;

			std_msgs::Float64MultiArray  robot_data_; 	 // local variable for the publishers 
			std_msgs::Float64MultiArray  robot_pose_meters_;
			std_msgs::Float64MultiArray  robot_pose_mm_;
			std_msgs::Float64MultiArray  robot_pose_meters_rpy_rad_;
			std_msgs::Float64MultiArray  robot_pose_mm_rpy_deg_;
			std_msgs::Float64MultiArray  robot_joint_states_rad_;
			std_msgs::Float64MultiArray  robot_joint_states_deg_;
			std_msgs::Float64MultiArray  robot_joint_velocity_rads_;
			std_msgs::Float64MultiArray  robot_joint_velocity_degs_;
			std_msgs::Float64MultiArray  robot_cartesian_velocity_m_rad_;
			std_msgs::Float64MultiArray  robot_cartesian_velocity_mm_deg_;
			
			std_msgs::Float64MultiArray  robot_joint_motor_torques_;  
			std_msgs::Float64MultiArray  robot_jacobian_;
	
			
			double roll_,pitch_,yaw_; // For roll pitch and yaw

			struct quaternion_
			{
				KDL::Vector v;
				double a;
			} quat_curr_, quat_des_;

			KDL::Vector v_temp_;
	
			int cmd_flag_;
	
			boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
			boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
			boost::scoped_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
			boost::scoped_ptr<KDL::ChainIkSolverPos_NR_JL> ik_pos_solver_;
			boost::scoped_ptr<KDL::ChainFkSolverVel_recursive> fk_vel_solver_;


	};
}

#endif
