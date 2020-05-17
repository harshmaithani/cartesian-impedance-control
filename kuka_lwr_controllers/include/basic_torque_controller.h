#ifndef LWR_CONTROLLERS_BASIC_TORQUE_CONTROLLER_H
#define LWR_CONTROLLERS_BASIC_TORQUE_CONTROLLER_H

// Controller base
#include "kinematic_chain_controller_base.h"

// hardware_interface 
#include <kuka_lwr_hw/lwr_kuka_interface.h> // contains definition of KUKAJointInterface

// Boost
#include <boost/scoped_ptr.hpp>

// KDL JntArray
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>

// KDL Solver
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chaindynparam.hpp>

// ROS msgs 
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/WrenchStamped.h>

// Activate Trace info
#define TRACE_Basic_Torque_Controller_ACTIVATED 1

namespace kuka_lwr_controllers
{
	class BasicTorqueController: public controller_interface::KinematicChainControllerBase<hardware_interface::KUKAJointInterface> 
	{
		public:

			BasicTorqueController();
			~BasicTorqueController();

			bool init(hardware_interface::KUKAJointInterface *robot, ros::NodeHandle &n); // Init the controller
			void starting(const ros::Time& time);	// Start the controller
			void stopping(const ros::Time& time);  // Stop the controller
			void update(const ros::Time& time, const ros::Duration& period); // Update the controller
			
			// Just an example of topic 'command' -> send an array of angular values (7 angular values)
			void command(const std_msgs::Float64MultiArray::ConstPtr &msg);

		private:
		
			std::string robot_namespace_;	// the current robot namespace
			
			int cmd_flag_; // flag set only to 1 when the controller receive a message to the command topic	
			
			boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;  // Jacobian Solver
			boost::scoped_ptr<KDL::ChainDynParam> id_solver_;
			
			ros::Subscriber sub_command_; // Ros subscriber command topic
			ros::Subscriber sub_ft_; // Ros subscriber force sensor values topic
			
			KDL::JntArray q_des_; // q desired setting by command topic
			KDL::Jacobian Jkdl_; // Kuka Jacobian
			KDL::JntSpaceInertiaMatrix 	M_; //Inertia matrix
			KDL::JntArray C_, G_; //Coriolis and Gravitational matrices
			
			KDL::JntArray tau_cmd_; // Torque command
			
			Eigen::VectorXd	FT_sensor_; // vector of Force sensor values
			
			void ft_readingsCB(const geometry_msgs::WrenchStamped& msg);
	};
}


#endif
