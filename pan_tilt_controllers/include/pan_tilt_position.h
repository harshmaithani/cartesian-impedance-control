/*
 *  Laurent LEQUIEVRE
 *  Institut Pascal UMR6602
 *  laurent.lequievre@univ-bpclermont.fr
 * 
*/

#ifndef PAN_TILT_CONTROLLER_H
#define PAN_TILT_CONTROLLER_H

// ROS
#include <controller_interface/controller.h>

// hardware_interface 
#include <hardware_interface/joint_command_interface.h> // contains definition of PositionJointInterface

// URDF
#include <urdf/model.h>

// KDL
#include <kdl/kdl.hpp>
#include <kdl/jntarrayacc.hpp>

// msgs Float64MultiArray
#include <std_msgs/Float64MultiArray.h>

// Ros messages generated
//#include <pan_tilt_controllers/Pan_tilt.h>

#define TRACE_ACTIVATED 0

namespace pan_tilt_controllers
{
	class PanTiltPosition: public controller_interface::Controller<hardware_interface::PositionJointInterface>
	{
		public:
			PanTiltPosition();
			~PanTiltPosition();
			bool init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n); // Init the controller
			
			void starting(const ros::Time& time);  // Start the controller
			void stopping(const ros::Time& time);  // Stop the controller
			void update(const ros::Time& time, const ros::Duration& period);  // Update the controller
			
		private:
			std::string robot_namespace_;
			ros::NodeHandle nh_;
			
			// configuration
			int n_joints_ = 4; // the pantilt has 4 joints
			std::vector<std::string> joint_names_; // vector of joints names : (pantilt_pivot1, pantilt_pivot2, ...)
			
			std::vector<hardware_interface::PositionJointInterface::ResourceHandleType> joint_handles_;
			
			struct limits_
			{
				KDL::JntArray min;
				KDL::JntArray max;
				KDL::JntArray center;
			} joint_limits_; // KDL structures to store limits min, max, center of each joint
			
			KDL::JntArrayAcc joint_msr_states_, joint_des_states_;  // joint states (measured and desired)
			
			void commandCB_(const std_msgs::Float64MultiArrayConstPtr& msg); // function associate to a subscribe command topic
			std::vector<std::string> getStrings_(const ros::NodeHandle& nh, const std::string& param_name);
			
			ros::Subscriber sub_command_;	
			std::vector<double> commands_buffer_;	// the vector of desired joint values
			int cmd_flag_;  // flag set only to 1 when the controller receive a message to the command topic
			std::vector<int> goal_buffer_; // the vector of index of desired joint values reached
			std::vector<int> goal_factor_; // the vector of signs of increase joint values (+1 -> increase, -1 -> decrement)
			bool isGoalChecked_(int value);  // function that verify for a joint index if the joint value is reached 
			bool first_trace = true;
	};
	
}

#endif
