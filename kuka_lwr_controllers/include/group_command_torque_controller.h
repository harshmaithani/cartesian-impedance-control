/*
 *  Laurent LEQUIEVRE
 *  Institut Pascal UMR6602
 *  laurent.lequievre@univ-bpclermont.fr
 * 
*/

#ifndef LWR_CONTROLLERS_GROUP_COMMAND_CONTROLLER_H
#define LWR_CONTROLLERS_GROUP_COMMAND_CONTROLLER_H

// Controller base
#include "kinematic_chain_controller_base.h"

// hardware_interface 
#include <hardware_interface/joint_command_interface.h> // contains definition of PositionJointInterface

// msgs Float64MultiArray
#include <std_msgs/Float64MultiArray.h>

#include <string>

#define TRACE_GroupCommandTorqueController_ACTIVATED 0

namespace kuka_lwr_controllers
{
	class GroupCommandTorqueController: 
	public controller_interface::KinematicChainControllerBase<hardware_interface::EffortJointInterface>
	{
		public:
			GroupCommandTorqueController();
			~GroupCommandTorqueController();

			bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n); // Init the controller
			void starting(const ros::Time& time);  // Start the controller
			void stopping(const ros::Time& time);  // Stop the controller
			void update(const ros::Time& time, const ros::Duration& period);  // Update the controller
			
		private:
			void commandCB(const std_msgs::Float64MultiArrayConstPtr& msg); // function associate to a subscribe command topic
			bool isGoalChecked(int value);  // function that verify for a joint index if the torque value is reached 
			ros::Subscriber sub_command_;	
			std::vector<double> commands_buffer_;	// the vector of desired torque values
			int cmd_flag_;  // flag set only to 1 when the controller receive a message to the command topic
			std::vector<int> goal_buffer_; // the vector of index of desired torque values reached
			std::vector<int> goal_factor_; // the vector of signs of increase torque values (+1 -> increase, -1 -> decrement)
			std::string robot_namespace_;
	};
}


#endif
