/*
 *  Kamal mohy el dine 
 *  Institut Pascal UMR6602
 *  kamal.mohy.el.dine@gmail.com
 * 
*/

#ifndef LWR_CONTROLLERS_TORQUE_BASED_POSITION_H
#define LWR_CONTROLLERS_TORQUE_BASED_POSITION_H

// Controller base
#include "kinematic_chain_controller_base.h"

// hardware_interface 
#include <kuka_lwr_hw/lwr_kuka_interface.h> // contains definition of KUKAJointInterface

// msgs Float64MultiArray
#include <std_msgs/Float64MultiArray.h>

// KDL JntArray
#include <kdl/jntarray.hpp>

#include <string>

#define TRACE_Torque_Based_Position_ACTIVATED 1

namespace kuka_lwr_controllers
{
	class TorqueBasedPositionController: 
	public controller_interface::KinematicChainControllerBase<hardware_interface::KUKAJointInterface>
	{
		public:
			TorqueBasedPositionController();
			~TorqueBasedPositionController();

			bool init(hardware_interface::KUKAJointInterface *robot, ros::NodeHandle &n); // Init the controller
			void starting(const ros::Time& time);  // Start the controller
			void stopping(const ros::Time& time);  // Stop the controller
			void update(const ros::Time& time, const ros::Duration& period);  // Update the controller
			
			
		private:
			void commandCB(const std_msgs::Float64MultiArrayConstPtr& msg); // function associate to a subscribe command topic
			
			void setKp(const std_msgs::Float64MultiArrayConstPtr& msg); // function associate to a subscribe setKp topic
			void setKd(const std_msgs::Float64MultiArrayConstPtr& msg); // function associate to a subscribe setKd topic
			
			ros::Subscriber sub_command_, sub_kp_, sub_kd_;
			int cmd_flag_;  // flag set only to 1 when the controller receive a message to the command topic	
			std::string robot_namespace_;
			
			KDL::JntArray q_des_, Kp_, Kd_;
			
	};
}


#endif

