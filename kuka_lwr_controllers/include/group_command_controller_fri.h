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

// FRI Type IRML
#include <TypeIRML.h>

#include <string>

// Service GetJointVelocity
#include <kuka_lwr_controllers/GetJointVelocity.h>

#ifndef PI
	#define PI 3.1415926535897932384626433832795
#endif

#ifndef RAD
	#define RAD(A)	((A) * PI / 180.0 )
#endif

#ifndef DEG
	#define DEG(A)	((A) * 180.0 / PI )
#endif

#define TRACE_GroupCommandController_ACTIVATED 0

namespace kuka_lwr_controllers
{
	class GroupCommandControllerFRI: 
	public controller_interface::KinematicChainControllerBase<hardware_interface::PositionJointInterface>
	{
		public:
			GroupCommandControllerFRI();
			~GroupCommandControllerFRI();

			bool init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n); // Init the controller
			void starting(const ros::Time& time);  // Start the controller
			void stopping(const ros::Time& time);  // Stop the controller
			void update(const ros::Time& time, const ros::Duration& period);  // Update the controller
			
		private:
			void commandCB(const std_msgs::Float64MultiArrayConstPtr& msg); // function associate to a subscribe command topic
			void setMaxVelocityCB(const std_msgs::Float64MultiArrayConstPtr& msg); // function associate to a subscribe setMaxVelocity topic
			ros::Subscriber sub_command_, sub_max_velovity_;
			ros::ServiceServer srv_get_velocity_;
			int cmd_flag_;  // flag set only to 1 when the controller receive a message to the command topic
			std::string robot_namespace_;
			std::vector<double> v_max_acc_;
			
			TypeIRML *RML_;
			TypeIRMLInputParameters *IP_;
			TypeIRMLOutputParameters *OP_;
			double cycleTime_;
			int resultValue_;
			
			bool getCurrentJointVelocity(kuka_lwr_controllers::GetJointVelocity::Request& req, kuka_lwr_controllers::GetJointVelocity::Response& resp);
			
	};
}


#endif
