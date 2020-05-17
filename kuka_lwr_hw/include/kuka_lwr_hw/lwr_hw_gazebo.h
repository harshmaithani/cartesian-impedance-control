#ifndef LWR_HW_GAZEBO
#define LWR_HW_GAZEBO

#include "lwr_hw.h"

// ROS
#include <angles/angles.h>

// Gazebo hook
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

#include <control_toolbox/filters.h>

namespace lwr_hw {
	
	#ifndef DEBUG
		#define DEBUG 0
	#endif
	
	class LWRHWGazebo : public LWRHW
	{
		public:

			LWRHWGazebo() : LWRHW() { ROS_INFO("LWRHWGazebo Contructor !"); }
			~LWRHWGazebo() {}
			
			void setParentModel(gazebo::physics::ModelPtr parent_model);
			
			// Init, read, and write, with Gazebo hooks
			bool init();
			
			void read(ros::Time time, ros::Duration period);
			
			void write(ros::Time time, ros::Duration period);
			
			void printInterfaces(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list);
			
			ControlStrategy getNewControlStrategy(const std::list< hardware_interface::ControllerInfo >& start_list, const std::list< hardware_interface::ControllerInfo >& stop_list, ControlStrategy default_control_strategy);
			
			void doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list);
			
		private:
			// Gazebo stuff
			std::vector<gazebo::physics::JointPtr> sim_joints_;
			gazebo::physics::ModelPtr parent_model_;
			bool parent_set_ = false;
			bool hasSwitched_;
	};
}

#endif
