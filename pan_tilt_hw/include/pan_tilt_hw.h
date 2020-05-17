/*
 *  Laurent LEQUIEVRE
 *  Institut Pascal UMR6602
 *  laurent.lequievre@univ-bpclermont.fr
 * 
*/

#ifndef KUKA_LWR_HW_H
#define KUKA_LWR_HW_H

// ROS controls
#include <hardware_interface/robot_hw.h>

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

#include <transmission_interface/transmission_info.h>
#include <transmission_interface/transmission_parser.h>

// ROS headers
#include <urdf/model.h>

// lib com pan tilt
#include <pan_tilt_com/UdpTransport.h>
#include <pan_tilt_com/Drive.h>

namespace pan_tilt_hw
{
	#define TRACE_ACTIVATED 1
	
	class PanTiltHW : public hardware_interface::RobotHW
	{
		public:
			PanTiltHW();
			virtual ~PanTiltHW();
			
			bool init();
			void create(std::string urdf_string, const ros::NodeHandle& nh);
			
			void read(ros::Time time, ros::Duration period);  // read 'measurement' joint values
			void write(ros::Time time, ros::Duration period);	// write 'cmd' joint cmd values

		private:
			// pan tilt lib com objects
			Pure::UdpTransport* udpTransport_;
			Pure::Drive::Client* client_;
			
			std::string robot_namespace_; // robot namespace
			
			// URDF Model
			std::string urdf_string_;
			urdf::Model urdf_model_;
			
			// configuration
			int n_joints_ = 4; // the pantilt has 4 joints
			std::vector<std::string> joint_names_; // vector of joints names : (pantilt_pivot1, pantilt_pivot2, ...)
			
			// state and commands
			std::vector<double>
			joint_position_,
			joint_position_prev_,
			joint_velocity_,
			joint_effort_,
			joint_position_command_,
			joint_velocity_command_,
			joint_effort_command_;
			
			// Transmissions in this plugin's scope
			std::vector<transmission_interface::TransmissionInfo> transmissions_;
			
			Pure::Drive::Mode current_mode_; // current control mode of pantilt
			
			bool parseTransmissionsFromURDF_(const std::string& urdf_string); // Parse Transmissions from the URDF string
			
			// Register all interfaces necessary
			void registerInterfaces_(const urdf::Model *const urdf_model, 
									 std::vector<transmission_interface::TransmissionInfo> transmissions);
									 
			void traceTransmissionInfo_(const transmission_interface::TransmissionInfo& info);
			
			// Hardware interfaces
			hardware_interface::JointStateInterface joint_state_interface_;
			hardware_interface::EffortJointInterface joint_effort_interface_;
			hardware_interface::PositionJointInterface joint_position_interface_;
									 
	};
}

#endif
