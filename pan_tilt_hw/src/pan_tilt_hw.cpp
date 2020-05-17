/*
 *  Laurent LEQUIEVRE
 *  Institut Pascal UMR6602
 *  laurent.lequievre@univ-bpclermont.fr
 * 
*/

#include "pan_tilt_hw.h"

#include <control_toolbox/filters.h>

namespace pan_tilt_hw
{
	PanTiltHW::PanTiltHW()
	{	
	}

	PanTiltHW::~PanTiltHW() 
	{ 
	}
	
	bool PanTiltHW::init()
	{
		udpTransport_ = new Pure::UdpTransport("192.168.100.252", 60000, 60000);
		sleep(2);
		client_ = new Pure::Drive::Client(*udpTransport_, 2);
		
		// Set current Mode of lib com pan tilt
		current_mode_ = Pure::Drive::Mode::Mode_Position;
		
		#if TRACE_ACTIVATED
			for(int i=0;i<n_joints_;i++)
			{
				ROS_INFO("PanTiltHW: get maxPosition of joint %i = %f",i,client_->drives[i].properties.maxPosition);
				ROS_INFO("PanTiltHW: get minPosition of joint %i = %f",i,client_->drives[i].properties.minPosition);
				ROS_INFO("PanTiltHW: get maxSpeed of joint %i = %f",i,client_->drives[i].properties.maxSpeed);
				ROS_INFO("PanTiltHW: get minSpeed of joint %i = %f",i,client_->drives[i].properties.minSpeed);
			}
		#endif
		
		return true;
	}
	
	// read 'measurement' joint values
	void PanTiltHW::read(ros::Time time, ros::Duration period)
	{
		client_->Receive();
		
		for(int i=0; i<n_joints_;i++)
		{
			joint_position_[i] = client_->drives[i].state.position;
			joint_position_prev_[i] = joint_position_[i];
		    joint_velocity_[i] = filters::exponentialSmoothing((joint_position_[i] - joint_position_prev_[i])/period.toSec(), joint_velocity_[i], 0.2);
		}
	}
	
	// write 'cmd' joint cmd values
	void PanTiltHW::write(ros::Time time, ros::Duration period)
	{
		/* gardes fous */

		for(int i=0;i<n_joints_;i++)
		{  
			if (current_mode_ == Pure::Drive::Mode::Mode_Position)
			{
			  if(joint_position_command_[i] > client_->drives[i].properties.maxPosition)
				joint_position_command_[i] = client_->drives[i].properties.maxPosition;
			  else if (joint_position_command_[i] < client_->drives[i].properties.minPosition)
				joint_position_command_[i] = client_->drives[i].properties.minPosition;
			}
			else if (current_mode_ == Pure::Drive::Mode::Mode_Velocity)
			{ 
			  if(joint_position_command_[i] > client_->drives[i].properties.maxSpeed)
				joint_position_command_[i] = client_->drives[i].properties.maxSpeed;
			  else if(joint_position_command_[i] < client_->drives[i].properties.minSpeed)
				joint_position_command_[i] = client_->drives[i].properties.minSpeed;
			}
		}

		/* insertion des commandes, chaque axe doit avoir sa consigne, indépendante des autres axes, et son mode (position ou vitesse) */  

		for(int i=0;i<n_joints_;i++)
		{
			client_->drives[i].command.target = joint_position_command_[i];
			client_->drives[i].command.mode = current_mode_;
			client_->drives[i].command.enable = true;
		}

		client_->Send(); //envoi de la commande
	}
	
	/*
	 * Initialize joint vector of joint names
	 * Parse transmissions from URDF
	 * Register interfaces
	 * name = namespace = "pantilt"
	 */
	void PanTiltHW::create(std::string urdf_string, const ros::NodeHandle& nh)
	{
		// Save the namespace in 'robot_namespace_' attribute
		// nh.getNamespace() --> return '//pantilt', so 2 first characters to remove !
		
		robot_namespace_ = nh.getNamespace();
		int nb_char_to_remove = 2;
		robot_namespace_ = robot_namespace_.substr(nb_char_to_remove,robot_namespace_.size()-nb_char_to_remove);
		
		#if TRACE_ACTIVATED
			ROS_INFO("PanTiltHW : Creating a PanTilt HW called: %s !",robot_namespace_.c_str());
		#endif
	
		// Get vector of joint names defined in the file 'pantilt_hw_interface.yaml'
		joint_names_.clear();
		nh.getParam("joints",joint_names_);

		#if TRACE_ACTIVATED
			ROS_INFO("PanTiltHW (create) : nb joints defined in 'pantilt_hw_interface.yaml' -> %zd", joint_names_.size());
		
			for (size_t i=0; i<joint_names_.size(); ++i)
				ROS_INFO("PanTiltHW (create) : name of joint %zd = %s", i, joint_names_[i].c_str());
		#endif

		// save the urdf name
		urdf_string_ = urdf_string;  // content of param : robot_namespace + "/robot_description".
		
		// Resize Vectors
		joint_position_.resize(n_joints_);
		joint_position_prev_.resize(n_joints_);
		joint_position_command_.resize(n_joints_);
		
		joint_velocity_.resize(n_joints_);
		joint_velocity_command_.resize(n_joints_);
		
		joint_effort_.resize(n_joints_);
		joint_effort_command_.resize(n_joints_);
		
		// reset all vectors with default values
		std::fill(joint_position_.begin(), joint_position_.end(), 0.0);
		std::fill(joint_position_prev_.begin(), joint_position_prev_.end(), 0.0);
		
		std::fill(joint_velocity_.begin(), joint_velocity_.end(), 0.0);
		std::fill(joint_effort_.begin(), joint_effort_.end(), 0.0);
		
		std::fill(joint_position_command_.begin(), joint_position_command_.end(), 0.0);  
		std::fill(joint_velocity_command_.begin(), joint_velocity_command_.end(), 0.0);
		std::fill(joint_effort_command_.begin(), joint_effort_command_.end(), 0.0);
		
		#if TRACE_ACTIVATED
			ROS_INFO("PanTiltHW : Parsing transmissions from the URDF...");
		#endif
		
		// GET TRANSMISSIONS THAT BELONG TO THIS LWR 4+ ARM
		if (!parseTransmissionsFromURDF_(urdf_string_))
		{
			ROS_INFO("PanTiltHW: Error parsing URDF of robot %s.",robot_namespace_.c_str());
			return;
		}
		
		#if TRACE_ACTIVATED
			ROS_INFO("PanTiltHW: Registering interfaces...");
		#endif

		const urdf::Model *const urdf_model_ptr = urdf_model_.initString(urdf_string_) ? &urdf_model_ : NULL;
		
		registerInterfaces_(urdf_model_ptr, transmissions_);
	}
	
	void PanTiltHW::traceTransmissionInfo_(const transmission_interface::TransmissionInfo& info)
	{
		ROS_INFO("PanTiltHW: Transmission name : %s", info.name_.c_str());
		ROS_INFO("PanTiltHW: Transmission nb joints : %d", (int)info.joints_.size());
		ROS_INFO("PanTiltHW: Transmission nb hardware_interfaces_ : %d", (int)info.joints_[0].hardware_interfaces_.size());
		for (int k=0; k < info.joints_[0].hardware_interfaces_.size(); k++)
			ROS_INFO("PanTiltHW: Transmission hardware_interface : %s", info.joints_[0].hardware_interfaces_[k].c_str());
	}
	
	// Register all interfaces necessary
	void PanTiltHW::registerInterfaces_(const urdf::Model *const urdf_model, 
					 std::vector<transmission_interface::TransmissionInfo> transmissions)
	{
		// Check that this transmission has one joint
		if( transmissions.empty() )
		{
			ROS_INFO("PanTiltHW: There are no transmission in this robot, all are non-driven joints ?");
			
			return;
		}
		
		// Initialize values
		for(int j=0; j < n_joints_; j++)
		{
			#if TRACE_ACTIVATED
				traceTransmissionInfo_(transmissions[j]);
			#endif
			
			// Check that this transmission has one joint
			if(transmissions[j].joints_.size() == 0)
			{
				ROS_INFO("PanTiltHW: Transmission %s has no associated joints.",transmissions[j].name_.c_str());
				continue;
			}
			else if(transmissions[j].joints_.size() > 1)
			{
				ROS_INFO("PanTiltHW: Transmission %s has more than one joint, and they can't be controlled simultaneously.",transmissions[j].name_.c_str());
				continue;
			}
			
			std::vector<std::string> joint_interfaces = transmissions[j].joints_[0].hardware_interfaces_;
			
			if( joint_interfaces.empty() )
			{
				ROS_INFO("PanTiltHW: Joint %s of transmission %s does not specify any hardware interface. You need to, otherwise the joint can't be controlled.", transmissions[j].joints_[0].name_.c_str(), transmissions[j].name_.c_str());
				continue;
			}
			
			const std::string& hardware_interface = joint_interfaces.front();
			
			#if TRACE_ACTIVATED
				ROS_INFO("PanTiltHW: Loading joint '%s', of hardware interface type '%s'", joint_names_[j].c_str(), hardware_interface.c_str());
			#endif
			
			// JOINT STATE **********************************************************************************************************************
			// Create a joint state handle for one joint and register it in a JointStateInterface.
			
			#if TRACE_ACTIVATED
				ROS_INFO("PanTiltHW: register 'hardware_interface::JointStateHandle' of joint %s", joint_names_[j].c_str());
			#endif
			
			// hardware_interface::JointStateHandle -> JointStateHandle(const std::string& name, const double* pos, const double* vel, const double* eff)
			// hardware_interface::ResourceManager -> void registerHandle(const ResourceHandle& handle)
			// hardware_interface::JointStateInterface : Hardware interface to support reading the state of an array of joints.
			joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(joint_names_[j], &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]));
			
			// EFFORT **********************************************************************************************************************
			// hardware_interface::EffortJointInterface : for commanding effort-based joints.
			
			#if TRACE_ACTIVATED
				ROS_INFO("PanTiltHW: register 'hardware_interface::EffortJointInterface' of joint %s", joint_names_[j].c_str());
			#endif
			
			hardware_interface::JointHandle joint_handle_effort;
			joint_handle_effort = hardware_interface::JointHandle(joint_state_interface_.getHandle(joint_names_[j]),
                                                       &joint_effort_command_[j]);
			joint_effort_interface_.registerHandle(joint_handle_effort);
			
			// POSITION **********************************************************************************************************************
			// hardware_interface::JointHandle : A handle used to read and command a single joint.
			// hardware_interface::PositionJointInterface : for commanding position-based joints.
			#if TRACE_ACTIVATED
				ROS_INFO("PanTiltHW: register 'hardware_interface::PositionJointInterface' of joint %s", joint_names_[j].c_str());
			#endif
			hardware_interface::JointHandle joint_handle_position;
			joint_handle_position = hardware_interface::JointHandle(joint_state_interface_.getHandle(joint_names_[j]),
													   &joint_position_command_[j]);
			joint_position_interface_.registerHandle(joint_handle_position);
			
		}
		
		// Register joints state, effort, position interfaces to 'hardware_interface::RobotHW'
		registerInterface(&joint_state_interface_);
		registerInterface(&joint_effort_interface_);
		registerInterface(&joint_position_interface_);
			
	}
	
	// Parse Transmissions from the URDF string
	bool PanTiltHW::parseTransmissionsFromURDF_(const std::string& urdf_string)
	{
		std::vector<transmission_interface::TransmissionInfo> transmissions;
		
		// Only *standard* transmission_interface are parsed
		transmission_interface::TransmissionParser::parse(urdf_string, transmissions);
		
		#if TRACE_ACTIVATED
			ROS_INFO("PanTiltHW::parseTransmissionsFromURDF -> transmissions size = %d", (int)transmissions.size());
		#endif

		// Now iterate and save only transmission from this robot
		for (int j = 0; j < n_joints_; ++j)
		{
		  std::vector<transmission_interface::TransmissionInfo>::iterator it = transmissions.begin();
		  
		  for(; it != transmissions.end(); ++it)
		  {
			if (joint_names_[j].compare(it->joints_[0].name_) == 0)
			{
			  transmissions_.push_back( *it );
			}
		  }
		}

		if( transmissions_.empty() )
		  return false;

		return true;
	}
}
