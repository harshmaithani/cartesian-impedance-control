/*
 *  Laurent LEQUIEVRE
 *  Institut Pascal UMR6602
 *  laurent.lequievre@univ-bpclermont.fr
 * 
*/

#include <pan_tilt_position.h>

// For plugin
#include <pluginlib/class_list_macros.h>

namespace pan_tilt_controllers 
{
	PanTiltPosition::PanTiltPosition() 
	{
		#if TRACE_ACTIVATED
			ROS_INFO("PanTiltPosition: constructor !");
		#endif
	}
	
	PanTiltPosition::~PanTiltPosition() 
	{
		#if TRACE_ACTIVATED
			ROS_INFO("PanTiltPosition: destructor !");
		#endif
	}
	
	
	std::vector<std::string> PanTiltPosition::getStrings_(const ros::NodeHandle& nh, const std::string& param_name)
	{
	  using namespace XmlRpc;
	  XmlRpcValue xml_array;
	  if (!nh.getParam(param_name, xml_array))
	  {
		ROS_ERROR_STREAM("PanTiltPosition:: Could not find '" << param_name << "' parameter (namespace: " << nh.getNamespace() << ").");
		return std::vector<std::string>();
	  }
	  if (xml_array.getType() != XmlRpcValue::TypeArray)
	  {
		ROS_ERROR_STREAM("PanTiltPosition:: The '" << param_name << "' parameter is not an array (namespace: " <<
						 nh.getNamespace() << ").");
		return std::vector<std::string>();
	  }

	  std::vector<std::string> out;
	  for (int i = 0; i < xml_array.size(); ++i)
	  {
		if (xml_array[i].getType() != XmlRpcValue::TypeString)
		{
		  ROS_ERROR_STREAM("PanTiltPosition:: The '" << param_name << "' parameter contains a non-string element (namespace: " <<
						   nh.getNamespace() << ").");
		  return std::vector<std::string>();
		}
		out.push_back(static_cast<std::string>(xml_array[i]));
	  }
	  return out;
	}
	
	
	bool PanTiltPosition::isGoalChecked_(int value)
	{
		if (goal_buffer_.empty()) return false;
		
		if (std::find(goal_buffer_.begin(), goal_buffer_.end(), value) != goal_buffer_.end())
			return true; 
		else 
			return false;
	}

	bool PanTiltPosition::init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n)
    {
		#if TRACE_ACTIVATED
			ROS_INFO("PanTiltPosition: Start init of robot !");
		#endif
		
		nh_ = n;
		robot_namespace_ = nh_.getNamespace();
		
		#if TRACE_ACTIVATED
			ROS_INFO("PanTiltPosition -> PanTiltPosition::init ros::NodeHandle namespace = %s", robot_namespace_.c_str());
		#endif
	
		// the 'getNamespace()' function return '/pantilt/pan_tilt_position_controller'
		// There is a parameter named '/pantilt/pan_tilt_position_controller/joints' 
		// This parameter is defined by the 'joints' field defined in the 'pantilt_control.yaml' file.
		// Set vector of joint names, by reading the 'joints' parameter defined in the 'pantilt_control.yaml' file
		joint_names_.clear();
		joint_names_ = getStrings_(nh_,"joints");
		
		#if TRACE_ACTIVATED
			ROS_INFO("PanTiltPosition::init : nb joints -> %zd", joint_names_.size());
		
			for (size_t i=0; i<joint_names_.size(); ++i)
				ROS_INFO("PanTiltPosition::init : name of joint %zd = %s", i, joint_names_[i].c_str());
		#endif

		// get URDF and name of root and tip from the parameter server
		std::string robot_description;

		if (!ros::param::search(robot_namespace_,"robot_description", robot_description))
		{
		    ROS_ERROR_STREAM("PanTiltPosition: No robot description (URDF) found on parameter server (" << robot_namespace_ << "/robot_description)");
		    return false;
		}
		
		// Construct an URDF model from the xml string
		std::string xml_string;

		if (n.hasParam(robot_description))
		    n.getParam(robot_description.c_str(), xml_string);
		else
		{
		    ROS_ERROR("PanTiltPosition -> (init) Parameter %s not set, shutting down node...",robot_description.c_str());
		    n.shutdown();
		    return false;
		}

		if (xml_string.size() == 0)
		{
		    ROS_ERROR("PanTiltPosition -> (init) Unable to load robot model from parameter %s",robot_description.c_str());
		    n.shutdown();
		    return false;
		}
		
		// Get urdf model out of robot_description
		urdf::Model model;
		if (!model.initString(xml_string))
		{
		    ROS_ERROR("PanTiltPosition -> (init) Failed to parse urdf file");
		    n.shutdown();
		    return false;
		}
		
		#if TRACE_ACTIVATED
			ROS_INFO("PanTiltPosition -> (init) Successfully parsed urdf file");
		#endif
		
		// Resize joints limits arrays with number of joints : n_joints_
		joint_limits_.min.resize(n_joints_);
		joint_limits_.max.resize(n_joints_);
		joint_limits_.center.resize(n_joints_);
		
		boost::shared_ptr<const urdf::Joint> a_joint;  // A Joint defined in a URDF structure
		
		for (int i = 0; i < n_joints_; i++)
		{
		    a_joint = model.getJoint(joint_names_[i]);
		    
			#if TRACE_ACTIVATED
				ROS_INFO("PanTiltPosition -> (init) Getting limits for joint: %s", a_joint->name.c_str());
			#endif

		    joint_limits_.min(i) = a_joint->limits->lower;
		    joint_limits_.max(i) = a_joint->limits->upper;
		    joint_limits_.center(i) = (joint_limits_.min(i) + joint_limits_.max(i))/2;
		    
		    joint_handles_.push_back(robot->getHandle(joint_names_[i]));
		}
		
		#if TRACE_ACTIVATED
			ROS_INFO("PanTiltPosition -> (init) Number of joints in handle = %lu", joint_handles_.size() );
		#endif

		joint_msr_states_.resize(n_joints_);
		joint_des_states_.resize(n_joints_);
		
		commands_buffer_.resize(joint_handles_.size());
		goal_factor_.clear();
        goal_buffer_.clear();
        
        // set joint positions, velocity inital values
        for (int i=0; i < joint_handles_.size(); i++)
        {
            joint_msr_states_.q(i) = joint_handles_[i].getPosition();
            joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
            
            // set initial value of desired state
            joint_des_states_.q(i) = joint_msr_states_.q(i);
            
            // set initial desired command values 
            commands_buffer_[i] = 0.0;
        }
        
		sub_command_ = nh_.subscribe("command", 1, &PanTiltPosition::commandCB_, this);

		cmd_flag_ = 0;  // set this flag to 0 to not to run the update method
		
		#if TRACE_ACTIVATED
			ROS_INFO("PanTiltPosition -> (init) Finish PanTiltPosition::init");
		#endif
		
		return true;
	}
	
	void PanTiltPosition::commandCB_(const std_msgs::Float64MultiArrayConstPtr& msg)
    {
		 #if TRACE_ACTIVATED
			ROS_INFO("PanTiltPosition: Start commandCB of robot %s!",robot_namespace_.c_str());
		 #endif
		
	     if(msg->data.size()!=joint_handles_.size())
	     { 
	       ROS_ERROR_STREAM("PanTiltPosition: Dimension (of robot " << robot_namespace_.c_str() << ") of command (" << msg->data.size() << ") does not match number of joints (" << joint_handles_.size() << ")! Not executing!");
	       cmd_flag_ = 0;
	       return; 
	     }
	     
	     // clear buffers for initial values
	     commands_buffer_.clear();
	     goal_buffer_.clear();
	     goal_factor_.clear();
	     
	     for (size_t i=0; i<joint_handles_.size(); ++i)
	     {
			std::cout << "msg->data[" << i << "]=" << msg->data[i] << std::endl;
			commands_buffer_.push_back(msg->data[i]);
			
			// set the factor of increment/decrement depending of the current joint value and the desired one
			if (joint_handles_[i].getPosition() > msg->data[i]) 
				goal_factor_.push_back(-1);
			else 
				goal_factor_.push_back(1);
		 }
	     
	     cmd_flag_ = 1; // set this flag to 1 to run the update method
	     
	     #if TRACE_ACTIVATED
			ROS_INFO("PanTiltPosition: Finish commandCB of robot %s !",robot_namespace_.c_str());
		 #endif
	}
	
	void PanTiltPosition::starting(const ros::Time& time)
    {
		#if TRACE_ACTIVATED
			ROS_INFO("PanTiltPosition: Starting of robot %s !",robot_namespace_.c_str());
		#endif
    }
    
    void PanTiltPosition::stopping(const ros::Time& time)
	{
		#if TRACE_ACTIVATED
			ROS_INFO("PanTiltPosition: Stopping of robot %s !",robot_namespace_.c_str());
		#endif
	}
	
	void PanTiltPosition::update(const ros::Time& time, const ros::Duration& period)
	{
		#if TRACE_ACTIVATED
			ROS_INFO("PanTiltPosition: Updating of robot %s !",robot_namespace_.c_str());
		#endif
		
		if (cmd_flag_)
		{	
			for (int i=0; i < joint_handles_.size(); i++)
			{
				if (!isGoalChecked_(i))
				{
					joint_msr_states_.q(i) = joint_handles_[i].getPosition();	// get current position
					joint_des_states_.q(i) = joint_msr_states_.q(i) + (goal_factor_[i]*0.01);  // set desired position
					
					// Verify the min joint limit
					if (joint_des_states_.q(i) <= joint_limits_.min(i))
					{
							joint_des_states_.q(i) = joint_limits_.min(i);
							goal_buffer_.push_back(i);
					}
					else
					{	
						// Verify the max joint limit
						if (joint_des_states_.q(i) >= joint_limits_.max(i))
						{
								joint_des_states_.q(i) = joint_limits_.max(i);
								goal_buffer_.push_back(i);
						}
						else
						{
							// Verify if the joint desired value is reached
							if ((goal_factor_[i]==1 && joint_des_states_.q(i) >= commands_buffer_[i]) 
							     || (goal_factor_[i]==-1 && joint_des_states_.q(i) <= commands_buffer_[i]))
							{
								joint_des_states_.q(i) = commands_buffer_[i];
								ROS_INFO("PanTiltPosition: GOAL FOR robot %s: j[%d]=%f, commands_buffer_[%d]=%f, name=%s",robot_namespace_.c_str(),i,joint_des_states_.q(i),i,commands_buffer_[i],joint_handles_[i].getName().c_str());
								goal_buffer_.push_back(i);
							}	
						}
					}
				}
			}
		
			// set control command for joints
			for (int i = 0; i < joint_handles_.size(); i++)
				joint_handles_[i].setCommand(joint_des_states_.q(i));
				
			if (first_trace)
			{
				for (int i = 0; i < joint_handles_.size(); i++)
				{
					std::cout << "initial value of joint_des_states_.q(" << i << ")=" << joint_des_states_.q(i) << std::endl;
					std::cout << "initial value of joint_msr_states_.q(" << i << ")=" << joint_msr_states_.q(i) << std::endl;
					first_trace = false;	
				}
			}
			
			// Verify if all joint values desired are reached
			if (goal_buffer_.size()==joint_handles_.size()) 
			{ 
				cmd_flag_=0; // all the joint values derired are reached, so set this flag to 0 to not to run the update method
				ROS_INFO("PanTiltPosition: The GOAL of robot %s is reached !!!!!!!!!!",robot_namespace_.c_str()); 
			}

		}
	}
}

PLUGINLIB_EXPORT_CLASS(pan_tilt_controllers::PanTiltPosition, controller_interface::ControllerBase)
