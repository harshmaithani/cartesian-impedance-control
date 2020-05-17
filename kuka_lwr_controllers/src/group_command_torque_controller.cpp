/*
 *  Laurent LEQUIEVRE
 *  Institut Pascal UMR6602
 *  laurent.lequievre@univ-bpclermont.fr
 * 
*/

#include <group_command_torque_controller.h>

// For plugin
#include <pluginlib/class_list_macros.h>

#include <algorithm>


namespace kuka_lwr_controllers 
{
    GroupCommandTorqueController::GroupCommandTorqueController() {}
    GroupCommandTorqueController::~GroupCommandTorqueController() {}

    bool GroupCommandTorqueController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
    {
		#if TRACE_GroupCommandTorqueController_ACTIVATED
			ROS_INFO("GroupCommandTorqueController: Start init of robot %s !",robot_namespace_.c_str());
		#endif
		
		
		robot_namespace_ = n.getNamespace();
			
        if( !(KinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(robot, n)) )
        {
            ROS_ERROR("GroupCommandTorqueController: Couldn't initilize GroupCommandController controller of robot %s!",robot_namespace_.c_str());
            return false;
        }
        
        commands_buffer_.resize(joint_handles_.size());
		goal_factor_.clear();
        goal_buffer_.clear();
        
        // get joint positions, velocity
        for (int i=0; i < joint_handles_.size(); i++)
        {
            joint_msr_states_.q(i) = joint_handles_[i].getPosition();
            joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
            torque_msr_states_(i) = joint_handles_[i].getEffort();
            
            // set initial value of desired state
            joint_des_states_.q(i) = joint_msr_states_.q(i);
            torque_des_states_(i) = torque_msr_states(i); 
            
            // set initial desired command values 
            commands_buffer_[i] = 0.0;
        }
        
		sub_command_ = nh_.subscribe("command", 1, &GroupCommandTorqueController::commandCB, this);

		cmd_flag_ = 0;  // set this flag to 0 to not to run the update method

		return true;
	}
	
	void GroupCommandTorqueController::starting(const ros::Time& time)
    {
		#if TRACE_GroupCommandTorqueController_ACTIVATED
			ROS_INFO("GroupCommandTorqueController: Starting of robot %s !",robot_namespace_.c_str());
		#endif
		
		cmd_flag_ = 0;  // set this flag to 0 to not to run the update method
    }
    
    void GroupCommandTorqueController::stopping(const ros::Time& time)
	{
		#if TRACE_GroupCommandTorqueController_ACTIVATED
			ROS_INFO_NAMED("GroupCommandTorqueController: Stopping of robot %s !",robot_namespace_.c_str());
		#endif
		
		cmd_flag_ = 0;  // set this flag to 0 to not to run the update method
	}
	
	bool GroupCommandTorqueController::isGoalChecked(int value)
	{
		if (goal_buffer_.empty()) return false;
		
		if (std::find(goal_buffer_.begin(), goal_buffer_.end(), value) != goal_buffer_.end())
			return true; 
		else 
			return false;
	}
	
    void GroupCommandTorqueController::update(const ros::Time& time, const ros::Duration& period)
    {	
		if (cmd_flag_)
		{	
			for (int i=0; i < joint_handles_.size(); i++)
			{
				if (!isGoalChecked(i))
				{
					joint_msr_states_.q(i) = joint_handles_[i].getPosition();	// get current position
					torque_msr_states_(i) = joint_handles_[i].getEffort();	// get current torque
					joint_des_states_.q(i) = joint_msr_states_.q(i) + (goal_factor_[i]*0.005);  // set desired torque
					
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
							if ((goal_factor_[i]==1 && joint_des_states_.q(i) >= commands_buffer_[i]) || (goal_factor_[i]==-1 && joint_des_states_.q(i) <= commands_buffer_[i]))
							{
								joint_des_states_.q(i) = commands_buffer_[i];
								ROS_INFO("GroupCommandTorqueController: GOAL FOR robot %s: j[%d]=%f, commands_buffer_[%d]=%f, name=%s",robot_namespace_.c_str(),i,joint_des_states_.q(i),i,commands_buffer_[i],joint_handles_[i].getName().c_str());
								goal_buffer_.push_back(i);
							}	
						}
					}	
				}
					
			}
			
		}  
		
		// Verify if all joint values desired are reached
		if (goal_buffer_.size()==joint_handles_.size() && cmd_flag_) 
		{ 
			cmd_flag_=0; // all the joint values derired are reached, so set this flag to 0 to not to run the update method
			ROS_INFO("GroupCommandTorqueController: GOAL of robot %s !!!!!!!!!!",robot_namespace_.c_str()); 
		}
		
		#if TRACE_GroupCommandTorqueController_ACTIVATED
			ROS_INFO("GroupCommandTorqueController: of robot %s -> j0=%f, j1=%f, j2=%f, j3=%f, j4=%f, j5=%f, j6=%f",robot_namespace_.c_str(),joint_des_states_.q(0), joint_des_states_.q(1), joint_des_states_.q(2), joint_des_states_.q(3), joint_des_states_.q(4), joint_des_states_.q(5), joint_des_states_.q(6));
		#endif
		
		 // set control command for joints
        for (int i = 0; i < joint_handles_.size(); i++)
			joint_handles_[i].setCommand(joint_des_states_.q(i));
	}
	
	void GroupCommandTorqueController::commandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
    {
		 #if TRACE_GroupCommandTorqueController_ACTIVATED
			ROS_INFO("GroupCommandTorqueController: Start commandCB of robot %s!",robot_namespace_.c_str());
		 #endif
		 
		
	     if(msg->data.size()!=joint_handles_.size())
	     { 
	       ROS_ERROR_STREAM("GroupCommandTorqueController: Dimension (of robot " << robot_namespace_.c_str() << ") of command (" << msg->data.size() << ") does not match number of joints (" << joint_handles_.size() << ")! Not executing!");
	       cmd_flag_ = 0;
	       return; 
	     }
	     
	     // clear buffers for initial values
	     commands_buffer_.clear();
	     goal_buffer_.clear();
	     goal_factor_.clear();
	     
	     for (size_t i=0; i<joint_handles_.size(); ++i)
	     {
			commands_buffer_.push_back(msg->data[i]);
			
			// set the factor of increment/decrement depending of the current joint value and the desired one
			if (joint_handles_[i].getPosition() > msg->data[i]) 
				goal_factor_.push_back(-1);
			else 
				goal_factor_.push_back(1);
		 }
		 
	     cmd_flag_ = 1; // set this flag to 1 to run the update method
	     
	     #if TRACE_GroupCommandTorqueController_ACTIVATED
			ROS_INFO("GroupCommandTorqueController: Finish commandCB of robot %s !",robot_namespace_.c_str());
			ROS_INFO("GroupCommandTorqueController: of robot %s -> j0=%f, j1=%f, j2=%f, j3=%f, j4=%f, j5=%f, j6=%f",robot_namespace_.c_str(),commands_buffer_[0],commands_buffer_[1],commands_buffer_[2],commands_buffer_[3],commands_buffer_[4],commands_buffer_[5],commands_buffer_[6]);

		 #endif
		 
	}
	
}

PLUGINLIB_EXPORT_CLASS(kuka_lwr_controllers::GroupCommandTorqueController, controller_interface::ControllerBase)
