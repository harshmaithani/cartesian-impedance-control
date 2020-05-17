/*
 *  Laurent LEQUIEVRE
 *  Institut Pascal UMR6602
 *  laurent.lequievre@univ-bpclermont.fr
 * 
*/

#include <group_command_controller_fri.h>

// For plugin
#include <pluginlib/class_list_macros.h>

#include <algorithm>


namespace kuka_lwr_controllers 
{
    GroupCommandControllerFRI::GroupCommandControllerFRI() {}
    GroupCommandControllerFRI::~GroupCommandControllerFRI() 
    {
			delete RML_;
			delete IP_;
			delete OP_;	
    }

    bool GroupCommandControllerFRI::init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n)
    {
		#if TRACE_GroupCommandController_ACTIVATED
			ROS_INFO("GroupCommandControllerFRI: Start init of robot %s !",robot_namespace_.c_str());
		#endif
	
		robot_namespace_ = n.getNamespace();
			
        if( !(KinematicChainControllerBase<hardware_interface::PositionJointInterface>::init(robot, n)) )
        {
            ROS_ERROR("GroupCommandControllerFRI: Couldn't initilize GroupCommandController controller of robot %s!",robot_namespace_.c_str());
            return false;
        }
        
        cycleTime_ = 0.002;
        
        RML_ = new TypeIRML(joint_handles_.size(),cycleTime_);
		IP_ = new TypeIRMLInputParameters(joint_handles_.size());
		OP_ = new TypeIRMLOutputParameters(joint_handles_.size());

		sub_command_ = nh_.subscribe("command", 1, &GroupCommandControllerFRI::commandCB, this);
		sub_max_velovity_ = nh_.subscribe("setMaxVelocity", 1, &GroupCommandControllerFRI::setMaxVelocityCB, this);
		
		srv_get_velocity_ = n.advertiseService("get_joint_velocity", &GroupCommandControllerFRI::getCurrentJointVelocity, this);

		cmd_flag_ = 0;  // set this flag to 0 to not run the update method
		
		v_max_acc_.resize(joint_handles_.size());
		
		// The max velocity of kuka lwr4 joints is
		/*
		Joint0 = 110;
		Joint1 = 110;
		Joint2 = 128;
		Joint3 = 128;
		Joint4 = 204;
		Joint5 = 184;
		Joint6 = 184;
		 */
		
		v_max_acc_[0] = 5;
		v_max_acc_[1] = 5;
		v_max_acc_[2] = 5;
		v_max_acc_[3] = 5;
		v_max_acc_[4] = 5;
		v_max_acc_[5] = 5;
		v_max_acc_[6] = 5;
		
		return true;
	}
	
    void GroupCommandControllerFRI::starting(const ros::Time& time)
    {
		#if TRACE_GroupCommandController_ACTIVATED
			ROS_INFO("GroupCommandControllerFRI: Starting of robot %s !",robot_namespace_.c_str());
		#endif
		
		cmd_flag_ = 0;  // set this flag to 0 to not run the update method
    }
    
    void GroupCommandControllerFRI::stopping(const ros::Time& time)
    {
		#if TRACE_GroupCommandController_ACTIVATED
			ROS_INFO_NAMED("GroupCommandControllerFRI: Stopping of robot %s !",robot_namespace_.c_str());
		#endif
		
		cmd_flag_ = 0;  // set this flag to 0 to not run the update method
    }
	
    void GroupCommandControllerFRI::update(const ros::Time& time, const ros::Duration& period)
    {	
		if (cmd_flag_)
		{
			if (resultValue_ != TypeIRML::RML_FINAL_STATE_REACHED)
			{
				resultValue_ = RML_->GetNextMotionState_Position(*IP_,OP_);
				
				if ((resultValue_ != TypeIRML::RML_WORKING) && (resultValue_ != TypeIRML::RML_FINAL_STATE_REACHED))
				{
					ROS_INFO("GroupCommandControllerFRI::update : ERROR during trajectory generation err n°%d",resultValue_);
				}
				
				// set control command for joints
				for (int i = 0; i < joint_handles_.size(); i++)
					joint_handles_[i].setCommand(RAD((double)(OP_->NewPosition->VecData[i])));
				
				*(IP_->CurrentPosition) = *(OP_->NewPosition);
				*(IP_->CurrentVelocity) = *(OP_->NewVelocity);
				
				#if TRACE_GroupCommandController_ACTIVATED
					ROS_INFO("GroupCommandControllerFRI: resultValue_ = %d", resultValue_);
					ROS_INFO("GroupCommandControllerFRI: of robot %s -> j0=%f, j1=%f, j2=%f, j3=%f, j4=%f, j5=%f, j6=%f",robot_namespace_.c_str(),joint_des_states_.q(0), joint_des_states_.q(1), joint_des_states_.q(2), joint_des_states_.q(3), joint_des_states_.q(4), joint_des_states_.q(5), joint_des_states_.q(6));
				#endif
			}
			else
			{
				cmd_flag_=0; // all the joint values derired are reached, so set this flag to 0 to not run the update method
				ROS_INFO("GroupCommandControllerFRI: GOAL of robot %s !!!!!!!!!!",robot_namespace_.c_str());
			}
		}
	}
	
	void GroupCommandControllerFRI::setMaxVelocityCB(const std_msgs::Float64MultiArrayConstPtr& msg)
	{
		#if TRACE_GroupCommandController_ACTIVATED
			ROS_INFO("GroupCommandControllerFRI: start setMaxVelocityCB of robot %s!",robot_namespace_.c_str());
		#endif
		
		if(msg->data.size()!=joint_handles_.size())
		{ 
			ROS_ERROR_STREAM("GroupCommandControllerFRI: Dimension (of robot " << robot_namespace_.c_str() << ") of set max velocity command (" << msg->data.size() << ") does not match number of joints (" << joint_handles_.size() << ")! Not executing!");
			return; 
		}
		
		for (size_t i=0; i<joint_handles_.size(); ++i)
		{
			v_max_acc_[i] = (double)DEG(msg->data[i]);
			ROS_INFO("GroupCommandControllerFRI::setMaxVelocityCB Joint[%zu] = %f rad , %f ° !",i, msg->data[i], v_max_acc_[i]);
		}
		
		
		
	}
	
	void GroupCommandControllerFRI::commandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
    	{
		#if TRACE_GroupCommandController_ACTIVATED
			ROS_INFO("GroupCommandControllerFRI: Start commandCB of robot %s!",robot_namespace_.c_str());
		#endif


		if(msg->data.size()!=joint_handles_.size())
		{ 
			ROS_ERROR_STREAM("GroupCommandControllerFRI: Dimension (of robot " << robot_namespace_.c_str() << ") of command (" << msg->data.size() << ") does not match number of joints (" << joint_handles_.size() << ")! Not executing!");
			cmd_flag_ = 0;
			return; 
		}

		for (size_t i=0; i<joint_handles_.size(); ++i)
		{
			IP_->CurrentPosition->VecData[i] = (double)DEG(joint_handles_[i].getPosition());  // set current position (transfrom to degrees) with current position of joint handles
			IP_->TargetPosition->VecData[i]	= (double)DEG(msg->data[i]); // set desired position (get it from msg data of topic)
			//IP_->MaxVelocity->VecData[i] = (double)5.0;
			//IP_->MaxAcceleration->VecData[i] = (double)20.0;
			IP_->MaxVelocity->VecData[i] = v_max_acc_[i];
			IP_->MaxAcceleration->VecData[i] = 0.5*v_max_acc_[i];
			IP_->SelectionVector->VecData[i] = true;
			
			ROS_INFO("GroupCommandControllerFRI::commandCB current Pos Joint[%zu] = %f rad , %f ° !",i, joint_handles_[i].getPosition(), (double)DEG(joint_handles_[i].getPosition()));
			ROS_INFO("GroupCommandControllerFRI::commandCB Joint[%zu] = %f rad , %f ° !",i, msg->data[i], (double)DEG(msg->data[i]));
			
		}

		resultValue_ = TypeIRML::RML_WORKING;

		cmd_flag_ = 1; // set this flag to 1 to run the update method

		#if TRACE_GroupCommandController_ACTIVATED
			ROS_INFO("GroupCommandControllerFRI: Finish commandCB of robot %s !",robot_namespace_.c_str());
			ROS_INFO("GroupCommandControllerFRI: of robot %s -> j0=%f, j1=%f, j2=%f, j3=%f, j4=%f, j5=%f, j6=%f",robot_namespace_.c_str(),msg->data[0],msg->data[1],msg->data[2],msg->data[3],msg->data[4],msg->data[5],msg->data[6]);
		#endif
		 
	}
	
	
	bool GroupCommandControllerFRI::getCurrentJointVelocity(kuka_lwr_controllers::GetJointVelocity::Request& req, kuka_lwr_controllers::GetJointVelocity::Response& resp)
	{
		
		resp.arrayVelocities.data.resize(joint_handles_.size());
		
		for (size_t i=0; i<joint_handles_.size(); ++i)
		{
			resp.arrayVelocities.data[i] = RAD(v_max_acc_[i]);
		}
		
		return true;
	}
	
}

PLUGINLIB_EXPORT_CLASS(kuka_lwr_controllers::GroupCommandControllerFRI, controller_interface::ControllerBase)
