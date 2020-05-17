/*
 *  Laurent LEQUIEVRE
 *  Institut Pascal UMR6602
 *  laurent.lequievre@univ-bpclermont.fr
 * 
*/

#include <gravity_compensation_controller.h>

// For plugin
#include <pluginlib/class_list_macros.h>

#include <algorithm>


namespace kuka_lwr_controllers 
{
    GravityCompensationController::GravityCompensationController() {}
    GravityCompensationController::~GravityCompensationController() {}

    bool GravityCompensationController::init(hardware_interface::KUKAJointInterface *robot, ros::NodeHandle &n)
    {
		#if TRACE_ACTIVATED
			ROS_INFO("GravityCompensationController: Start init of robot %s !",robot_namespace_.c_str());
		#endif
		
		robot_namespace_ = n.getNamespace();
			
        if( !(KinematicChainControllerBase<hardware_interface::KUKAJointInterface>::init(robot, n)) )
        {
            ROS_ERROR("GravityCompensationController: Couldn't initilize GravityCompensationController controller of robot %s!",robot_namespace_.c_str());
            return false;
        }
        
        sub_stiffness_damping_ = n.subscribe("setStiffnessDamping"		,1, &GravityCompensationController::setStiffnessDamping	, this); 
        sub_command_ = n.subscribe("command", 1, &GravityCompensationController::commandCB, this);
        
        stiff_.resize(joint_handles_.size());
		damp_.resize(joint_handles_.size());
		q_des_.resize(joint_handles_.size());

		for (std::size_t i=0; i<joint_handles_.size(); i++)
		{
			stiff_(i) = 200.0;
			damp_(i) = 0.7;
			q_des_(i) = 0.0;
		}

		return true;
	}
	
	void GravityCompensationController::starting(const ros::Time& time)
    {
        
		#if TRACE_ACTIVATED
			ROS_INFO("GravityCompensationController: Starting of robot %s !",robot_namespace_.c_str());
		#endif
    }
    
    void GravityCompensationController::stopping(const ros::Time& time)
	{
		#if TRACE_ACTIVATED
			ROS_INFO_NAMED("GravityCompensationController: Stopping of robot %s !",robot_namespace_.c_str());
		#endif
		
	}
	
    void GravityCompensationController::update(const ros::Time& time, const ros::Duration& period)
    {	
		
		// update the commanded position to the actual, so that the robot doesn't 
        // go back at full speed to the last commanded position when the stiffness 
        // is raised again
        for(size_t i=0; i<joint_handles_.size(); i++) 
        {
			joint_handles_[i].setCommandPosition(joint_handles_[i].getPosition());
			//joint_handles_[i].setCommandPosition(q_des_(i));
            joint_handles_[i].setCommandTorque(0.0); // Set a value of torque to 0.0 for each joint.
            joint_handles_[i].setCommandStiffness(stiff_(i));
			joint_handles_[i].setCommandDamping(damp_(i));
        }
        
        joint_handles_[2].setCommandTorque(0.5); // Set a value of torque to 0.0 for each joint.
        
	}
	
	void GravityCompensationController::setStiffnessDamping(const kuka_lwr_controllers::StiffnessDamping::ConstPtr & msg)
	{
		
		
		if(msg->stiffness.data.size()!=joint_handles_.size()  )
		{ 
			ROS_ERROR_STREAM("TorqueBasedPositionController: stiffness Dimension (of robot " << robot_namespace_.c_str() << ") of command (" << msg->stiffness.data.size() << ") does not match number of joints (" << joint_handles_.size() << ")! Not executing!");
			return; 
		}
			if(msg->damping.data.size()!=joint_handles_.size()  )
		{ 
			ROS_ERROR_STREAM("TorqueBasedPositionController: Damping Dimension (of robot " << robot_namespace_.c_str() << ") of command (" << msg->damping.data.size() << ") does not match number of joints (" << joint_handles_.size() << ")! Not executing!");
			return; 
		}
		
		for (std::size_t i=0; i<msg->stiffness.data.size(); i++)
		{
			stiff_(i) = (double)msg->stiffness.data[i];
			damp_(i) = (double)msg->damping.data[i];
		}
	}
	
	
	void GravityCompensationController::commandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
    {
		#if TRACE_Torque_Based_Position_ACTIVATED
			ROS_INFO("TorqueBasedPositionController: Start commandCB of robot %s!",robot_namespace_.c_str());
		#endif

		if(msg->data.size()!=joint_handles_.size())
		{ 
			ROS_ERROR_STREAM("TorqueBasedPositionController: Dimension (of robot " << robot_namespace_.c_str() << ") of command (" << msg->data.size() << ") does not match number of joints (" << joint_handles_.size() << ")! Not executing!");
			return; 
		}
		
		for (std::size_t i=0; i<msg->data.size(); i++)
		{
			q_des_(i) = (double)msg->data[i];
		}
		
		ROS_INFO("-> qdes values : %f, %f, %f, %f, %f, %f, %f!",q_des_(0), q_des_(1), q_des_(2), q_des_(3), q_des_(4), q_des_(5), q_des_(6));
		
	}
	
}

PLUGINLIB_EXPORT_CLASS(kuka_lwr_controllers::GravityCompensationController, controller_interface::ControllerBase)
