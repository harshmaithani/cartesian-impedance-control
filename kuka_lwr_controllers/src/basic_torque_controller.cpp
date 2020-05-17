#include <pluginlib/class_list_macros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <math.h>

#include <basic_torque_controller.h>

namespace kuka_lwr_controllers
{
	
	BasicTorqueController::BasicTorqueController() {}
	BasicTorqueController::~BasicTorqueController() {}
	
	bool BasicTorqueController::init(hardware_interface::KUKAJointInterface *robot, ros::NodeHandle &n)
	{
		robot_namespace_ = n.getNamespace();
		
		if( !(KinematicChainControllerBase<hardware_interface::KUKAJointInterface>::init(robot, n)) )
		{
            ROS_ERROR("BasicTorqueController: Couldn't initilize BasicTorqueController controller of robot %s!",robot_namespace_.c_str());
            return false;
        }
        
        //initializes the KDL solvers
        jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
        
        // Subscribe to command
        sub_command_ = n.subscribe("command", 1,&BasicTorqueController::command, this);
        
        // Subscribe to force sensor topic
        sub_ft_ = n.subscribe("sensor_readings"	,1, &BasicTorqueController::ft_readingsCB, this);
        
        //resizing and init the vectors, jacobian used
		M_.resize(kdl_chain_.getNrOfJoints());
		SetToZero(M_);
		C_.resize(kdl_chain_.getNrOfJoints());
		SetToZero(C_);
		G_.resize(kdl_chain_.getNrOfJoints());
		SetToZero(G_);
		Jkdl_.resize(kdl_chain_.getNrOfJoints());
		SetToZero(Jkdl_);
		
		q_des_.resize(kdl_chain_.getNrOfJoints());
		SetToZero(q_des_);
		
		tau_cmd_.resize(kdl_chain_.getNrOfJoints());
		SetToZero(tau_cmd_);
		 
		return true;
	}
	
	
	void BasicTorqueController::stopping(const ros::Time& time)
	{
		#if TRACE_Basic_Torque_Controller_ACTIVATED
			ROS_INFO("BasicTorqueController: Stopping of robot %s !",robot_namespace_.c_str());
		#endif
		
		cmd_flag_ = 0;  // set this flag to 0 to not run the update method
	}
	
	
	void BasicTorqueController::starting(const ros::Time& time)
	{
		#if TRACE_Basic_Torque_Controller_ACTIVATED
			ROS_INFO("BasicTorqueController: Starting of robot %s !",robot_namespace_.c_str());
		#endif
		
		// get joint positions
		// KDL::JntArrayAcc -> joint_msr_states_, joint_des_states_
  		for(size_t i=0; i<joint_handles_.size(); i++) 
  		{
    		joint_msr_states_.q(i) = joint_handles_[i].getPosition(); // get current position mes
    		joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity(); // get current velocity mes
    		joint_msr_states_.qdotdot(i) = joint_handles_[i].getAcceleration(); // get current acc mes
    		joint_des_states_.q(i) = joint_msr_states_.q(i); // set joint desired
    	}
    	
    	cmd_flag_ = 0;
    	
		#if TRACE_Basic_Torque_Controller_ACTIVATED
			ROS_INFO(" Number of joints in handle = %lu", joint_handles_.size() );
		#endif
	}
	
	
	void BasicTorqueController::update(const ros::Time& time, const ros::Duration& period)
	{
    	// joint_msr_states_(KDL::JntArrayAcc) -> Joint measurement
    	// joint positions : q
    	// joint velocity : qdot
    	// joint acc : qdotdot
    	// joint_handles_[i] -> Joint number i
		for(size_t i=0; i<joint_handles_.size(); i++) 
		{
			joint_msr_states_.q(i) = joint_handles_[i].getPosition();  // get current position
			joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity(); // get current velocity
			joint_msr_states_.qdotdot(i) = joint_handles_[i].getAcceleration(); // get current acc
		}
		
		if (cmd_flag_)
		{ 
			jnt_to_jac_solver_->JntToJac(joint_msr_states_.q, Jkdl_);						// computing Jacobian with KDL
			
			// Examples of calculate : Inertia matrix, Coriolis torques, Gravity torques
			id_solver_->JntToMass(joint_msr_states_.q, M_);									// computing Inertia matrix		
			id_solver_->JntToCoriolis(joint_msr_states_.q, joint_msr_states_.qdot, C_);		// computing Coriolis torques	
			id_solver_->JntToGravity(joint_msr_states_.q, G_);								// computing Gravity torques
			
			
			// Send Torque command
			for(size_t i=0; i<joint_handles_.size(); i++)
			{
				joint_handles_[i].setCommandTorque(tau_cmd_(i));
				joint_handles_[i].setCommandPosition(joint_handles_[i].getPosition());
			}
		}
		
	}
	
	void BasicTorqueController::command(const std_msgs::Float64MultiArray::ConstPtr &msg)
	{
		
		#if TRACE_Basic_Torque_Controller_ACTIVATED
			ROS_INFO("BasicTorqueController: Start commandCB of robot %s!",robot_namespace_.c_str());
		#endif

		if(msg->data.size()!=joint_handles_.size())
		{ 
			ROS_ERROR_STREAM("BasicTorqueController: Dimension (of robot " << robot_namespace_.c_str() << ") of command (" << msg->data.size() << ") does not match number of joints (" << joint_handles_.size() << ")! Not executing!");
			cmd_flag_ = 0;
			return; 
		}
		
		q_des_.resize(msg->data.size());
		for (std::size_t i=0; i<msg->data.size(); i++)
		{
			q_des_(i) = (double)msg->data[i];
		}
		
		cmd_flag_ = 1;
		
	}
	
	
	void BasicTorqueController::ft_readingsCB(const geometry_msgs::WrenchStamped& msg)
    {
		#if TRACE_Basic_Torque_Controller_ACTIVATED
			ROS_INFO("BasicTorqueController: Start ft_readingsCB of robot %s!",robot_namespace_.c_str());
		#endif
		FT_sensor_[0] = msg.wrench.force.x ;
		FT_sensor_[1] = msg.wrench.force.y ;
		FT_sensor_[2] = msg.wrench.force.z ;
		FT_sensor_[3] = msg.wrench.torque.x ;
		FT_sensor_[4] = msg.wrench.torque.y ;
		FT_sensor_[5] = msg.wrench.torque.z ;

	}
	
}

PLUGINLIB_EXPORT_CLASS(kuka_lwr_controllers::BasicTorqueController, controller_interface::ControllerBase)
