/*
 * Harsh Maithani
 * Universite Clermont Auvergne
 * harshmaithani09@gmail.com
 * V1
*/

#include <harsh_cartesian_impedance_controller.h>

#include <algorithm>
#include <Eigen/Dense>
#include <utils/pseudo_inversion.h>
#include "math.h"

// For plugin
#include <pluginlib/class_list_macros.h>

// For control toolbox filters - Laurent
#include <control_toolbox/filters.h>

namespace kuka_lwr_controllers 
{
    HarshCartesianImpedanceController::HarshCartesianImpedanceController() {}
    HarshCartesianImpedanceController::~HarshCartesianImpedanceController() 
    {
		delete RML_;
		delete IP_;
		delete OP_;	
	}
    
     bool HarshCartesianImpedanceController::init(hardware_interface::KUKACartesianInterface *robot, ros::NodeHandle &n)
     {
		robot_namespace_ = n.getNamespace();
		// robot_namespace_ contains the namespace concatenate with the name of this controller
		// example : /kuka_lwr_left/kuka_simple_cartesian_impedance_controller
		// Only need to get the namespace --------------------------------------------
		int pos_found = robot_namespace_.find("/",1);
		std::string robot_namespace_only = robot_namespace_.substr(1,pos_found-1);
		// ----------------------------------------------------------------------------
		 
		#if TRACE_CARTESIAN_IMPEDANCE_CONTROLLER_ACTIVATED
			ROS_INFO("HarshCartesianImpedanceController: Start init of robot %s !",robot_namespace_.c_str());
			//ROS_INFO("HarshCartesianImpedanceController: Start init of robot ns only %s !",robot_namespace_only.c_str());
		#endif
		
			
        if( !(KinematicChainControllerBase<hardware_interface::KUKACartesianInterface>::init(robot, n)) )
        {
            ROS_ERROR("HarshCartesianImpedanceController: Couldn't initilize HarshCartesianImpedanceController controller of robot %s!",robot_namespace_.c_str());
            return false;
        }
        
        cycleTime_ = 0.002;
        
        RML_ = new TypeIRML(3,cycleTime_);
		IP_ = new TypeIRMLInputParameters(3);
		OP_ = new TypeIRMLOutputParameters(3);
        
        //initializes the solvers
        jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
		fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
		fk_vel_solver_.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain_));
        id_solver_.reset(new KDL::ChainDynParam( kdl_chain_, gravity_));
                
        // Get Cartesian Stiffness and Damping Interfaces Handles
        kuka_cart_stiff_handle_ = robot->getHandle(robot_namespace_only + "_cart_stiffness");
        kuka_cart_damp_handle_ = robot->getHandle(robot_namespace_only + "_cart_damping");
        
        // Get Cartesian Pose and Wrench Interfaces Handles
        kuka_cart_pose_handle_ = robot->getHandle(robot_namespace_only + "_cart_pose");
        kuka_cart_wrench_handle_ = robot->getHandle(robot_namespace_only + "_cart_wrench");
        
        // Subscribers
        sub_cart_stiffness_command_ = n.subscribe("setCartesianStiffness", 1, &HarshCartesianImpedanceController::setCartesianStiffness, this); 
        sub_cart_damping_command_   = n.subscribe("setCartesianDamping", 1, &HarshCartesianImpedanceController::setCartesianDamping, this); 
        sub_cart_pose_command_      = n.subscribe("setCartesianPose", 1, &HarshCartesianImpedanceController::setCartesianPose, this); 
        sub_cart_wrench_command_    = n.subscribe("setCartesianWrench", 1, &HarshCartesianImpedanceController::setCartesianWrench, this); 
        
        // Publishers
       	pub_cart_imp_data_         				= n.advertise<std_msgs::Float64MultiArray>("/cart_imp_data", 1000);
	    pub_cart_imp_pose_meters_     			= n.advertise<std_msgs::Float64MultiArray>("/cart_imp_pose_meters", 1000);
	    pub_cart_imp_pose_mm_     				= n.advertise<std_msgs::Float64MultiArray>("/cart_imp_pose_mm", 1000);
	    pub_cart_imp_pose_meters_rpy_rad_  		= n.advertise<std_msgs::Float64MultiArray>("/cart_imp_pose_meters_rpy_rad", 1000);
	    pub_cart_imp_pose_mm_rpy_deg_    		= n.advertise<std_msgs::Float64MultiArray>("/cart_imp_pose_mm_rpy_deg", 1000);
	    pub_cart_imp_joint_states_rad_ 			= n.advertise<std_msgs::Float64MultiArray>("/cart_imp_joint_states_rad", 1000);
	    pub_cart_imp_joint_states_deg_ 			= n.advertise<std_msgs::Float64MultiArray>("/cart_imp_joint_states_deg", 1000);
	    pub_cart_imp_joint_velocity_rads_  		= n.advertise<std_msgs::Float64MultiArray>("/cart_imp_joint_velocity_rads", 1000);
	    pub_cart_imp_joint_velocity_degs_  		= n.advertise<std_msgs::Float64MultiArray>("/cart_imp_joint_velocity_degs", 1000);
	    pub_cart_imp_kdl_cart_velocity_m_rad_  	= n.advertise<std_msgs::Float64MultiArray>("/cart_imp_kdl_cart_velocity_m_rad", 1000);
	    pub_cart_imp_kdl_cart_velocity_mm_deg_  = n.advertise<std_msgs::Float64MultiArray>("/cart_imp_kdl_cart_velocity_mm_deg", 1000);
	    pub_cart_imp_kdl_joint_acceleration_radss_     = n.advertise<std_msgs::Float64MultiArray>("/cart_imp_kdl_joint_acceleration_radss_", 1000);
	   // pub_robot_joint_acceleration_degss_  = n.advertise<std_msgs::Float64MultiArray>("/robot_joint_acceleration_degss", 1000);
	  
	    pub_cart_imp_joint_motor_torques_  		= n.advertise<std_msgs::Float64MultiArray>("/cart_imp_joint_motor_torques", 1000);
	    pub_cart_imp_estimated_forces_			= n.advertise<std_msgs::Float64MultiArray>("/cart_imp_estimated_forces", 1000);
	    pub_cart_imp_commanded_forces_ 			= n.advertise<std_msgs::Float64MultiArray>("/cart_imp_commanded_forces", 1000);
	    pub_cart_imp_kdl_jacobian_				= n.advertise<std_msgs::Float64MultiArray>("/cart_imp_kdl_jacobian", 1000);
	    pub_cart_imp_stiffness_       			= n.advertise<std_msgs::Float64MultiArray>("/cart_imp_stiffness", 1000);
	    pub_cart_imp_damping_         			= n.advertise<std_msgs::Float64MultiArray>("/cart_imp_damping", 1000);
	    pub_cart_imp_kdl_mass_inertia_matrix_	= n.advertise<std_msgs::Float64MultiArray>("/cart_imp_kdl_mass_inertia_matrix", 1000);
	    pub_cart_imp_kdl_coriolis_torques_      = n.advertise<std_msgs::Float64MultiArray>("/cart_imp_kdl_coriolis_torques", 1000);
	    pub_cart_imp_kdl_gravity_torques_       = n.advertise<std_msgs::Float64MultiArray>("/cart_imp_kdl_gravity_torques", 1000);
	
        realtime_pose_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>(n, "cartesianPose", 4));
       
        cur_Pose_FRI_.resize(NUMBER_OF_FRAME_ELEMENTS);
       
       // Resize variables here
       
        robot_data_.data.resize(2);
        robot_pose_meters_.data.resize(16); 
        robot_pose_mm_.data.resize(16); 
        robot_pose_meters_rpy_rad_.data.resize(6); 
        robot_pose_mm_rpy_deg_.data.resize(6);
        robot_joint_states_rad_.data.resize(7);
        robot_joint_states_deg_.data.resize(7);
        robot_joint_velocity_rads_.data.resize(7);
        robot_joint_velocity_degs_.data.resize(7);
        robot_cartesian_velocity_m_rad_.data.resize(6);
        robot_cartesian_velocity_mm_deg_.data.resize(6);
        robot_joint_acceleration_radss_.data.resize(7);
        //robot_joint_acceleration_degss_.data.resize(7);
        robot_joint_motor_torques_.data.resize(7);
        robot_estimated_forces_.data.resize(6);
        robot_commanded_forces_.data.resize(6);
        robot_jacobian_.data.resize(42);
       	robot_stiffness_.data.resize(6);
        robot_damping_.data.resize(6);
		robot_mass_inertia_matrix_.data.resize(49); 
		robot_coriolis_torques_.data.resize(kdl_chain_.getNrOfJoints());
		robot_gravity_torques_.data.resize(kdl_chain_.getNrOfJoints());

        // Laurent
        joint_velocity_prev_.resize(kdl_chain_.getNrOfJoints());
        joint_acceleration_.resize(kdl_chain_.getNrOfJoints());
        
        
        
        // Initialize local variables here
        
        jacobian_.resize(kdl_chain_.getNrOfJoints());
        
        joint_positions_.resize(7);
        //joint_velocities_.resize(6);
       
        joint_velocities_.resize(kdl_chain_.getNrOfJoints());
		joint_velocities_.q		=	joint_msr_states_.q;
		joint_velocities_.qdot	=	joint_msr_states_.qdot;
       
		mass_inertia_matrix_.resize(kdl_chain_.getNrOfJoints());   // Mass Inertia Matrix 
		coriolis_torques_.resize(kdl_chain_.getNrOfJoints());			   // Coriolis 
		gravity_torques_.resize(kdl_chain_.getNrOfJoints());        // Gravity 
       
 
		cmd_flag_ = 0;  // set this flag to 0 to not run the update method
       
        return true;
        
	 }
	 
	 void HarshCartesianImpedanceController::starting(const ros::Time& time)
     {
        
		#if TRACE_CARTESIAN_IMPEDANCE_CONTROLLER_ACTIVATED
			ROS_INFO("HarshCartesianImpedanceController: Starting of robot %s !",robot_namespace_.c_str());
		#endif
		
		trace_count_ = 0;
			
		// Get current Pose (Rotation && Translation matrix values)
		getCurrentPose_(pose_init_);
		
		// Get Estimated External Forces
		getExternalEstimatedForces_(wrench_external_estimated_);
		
		// Initial Cartesian stiffness
		KDL::Stiffness init_stiffness( 5000.0, 5000.0, 5000.0, 300.0, 300.0, 300.0 );
		stiff_cur_ = init_stiffness;
		
		// Initial Cartesian damping
		KDL::Stiffness init_damping(1.0,1.0,1.0,1.0,1.0,1.0);
		damp_cur_ = init_damping;
		
		// Initial Force/torque wrench
		KDL::Wrench init_wrench(KDL::Vector(0.0, 0.0, 0.0), KDL::Vector(0.0, 0.0, 0.0));
		wrench_cur_ = init_wrench;
		
		for (int i = 0; i < joint_handles_.size(); i++)
			{
             joint_positions_(i) = joint_handles_[i].getPosition();       
			}
		
		for(int i=0; i<joint_handles_.size(); i++) 
		{		
			joint_msr_states_.q(i) = joint_handles_[i].getPosition();
			joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
			//joint_msr_states_.qdotdot(i) = joint_handles_[i].getAcceleration();
            // Laurent
            joint_velocity_prev_[i] = 0.0;
            joint_acceleration_[i] = 0.0;
		}
		
		// Initial Jacobian
		jnt_to_jac_solver_->JntToJac(joint_positions_, jacobian_);		
		
		// Inverse Dynamics Parameters
		id_solver_->JntToMass(joint_msr_states_.q, mass_inertia_matrix_);				        		// computing Inertia matrix		
		id_solver_->JntToCoriolis(joint_msr_states_.q, joint_msr_states_.qdot, coriolis_torques_);		// computing Coriolis torques
		id_solver_->JntToGravity(joint_msr_states_.q, gravity_torques_); 								// computing Gravity torques
		
		
		// forward initial commands to HW
		forwardCmdFRI_(pose_init_,stiff_cur_,damp_cur_,wrench_cur_);
		
		cmd_flag_ = 0;  // set this flag to 0 to not run the update method
		
		 last_time_ = ros::Time::now().toSec();
		
     }
    
     void HarshCartesianImpedanceController::stopping(const ros::Time& time)
	 {
		#if TRACE_CARTESIAN_IMPEDANCE_CONTROLLER_ACTIVATED
			ROS_INFO("HarshCartesianImpedanceController: Stopping of robot %s !",robot_namespace_.c_str());
		#endif
		
		cmd_flag_ = 0;  // set this flag to 0 to not run the update method
			
	 }
	 
	 void HarshCartesianImpedanceController::update(const ros::Time& time, const ros::Duration& period)
     {
		
		current_time_ = ros::Time::now().toSec();
		dt =  current_time_ - last_time_;
		last_time_ = ros::Time::now().toSec();
		
		getCurrentPose_(pose_cur_);


        // Laurent
        for(int j=0; j<joint_handles_.size(); j++)
        {
         joint_acceleration_[j] = filters::exponentialSmoothing((joint_handles_[j].getVelocity()-joint_velocity_prev_[j])/period.toSec(), joint_acceleration_[j], 0.2);
         joint_velocity_prev_[j] = joint_handles_[j].getVelocity();
        }


		if (cmd_flag_)
       {
			if (resultValue_ != TypeIRML::RML_FINAL_STATE_REACHED)
			{
				resultValue_ = RML_->GetNextMotionState_Position(*IP_,OP_);
				
				if ((resultValue_ != TypeIRML::RML_WORKING) && (resultValue_ != TypeIRML::RML_FINAL_STATE_REACHED))
				{
					ROS_INFO("::update : ERROR during trajectory generation err n°%d",resultValue_);
				}
			
			/*	For robot to stay in the same orientation as current 
				KDL::Rotation des_R(kuka_cart_pose_handle_.getRXX(),
							kuka_cart_pose_handle_.getRXY(),
							kuka_cart_pose_handle_.getRXZ(),
							kuka_cart_pose_handle_.getRYX(),
							kuka_cart_pose_handle_.getRYY(),
							kuka_cart_pose_handle_.getRYZ(),
							kuka_cart_pose_handle_.getRZX(),
							kuka_cart_pose_handle_.getRZY(),
							kuka_cart_pose_handle_.getRZZ());
			*/
			  
			 /* For robot to stay in the same orientation as initial orientation
			   KDL::Rotation des_R(pose_init_.M.UnitX().x(),
							pose_init_.M.UnitY().x(),
							pose_init_.M.UnitZ().x(),
							pose_init_.M.UnitX().y(),
							pose_init_.M.UnitY().y(),
							pose_init_.M.UnitZ().y(),
							pose_init_.M.UnitX().z(),
							pose_init_.M.UnitY().z(),
							pose_init_.M.UnitZ().z());
			 */ 
			 
			    KDL::Rotation des_R(0,0,1,
				    			    0,1,0,
				                   -1,0,0);	
				                   
				KDL::Vector des_T((double)(IP_->TargetPosition->VecData[0]),
						          (double)(IP_->TargetPosition->VecData[1]),
						          (double)(IP_->TargetPosition->VecData[2]));                   
				
				/*
				KDL::Rotation des_R(1,0,0,
				                    0,1,0,
				                    0,0,1);
			 	
			 	  
				KDL::Vector des_T((double)(OP_->NewPosition->VecData[0]),
						          (double)(OP_->NewPosition->VecData[1]),
						          (double)(OP_->NewPosition->VecData[2]));
				*/
						 
		    //ROS_INFO("x_cur: %f x_target: %f y_cur: %f y_target: %f z_cur: %f z_target: %f",kuka_cart_pose_handle_.getTX(),(double)(OP_->NewPosition->VecData[0]),kuka_cart_pose_handle_.getTY(),(double)(OP_->NewPosition->VecData[1]),kuka_cart_pose_handle_.getTZ(),(double)(OP_->NewPosition->VecData[2]));
			  
				pose_des_ = KDL::Frame(des_R, des_T);
				
			//	ROS_INFO("desired z is: %f",pose_des_.p.z());
				
			//	pose_des_.p = KDL::Vector(pose_des_.p.x(),pose_des_.p.y(),(double)(OP_->NewPosition->VecData[0]));
				
				*(IP_->CurrentPosition) = *(OP_->NewPosition);
				*(IP_->CurrentVelocity) = *(OP_->NewVelocity);
				
				#if TRACE_GroupCommandController_ACTIVATED
					ROS_INFO("HarshCartesianImpedanceController: resultValue_ = %d", resultValue_);
					ROS_INFO("HarshCartesianImpedanceController: of robot %s -> j0=%f, j1=%f, j2=%f, j3=%f, j4=%f, j5=%f, j6=%f",robot_namespace_.c_str(),joint_des_states_.q(0), joint_des_states_.q(1), joint_des_states_.q(2), joint_des_states_.q(3), joint_des_states_.q(4), joint_des_states_.q(5), joint_des_states_.q(6));
				#endif
				
				forwardCmdFRI_(pose_des_,stiff_cur_,damp_cur_,wrench_cur_);
        
			}
			else
			{
				cmd_flag_=0; // all the joint values derired are reached, so set this flag to 0 to not run the update method
                ROS_INFO("Harshjoint_acceleration_[j] = filters::exponentialSmoothing((joint_velocity_[j]-joint_velocity_prev_[j])/period.toSec(), joint_acceleration_[j], 0.2);CartesianImpedanceController: GOAL of robot %s !!!!!!!!!!",robot_namespace_.c_str());
			}	
		}
	  else
		{
	    // forward initial commands to HW
		forwardCmdFRI_(pose_cur_,stiff_cur_,damp_cur_,wrench_cur_);  
	    }
	 
		// Get current Pose (Rotation && Translation matrix values)
		/*
		getCurrentPose_(pose_cur_);
		
		KDL::Rotation des_R(kuka_cart_pose_handle_.getRXX(),
							kuka_cart_pose_handle_.getRXY(),
							kuka_cart_pose_handle_.getRXZ(),
							kuka_cart_pose_handle_.getRYX(),
							kuka_cart_pose_handle_.getRYY(),
							kuka_cart_pose_handle_.getRYZ(),
							kuka_cart_pose_handle_.getRZX(),
							kuka_cart_pose_handle_.getRZY(),
							kuka_cart_pose_handle_.getRZZ());
							
		KDL::Vector des_T(kuka_cart_pose_handle_.getTX(),
						  kuka_cart_pose_handle_.getTY(),
						  kuka_cart_pose_handle_.getTZ()+0.01);
						  
		pose_des_ = KDL::Frame(des_R, des_T);
		
		ROS_INFO("desired z is: %f",pose_des_.p.z());
	    
	    forwardCmdFRI_(pose_des_,stiff_cur_,damp_cur_,wrench_cur_);
		*/
	
		// Publish Robot Data	
	 
		//robot_data_.data[0] = (float) ros::Time::now().toSec();
		robot_data_.data[0] = dt;
		robot_data_.data[1] = period.toSec();

		// Pose in meters
		robot_pose_meters_.data[0]  = kuka_cart_pose_handle_.getRXX();   //Rxx
		robot_pose_meters_.data[1]  = kuka_cart_pose_handle_.getRXY();   //Rxy
		robot_pose_meters_.data[2]  = kuka_cart_pose_handle_.getRXZ();   //Rxz
		robot_pose_meters_.data[3]  = kuka_cart_pose_handle_.getTX();    //Tx
		robot_pose_meters_.data[4]  = kuka_cart_pose_handle_.getRYX();   //Ryx
	    robot_pose_meters_.data[5]  = kuka_cart_pose_handle_.getRYY();   //Ryy
		robot_pose_meters_.data[6]  = kuka_cart_pose_handle_.getRYZ();   //Ryz
		robot_pose_meters_.data[7]  = kuka_cart_pose_handle_.getTY();    //Ty
		robot_pose_meters_.data[8]  = kuka_cart_pose_handle_.getRZX();   //Rzx
		robot_pose_meters_.data[9]  = kuka_cart_pose_handle_.getRZY();   //Rzy
		robot_pose_meters_.data[10] = kuka_cart_pose_handle_.getRZZ();   //Rzz
	    robot_pose_meters_.data[11] = kuka_cart_pose_handle_.getTZ();    //Tz
	    robot_pose_meters_.data[12] = 0;
	    robot_pose_meters_.data[13] = 0;
	    robot_pose_meters_.data[14] = 0;
	    robot_pose_meters_.data[15] = 1;
	 
		// Pose in mm
		robot_pose_mm_.data[0]  = kuka_cart_pose_handle_.getRXX();   //Rxx
		robot_pose_mm_.data[1]  = kuka_cart_pose_handle_.getRXY();   //Rxy
		robot_pose_mm_.data[2]  = kuka_cart_pose_handle_.getRXZ();   //Rxz
		robot_pose_mm_.data[3]  = 1000*kuka_cart_pose_handle_.getTX();    //Tx
		robot_pose_mm_.data[4]  = kuka_cart_pose_handle_.getRYX();   //Ryx
	    robot_pose_mm_.data[5]  = kuka_cart_pose_handle_.getRYY();   //Ryy
		robot_pose_mm_.data[6]  = kuka_cart_pose_handle_.getRYZ();   //Ryz
		robot_pose_mm_.data[7]  = 1000*kuka_cart_pose_handle_.getTY();    //Ty
		robot_pose_mm_.data[8]  = kuka_cart_pose_handle_.getRZX();   //Rzx
		robot_pose_mm_.data[9]  = kuka_cart_pose_handle_.getRZY();   //Rzy
		robot_pose_mm_.data[10] = kuka_cart_pose_handle_.getRZZ();   //Rzz
	    robot_pose_mm_.data[11] = 1000*kuka_cart_pose_handle_.getTZ();    //Tz
	    robot_pose_mm_.data[12] = 0;
	    robot_pose_mm_.data[13] = 0;
	    robot_pose_mm_.data[14] = 0;
	    robot_pose_mm_.data[15] = 1;
	 
		// Pose in meters and rpy in rad
		
		for (int i = 0; i < joint_handles_.size(); i++)
			{
             joint_positions_(i) = joint_handles_[i].getPosition(); 
             joint_velocities_.q(i) = joint_handles_[i].getPosition(); 
		     joint_velocities_.qdot(i) = joint_handles_[i].getVelocity(); 
             
			}
			
		fk_pos_solver_->JntToCart(joint_positions_,pose_current_); 			// Computing Forward Kinematics
		pose_current_.M.GetRPY (roll_,pitch_,yaw_);
	  
		robot_pose_meters_rpy_rad_.data[0]  = kuka_cart_pose_handle_.getTX();    //Tx
		robot_pose_meters_rpy_rad_.data[1]  = kuka_cart_pose_handle_.getTY();    //Ty
		robot_pose_meters_rpy_rad_.data[2]  = kuka_cart_pose_handle_.getTZ();    //Tz
		robot_pose_meters_rpy_rad_.data[3]  = yaw_; 			// Earlier this was roll_ by mistake
		robot_pose_meters_rpy_rad_.data[4]  = pitch_;	
	    robot_pose_meters_rpy_rad_.data[5]  = roll_; 			// Earlier this was yaw_ by mistake
	
		// Pose in mm and rpy in deg
		
		robot_pose_mm_rpy_deg_.data[0]  = 1000*kuka_cart_pose_handle_.getTX();    //Tx
		robot_pose_mm_rpy_deg_.data[1]  = 1000*kuka_cart_pose_handle_.getTY();    //Ty
		robot_pose_mm_rpy_deg_.data[2]  = 1000*kuka_cart_pose_handle_.getTZ();    //Tz
		robot_pose_mm_rpy_deg_.data[3]  = (180/3.14)*yaw_;    	// Earlier this was roll_ by mistake
		robot_pose_mm_rpy_deg_.data[4]  = (180/3.14)*pitch_;
	    robot_pose_mm_rpy_deg_.data[5]  = (180/3.14)*roll_;   	// Earlier this was yaw_ by mistake
	
	 	// Joint States in Radians
		
		robot_joint_states_rad_.data[0]  =	joint_handles_[0].getPosition();
		robot_joint_states_rad_.data[1]  =	joint_handles_[1].getPosition();
		robot_joint_states_rad_.data[2]  =	joint_handles_[2].getPosition();
		robot_joint_states_rad_.data[3]  =	joint_handles_[3].getPosition();
		robot_joint_states_rad_.data[4]  =	joint_handles_[4].getPosition();
		robot_joint_states_rad_.data[5]  =	joint_handles_[5].getPosition();
		robot_joint_states_rad_.data[6]  =	joint_handles_[6].getPosition();
		
		// Joint Positions in Degrees
		
		robot_joint_states_deg_.data[0]  =	(180/3.14)*joint_handles_[0].getPosition();
		robot_joint_states_deg_.data[1]  =	(180/3.14)*joint_handles_[1].getPosition();
		robot_joint_states_deg_.data[2]  =	(180/3.14)*joint_handles_[2].getPosition();
		robot_joint_states_deg_.data[3]  =	(180/3.14)*joint_handles_[3].getPosition();
		robot_joint_states_deg_.data[4]  =	(180/3.14)*joint_handles_[4].getPosition();
		robot_joint_states_deg_.data[5]  =	(180/3.14)*joint_handles_[5].getPosition();
		robot_joint_states_deg_.data[6]  =	(180/3.14)*joint_handles_[6].getPosition();
		
		// Joint Velocity in Radians/s
		
		robot_joint_velocity_rads_.data[0]  =	joint_handles_[0].getVelocity();  
		robot_joint_velocity_rads_.data[1]  =	joint_handles_[1].getVelocity();  
		robot_joint_velocity_rads_.data[2]  =	joint_handles_[2].getVelocity();  
		robot_joint_velocity_rads_.data[3]  =	joint_handles_[3].getVelocity();  
		robot_joint_velocity_rads_.data[4]  =	joint_handles_[4].getVelocity();  
		robot_joint_velocity_rads_.data[5]  =	joint_handles_[5].getVelocity();  
		robot_joint_velocity_rads_.data[6]  =	joint_handles_[6].getVelocity();  
		
		// Joint Velocity in Degrees/s
		
		robot_joint_velocity_degs_.data[0]  =	(180/3.14)*joint_handles_[0].getVelocity();  
		robot_joint_velocity_degs_.data[1]  =	(180/3.14)*joint_handles_[1].getVelocity();  
		robot_joint_velocity_degs_.data[2]  =	(180/3.14)*joint_handles_[2].getVelocity();  
		robot_joint_velocity_degs_.data[3]  =	(180/3.14)*joint_handles_[3].getVelocity();  
		robot_joint_velocity_degs_.data[4]  =	(180/3.14)*joint_handles_[4].getVelocity();  
		robot_joint_velocity_degs_.data[5]  =	(180/3.14)*joint_handles_[5].getVelocity();  
		robot_joint_velocity_degs_.data[6]  =	(180/3.14)*joint_handles_[6].getVelocity();  
		
		// Cartesian Velocity in m/s and rad/s 
		
		fk_vel_solver_->JntToCart(joint_velocities_ , cartesian_velocity_current_);
		
		robot_cartesian_velocity_m_rad_.data[0]  = cartesian_velocity_current_.GetTwist().vel.x();
		robot_cartesian_velocity_m_rad_.data[1]  = cartesian_velocity_current_.GetTwist().vel.y();
		robot_cartesian_velocity_m_rad_.data[2]  = cartesian_velocity_current_.GetTwist().vel.z();
		robot_cartesian_velocity_m_rad_.data[3]  = cartesian_velocity_current_.GetTwist().rot.x();
		robot_cartesian_velocity_m_rad_.data[4]  = cartesian_velocity_current_.GetTwist().rot.y();
		robot_cartesian_velocity_m_rad_.data[5]  = cartesian_velocity_current_.GetTwist().rot.z(); 
		
		// Cartesian Velocity in mm/s and deg/s
		
		robot_cartesian_velocity_mm_deg_.data[0]  = 1000*cartesian_velocity_current_.GetTwist().vel.x();
		robot_cartesian_velocity_mm_deg_.data[1]  = 1000*cartesian_velocity_current_.GetTwist().vel.y();
		robot_cartesian_velocity_mm_deg_.data[2]  = 1000*cartesian_velocity_current_.GetTwist().vel.z();
		robot_cartesian_velocity_mm_deg_.data[3]  = (180/3.14)*cartesian_velocity_current_.GetTwist().rot.x();
		robot_cartesian_velocity_mm_deg_.data[4]  = (180/3.14)*cartesian_velocity_current_.GetTwist().rot.y();
		robot_cartesian_velocity_mm_deg_.data[5]  = (180/3.14)*cartesian_velocity_current_.GetTwist().rot.z(); 
		
		
		// Joint Acceleration in Radians/s/s
		
		robot_joint_acceleration_radss_.data[0]  =	joint_acceleration_[0];  
		robot_joint_acceleration_radss_.data[1]  =	joint_acceleration_[1]; 
		robot_joint_acceleration_radss_.data[2]  =	joint_acceleration_[2];  
		robot_joint_acceleration_radss_.data[3]  =	joint_acceleration_[3];
		robot_joint_acceleration_radss_.data[4]  =	joint_acceleration_[4];
		robot_joint_acceleration_radss_.data[5]  =	joint_acceleration_[5];  
		robot_joint_acceleration_radss_.data[6]  =	joint_acceleration_[6];  
		
		// Joint Acceleration in Degs/s/s
		/*
		robot_joint_acceleration_degss_.data[0]  =	(180/3.14)*joint_handles_[0].getAcceleration();  
		robot_joint_acceleration_degss_.data[1]  =	(180/3.14)*joint_handles_[1].getAcceleration();  
		robot_joint_acceleration_degss_.data[2]  =	(180/3.14)*joint_handles_[2].getAcceleration();  
		robot_joint_acceleration_degss_.data[3]  =	(180/3.14)*joint_handles_[3].getAcceleration();  
		robot_joint_acceleration_degss_.data[4]  =	(180/3.14)*joint_handles_[4].getAcceleration();  
		robot_joint_acceleration_degss_.data[5]  =	(180/3.14)*joint_handles_[5].getAcceleration();  
		robot_joint_acceleration_degss_.data[6]  =	(180/3.14)*joint_handles_[6].getAcceleration();  
		*/
		
		// Joint Motor Torques in N-m
	    
	    robot_joint_motor_torques_.data[0]  =	joint_handles_[0].getEffort();
		robot_joint_motor_torques_.data[1]  =	joint_handles_[1].getEffort();
		robot_joint_motor_torques_.data[2]  =	joint_handles_[2].getEffort();
		robot_joint_motor_torques_.data[3]  =	joint_handles_[3].getEffort();
		robot_joint_motor_torques_.data[4]  =	joint_handles_[4].getEffort();
		robot_joint_motor_torques_.data[5]  =	joint_handles_[5].getEffort();
		robot_joint_motor_torques_.data[6]  =	joint_handles_[6].getEffort();
		
		// Stiffness 
		
		robot_stiffness_.data[0]  = stiff_cur_[0];  // Kx
		robot_stiffness_.data[1]  = stiff_cur_[1];  // Ky
		robot_stiffness_.data[2]  = stiff_cur_[2];  // Kz
		robot_stiffness_.data[3]  = stiff_cur_[3];  // Kpitch
		robot_stiffness_.data[4]  = stiff_cur_[4];  // Kyaw
		robot_stiffness_.data[5]  = stiff_cur_[5];  // Kroll
			
		// Damping
		
		robot_damping_.data[0]  = damp_cur_[0];  // Bx
		robot_damping_.data[1]  = damp_cur_[1];  // By
		robot_damping_.data[2]  = damp_cur_[2];  // Bz
		robot_damping_.data[3]  = damp_cur_[3];  // B_pitch
		robot_damping_.data[4]  = damp_cur_[4];  // B_yaw
		robot_damping_.data[5]  = damp_cur_[5];  // B_roll
		
		// External Wrench 
	
		robot_commanded_forces_.data[0]  = wrench_cur_.force.x();   //Fx
		robot_commanded_forces_.data[1]  = wrench_cur_.force.y();   //Fy
		robot_commanded_forces_.data[2]  = wrench_cur_.force.z();   //Fz
	    robot_commanded_forces_.data[3]  = wrench_cur_.torque.x();  //Taux
	    robot_commanded_forces_.data[4]  = wrench_cur_.torque.y();  //Tauy
	    robot_commanded_forces_.data[5]  = wrench_cur_.torque.z();  //Tauz
	    
	    // Estimated Forces
	    
	    getExternalEstimatedForces_(wrench_external_estimated_);
	    
	    robot_estimated_forces_.data[0]  = wrench_external_estimated_.force.x();   //Fx
		robot_estimated_forces_.data[1]  = wrench_external_estimated_.force.y();   //Fy
		robot_estimated_forces_.data[2]  = wrench_external_estimated_.force.z();   //Fz
	    robot_estimated_forces_.data[3]  = wrench_external_estimated_.torque.x();  //Taux
	    robot_estimated_forces_.data[4]  = wrench_external_estimated_.torque.y();  //Tauy
	    robot_estimated_forces_.data[5]  = wrench_external_estimated_.torque.z();  //Tauz
	    
		// Jacobian
		
		jnt_to_jac_solver_->JntToJac(joint_positions_, jacobian_);	
		
		robot_jacobian_.data[0]=jacobian_(0,0);
		robot_jacobian_.data[1]=jacobian_(0,1);
		robot_jacobian_.data[2]=jacobian_(0,2);
		robot_jacobian_.data[3]=jacobian_(0,3);
		robot_jacobian_.data[4]=jacobian_(0,4);
		robot_jacobian_.data[5]=jacobian_(0,5);
		robot_jacobian_.data[6]=jacobian_(0,6);
		robot_jacobian_.data[7]=jacobian_(1,0);
		robot_jacobian_.data[8]=jacobian_(1,1);
		robot_jacobian_.data[9]=jacobian_(1,2);
		robot_jacobian_.data[10]=jacobian_(1,3);
		robot_jacobian_.data[11]=jacobian_(1,4);
		robot_jacobian_.data[12]=jacobian_(1,5);
		robot_jacobian_.data[13]=jacobian_(1,6);
		robot_jacobian_.data[14]=jacobian_(2,0);
		robot_jacobian_.data[15]=jacobian_(2,1);
		robot_jacobian_.data[16]=jacobian_(2,2);
		robot_jacobian_.data[17]=jacobian_(2,3);
		robot_jacobian_.data[18]=jacobian_(2,4);
		robot_jacobian_.data[19]=jacobian_(2,5);
		robot_jacobian_.data[20]=jacobian_(2,6);
		robot_jacobian_.data[21]=jacobian_(3,0);
		robot_jacobian_.data[22]=jacobian_(3,1);
		robot_jacobian_.data[23]=jacobian_(3,2);
		robot_jacobian_.data[24]=jacobian_(3,3);
		robot_jacobian_.data[25]=jacobian_(3,4);
		robot_jacobian_.data[26]=jacobian_(3,5);
		robot_jacobian_.data[27]=jacobian_(3,6);
		robot_jacobian_.data[28]=jacobian_(4,0);
		robot_jacobian_.data[29]=jacobian_(4,1);
		robot_jacobian_.data[30]=jacobian_(4,2);
		robot_jacobian_.data[31]=jacobian_(4,3);
		robot_jacobian_.data[32]=jacobian_(4,4);
		robot_jacobian_.data[33]=jacobian_(4,5);
		robot_jacobian_.data[34]=jacobian_(4,6);
		robot_jacobian_.data[35]=jacobian_(5,0);
		robot_jacobian_.data[36]=jacobian_(5,1);
		robot_jacobian_.data[37]=jacobian_(5,2);
		robot_jacobian_.data[38]=jacobian_(5,3);
		robot_jacobian_.data[39]=jacobian_(5,4);
		robot_jacobian_.data[40]=jacobian_(5,5);
		robot_jacobian_.data[41]=jacobian_(5,6);

		for(size_t i=0; i<joint_handles_.size(); i++) 
		{		
			joint_msr_states_.q(i) = joint_handles_[i].getPosition();
			joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
            joint_msr_states_.qdotdot(i) = joint_acceleration_[i];
            //joint_msr_states_.qdotdot(i) = joint_handles_[i].getAcceleration(); Displaying error
		}
		
		id_solver_->JntToMass(joint_msr_states_.q, mass_inertia_matrix_);				        		// computing Inertia matrix		
		id_solver_->JntToCoriolis(joint_msr_states_.q, joint_msr_states_.qdot, coriolis_torques_);		// computing Coriolis torques
		id_solver_->JntToGravity(joint_msr_states_.q, gravity_torques_); 								// computing Gravity torques

		// Mass Inertia Matrix 
		robot_mass_inertia_matrix_.data[0] = mass_inertia_matrix_(0,0);
		robot_mass_inertia_matrix_.data[1] = mass_inertia_matrix_(0,1);
		robot_mass_inertia_matrix_.data[2] = mass_inertia_matrix_(0,2);
		robot_mass_inertia_matrix_.data[3] = mass_inertia_matrix_(0,3);
		robot_mass_inertia_matrix_.data[4] = mass_inertia_matrix_(0,4);
		robot_mass_inertia_matrix_.data[5] = mass_inertia_matrix_(0,5);
		robot_mass_inertia_matrix_.data[6] = mass_inertia_matrix_(0,6);
		
		robot_mass_inertia_matrix_.data[7] = mass_inertia_matrix_(1,0);
		robot_mass_inertia_matrix_.data[8] = mass_inertia_matrix_(1,1);
		robot_mass_inertia_matrix_.data[9] = mass_inertia_matrix_(1,2);
		robot_mass_inertia_matrix_.data[10]= mass_inertia_matrix_(1,3);
		robot_mass_inertia_matrix_.data[11]= mass_inertia_matrix_(1,4);
		robot_mass_inertia_matrix_.data[12]= mass_inertia_matrix_(1,5);
		robot_mass_inertia_matrix_.data[13]= mass_inertia_matrix_(1,6);
		
		robot_mass_inertia_matrix_.data[14]= mass_inertia_matrix_(2,0);
		robot_mass_inertia_matrix_.data[15]= mass_inertia_matrix_(2,1);
		robot_mass_inertia_matrix_.data[16]= mass_inertia_matrix_(2,2);
		robot_mass_inertia_matrix_.data[17]= mass_inertia_matrix_(2,3);
		robot_mass_inertia_matrix_.data[18]= mass_inertia_matrix_(2,4);
		robot_mass_inertia_matrix_.data[19]= mass_inertia_matrix_(2,5);
		robot_mass_inertia_matrix_.data[20]= mass_inertia_matrix_(2,6);
		
		robot_mass_inertia_matrix_.data[21]= mass_inertia_matrix_(3,0);
		robot_mass_inertia_matrix_.data[22]= mass_inertia_matrix_(3,1);
		robot_mass_inertia_matrix_.data[23]= mass_inertia_matrix_(3,2);
		robot_mass_inertia_matrix_.data[24]= mass_inertia_matrix_(3,3);
		robot_mass_inertia_matrix_.data[25]= mass_inertia_matrix_(3,4);
		robot_mass_inertia_matrix_.data[26]= mass_inertia_matrix_(3,5);
		robot_mass_inertia_matrix_.data[27]= mass_inertia_matrix_(3,6);
		
		robot_mass_inertia_matrix_.data[28]= mass_inertia_matrix_(4,0);
		robot_mass_inertia_matrix_.data[29]= mass_inertia_matrix_(4,1);
		robot_mass_inertia_matrix_.data[30]= mass_inertia_matrix_(4,2);
		robot_mass_inertia_matrix_.data[31]= mass_inertia_matrix_(4,3);
		robot_mass_inertia_matrix_.data[32]= mass_inertia_matrix_(4,4);
		robot_mass_inertia_matrix_.data[33]= mass_inertia_matrix_(4,5);
		robot_mass_inertia_matrix_.data[34]= mass_inertia_matrix_(4,6);
		
		robot_mass_inertia_matrix_.data[35]= mass_inertia_matrix_(5,0);
		robot_mass_inertia_matrix_.data[36]= mass_inertia_matrix_(5,1);
		robot_mass_inertia_matrix_.data[37]= mass_inertia_matrix_(5,2);
		robot_mass_inertia_matrix_.data[38]= mass_inertia_matrix_(5,3);
		robot_mass_inertia_matrix_.data[39]= mass_inertia_matrix_(5,4);
		robot_mass_inertia_matrix_.data[40]= mass_inertia_matrix_(5,5);
		robot_mass_inertia_matrix_.data[41]= mass_inertia_matrix_(5,6);
		
		robot_mass_inertia_matrix_.data[42]= mass_inertia_matrix_(6,0);
		robot_mass_inertia_matrix_.data[43]= mass_inertia_matrix_(6,1);
		robot_mass_inertia_matrix_.data[44]= mass_inertia_matrix_(6,2);
		robot_mass_inertia_matrix_.data[45]= mass_inertia_matrix_(6,3);
		robot_mass_inertia_matrix_.data[46]= mass_inertia_matrix_(6,4);
		robot_mass_inertia_matrix_.data[47]= mass_inertia_matrix_(6,5);
		robot_mass_inertia_matrix_.data[48]= mass_inertia_matrix_(6,6);
		

		// Coriolis Torques
		robot_coriolis_torques_.data[0] = coriolis_torques_(0);
		robot_coriolis_torques_.data[1] = coriolis_torques_(1);
		robot_coriolis_torques_.data[2] = coriolis_torques_(2);
		robot_coriolis_torques_.data[3] = coriolis_torques_(3);
		robot_coriolis_torques_.data[4] = coriolis_torques_(4);
		robot_coriolis_torques_.data[5] = coriolis_torques_(5);
		robot_coriolis_torques_.data[6] = coriolis_torques_(6);
		
		// Gravity Torques 
		robot_gravity_torques_.data[0] = gravity_torques_(0);
		robot_gravity_torques_.data[1] = gravity_torques_(1);
		robot_gravity_torques_.data[2] = gravity_torques_(2);
		robot_gravity_torques_.data[3] = gravity_torques_(3);
		robot_gravity_torques_.data[4] = gravity_torques_(4);
		robot_gravity_torques_.data[5] = gravity_torques_(5);
		robot_gravity_torques_.data[6] = gravity_torques_(6);
		
		// Publish on ROS
		
		pub_cart_imp_data_.publish(robot_data_);
	    pub_cart_imp_pose_meters_.publish(robot_pose_meters_);
	    pub_cart_imp_pose_mm_.publish(robot_pose_mm_);
	    pub_cart_imp_pose_meters_rpy_rad_.publish(robot_pose_meters_rpy_rad_);
	    pub_cart_imp_pose_mm_rpy_deg_.publish(robot_pose_mm_rpy_deg_);
	    pub_cart_imp_joint_states_deg_.publish(robot_joint_states_deg_);
	    pub_cart_imp_joint_states_rad_.publish(robot_joint_states_rad_);
	    pub_cart_imp_joint_velocity_rads_.publish(robot_joint_velocity_rads_);
	    pub_cart_imp_joint_velocity_degs_.publish(robot_joint_velocity_degs_);
	   
	    pub_cart_imp_kdl_joint_acceleration_radss_.publish(robot_joint_acceleration_radss_);
	    //pub_robot_joint_acceleration_degss_.publish(robot_joint_acceleration_degss_);
	    pub_cart_imp_joint_motor_torques_.publish(robot_joint_motor_torques_);
	    pub_cart_imp_estimated_forces_.publish(robot_estimated_forces_);
	    pub_cart_imp_commanded_forces_.publish(robot_commanded_forces_);		
	    pub_cart_imp_stiffness_.publish(robot_stiffness_);
	    pub_cart_imp_damping_.publish(robot_damping_);
	 
		pub_cart_imp_kdl_jacobian_.publish(robot_jacobian_);
		pub_cart_imp_kdl_cart_velocity_m_rad_.publish(robot_cartesian_velocity_m_rad_);
	    pub_cart_imp_kdl_cart_velocity_mm_deg_.publish(robot_cartesian_velocity_mm_deg_);
	    
	    pub_cart_imp_kdl_mass_inertia_matrix_.publish(robot_mass_inertia_matrix_);
	    pub_cart_imp_kdl_coriolis_torques_.publish(robot_coriolis_torques_);
	    pub_cart_imp_kdl_gravity_torques_.publish(robot_gravity_torques_);
	    
	 
		 #if TRACE_CARTESIAN_IMPEDANCE_CONTROLLER_ACTIVATED
			 trace_count_++;
			 if (trace_count_%1000000)
			 {
				//ROS_INFO("-> stiffness -> X = %f, Y = %f, Z = %f, A = %f, B = %f, C = %f", kuka_cart_stiff_handle_.getX(), kuka_cart_stiff_handle_.getY(), kuka_cart_stiff_handle_.getZ(), kuka_cart_stiff_handle_.getA(), kuka_cart_stiff_handle_.getB(), kuka_cart_stiff_handle_.getC());
				/*for (int i=0; i<7; i++)
				{
					ROS_INFO("Joint Pos(%d) = %f", i, joint_handles_[i].getPosition());
				}*/
			 }
		 #endif
	 }
	 
	 
	 void HarshCartesianImpedanceController::setCartesianStiffness(const std_msgs::Float64MultiArrayConstPtr& msg)
	 {
		#if TRACE_CARTESIAN_IMPEDANCE_CONTROLLER_ACTIVATED
			ROS_INFO("HarshCartesianImpedanceController: Start setCartesianStiffness of robot %s!",robot_namespace_.c_str());
		#endif

		if(msg->data.size()!=NUMBER_OF_CART_DOFS)
		{ 
			ROS_ERROR_STREAM("HarshCartesianImpedanceController: Dimension (of robot " << robot_namespace_.c_str() << ") of command (" << msg->data.size() << ") does not match number of DOF(" << NUMBER_OF_CART_DOFS << ")! Not executing!");
			return; 
		}
		
		#if TRACE_CARTESIAN_IMPEDANCE_CONTROLLER_ACTIVATED
			ROS_INFO("Get HarshCartesianImpedanceController/setCartesianStiffness X=%f, Y=%f, Z=%f, A=%f, B=%f, C=%f",msg->data[0],msg->data[1],msg->data[2],msg->data[3],msg->data[4],msg->data[5]);
		#endif
		
		// 0: X, 1: Y, 2: Z, 3: A, 4: B, 5: C
		for (int i=0; i<NUMBER_OF_CART_DOFS; i++)
		{
			stiff_cur_[i] = msg->data[i];
		}
	
	 }
	 
	 void HarshCartesianImpedanceController::setCartesianDamping(const std_msgs::Float64MultiArrayConstPtr& msg)
	 {
		#if TRACE_CARTESIAN_IMPEDANCE_CONTROLLER_ACTIVATED
			ROS_INFO("HarshCartesianImpedanceController: Start setCartesianDamping of robot %s!",robot_namespace_.c_str());
		#endif

		if(msg->data.size()!=NUMBER_OF_CART_DOFS)
		{ 
			ROS_ERROR_STREAM("HarshCartesianImpedanceController: Dimension (of robot " << robot_namespace_.c_str() << ") of command (" << msg->data.size() << ") does not match number of DOF(" << NUMBER_OF_CART_DOFS << ")! Not executing!");
			return; 
		}
		
		#if TRACE_CARTESIAN_IMPEDANCE_CONTROLLER_ACTIVATED
			ROS_INFO("Get HarshCartesianImpedanceController/setCartesianDamping X=%f, Y=%f, Z=%f, A=%f, B=%f, C=%f",msg->data[0],msg->data[1],msg->data[2],msg->data[3],msg->data[4],msg->data[5]);
		#endif
		
		// 0: X, 1: Y, 2: Z, 3: A, 4: B, 5: C
		// 0: X, 1: Y, 2: Z, 3: A, 4: B, 5: C
		for (int i=0; i<NUMBER_OF_CART_DOFS; i++)
		{
			damp_cur_[i] = msg->data[i];
		}
		
	 }
	 
	 void HarshCartesianImpedanceController::setCartesianPose(const std_msgs::Float64MultiArrayConstPtr& msg)
	 {
		#if TRACE_CARTESIAN_IMPEDANCE_CONTROLLER_ACTIVATED
			ROS_INFO("HarshCartesianImpedanceController: Start setCartesianPose of robot %s!",robot_namespace_.c_str());
		#endif

		if(msg->data.size()!=NUMBER_OF_FRAME_ELEMENTS)
		{ 
			ROS_ERROR_STREAM("HarshCartesianImpedanceController: Dimension (of robot " << robot_namespace_.c_str() << ") of command (" << msg->data.size() << ") does not match number of DOF(" << NUMBER_OF_FRAME_ELEMENTS << ")! Not executing!");
			return; 
		}
		
		#if TRACE_CARTESIAN_IMPEDANCE_CONTROLLER_ACTIVATED
			ROS_INFO("Get HarshCartesianImpedanceController/setCartesianPose :");
			ROS_INFO("[%.3f, %.3f, %.3f, %.3f",msg->data[0],msg->data[1],msg->data[2],msg->data[3]);
			ROS_INFO("%.3f, %.3f, %.3f, %.3f",msg->data[4],msg->data[5],msg->data[6],msg->data[7]);
			ROS_INFO("%.3f, %.3f, %.3f, %.3f",msg->data[8],msg->data[9],msg->data[10],msg->data[11]);
			ROS_INFO("0, 0, 0, 1]");
		#endif
		
		// 0: RXX, 1: RXY, 2: RXZ, 3: TX, 4: RYX, 5: RYY, 6: RYZ, 7: TY, 8: RZX, 9: RZY, 10: RZZ, 11: TZ
		pose_des_.M = KDL::Rotation(msg->data[0],msg->data[1],msg->data[2],msg->data[4],msg->data[5],msg->data[6],msg->data[8],msg->data[9],msg->data[10]);
		pose_des_.p = KDL::Vector(msg->data[3],msg->data[7],msg->data[11]);
		
		cmd_flag_ = 1;  // set this flag to 0 to not run the update method
		
		IP_->CurrentPosition->VecData[0] = kuka_cart_pose_handle_.getTX();  // (double) ? set current position (transfrom to degrees) with current position of joint handles
		IP_->CurrentPosition->VecData[1] = kuka_cart_pose_handle_.getTY();  // (double) ? set current position (transfrom to degrees) with current position of joint handles
		IP_->CurrentPosition->VecData[2] = kuka_cart_pose_handle_.getTZ();  // (double) ? set current position (transfrom to degrees) with current position of joint handles
		
		IP_->TargetPosition->VecData[0]	= msg->data[3]; // (double) ? set desired position (get it from msg data of topic)
		IP_->TargetPosition->VecData[1]	= msg->data[7]; // (double) ? set desired position (get it from msg data of topic)
		IP_->TargetPosition->VecData[2]	= msg->data[11]; // (double) ? set desired position (get it from msg data of topic)
		
		//IP_->MaxVelocity->VecData[i] = (double)5.0;
		//IP_->MaxAcceleration->VecData[i] = (double)20.0;
		IP_->MaxVelocity->VecData[0] = (double)0.01;
		IP_->MaxVelocity->VecData[1] = (double)0.01;
		IP_->MaxVelocity->VecData[2] = (double)0.01;
		
		IP_->MaxAcceleration->VecData[0] = (double)0.005;
		IP_->MaxAcceleration->VecData[1] = (double)0.005;
		IP_->MaxAcceleration->VecData[2] = (double)0.005;
		
		IP_->SelectionVector->VecData[0] = true;
		IP_->SelectionVector->VecData[1] = true;
		IP_->SelectionVector->VecData[2] = true;
		
		resultValue_ = TypeIRML::RML_WORKING;
		
		cmd_flag_ = 1;  // set this flag to 0 to not run the update method
		
	//("Pos Act: %f, Pos IP Cur : %f, Pos IP Target %f",kuka_cart_pose_handle_.getTZ(),IP_->CurrentPosition->VecData[0],IP_->TargetPosition->VecData[0]);
		
		
	 }
	 
	 void HarshCartesianImpedanceController::setCartesianWrench(const std_msgs::Float64MultiArrayConstPtr& msg)
	 {
		#if TRACE_CARTESIAN_IMPEDANCE_CONTROLLER_ACTIVATED
			ROS_INFO("HarshCartesianImpedanceController: Start setCartesianWrench of robot %s!",robot_namespace_.c_str());
		#endif

		 if(msg->data.size()!=NUMBER_OF_CART_DOFS)
		 { 
			ROS_ERROR_STREAM("HarshCartesianImpedanceController: Dimension (of robot " << robot_namespace_.c_str() << ") of command (" << msg->data.size() << ") does not match number of DOF(" << NUMBER_OF_CART_DOFS << ")! Not executing!");
			return; 
		 }
		 
		 // 0: X, 1: Y, 2: Z, 3: A, 4: B, 5: C
		 wrench_cur_.force = KDL::Vector(msg->data[0],msg->data[1],msg->data[2]);
		 wrench_cur_.torque = KDL::Vector(msg->data[3],msg->data[4],msg->data[5]);
		  
	  }
	 	  
	void HarshCartesianImpedanceController::getCurrentPose_(KDL::Frame& frame)
	{
		 KDL::Rotation cur_R(kuka_cart_pose_handle_.getRXX(),
							kuka_cart_pose_handle_.getRXY(),
							kuka_cart_pose_handle_.getRXZ(),
							kuka_cart_pose_handle_.getRYX(),
							kuka_cart_pose_handle_.getRYY(),
							kuka_cart_pose_handle_.getRYZ(),
							kuka_cart_pose_handle_.getRZX(),
							kuka_cart_pose_handle_.getRZY(),
							kuka_cart_pose_handle_.getRZZ());
							
		KDL::Vector cur_T(kuka_cart_pose_handle_.getTX(),
						  kuka_cart_pose_handle_.getTY(),
						  kuka_cart_pose_handle_.getTZ());
						  
		frame = KDL::Frame(cur_R, cur_T);
	 
	}
	 
	void HarshCartesianImpedanceController::getExternalEstimatedForces_(KDL::Wrench& wrench)
	{
		wrench.force  = KDL::Vector(kuka_cart_wrench_handle_.getX(),kuka_cart_wrench_handle_.getY(),kuka_cart_wrench_handle_.getZ());
		wrench.torque = KDL::Vector(kuka_cart_wrench_handle_.getA(),kuka_cart_wrench_handle_.getB(),kuka_cart_wrench_handle_.getC());
	}
	 
	 
	void HarshCartesianImpedanceController::fromKDLtoFRI_(const KDL::Frame& frame_in, std::vector<double>& vect_Pose_FRI_out)
	{
		assert(vect_Pose_FRI_out.size() == NUMBER_OF_FRAME_ELEMENTS);
		vect_Pose_FRI_out[0] = frame_in.M.UnitX().x(); // RXX
		vect_Pose_FRI_out[1] = frame_in.M.UnitY().x(); // RXY
		vect_Pose_FRI_out[2] = frame_in.M.UnitZ().x(); // RXZ
		vect_Pose_FRI_out[3] = frame_in.p.x(); // TX
		vect_Pose_FRI_out[4] = frame_in.M.UnitX().y(); // RYX
		vect_Pose_FRI_out[5] = frame_in.M.UnitY().y(); // RYY
		vect_Pose_FRI_out[6] = frame_in.M.UnitZ().y(); // RYZ
		vect_Pose_FRI_out[7] = frame_in.p.y(); // TY
		vect_Pose_FRI_out[8] = frame_in.M.UnitX().z(); // RZX
		vect_Pose_FRI_out[9] = frame_in.M.UnitY().z(); // RZY
		vect_Pose_FRI_out[10] = frame_in.M.UnitZ().z(); // RZZ
		vect_Pose_FRI_out[11] = frame_in.p.z(); // TZ

	}
	 
	 
	void HarshCartesianImpedanceController::forwardCmdFRI_(const KDL::Frame& frame, const KDL::Stiffness& stiffness, const KDL::Stiffness& damping, const KDL::Wrench& wrench)
	{
	 // Transform a KDL frame to a 'FRI Pose vector' (Rotation && Translation)
	 fromKDLtoFRI_(frame, cur_Pose_FRI_);
	 
	 // Set Cartesian Pose command
	 kuka_cart_pose_handle_.setCommandRXX(cur_Pose_FRI_[0]);
	 kuka_cart_pose_handle_.setCommandRXY(cur_Pose_FRI_[1]);
	 kuka_cart_pose_handle_.setCommandRXZ(cur_Pose_FRI_[2]);
	 kuka_cart_pose_handle_.setCommandTX(cur_Pose_FRI_[3]);
	 
	 kuka_cart_pose_handle_.setCommandRYX(cur_Pose_FRI_[4]);
	 kuka_cart_pose_handle_.setCommandRYY(cur_Pose_FRI_[5]);
	 kuka_cart_pose_handle_.setCommandRYZ(cur_Pose_FRI_[6]);
	 kuka_cart_pose_handle_.setCommandTY(cur_Pose_FRI_[7]);
	 
	 kuka_cart_pose_handle_.setCommandRZX(cur_Pose_FRI_[8]);
	 kuka_cart_pose_handle_.setCommandRZY(cur_Pose_FRI_[9]);
	 kuka_cart_pose_handle_.setCommandRZZ(cur_Pose_FRI_[10]);
	 kuka_cart_pose_handle_.setCommandTZ(cur_Pose_FRI_[11]);
	 
	 //ROS_INFO("x: %f y: %f z: %f",cur_Pose_FRI_[3],cur_Pose_FRI_[7],cur_Pose_FRI_[11]);
		 
	 // Set Cartesian Stiffness command
	 kuka_cart_stiff_handle_.setCommandX(stiffness[0]); 
	 kuka_cart_stiff_handle_.setCommandY(stiffness[1]);
	 kuka_cart_stiff_handle_.setCommandZ(stiffness[2]);
	 kuka_cart_stiff_handle_.setCommandA(stiffness[3]); 
	 kuka_cart_stiff_handle_.setCommandB(stiffness[4]);
	 kuka_cart_stiff_handle_.setCommandC(stiffness[5]);
	 
	 // Set Cartesian Damping command
	 kuka_cart_damp_handle_.setCommandX(damping[0]); 
	 kuka_cart_damp_handle_.setCommandY(damping[1]);
	 kuka_cart_damp_handle_.setCommandZ(damping[2]);
	 kuka_cart_damp_handle_.setCommandA(damping[3]); 
	 kuka_cart_damp_handle_.setCommandB(damping[4]);
	 kuka_cart_damp_handle_.setCommandC(damping[5]);
	 
	 // Set Cartesian Wrench command 
	 kuka_cart_wrench_handle_.setCommandX(wrench.force.x());
	 kuka_cart_wrench_handle_.setCommandY(wrench.force.y());
	 kuka_cart_wrench_handle_.setCommandZ(wrench.force.z());
	 kuka_cart_wrench_handle_.setCommandA(wrench.torque.x());
	 kuka_cart_wrench_handle_.setCommandB(wrench.torque.y());
	 kuka_cart_wrench_handle_.setCommandC(wrench.torque.z());
		 
	}
	
	
	void HarshCartesianImpedanceController::publishCurrentPose(const KDL::Frame& f)
    {
        if (realtime_pose_pub_->trylock()) 
        {
            realtime_pose_pub_->msg_.header.stamp = ros::Time::now();
            tf::poseKDLToMsg(f, realtime_pose_pub_->msg_.pose);
            realtime_pose_pub_->unlockAndPublish();
        }
	}
	 
}

PLUGINLIB_EXPORT_CLASS(kuka_lwr_controllers::HarshCartesianImpedanceController, controller_interface::ControllerBase)
