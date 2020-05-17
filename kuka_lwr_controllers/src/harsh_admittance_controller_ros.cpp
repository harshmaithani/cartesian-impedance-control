#include <harsh_admittance_controller_ros.h>

#include <algorithm>
#include <Eigen/Dense>

// Utils for pseudo inverse and skew_symmetric
#include <utils/pseudo_inversion.h>
#include <utils/skew_symmetric.h>

// For plugin
#include <pluginlib/class_list_macros.h>

// Ros messages generated
#include <kuka_lwr_controllers/PoseRPY.h>

namespace kuka_lwr_controllers 
{
    HarshAdmittanceControllerROS::HarshAdmittanceControllerROS() {}
    HarshAdmittanceControllerROS::~HarshAdmittanceControllerROS() {}

    bool HarshAdmittanceControllerROS::init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n)
    {
      // ROS_INFO("***** START HarshAdmittanceControllerROS::init ************");
	
        if( !(KinematicChainControllerBase<hardware_interface::PositionJointInterface>::init(robot, n)) )
        {
            ROS_ERROR("Couldn't initilize HarshAdmittanceControllerROS controller.");
            return false;
        }

		// Create Solvers
        jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
        fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
        ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));
        ik_pos_solver_.reset(new KDL::ChainIkSolverPos_NR_JL(kdl_chain_,joint_limits_.min,joint_limits_.max,*fk_pos_solver_,*ik_vel_solver_));
        fk_vel_solver_.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain_));

        q_cmd_.resize(kdl_chain_.getNrOfJoints());
        J_.resize(kdl_chain_.getNrOfJoints());
        
        sub_command_ = nh_.subscribe("command", 1, &HarshAdmittanceControllerROS::command, this);
   	
		// Publishers
	    pub_cart_adm_kdl_pose_meters_     			= n.advertise<std_msgs::Float64MultiArray>("/cart_adm_kdl_pose_meters", 1000);
	    pub_cart_adm_kdl_pose_mm_     				= n.advertise<std_msgs::Float64MultiArray>("/cart_adm_kdl_pose_mm", 1000);
	    pub_cart_adm_kdl_pose_meters_rpy_rad_  		= n.advertise<std_msgs::Float64MultiArray>("/cart_adm_kdl_pose_meters_rpy_rad", 1000);
	    pub_cart_adm_kdl_pose_mm_rpy_deg_    		= n.advertise<std_msgs::Float64MultiArray>("/cart_adm_kdl_pose_mm_rpy_deg", 1000);
	    pub_cart_adm_joint_states_rad_ 				= n.advertise<std_msgs::Float64MultiArray>("/cart_adm_joint_states_rad", 1000);
	    pub_cart_adm_joint_states_deg_ 				= n.advertise<std_msgs::Float64MultiArray>("/cart_adm_joint_states_deg", 1000);
	    pub_cart_adm_joint_velocity_rads_  			= n.advertise<std_msgs::Float64MultiArray>("/cart_adm_joint_velocity_rads", 1000);
	    pub_cart_adm_joint_velocity_degs_  			= n.advertise<std_msgs::Float64MultiArray>("/cart_adm_joint_velocity_degs", 1000);
	    pub_cart_adm_kdl_cart_velocity_m_rad_  		= n.advertise<std_msgs::Float64MultiArray>("/cart_adm_kdl_cart_velocity_m_rad", 1000);
	    pub_cart_adm_kdl_cart_velocity_mm_deg_  	= n.advertise<std_msgs::Float64MultiArray>("/cart_adm_kdl_cart_velocity_mm_deg", 1000);

	    pub_cart_adm_joint_motor_torques_  			= n.advertise<std_msgs::Float64MultiArray>("/cart_adm_joint_motor_torques", 1000);
	    pub_cart_adm_kdl_jacobian_					= n.advertise<std_msgs::Float64MultiArray>("/cart_adm_kdl_jacobian", 1000);
	 	
        pub_current_joint_position_                 = n.advertise<kuka_lwr_controllers::PoseRPY>("/kuka_lwr_right/harsh_admittance_controller/current_joint_position", 1000);

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
        robot_joint_motor_torques_.data.resize(7);
        robot_jacobian_.data.resize(42);
		
  
	 	// Initialize local variables here
        
        jacobian_.resize(kdl_chain_.getNrOfJoints());
        
        joint_positions_.resize(kdl_chain_.getNrOfJoints());
        joint_velocities_.resize(kdl_chain_.getNrOfJoints());
        joint_positions_test_.resize(kdl_chain_.getNrOfJoints());
	 	
        // Get Current Joint positions, Joint Velocities, 
        for(int i=0; i < joint_handles_.size(); i++)
        {
            joint_msr_states_.q(i) = joint_handles_[i].getPosition();
            joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
            joint_des_states_.q(i) = joint_msr_states_.q(i);
        }

        // Computing Forward kinematics
        fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);
		fk_pos_solver_->JntToCart(joint_msr_states_.q, pose_current_);
		
		// Computing Inverse Kinematics
		ik_pos_solver_->CartToJnt(joint_msr_states_.q, x_, joint_positions_test_);

        // Desired posture is the current one
        x_des_ = x_;

        cmd_flag_ = 0;

       
   
        return true;
    }

    void HarshAdmittanceControllerROS::starting(const ros::Time& time)
    {
		ROS_INFO("***** HarshAdmittanceControllerROS::starting ************");
		cmd_flag_ = 0;  // set this flag to 0 to not run the update method
    }

    void HarshAdmittanceControllerROS::update(const ros::Time& time, const ros::Duration& period)
    {
	//ROS_INFO("***** HarshAdmittanceControllerROS::update debut ************");

        // get joint positions
        for(int i=0; i < joint_handles_.size(); i++)
        {
            joint_msr_states_.q(i) = joint_handles_[i].getPosition();
        }

        if (cmd_flag_)
        {
            // computing Jacobian
            jnt_to_jac_solver_->JntToJac(joint_msr_states_.q, J_);

            // computing J_pinv_
            pseudo_inverse(J_.data, J_pinv_);

            // computing forward kinematics
            fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);
            
           //std::cout<<"Current Pose x:"<<x_.p[0]<<" y:"<<x_.p[1]<<" z:"<<x_.p[2]<<std::endl;
            
            current_joint_position.position.x=x_.p[0];
            current_joint_position.position.y=x_.p[1];
            current_joint_position.position.z=x_.p[2];
            current_joint_position.orientation.roll=0;
            current_joint_position.orientation.pitch=0;
            current_joint_position.orientation.yaw=0;
            
            
           
            // Position Method
             
            // Computing Inverse Kinematics
		    ik_pos_solver_->CartToJnt(joint_msr_states_.q,x_des_,joint_positions_test_);
           // ROS_INFO("Measured Angles are: %f %f %f %f %f %f %f",joint_msr_states_.q(0),joint_msr_states_.q(1),joint_msr_states_.q(2),joint_msr_states_.q(3),joint_msr_states_.q(4),joint_msr_states_.q(5),joint_msr_states_.q(6));
           // ROS_INFO("IK Angles are: %f %f %f %f %f %f %f", joint_positions_test_(0), joint_positions_test_(1),joint_positions_test_(2),joint_positions_test_(3),joint_positions_test_(4),joint_positions_test_(5),joint_positions_test_(6));
            
            joint_des_states_.q(0) = joint_positions_test_(0);
            joint_des_states_.q(1) = joint_positions_test_(1);
            joint_des_states_.q(2) = joint_positions_test_(2);
            joint_des_states_.q(3) = joint_positions_test_(3);
            joint_des_states_.q(4) = joint_positions_test_(4);
            joint_des_states_.q(5) = joint_positions_test_(5);
            joint_des_states_.q(6) = joint_positions_test_(6);
            
            //
                        
            /* Velocity Method
            
            // end-effector position error
            x_err_.vel = (x_des_.p - x_.p);

            // getting quaternion from rotation matrix
            x_.M.GetQuaternion(quat_curr_.v(0),quat_curr_.v(1),quat_curr_.v(2),quat_curr_.a);
            x_des_.M.GetQuaternion(quat_des_.v(0),quat_des_.v(1),quat_des_.v(2),quat_des_.a);

            skew_symmetric(quat_des_.v, skew_);

            for (int i = 0; i < skew_.rows(); i++)
            {
                v_temp_(i) = 0.0;
                for (int k = 0; k < skew_.cols(); k++)
                    v_temp_(i) += skew_(i,k)*(quat_curr_.v(k));
            }

            // end-effector orientation error
            x_err_.rot = quat_curr_.a*quat_des_.v - quat_des_.a*quat_curr_.v - v_temp_;

            // computing q_dot
            for (int i = 0; i < J_pinv_.rows(); i++)
            {
                joint_des_states_.qdot(i) = 0.0;
                for (int k = 0; k < J_pinv_.cols(); k++)
                    joint_des_states_.qdot(i) += 0.07*J_pinv_(i,k)*x_err_(k); //removed scaling factor of .7
          
            }

            // integrating q_dot -> getting q (Euler method)
            for (int i = 0; i < joint_handles_.size(); i++)
                joint_des_states_.q(i) += period.toSec()*joint_des_states_.qdot(i);

			*/

            // joint limits saturation
            for (int i =0;  i < joint_handles_.size(); i++)
            {
                if (joint_des_states_.q(i) < joint_limits_.min(i))
                    joint_des_states_.q(i) = joint_limits_.min(i);
                if (joint_des_states_.q(i) > joint_limits_.max(i))
                    joint_des_states_.q(i) = joint_limits_.max(i);
            }

			
            if (Equal(x_, x_des_, 0.00001))
            {
                //ROS_INFO("On target");
                cmd_flag_ = 0;
            }
           
           
			// set controls for joints
            for (int i = 0; i < joint_handles_.size(); i++)
            {
               joint_handles_[i].setCommand(joint_des_states_.q(i));
            }

        }
        
		// Publish Robot Data	
	 

		for(int i=0; i < joint_handles_.size(); i++)
        {
            joint_msr_states_.q(i) = joint_handles_[i].getPosition();
        }
		
		// computing Jacobian
        jnt_to_jac_solver_->JntToJac(joint_msr_states_.q, jacobian_);

        // computing J_pinv_
        pseudo_inverse(jacobian_.data, J_pinv_);

        // computing forward kinematics
        fk_pos_solver_->JntToCart(joint_msr_states_.q, pose_current_);
        	
		// Pose in meters
		
		robot_pose_meters_.data[0]  = pose_current_.M(0,0);   //Rxx
		robot_pose_meters_.data[1]  = pose_current_.M(0,1);   //Rxy
		robot_pose_meters_.data[2]  = pose_current_.M(0,2);   //Rxz
		robot_pose_meters_.data[3]  = pose_current_.p[0];    //Tx
		robot_pose_meters_.data[4]  = pose_current_.M(1,0);   //Ryx
	    robot_pose_meters_.data[5]  = pose_current_.M(1,1);   //Ryy
		robot_pose_meters_.data[6]  = pose_current_.M(1,2);   //Ryz
		robot_pose_meters_.data[7]  = pose_current_.p[1];    //Ty
		robot_pose_meters_.data[8]  = pose_current_.M(2,0);   //Rzx
		robot_pose_meters_.data[9]  = pose_current_.M(2,1);   //Rzy
		robot_pose_meters_.data[10] = pose_current_.M(2,2);   //Rzz
	    robot_pose_meters_.data[11] = pose_current_.p[2];    //Tz
	    robot_pose_meters_.data[12] = 0;
	    robot_pose_meters_.data[13] = 0;
	    robot_pose_meters_.data[14] = 0;
	    robot_pose_meters_.data[15] = 1;
	    
	    
	    // Pose in mm
		robot_pose_mm_.data[0]  = pose_current_.M(0,0);   //Rxx
		robot_pose_mm_.data[1]  = pose_current_.M(0,1);   //Rxy
		robot_pose_mm_.data[2]  = pose_current_.M(0,2);   //Rxz
		robot_pose_mm_.data[3]  = 1000*pose_current_.p[0];    //Tx
		robot_pose_mm_.data[4]  = pose_current_.M(1,0);   //Ryx
	    robot_pose_mm_.data[5]  = pose_current_.M(1,1);   //Ryy
		robot_pose_mm_.data[6]  = pose_current_.M(1,2);   //Ryz
		robot_pose_mm_.data[7]  = 1000*pose_current_.p[1];    //Ty
		robot_pose_mm_.data[8]  = pose_current_.M(2,0);   //Rzx
		robot_pose_mm_.data[9]  = pose_current_.M(2,1);   //Rzy
		robot_pose_mm_.data[10] = pose_current_.M(2,2);   //Rzz
	    robot_pose_mm_.data[11] = 1000*pose_current_.p[2];    //Tz
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
			
		fk_pos_solver_->JntToCart(joint_positions_,pose_current_); 			// computing forward kinematics
		pose_current_.M.GetRPY (roll_,pitch_,yaw_);
	  
		robot_pose_meters_rpy_rad_.data[0]  = pose_current_.p[0];    //Tx
		robot_pose_meters_rpy_rad_.data[1]  = pose_current_.p[1];    //Ty
		robot_pose_meters_rpy_rad_.data[2]  = pose_current_.p[2];    //Tz
		robot_pose_meters_rpy_rad_.data[3]  = roll_;
		robot_pose_meters_rpy_rad_.data[4]  = pitch_;
	    robot_pose_meters_rpy_rad_.data[5]  = yaw_;
	
		// Pose in mm and rpy in deg
		
		robot_pose_mm_rpy_deg_.data[0]  = 1000*pose_current_.p[0];    //Tx
		robot_pose_mm_rpy_deg_.data[1]  = 1000*pose_current_.p[1];    //Ty
		robot_pose_mm_rpy_deg_.data[2]  = 1000*pose_current_.p[2];    //Tz
		robot_pose_mm_rpy_deg_.data[3]  = (180/3.14)*roll_;
		robot_pose_mm_rpy_deg_.data[4]  = (180/3.14)*pitch_;
	    robot_pose_mm_rpy_deg_.data[5]  = (180/3.14)*yaw_;
	
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
        
        // Joint Motor Torques in N-m
	    
	    robot_joint_motor_torques_.data[0]  =	joint_handles_[0].getEffort();
		robot_joint_motor_torques_.data[1]  =	joint_handles_[1].getEffort();
		robot_joint_motor_torques_.data[2]  =	joint_handles_[2].getEffort();
		robot_joint_motor_torques_.data[3]  =	joint_handles_[3].getEffort();
		robot_joint_motor_torques_.data[4]  =	joint_handles_[4].getEffort();
		robot_joint_motor_torques_.data[5]  =	joint_handles_[5].getEffort();
		robot_joint_motor_torques_.data[6]  =	joint_handles_[6].getEffort();
		
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
		
		
		// Publish on ROS
		
	    pub_cart_adm_kdl_pose_meters_.publish(robot_pose_meters_);
	    
	    pub_cart_adm_kdl_pose_mm_.publish(robot_pose_mm_);
	    pub_cart_adm_kdl_pose_meters_rpy_rad_.publish(robot_pose_meters_rpy_rad_);
	    pub_cart_adm_kdl_pose_mm_rpy_deg_.publish(robot_pose_mm_rpy_deg_);
	    pub_cart_adm_joint_states_deg_.publish(robot_joint_states_deg_);
	    pub_cart_adm_joint_states_rad_.publish(robot_joint_states_rad_);
	    pub_cart_adm_joint_velocity_rads_.publish(robot_joint_velocity_rads_);
	    pub_cart_adm_joint_velocity_degs_.publish(robot_joint_velocity_degs_);
	  
	    pub_cart_adm_joint_motor_torques_.publish(robot_joint_motor_torques_);
	
		pub_cart_adm_kdl_jacobian_.publish(robot_jacobian_);
		pub_cart_adm_kdl_cart_velocity_m_rad_.publish(robot_cartesian_velocity_m_rad_);
	    pub_cart_adm_kdl_cart_velocity_mm_deg_.publish(robot_cartesian_velocity_mm_deg_);
        
	pub_current_joint_position_.publish(current_joint_position);

	//ROS_INFO("***** HarshAdmittanceControllerROS::update fin ************");
    }

    void HarshAdmittanceControllerROS::command(const std_msgs::Float64MultiArrayConstPtr& msg)
    {
		//ROS_INFO("***** START HarshAdmittanceControllerROS::command ************");

        KDL::Frame frame_des_;

		frame_des_ = KDL::Frame(
        KDL::Rotation::RPY(msg->data[3],
        msg->data[4],
        msg->data[5]),
        KDL::Vector(msg->data[0],
        msg->data[1],
        msg->data[2]));
           
        x_des_ = frame_des_;
        cmd_flag_ = 1;
        
       // ROS_INFO("***** FINISH HarshAdmittanceControllerROS::command ************");
    }

}

PLUGINLIB_EXPORT_CLASS(kuka_lwr_controllers::HarshAdmittanceControllerROS, controller_interface::ControllerBase)

