/*
 *  Kamal mohy el dine 
 *  Institut Pascal UMR6602
 *  kamal.mohy.el.dine@gmail.com
 * 
*/

#include <operational_space_controller.h>
#include <algorithm>
#include <Eigen/Dense>
#include <utils/pseudo_inversion.h>
#include "math.h"

// For plugin
#include <pluginlib/class_list_macros.h>

namespace kuka_lwr_controllers 
{

	OperationalSpaceController::OperationalSpaceController() {}
    	OperationalSpaceController::~OperationalSpaceController() {}

   	bool OperationalSpaceController::init(hardware_interface::KUKAJointInterface *robot, ros::NodeHandle &n)
    	{

		robot_namespace_ = n.getNamespace();
		
		#if TRACE_OperationalSpaceController_ACTIVATED
			ROS_INFO("OperationalSpaceController: Start init of robot %s !",robot_namespace_.c_str());
		#endif
			
        	if( !(KinematicChainControllerBase<hardware_interface::KUKAJointInterface>::init(robot, n)) )
        	{
            		ROS_ERROR("OperationalSpaceController: Couldn't initilize OperationalSpaceController controller of robot %s!",robot_namespace_.c_str());
            		return false;
        	}
        //initializes the solvers
        jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
		fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
		fk_vel_solver_.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain_));
        id_solver_.reset(new KDL::ChainDynParam( kdl_chain_, gravity_));

		
		// publishers
		pub_robot_data_         	= n.advertise<std_msgs::Float64MultiArray>("/robot_data", 1000);
		pub_endeffector_position_ 	= n.advertise<geometry_msgs::PointStamped>("/end_effector_position", 1000);
		//subscribers
		sub_imu_					= n.subscribe("/acceleration",1, &OperationalSpaceController::imuCB, this);
		sub_ft_ 					= n.subscribe("/FT13855/sensor_readings",1, &OperationalSpaceController::ft_readingsCB, this);
        sub_command_ 				= n.subscribe("/command", 1, &OperationalSpaceController::qcommandCB, this);
        sub_kp_cartesian_ 			= n.subscribe("setcartesianKp"		,1, &OperationalSpaceController::setcartesianKp	, this);
        sub_kd_cartesian_ 			= n.subscribe("setcartesianKd"		,1, &OperationalSpaceController::setcartesianKd	, this);
		sub_traj_			        = n.subscribe("/cart_traj_cmd"			,1, &OperationalSpaceController::PathPointDesiredCB	, this);

		/************** init variables here ********************************/
		robot_data_.data.resize(47);
		end_effector_pos_msg_.header.frame_id = "kuka_lwr_right_base_link";

		//Creating Jnt_vel_ object of type JntArrayVel as attribute for the fk_vel_solver_->JntToCart(Jnt_vel_, V_current_);
		Jnt_vel_.resize(kdl_chain_.getNrOfJoints());
		Jnt_vel_.q		=	joint_msr_states_.q;
		Jnt_vel_.qdot	=	joint_msr_states_.qdot;
		
		stiff_.resize(joint_handles_.size());
		damp_.resize(joint_handles_.size());
		
		for (std::size_t i=0; i<joint_handles_.size(); i++)
		{
			stiff_(i) 	= 0.0;
			damp_(i)	= 0.0;
		}
		
		FT_sensor_	= VectorXd::Zero(6);

		q_des_.resize(kdl_chain_.getNrOfJoints());
		for (std::size_t i=0; i<joint_handles_.size(); i++)
		{
			q_des_(i) 	= joint_handles_[i].getPosition();
		}
		
		M_.resize(kdl_chain_.getNrOfJoints());
		C_.resize(kdl_chain_.getNrOfJoints());
		G_.resize(kdl_chain_.getNrOfJoints());
		Jkdl_.resize(kdl_chain_.getNrOfJoints());
		Tau_cmd_	= VectorXd::Zero(kdl_chain_.getNrOfJoints());
		LAMDA_.resize(6,6);
		p_resp_ = MatrixXd:: Zero(6,1);
		v_resp_ = MatrixXd:: Zero(6,1);
		traj_des_.resize(6);
		SetToZero (traj_des_);
		Sv_         = MatrixXd::Zero(6,6);
		Sf_         = MatrixXd::Zero(6,6);
		F_lamda_.resize(6);
		Hc_.resize(6);
		KDv_ 		= MatrixXd::Zero(6,6);
		KPv_ 		= MatrixXd::Zero(6,6);
		KPl_        = MatrixXd::Zero(6,6);
		Alpha_v_ 	= MatrixXd::Zero(6,1);
		force_des_.resize(6);
		force_des_<<0,0,2,0,0,0;
		std::cout<<"tau is ="<<force_des_<<"\n";
		Kp_cartesian_.resize(6);
		Kd_cartesian_.resize(6);
        Kdf_force_.resize(6);
		for (std::size_t i=0; i<3; i++)
		{		
			Kp_cartesian_(i) = 50.0;   //800.0
			Kd_cartesian_(i) = 10.0;    //50.0
			
			KPv_(i,i )=Kp_cartesian_(i);
			KDv_(i,i )=Kd_cartesian_(i);
		} 
		for (std::size_t i=3; i<6; i++)
		{		
			Kp_cartesian_(i) = 50.0;   //200.0
			Kd_cartesian_(i) = 10.0;    //50.0
			
			KPv_(i,i )=Kp_cartesian_(i);
			KDv_(i,i )=Kd_cartesian_(i);
			for(std::size_t i=0; i<6; i++)
		        {
		        Kdf_force_(i)=0.8;
		        KPl_ (i,i)=Kdf_force_(i);
	            }
	             using Eigen::MatrixXd;
	   
	     MatrixXd  matB(6,1);
         matB<<1,1,0,1,1,1;   //Selection Matrix
          for(int i=0; i<6;i++)
           {
                Sv_(i,i) += matB(i); // this addition creat a diagonal with vector(1,1,0,1,1,1)
           }
     Eigen::Matrix< double, 6, 1> v ;
             v << 1, 1, 1,1,1,1;// vector 6,1
     Eigen::Matrix< double, 6, 6> ID = v.array().matrix().asDiagonal(); // create a matrix(6,6)  with 1,1,1,1,1,1 in diagonal 
    Sf_=ID-Sv_;
		}	
		return true;
	}

	void OperationalSpaceController::starting(const ros::Time& time)
    {      
		#if TRACE_ACTIVATED
			ROS_INFO("OperationalSpaceController: Starting of robot %s !",robot_namespace_.c_str());
		#endif
		count = 0;
		for(size_t i=0; i<joint_handles_.size(); i++) 
		{		
			joint_msr_states_.q(i) = joint_handles_[i].getPosition();
			joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
			joint_msr_states_.qdotdot(i) = joint_handles_[i].getAcceleration();
		}
		fk_pos_solver_->JntToCart(joint_msr_states_.q, P_current_); 					
		Jnt_vel_.q=joint_msr_states_.q;													
		Jnt_vel_.qdot=joint_msr_states_.qdot;
		fk_vel_solver_->JntToCart(Jnt_vel_, V_current_);
		jnt_to_jac_solver_->JntToJac(joint_msr_states_.q, Jkdl_);						

		id_solver_->JntToMass(joint_msr_states_.q, M_);									
		id_solver_->JntToCoriolis(joint_msr_states_.q, joint_msr_states_.qdot, C_);		
		id_solver_->JntToGravity(joint_msr_states_.q, G_);								
		
		traj_des_.q.data(0) = P_current_.p.x();
		traj_des_.q.data(1) = P_current_.p.y();
		traj_des_.q.data(2) = P_current_.p.z();
		P_current_.M.GetRPY (roll_,pitch_,yaw_);
	    //P_current_.M.GetEulerZYX (roll_,pitch_,yaw_);

		traj_des_.q.data(3) = roll_;
		traj_des_.q.data(4) = pitch_;
		traj_des_.q.data(5) = yaw_;
		std::cout<<"pose_init:\n"<<P_current_.p.x()<<" "<<P_current_.p.y()<<" "<<P_current_.p.z()<<" "<<roll_<<" "<<pitch_<<" "<<yaw_<<std::endl;
	}

	void OperationalSpaceController::stopping(const ros::Time& time)
	{
		#if TRACE_ACTIVATED
			ROS_INFO_NAMED("OperationalSpaceController: Stopping of robot %s !",robot_namespace_.c_str());
		#endif
	}

	void OperationalSpaceController::update(const ros::Time& time, const ros::Duration& period)
    {
		for(size_t i=0; i<joint_handles_.size(); i++) 
		{		
			joint_msr_states_.q(i) = joint_handles_[i].getPosition();
			joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
			joint_msr_states_.qdotdot(i) = joint_handles_[i].getAcceleration();
		}
		

		fk_pos_solver_->JntToCart(joint_msr_states_.q, P_current_); 					// computing forward kinematics
		Jnt_vel_.q=joint_msr_states_.q;													
		Jnt_vel_.qdot=joint_msr_states_.qdot;
		fk_vel_solver_->JntToCart(Jnt_vel_, V_current_);
		jnt_to_jac_solver_->JntToJac(joint_msr_states_.q, Jkdl_);						// computing Jacobian with KDL

		id_solver_->JntToMass(joint_msr_states_.q, M_);									// computing Inertia matrix		
		id_solver_->JntToCoriolis(joint_msr_states_.q, joint_msr_states_.qdot, C_);		// computing Coriolis torques	
		id_solver_->JntToGravity(joint_msr_states_.q, G_);								// computing Gravity torques
		
		// screen output
		if(count%1000==0)
		{	
		std::cout<<"pose:\n"<<P_current_.p.x()<<" "<<P_current_.p.y()<<" "<<P_current_.p.z()<<" "<<roll_<<" "<<pitch_<<" "<<yaw_<<std::endl;
		}
		/*****************Control functions***************/
		
		CalculatePsuedoInertia(Jkdl_.data, M_.data, LAMDA_ );
		CalculateAccelerationCommand(P_current_, V_current_,traj_des_,Alpha_v_ );		
		CalculateJointTorques(Jkdl_.data,Alpha_v_, Tau_cmd_);	
		CheckTorqueMax_(Tau_cmd_);
		CalculateForce( FT_sensor_, KPl_	, force_des_,  F_lamda_ );
		controlWrench_h_c( Sv_,  Sf_ ,  LAMDA_ ,   Alpha_v_  , F_lamda_, Hc_ );
		/*************************************************/
		for(size_t i=0; i<joint_handles_.size(); i++) 
        {
			joint_handles_[i].setCommandPosition(joint_handles_[i].getPosition());
            joint_handles_[i].setCommandTorque(Tau_cmd_(i)); 
            joint_handles_[i].setCommandStiffness(stiff_(i)*0);
			joint_handles_[i].setCommandDamping(damp_(i)*0);
        }
        joint_handles_[2].setCommandPosition(0);

		/*************************************************/
		/*
		for(size_t i=0; i<joint_handles_.size(); i++) 
        {
			joint_handles_[i].setCommandPosition(joint_handles_[i].getPosition());
            joint_handles_[i].setCommandTorque(0.0); 
            joint_handles_[i].setCommandStiffness(stiff_(i));
			joint_handles_[i].setCommandDamping(damp_(i));
        }
		*/
		/**************************************************/
		/****** messages to be published for analyses *****/
		/**************************************************/
		// end effector position topic publishing 
		end_effector_pos_msg_.point.x = P_current_.p.x();
		end_effector_pos_msg_.point.y = P_current_.p.y();
		end_effector_pos_msg_.point.z = P_current_.p.z();
		pub_endeffector_position_.publish(end_effector_pos_msg_);

		// publishing robot data
		robot_data_.data[0] = (float) ros::Time::now().toSec();

		robot_data_.data[1] = P_current_.p.x();
		robot_data_.data[2] = P_current_.p.y();
		robot_data_.data[3] = P_current_.p.z();
		
		P_current_.M.GetQuaternion (orientation_x_, orientation_y_, orientation_z_, orientation_w_);
		robot_data_.data[4] = orientation_x_;
		robot_data_.data[5] = orientation_y_;
		robot_data_.data[6] = orientation_z_;
		robot_data_.data[7] = orientation_w_;
		
		robot_data_.data[8]   = V_current_.GetTwist ().vel.x();
		robot_data_.data[9]   = V_current_.GetTwist ().vel.y();
		robot_data_.data[10]  = V_current_.GetTwist ().vel.z();
		robot_data_.data[11]  = V_current_.GetTwist ().rot.x();
		robot_data_.data[12]  = V_current_.GetTwist ().rot.y();
		robot_data_.data[13]  = V_current_.GetTwist ().rot.z(); 
		// arduino 
		robot_data_.data[14]  = IMU_.accel.linear.x;
		robot_data_.data[15]  = IMU_.accel.linear.y;
		robot_data_.data[16]  = IMU_.accel.linear.z;
		robot_data_.data[17]  = IMU_.accel.angular.x;
		robot_data_.data[18]  = IMU_.accel.angular.y;
		robot_data_.data[19]  = IMU_.accel.angular.z;
		// force sensor
		robot_data_.data[20]  =	FT_sensor_(0);
		robot_data_.data[21]  =	FT_sensor_(1);
		robot_data_.data[22]  =	FT_sensor_(2);
		robot_data_.data[23]  =	FT_sensor_(3);
		robot_data_.data[24]  =	FT_sensor_(4);
		robot_data_.data[25]  =	FT_sensor_(5);
		// joint positions
		robot_data_.data[26]  =	joint_msr_states_.q(0);
		robot_data_.data[27]  =	joint_msr_states_.q(1);
		robot_data_.data[28]  =	joint_msr_states_.q(2);
		robot_data_.data[29]  =	joint_msr_states_.q(3);
		robot_data_.data[30]  =	joint_msr_states_.q(4);
		robot_data_.data[31]  =	joint_msr_states_.q(5);
		robot_data_.data[32]  =	joint_msr_states_.q(6);
		// joint velocities
		robot_data_.data[33]  =	joint_msr_states_.qdot(0);
		robot_data_.data[34]  =	joint_msr_states_.qdot(1);
		robot_data_.data[35]  =	joint_msr_states_.qdot(2);
		robot_data_.data[36]  =	joint_msr_states_.qdot(3);
		robot_data_.data[37]  =	joint_msr_states_.qdot(4);
		robot_data_.data[38]  =	joint_msr_states_.qdot(5);
		robot_data_.data[39]  =	joint_msr_states_.qdot(6);	
		// joint torque sensors
		robot_data_.data[40]  =	joint_handles_[0].getTorque();
		robot_data_.data[41]  =	joint_handles_[1].getTorque();
		robot_data_.data[42]  =	joint_handles_[2].getTorque();
		robot_data_.data[43]  =	joint_handles_[3].getTorque();
		robot_data_.data[44]  =	joint_handles_[4].getTorque();
		robot_data_.data[45]  =	joint_handles_[5].getTorque();
		robot_data_.data[46]  =	joint_handles_[6].getTorque();
			
		pub_robot_data_.publish(robot_data_);
		count ++;
	}
	void OperationalSpaceController::PathPointDesiredCB(const kuka_lwr_controllers::PathPointCaracteristics::ConstPtr & msg)
    {
		
			// position command
			traj_des_.q.data(0) = (double)msg->pos.x;
			traj_des_.q.data(1) = (double)msg->pos.y;
			traj_des_.q.data(2) = (double)msg->pos.z;
			traj_des_.q.data(3) = (double)msg->ori.x;
			traj_des_.q.data(4) = (double)msg->ori.y;
			traj_des_.q.data(5) = (double)msg->ori.z;
			// velocity command
			traj_des_.qdot.data(0) = (double)msg->vel.x;
			traj_des_.qdot.data(1) = (double)msg->vel.y;
			traj_des_.qdot.data(2) = (double)msg->vel.z;
			traj_des_.qdot.data(3) = (double)msg->dot_ori.x;
			traj_des_.qdot.data(4) = (double)msg->dot_ori.y;
			traj_des_.qdot.data(5) = (double)msg->dot_ori.z;		
			// acceleration command
			traj_des_.qdotdot.data(0) = (double)msg->acc.x;
			traj_des_.qdotdot.data(1) = (double)msg->acc.y;
			traj_des_.qdotdot.data(2) = (double)msg->acc.z;
			traj_des_.qdotdot.data(3) = (double)msg->dotdot_ori.x;
			traj_des_.qdotdot.data(4) = (double)msg->dotdot_ori.y;
			traj_des_.qdotdot.data(5) = (double)msg->dotdot_ori.z;
			//ROS_INFO("OperationalSpaceController: PathPointDesiredCB  msg->pos.x = %f !",(double)msg->pos.x);
		    
	}
	
	void OperationalSpaceController::imuCB(const  geometry_msgs::AccelStamped& msg){
		IMU_ = msg;
	}
	void OperationalSpaceController::ft_readingsCB(const geometry_msgs::WrenchStamped& msg)
    {		
		FT_sensor_(0) = msg.wrench.force.x ;
		FT_sensor_(1) = msg.wrench.force.y ;
		FT_sensor_(2) = msg.wrench.force.z ;
		FT_sensor_(3) = msg.wrench.torque.x ;
		FT_sensor_(4) = msg.wrench.torque.y ;
		FT_sensor_(5) = msg.wrench.torque.z ;	
	}
	void OperationalSpaceController::qcommandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
    {
		#if TRACE_Torque_Based_Position_ACTIVATED
			ROS_INFO("OperationalSpaceController: Start qcommandCB of robot %s!",robot_namespace_.c_str());
		#endif

		if(msg->data.size()!=joint_handles_.size())
		{ 
			ROS_ERROR_STREAM("OperationalSpaceController: Dimension (of robot " << robot_namespace_.c_str() << ") of command (" << msg->data.size() << ") does not match number of joints (" << joint_handles_.size() << ")! Not executing!");
			return; 
		}
		
		for (std::size_t i=0; i<msg->data.size(); i++)
		{
			q_des_(i) = (double)msg->data[i];
		}

		
		
		ROS_INFO("-> qdes values : %f, %f, %f, %f, %f, %f, %f!",q_des_(0), q_des_(1), q_des_(2), q_des_(3), q_des_(4), q_des_(5), q_des_(6));
		
	}
	void OperationalSpaceController::CheckTorqueMax_(Eigen::VectorXd & TorqueCommandVector)
	{
		
		double maxTorqueArray[] = { 176.0, 176.0, 100.0, 100.0, 100.0, 38.0, 38.0 };
		double minTorqueArray[] = { -176.0, -176.0, -100.0, -100.0, -100.0, -38.0, -38.0 };

		for (size_t i=0; i<TorqueCommandVector.size(); ++i)
		{
			if (TorqueCommandVector(i) > maxTorqueArray[i])
			{
				ROS_INFO("Current joint n%i torque = %f -> Max allowed value = %f", (int)i, TorqueCommandVector(i), maxTorqueArray[i]);
				TorqueCommandVector(i) = maxTorqueArray[i] - 5.0;	 
			}
			
			if (TorqueCommandVector(i) < minTorqueArray[i])
			{
				ROS_INFO("Current joint n%i torque = %f -> Min allowed value = %f", (int)i, TorqueCommandVector(i), minTorqueArray[i]);
				TorqueCommandVector(i) = minTorqueArray[i] + 5.0;	 
			}
		}
	}
	void OperationalSpaceController::setcartesianKp(const std_msgs::Float64MultiArrayConstPtr& msg)
    {
			ROS_INFO("OperationalSpaceController: In setcartesianKp of robot %s!",robot_namespace_.c_str());

		if(msg->data.size()!=6)
		{ 
			ROS_ERROR_STREAM("OperationalSpaceController: setcartesianKp Dimension (of robot " << robot_namespace_.c_str() << ") of command (" << msg->data.size() << ") does not match number of joints (" << joint_handles_.size() << ")! Not executing!");
			return; 
		}
				
		for (std::size_t i=0; i<msg->data.size(); i++)
		{
			KPv_(i,i) = (double)msg->data[i];
		}
		
	}
	void OperationalSpaceController::setcartesianKd(const std_msgs::Float64MultiArrayConstPtr& msg)
    {
			ROS_INFO("OperationalSpaceController: In setcartesianKd of robot %s!",robot_namespace_.c_str());

		if(msg->data.size()!=6)
		{ 
			ROS_ERROR_STREAM("OperationalSpaceController: setcartesianKd Dimension (of robot " << robot_namespace_.c_str() << ") of command (" << msg->data.size() << ") does not match number of joints (" << joint_handles_.size() << ")! Not executing!");
			return; 
		}
		for (std::size_t i=0; i<msg->data.size(); i++)
		{
			KDv_(i,i) = (double)msg->data[i];
		}
	}
	
	void OperationalSpaceController:: CalculatePsuedoInertia(const Eigen::MatrixXd  & j, const Eigen::MatrixXd & i, Eigen::MatrixXd & lamda )
	{
		lamda = ((j)*(i.inverse())*(j.transpose())).inverse();
	}
	void OperationalSpaceController:: CalculateAccelerationCommand(const KDL::Frame  & p_current, const KDL::FrameVel & v_current, const KDL::JntArrayAcc & traj_des, Eigen::MatrixXd & alpha_v )
	{		
		p_resp_(0,0) = p_current.p.x();
		p_resp_(1,0) = p_current.p.y();
		p_resp_(2,0) = p_current.p.z();
		
		p_current.M.GetRPY (roll_,pitch_,yaw_);
	    //p_current.M.GetEulerZYX (roll_,pitch_,yaw_);

		p_resp_(3,0) = roll_;
		p_resp_(4,0) = pitch_;
		p_resp_(5,0) = yaw_;
		
		v_resp_(0,0) = v_current.GetTwist ().vel.x();
		v_resp_(1,0) = v_current.GetTwist ().vel.y();
		v_resp_(2,0) = v_current.GetTwist ().vel.z();
		v_resp_(3,0) = v_current.GetTwist ().rot.x();
		v_resp_(4,0) = v_current.GetTwist ().rot.y();
		v_resp_(5,0) = v_current.GetTwist ().rot.z(); 
		 
		alpha_v = (traj_des_.qdotdot.data + KDv_*(traj_des.qdot.data - v_resp_)  + KPv_*(traj_des.q.data - p_resp_));

		//alpha_v =   KPv_*(traj_des.q.data - p_resp_);

	}
	void OperationalSpaceController::CalculateForce(Eigen::VectorXd & FT_sensor_,Eigen::MatrixXd KPl_, Eigen::VectorXd & force_des_, Eigen::VectorXd	 & F_lamda_ )
  {
		
   F_lamda_=force_des_+KPl_*(force_des_-FT_sensor_);
   
   std::cout<<"F_lamda_ is ="<<F_lamda_<<"\n";
  }
    void OperationalSpaceController::controlWrench_h_c( Eigen::MatrixXd	& Sv_, Eigen::MatrixXd & Sf_ , Eigen::MatrixXd & lamda ,  Eigen::MatrixXd & alpha_v , Eigen::VectorXd	 & F_lamda_,Eigen::VectorXd & Hc_ )
    {
    Hc_=LAMDA_*Sv_*Alpha_v_ +Sf_*F_lamda_;
    std::cout<<"Inertia Matrix is ="<<lamda<<" "<<"\n";
    //std::cout<<"Sv is ="<<Sv_<<" "<<"\n";
    //std::cout<<"Sf is ="<<Sf_<<" "<<"\n";
    std::cout<<"Force result is ="<<F_lamda_<<"\n";
    std::cout<<"Decoupling position/force is = "<<Hc_<<" "<<"\n";
    
    }
	void OperationalSpaceController :: CalculateJointTorques(const Eigen::MatrixXd & j,const Eigen::MatrixXd & alpha_v, Eigen::VectorXd & Tau_cmd_  )
	{
		std::cout<<"Acceleration control is"<<alpha_v<<"\n";
		std::cout<<LAMDA_*alpha_v<<std::endl;
		
		//Tau_cmd_ = j.transpose()*(LAMDA_*alpha_v);   // Position Control Equation 
		Tau_cmd_ = j.transpose()*(Hc_);            // Hybrid Control Equation 
		std::cout<<"Tau is ="<<Tau_cmd_ <<"\n";
		/*
		std::cout<<"Jacobian:\n";
		std::cout<< j;
		std::cout<<"\n LAMDA_:\n";
		std::cout<< LAMDA_;
		std::cout<<"\n alpha_v:\n";
		std::cout<< alpha_v;
		std::cout<<"\n";
		*/
		/*
		std::cout<<"rotation matrix data:\n";
		for (std::size_t i=0; i<9; i++)
		std::cout<< P_current_.M.data[i]<<";";
		std::cout<<"\n";
		*/
	}
}

PLUGINLIB_EXPORT_CLASS(kuka_lwr_controllers::OperationalSpaceController, controller_interface::ControllerBase)


