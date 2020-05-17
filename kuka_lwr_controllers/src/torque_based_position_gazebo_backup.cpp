/*
 *  Kamal mohy el dine 
 *  Institut Pascal UMR6602
 *  kamal.mohy.el.dine@gmail.com
 * 
*/

#include <torque_based_position_gazebo.h>
#include "math.h"
// For plugin
#include <pluginlib/class_list_macros.h>

#include <algorithm>
#include <Eigen/Dense>
#include <utils/pseudo_inversion.h>
#include "pid.h"

namespace kuka_lwr_controllers 
{
    TorqueBasedPositionControllerGazebo::TorqueBasedPositionControllerGazebo() {}
    TorqueBasedPositionControllerGazebo::~TorqueBasedPositionControllerGazebo() 
    {
		delete RML_;
		delete IP_;
		delete OP_;	
		
		delete RML_Q_;
		delete IP_Q_;
		delete OP_Q_;	
	}

   bool TorqueBasedPositionControllerGazebo::init(hardware_interface::KUKAJointInterface *robot, ros::NodeHandle &n)
    {
		
		robot_namespace_ = n.getNamespace();
		
		#if TRACE_Torque_Based_Position_ACTIVATED
			ROS_INFO("TorqueBasedPositionController: Start init of robot %s !",robot_namespace_.c_str());
		#endif
			
        if( !(KinematicChainControllerBase<hardware_interface::KUKAJointInterface>::init(robot, n)) )
        {
            ROS_ERROR("TorqueBasedPositionController: Couldn't initilize TorqueBasedPositionController controller of robot %s!",robot_namespace_.c_str());
            return false;
        }
        //initializes the solvers
        jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
        id_solver_.reset(new KDL::ChainDynParam( kdl_chain_, gravity_));
		fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
		fk_vel_solver_.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain_));
		
		
		
       // sub_command_ 		= n.subscribe("command"				,1, &TorqueBasedPositionControllerGazebo::commandCB			, this);
        sub_kp_cartesian_ 	= n.subscribe("setcartesianKp"		,1, &TorqueBasedPositionControllerGazebo::setcartesianKp	, this);
        sub_kd_cartesian_ 	= n.subscribe("setcartesianKd"		,1, &TorqueBasedPositionControllerGazebo::setcartesianKd	, this);
		sub_traj_			= n.subscribe("/traj_cmd"			,1, &TorqueBasedPositionControllerGazebo::TrajPathPointCB	, this);
		sub_traj_initial_	= n.subscribe("/traj_cmd_initial"	,1, &TorqueBasedPositionControllerGazebo::TrajPathPointCBInitial, this);
		sub_ft_ 			= n.subscribe("/sensor_readings"		,1, &TorqueBasedPositionControllerGazebo::ft_readingsCB		, this);
		sub_kp_joints_  	= n.subscribe("setjointsKp"			,1, &TorqueBasedPositionControllerGazebo::setjointsKp		, this); 
		sub_kd_joints_  	= n.subscribe("setjointsKd"			,1, &TorqueBasedPositionControllerGazebo::setjointsKd		, this); 
		sub_kp_force_		= n.subscribe("setforceKp"			,1, &TorqueBasedPositionControllerGazebo::setforceKp		, this); 
		sub_ee_pos_			= n.subscribe("/aruco_single/pose"	,1, &TorqueBasedPositionControllerGazebo::setRollPitchYaw	, this); 
		sub_stiffness_damping_ = n.subscribe("setStiffnessDamping"		,1, &TorqueBasedPositionControllerGazebo::setStiffnessDamping	, this); 

		pub_traj_resp_		= n.advertise<kuka_lwr_controllers::TrajPathPoint>("traj_resp", 1000);
		pub_tau_cmd_		= n.advertise<std_msgs::Float64MultiArray>("tau_cmd", 1000);
		pub_F_des_			= n.advertise<geometry_msgs::WrenchStamped>("F_des", 1000);
		pub_Sv				= n.advertise<std_msgs::Float64>("Sv", 1000);
		pub_Sf 				= n.advertise<std_msgs::Float64>("Sf", 1000);
		cmd_flag_ = 0;  // set this flag to 0 to not run the update method
		d_wall_ = 0.5; // 
	

		//resizing the used vectors
		M_.resize(kdl_chain_.getNrOfJoints());
		C_.resize(kdl_chain_.getNrOfJoints());
		G_.resize(kdl_chain_.getNrOfJoints());
		Jkdl_.resize(kdl_chain_.getNrOfJoints());
		Jnt_vel_.resize(kdl_chain_.getNrOfJoints());
		//Creating Jnt_vel_ object of type JntArrayVel as attribute for the fk_vel_solver_->JntToCart(Jnt_vel_, V_current_);
		Jnt_vel_.q=joint_msr_states_.q;
		Jnt_vel_.qdot=joint_msr_states_.qdot;
		
		Kp_joints_.resize(kdl_chain_.getNrOfJoints());
		Kd_joints_.resize(kdl_chain_.getNrOfJoints());
		stiff_.resize(kdl_chain_.getNrOfJoints());
		damp_.resize(kdl_chain_.getNrOfJoints());
		
		initialPositions_.resize(kdl_chain_.getNrOfJoints());

		
		for (std::size_t i=0; i<joint_handles_.size()-3; i++)
		{
			Kp_joints_(i) = 200.0;
			Kd_joints_(i) = 50.0;
		}
		for (std::size_t i=4; i<joint_handles_.size(); i++)
		{
			Kp_joints_(i) = 50.0;
			Kd_joints_(i) = 10.0;
		}
		for (std::size_t i=0; i<joint_handles_.size()-3; i++)
		{
			stiff_(i) = 1000.0;
			damp_(i) = 1.0;
		}
		/*for (std::size_t i=4; i<joint_handles_.size(); i++)
		{
			stiff_(i) = 500.0;
			damp_(i) = 1.0;
		}*/
			stiff_(4) = 500.0;
			damp_(4) = 1.0;
			stiff_(5) = 500.0;
			damp_(5) = 1.0;
			stiff_(6) = 500.0;
			damp_(6) = 1.0;
		Kp_cartesian_ .resize(6);
		Kd_cartesian_ .resize(6);
		
		KDv_ 		= MatrixXd::Zero(6,6);
		KPv_ 		= MatrixXd::Zero(6,6);
		KPf_ 		= MatrixXd::Zero(6,6);

						
 		for (std::size_t i=0; i<3; i++)
		{		
			Kp_cartesian_(i) = 1000.0;
			Kd_cartesian_(i) = 50.0;
			
			KPv_(i,i )=Kp_cartesian_(i);
			KDv_(i,i )=Kd_cartesian_(i);
		}    
		KPf_(0,0) = 10;
		
		S_v_    	= MatrixXd::Identity(6,6);				// selection matrix for position controlled direction
		S_v_.bottomRightCorner(3,3) = MatrixXd::Zero(3, 3);
		S_f_    	= MatrixXd::Zero(6,6);					// selection matrix for velocity controlled direction
		
		// setting the values of the initial point 
		traj_des_.resize(6);
		SetToZero (traj_des_);
		
		/*traj_des_.q.data(0) = 0.281;
		traj_des_.q.data(1) = -0.231;
		traj_des_.q.data(2) = 0.8547;*/
		
		// initial position for testing the force control
		/*
		traj_des_.q.data(0) = 0.5905;
		traj_des_.q.data(1) = -0.001;
		traj_des_.q.data(2) = 0.72117;
		*/
		q_des_.resize(kdl_chain_.getNrOfJoints());
		SetToZero (q_des_);
		

	
		LAMDA_.resize(6,6);
		Alpha_v_ 	= MatrixXd::Zero(6, 1);
		FT_sensor_	= VectorXd::Zero(6);
		F_des_		= VectorXd::Zero(6);
		F_des_max_		= VectorXd::Zero(6);

		Tau_cmd_	= VectorXd::Zero(6);
		
		F_des_max_(0) = 10;
		
		robotState_ = State::NoMove;
		
		cycleTime_ = 0.002;
		cycleTime_Q_ = 0.001;
        
        RML_ = new ReflexxesAPI(3,cycleTime_);
		IP_ = new RMLPositionInputParameters(3);
		OP_ = new RMLPositionOutputParameters(3);
		
		RML_Q_ = new ReflexxesAPI(2,cycleTime_Q_);
		IP_Q_ = new RMLPositionInputParameters(2);
		OP_Q_ = new RMLPositionOutputParameters(2);
		
		setInitialPositions_ = true;
		count_pos_reached=0;
		// fill pid object for force control along x direction
		PID Force_PIDx_ = PID( (double)0.001/*dt*/, (double)100.0/*max*/, (double)-100.0/*min*/, (double)10.0/*Kp*/, (double)2.0/*Kd*/,  (double)0.0/*Ki*/);

		#if TRACE_Torque_Based_Position_ACTIVATED
			ROS_INFO("TorqueBasedPositionController: End of Start init of robot %s !",robot_namespace_.c_str());
		#endif

		return true;
	}
	
	void TorqueBasedPositionControllerGazebo::starting(const ros::Time& time)
    {
        
		#if TRACE_Torque_Based_Position_ACTIVATED
			ROS_INFO("TorqueBasedPositionController: Starting of robot %s !",robot_namespace_.c_str());
		#endif
		
		cmd_flag_ = 0;  // set this flag to 0 to not run the update method
		count = 0;
		setInitialPositions_ = true;
		robotState_ = State::NoMove;
		count_pos_reached=0;
    }
    
    void TorqueBasedPositionControllerGazebo::stopping(const ros::Time& time)
	{
		#if TRACE_Torque_Based_Position_ACTIVATED
			ROS_INFO("TorqueBasedPositionController: Stopping of robot %s !",robot_namespace_.c_str());
		#endif
		
		cmd_flag_ = 0;  // set this flag to 0 to not run the update method
		robotState_ = State::NoMove;
		count_pos_reached=0;
		
	}
	
	
	void TorqueBasedPositionControllerGazebo::checkTorqueMax_(std_msgs::Float64MultiArray& torqueArray)
	{
		
		double maxTorqueArray[] = { 176.0, 176.0, 100.0, 100.0, 100.0, 38.0, 38.0 };
		double minTorqueArray[] = { -176.0, -176.0, -100.0, -100.0, -100.0, -38.0, -38.0 };

		for (size_t i=0; i<torqueArray.data.size(); ++i)
		{
			if (torqueArray.data[i] > maxTorqueArray[i])
			{
				ROS_INFO("Current joint n°%i torque = %f -> Max allowed value = %f", (int)i, torqueArray.data[i], maxTorqueArray[i]);
				torqueArray.data[i] = maxTorqueArray[i] - 5.0;
				 
			}
			
			if (torqueArray.data[i] < minTorqueArray[i])
			{
				ROS_INFO("Current joint n°%i torque = %f -> Min allowed value = %f", (int)i, torqueArray.data[i], minTorqueArray[i]);
				torqueArray.data[i] = minTorqueArray[i] + 5.0;
				 
			}
		}
		
		
	}
	
	
    void TorqueBasedPositionControllerGazebo::update(const ros::Time& time, const ros::Duration& period)
    {	
		ros::Time begin = ros::Time::now();
		
		current_ = ros::Time::now();
		
		ros::Duration diff_time = current_ - previous_;
		
		/****** messages to be published for analyses *****/
		kuka_lwr_controllers::TrajPathPoint traj_resp_msg;
		std_msgs::Float64MultiArray tau_cmd_msg;	 tau_cmd_msg.data.resize(7);	
		std_msgs::Float64 Sv_msg, Sf_msg;
        geometry_msgs::WrenchStamped  F_des_msg;
        double torque;
        
        for(size_t i=0; i<joint_handles_.size(); i++) 
		{
			
			joint_msr_states_.q(i) = joint_handles_[i].getPosition();
			joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
			joint_msr_states_.qdotdot(i) = joint_handles_[i].getAcceleration();
					
		}
				
		//fcn->JntToCart(const JntArray& q_in, Frame& p_out, int seg_nr) 
		fk_pos_solver_->JntToCart(joint_msr_states_.q, P_current_); 					// computing forward kinematics
		
		Jnt_vel_.q=joint_msr_states_.q;													
		Jnt_vel_.qdot=joint_msr_states_.qdot;
		fk_vel_solver_->JntToCart(Jnt_vel_, V_current_);								// computing velocities
		
		count++;
		
		if(count%500==0)
		{
			ROS_INFO("Current position x=%lf y=%lf z=%lf ",P_current_.p.x(),P_current_.p.y(),P_current_.p.z());
			ROS_INFO("diff time = %f , period = %f", diff_time.toSec(), period.toSec());
		}
		
		/*IP_Q_->CurrentPositionVector->VecData[0] = (double)joint_handles_[5].getPosition(); 
		IP_Q_->CurrentPositionVector->VecData[1] = (double)joint_handles_[6].getPosition();
		
		
		IP_Q_->TargetPositionVector->VecData[0]	= (double)((-joint_handles_[1].getPosition())+joint_handles_[3].getPosition()); 
		IP_Q_->TargetPositionVector->VecData[1]	= (double)(-joint_handles_[0].getPosition());
		
		
		IP_Q_->MaxVelocityVector->VecData[0] = (double)3.0;
		IP_Q_->MaxVelocityVector->VecData[1] = (double)3.0;
	
		
		IP_Q_->MaxAccelerationVector->VecData[0] = (double)5.0;
		IP_Q_->MaxAccelerationVector->VecData[1] = (double)5.0;
		
		
		IP_Q_->CurrentVelocityVector->VecData[0] = joint_handles_[5].getVelocity();
		IP_Q_->CurrentVelocityVector->VecData[1] = joint_handles_[6].getVelocity();
	
		
		IP_Q_->SelectionVector->VecData[0] = true;
		IP_Q_->SelectionVector->VecData[1] = true;
		
		for (size_t i=0; i<10; i++)
		{
			resultValue_Q_ = RML_Q_->RMLPosition(*IP_Q_,OP_Q_, Flags_Q_);
			if ( resultValue_Q_ < 0 )
			{
				ROS_INFO("RML_Q_ -> ERROR during trajectory generation err n°%d",resultValue_Q_);
			}
			*IP_Q_->CurrentPositionVector      =   *OP_Q_->NewPositionVector      ;
			*IP_Q_->CurrentVelocityVector      =   *OP_Q_->NewVelocityVector      ;
			*IP_Q_->CurrentAccelerationVector  =   *OP_Q_->NewAccelerationVector  ;
		}*/
		
	/*	if(count%100==0)
		{
			//ROS_INFO("q4= %f, -q2+q'= %f ", joint_handles_[5].getPosition(), -joint_handles_[1].getPosition()+joint_handles_[3].getPosition());
			
			ROS_INFO("current q5 = %f, q6 = %f, irml q5 = %f, q6 = %f", joint_handles_[5].getPosition(), joint_handles_[6].getPosition(), OP_Q_->NewPositionVector->VecData[0], OP_Q_->NewPositionVector->VecData[1]);
		}*/
					
		switch (robotState_)
		{
			
			case State::NoMove :
			{
			}
			
			break;
			
			case State::MoveInitial :
			{
				//if (resultValue_ != ReflexxesAPI::RML_FINAL_STATE_REACHED)
				//{
				
					
					if (resultValue_ == ReflexxesAPI::RML_FINAL_STATE_REACHED)
					{
							count_pos_reached++;
							if (count_pos_reached == 1)
								ROS_INFO("Initial Position Reached !");
							
					}
					else
					{
						resultValue_ = RML_->RMLPosition(*IP_,OP_, Flags_);
						if ( resultValue_ < 0 )
						{
							ROS_INFO("GroupCommandControllerFRI::update : ERROR during trajectory generation err n°%d",resultValue_);
						}
					}
					
					
											
					
					traj_des_.q.data(0) = OP_->NewPositionVector->VecData[0];
					traj_des_.q.data(1) = OP_->NewPositionVector->VecData[1];
					traj_des_.q.data(2) = OP_->NewPositionVector->VecData[2];
							
					traj_des_.qdot.data(0) = OP_->NewVelocityVector->VecData[0];
					traj_des_.qdot.data(1) = OP_->NewVelocityVector->VecData[1];
					traj_des_.qdot.data(2) = OP_->NewVelocityVector->VecData[2];
					
					traj_des_.qdotdot.data(0) = OP_->NewAccelerationVector->VecData[0];
					traj_des_.qdotdot.data(1) = OP_->NewAccelerationVector->VecData[1];
					traj_des_.qdotdot.data(2) = OP_->NewAccelerationVector->VecData[2];
						
					jnt_to_jac_solver_->JntToJac(joint_msr_states_.q, Jkdl_);						// computing Jacobian with KDL
					id_solver_->JntToMass(joint_msr_states_.q, M_);									// computing Inertia matrix		
					id_solver_->JntToCoriolis(joint_msr_states_.q, joint_msr_states_.qdot, C_);		// computing Coriolis torques	
					id_solver_->JntToGravity(joint_msr_states_.q, G_);								// computing Gravity torques
					
					//ROS_INFO("1");
					J6x6_ =   Jkdl_.data;        // save the KDL jacoian to the 6x6 one to modify it
					removeColumn(J6x6_, 2);      // removing the column corresponding to the redundant joint and resising the matrix
					J_trans_= J6x6_.transpose();
					//ROS_INFO("2");
				   	I6x6_= M_.data;             // save the KDL Inertia matrix to the 6x6 one to modify it
					removeColumn(I6x6_, 2);     // removing the column corresponding to the redundant joint and resising the matrix
					removeRow(I6x6_, 2);        // removing the row corresponding to the redundant joint and resising the matrix
					//ROS_INFO("3");
					calculatePsuedoInertia(Jkdl_.data, M_.data, LAMDA_ );	
					//ROS_INFO("4");
					calculateAccelerationCommand(P_current_, V_current_,traj_des_,Alpha_v_ );		
					//ROS_INFO("5");	
					calculateJointTorques(J6x6_,Alpha_v_, Tau_cmd_);	
					//ROS_INFO("6");
					
					tau_cmd_msg.data[0] = Tau_cmd_(0)+C_(0)+G_(0);
					tau_cmd_msg.data[1] = Tau_cmd_(1)+C_(1)+G_(1);
					tau_cmd_msg.data[2] = (Kp_joints_(2)*(0-joint_handles_[2].getPosition()))+(Kd_joints_(2)*(-joint_handles_[2].getVelocity()))+C_(2)+G_(2);
					tau_cmd_msg.data[3] = Tau_cmd_(2)+C_(3)+G_(3);
					
					tau_cmd_msg.data[4] = 0.0;
					tau_cmd_msg.data[5] = ((Kp_joints_(5)*((-joint_handles_[1].getPosition()+joint_handles_[3].getPosition())- joint_handles_[5].getPosition()
								))+(Kd_joints_(5)*((-joint_handles_[1].getVelocity()+joint_handles_[3].getVelocity())-joint_handles_[5].getVelocity())))+C_(5)+G_(4);
					tau_cmd_msg.data[6] = (Kp_joints_(6)*(0-joint_handles_[6].getPosition()))+(Kd_joints_(6)*(-joint_handles_[6].getVelocity()))+C_(6)+G_(6);
					
					
					checkTorqueMax_(tau_cmd_msg);
					
					joint_handles_[0].setCommandPosition(joint_handles_[0].getPosition());
					joint_handles_[0].setCommandStiffness(stiff_(0)*0);
					joint_handles_[0].setCommandDamping(damp_(0)*0);
					
					joint_handles_[1].setCommandPosition(joint_handles_[1].getPosition());
					joint_handles_[1].setCommandStiffness(stiff_(1)*0);
					joint_handles_[1].setCommandDamping(damp_(1)*0);
					
					joint_handles_[2].setCommandPosition(initialPositions_(2));
					joint_handles_[2].setCommandStiffness(stiff_(2));
					joint_handles_[2].setCommandDamping(damp_(2));
					
					joint_handles_[3].setCommandPosition(joint_handles_[3].getPosition());
					joint_handles_[3].setCommandStiffness(stiff_(3)*0);
					joint_handles_[3].setCommandDamping(damp_(3)*0);
					
					joint_handles_[4].setCommandPosition(initialPositions_(4));
					joint_handles_[4].setCommandStiffness(stiff_(4));
					joint_handles_[4].setCommandDamping(damp_(4));
					
					
					joint_handles_[5].setCommandPosition(joint_handles_[5].getPosition());
					//joint_handles_[5].setCommandPosition(-initialPositions_(1)+initialPositions_(3));
					//joint_handles_[5].setCommandPosition(OP_Q_->NewPositionVector->VecData[0]);
					joint_handles_[5].setCommandStiffness(stiff_(5));
					joint_handles_[5].setCommandDamping(damp_(5));
					
					//joint_handles_[6].setCommandPosition(OP_Q_->NewPositionVector->VecData[1]);
					joint_handles_[6].setCommandPosition(joint_handles_[6].getPosition());
					//joint_handles_[6].setCommandPosition(-joint_handles_[0].getPosition());
					joint_handles_[6].setCommandStiffness(stiff_(6));
					joint_handles_[6].setCommandDamping(damp_(6));
					
					joint_handles_[0].setCommandTorque(tau_cmd_msg.data[0]) ;
					joint_handles_[1].setCommandTorque(tau_cmd_msg.data[1]);
					joint_handles_[2].setCommandTorque(0.0); // Set a value of redundant joint.
					joint_handles_[3].setCommandTorque(tau_cmd_msg.data[3]);
					joint_handles_[4].setCommandTorque(0.0);
					joint_handles_[5].setCommandTorque(tau_cmd_msg.data[5]*0.0);
					joint_handles_[6].setCommandTorque(tau_cmd_msg.data[6]*0.0);
					
					if (resultValue_ != ReflexxesAPI::RML_FINAL_STATE_REACHED)
					{
						*IP_->CurrentPositionVector      =   *OP_->NewPositionVector      ;
						*IP_->CurrentVelocityVector      =   *OP_->NewVelocityVector      ;
						*IP_->CurrentAccelerationVector  =   *OP_->NewAccelerationVector  ;
					}
					
					#if TRACE_GroupCommandController_ACTIVATED
						ROS_INFO("GroupCommandControllerFRI: resultValue_ = %d", resultValue_);
						ROS_INFO("GroupCommandControllerFRI: of robot %s -> j0=%f, j1=%f, j2=%f, j3=%f, j4=%f, j5=%f, j6=%f",robot_namespace_.c_str(),joint_des_states_.q(0), joint_des_states_.q(1), joint_des_states_.q(2), joint_des_states_.q(3), joint_des_states_.q(4), joint_des_states_.q(5), joint_des_states_.q(6));
					#endif
				/*}
				else
				{
					
					if(count%500==0)
					{
						ROS_INFO("Initial Position Reached !");
					}
					// robotState_= State::NoMove;
				}*/
				
				
			}
			break;
			
			
			case State::MoveTrajectory :
			{
					
					jnt_to_jac_solver_->JntToJac(joint_msr_states_.q, Jkdl_);						// computing Jacobian with KDL
					id_solver_->JntToMass(joint_msr_states_.q, M_);									// computing Inertia matrix		
					id_solver_->JntToCoriolis(joint_msr_states_.q, joint_msr_states_.qdot, C_);		// computing Coriolis torques	
					id_solver_->JntToGravity(joint_msr_states_.q, G_);								// computing Gravity torques
					
					//ROS_INFO("1");
					J6x6_ =   Jkdl_.data;        // save the KDL jacoian to the 6x6 one to modify it
					removeColumn(J6x6_, 2);      // removing the column corresponding to the redundant joint and resising the matrix
					J_trans_= J6x6_.transpose();
					//ROS_INFO("2");
				   	I6x6_= M_.data;             // save the KDL Inertia matrix to the 6x6 one to modify it
					removeColumn(I6x6_, 2);     // removing the column corresponding to the redundant joint and resising the matrix
					removeRow(I6x6_, 2);        // removing the row corresponding to the redundant joint and resising the matrix
					//ROS_INFO("3");
					calculatePsuedoInertia(Jkdl_.data, M_.data, LAMDA_ );	
					//ROS_INFO("4");
					calculateAccelerationCommand(P_current_, V_current_,traj_des_,Alpha_v_ );		
					//ROS_INFO("5");
					//5 SelectionMatricesTransitionControl(); 	
					
					/*3 // X direction
					 if (S_f_(0,0) ==1 && F_des_(0) < F_des_max_(0))
					{
						F_des_(0) = F_des_(0)+0.001;
					} 
					else
					{
						F_des_(0) = 0.0;
					}
					* */
					//ROS_INFO("6");
					//4 calculateForceCommand(FT_sensor_, F_des_, F_cmd_);
					//ROS_INFO("7");
					calculateJointTorques(J6x6_,Alpha_v_, Tau_cmd_);	
					//
					
					tau_cmd_msg.data[0] = Tau_cmd_(0)+C_(0)+G_(0);
					tau_cmd_msg.data[1] = Tau_cmd_(1)+C_(1)+G_(1);
					tau_cmd_msg.data[2] = (Kp_joints_(2)*(0-joint_handles_[2].getPosition()))+(Kd_joints_(2)*(-joint_handles_[2].getVelocity()))+C_(2)+G_(2);
					tau_cmd_msg.data[3] = Tau_cmd_(2)+C_(3)+G_(3);
					
					tau_cmd_msg.data[4] = 0.0;
					tau_cmd_msg.data[5] = ((Kp_joints_(5)*((-joint_handles_[1].getPosition()+joint_handles_[3].getPosition())- joint_handles_[5].getPosition()
								))+(Kd_joints_(5)*((-joint_handles_[1].getVelocity()+joint_handles_[3].getVelocity())-joint_handles_[5].getVelocity())))+C_(5)+G_(4);
					tau_cmd_msg.data[6] = (Kp_joints_(6)*(0-joint_handles_[6].getPosition()))+(Kd_joints_(6)*(-joint_handles_[6].getVelocity()))+C_(6)+G_(6);
					
					
					checkTorqueMax_(tau_cmd_msg);
						
					
					joint_handles_[0].setCommandPosition(joint_handles_[0].getPosition());
					joint_handles_[0].setCommandStiffness(stiff_(0)*0);
					joint_handles_[0].setCommandDamping(damp_(0)*0);
					
					joint_handles_[1].setCommandPosition(joint_handles_[1].getPosition());
					joint_handles_[1].setCommandStiffness(stiff_(1)*0);
					joint_handles_[1].setCommandDamping(damp_(1)*0);

					joint_handles_[2].setCommandPosition(initialPositions_(2));			
					//joint_handles_[2].setCommandPosition(joint_handles_[2].getPosition());
					joint_handles_[2].setCommandStiffness(stiff_(2));
					joint_handles_[2].setCommandDamping(damp_(2));
					
					joint_handles_[3].setCommandPosition(joint_handles_[3].getPosition());
					joint_handles_[3].setCommandStiffness(stiff_(3)*0);
					joint_handles_[3].setCommandDamping(damp_(3)*0);
					
					joint_handles_[4].setCommandPosition(initialPositions_(4));			
					//joint_handles_[4].setCommandPosition(joint_handles_[4].getPosition());
					joint_handles_[4].setCommandStiffness(stiff_(4));
					joint_handles_[4].setCommandDamping(damp_(4));
					
					
					joint_handles_[5].setCommandPosition(joint_handles_[3].getPosition());
					//joint_handles_[5].setCommandPosition(-joint_handles_[1].getPosition()+joint_handles_[3].getPosition()+initialPositions_(5));
					//joint_handles_[5].setCommandPosition(initialPositions_(5));
					//joint_handles_[5].setCommandPosition(-joint_handles_[1].getPosition()+joint_handles_[3].getPosition());
					//joint_handles_[5].setCommandPosition(OP_Q_->NewPositionVector->VecData[0]);
					joint_handles_[5].setCommandStiffness(stiff_(5));
					joint_handles_[5].setCommandDamping(damp_(5));
					
					
					//joint_handles_[6].setCommandPosition(initialPositions_(6));
					//joint_handles_[6].setCommandPosition(-joint_handles_[0].getPosition()+initialPositions_(6));
					joint_handles_[6].setCommandPosition(-joint_handles_[0].getPosition());
					//joint_handles_[6].setCommandPosition(OP_Q_->NewPositionVector->VecData[1]);
					joint_handles_[6].setCommandStiffness(stiff_(6));
					joint_handles_[6].setCommandDamping(damp_(6));
					
					
					
					joint_handles_[0].setCommandTorque(tau_cmd_msg.data[0]) ;
					joint_handles_[1].setCommandTorque(tau_cmd_msg.data[1]);
					joint_handles_[2].setCommandTorque(0.0); // Set a value of redundant joint.
					joint_handles_[3].setCommandTorque(tau_cmd_msg.data[3]);
					joint_handles_[4].setCommandTorque(0.0);
					joint_handles_[5].setCommandTorque(tau_cmd_msg.data[5]*0.0);
					joint_handles_[6].setCommandTorque(tau_cmd_msg.data[6]*0.0);
					
					stiff_(5) = 10.0;
					damp_(5) = 1.0;
			}
			break;		
			
		}
		
		
		previous_ = current_;
 
        
	}
	
	
	void TorqueBasedPositionControllerGazebo:: calculatePsuedoInertia(const Eigen::MatrixXd  & j, const Eigen::MatrixXd & i, Eigen::MatrixXd & lamda ){
		
		lamda = ((j)*(i.inverse())*(j.transpose())).inverse();
	}
	
	void TorqueBasedPositionControllerGazebo::transformKDLToEigen_(const KDL::Frame  & frame, Eigen::MatrixXd & matrix) const
	{
	
		for (size_t i=0; i<3; ++i)
			matrix(i,0) = frame.p(i);
		
		for (size_t i=3; i<6; ++i)
			matrix(i,0) = 0.0;
		
	}
	void TorqueBasedPositionControllerGazebo:: calculateAccelerationCommand(const KDL::Frame  & p_current, const KDL::FrameVel & v_current, const KDL::JntArrayAcc & traj_des, Eigen::MatrixXd & alpha_v )
	{
		
		MatrixXd p_resp(6,1);
		MatrixXd v_resp(6,1);
		p_resp = MatrixXd::Zero(6, 1);
		v_resp = MatrixXd::Zero(6, 1);
		
		for (size_t i=0; i<3; ++i)
		{
			p_resp(i,0) = p_current.p(i);
			v_resp(i,0) = v_current.p.v(i);
		}
	
		alpha_v = (traj_des_.qdotdot.data + KDv_*(traj_des.qdot.data - v_resp)  + KPv_*(traj_des.q.data - p_resp));
	}
	void TorqueBasedPositionControllerGazebo :: SelectionMatricesTransitionControl()
	{
		double offset    = 0.290-0.2211;              // offset between radars and wall at distance = 0
		double Dist      = d_wall_ - offset;	
		
		if (Dist < 0) 
		{
			Dist = 0;
		}
		
		double S_f_0     = 1;
		double x_initial = 0;
		double x_final   = 2.5;   // domain of work
		double S_f_final = 0.0001;
		double gain      = 10000;
		double eps       = (x_final - x_initial)*gain; // value to adjust the shape of the transition curve
		double ka        = (-log(S_f_final/S_f_0)/x_final )+eps;
		
		S_f_(0,0) = S_f_0*exp(-ka*Dist); // the term corresponding to the direction controlled by position then force
		S_v_(0,0) = 1-S_f_(0,0);
	}
	
	void TorqueBasedPositionControllerGazebo ::calculateForceCommand(const Eigen::VectorXd & FT_sensor, const Eigen::VectorXd & F_des, Eigen::MatrixXd & F_cmd )
	{
		Eigen::VectorXd FT_sensor_des;
		
		FT_sensor_des = FT_sensor_;
		
		// X direction
		if(FT_sensor(0)>(F_des_max_(0)+3))
		FT_sensor_des(0) = F_des_max_(0)+3;
		
		if(FT_sensor(0)<0)
		FT_sensor_des(0) = 0;
		
		// Returns the manipulated variable given a setpoint and current process value
        //2 double fx_cmd =  Force_PIDx_.calculate( F_des(0),FT_sensor_des(0) );
		
		F_cmd_ = F_des + KPf_*(F_des - FT_sensor_des);  // pay attention to the sign of the force sensor
		//1 F_cmd_(0) = fx_cmd;
	}
	
	void TorqueBasedPositionControllerGazebo :: calculateJointTorques(const Eigen::MatrixXd & j6x6,const Eigen::MatrixXd & alpha_v, Eigen::VectorXd & Tau_cmd_  )
	{
		//Tau_cmd_ = Jkdl_.data.transpose()*(LAMDA_*S_v_*alpha_v);
		//Tau_cmd_ = J6x6_.transpose()*((LAMDA_*S_v_*alpha_v)+(S_f_*F_cmd_));
		Tau_cmd_ = J6x6_.transpose()*(LAMDA_*S_v_*alpha_v);	
	}
	void TorqueBasedPositionControllerGazebo :: removeColumn(Eigen::MatrixXd& matrix_in, unsigned int colToRemove)
	{
		unsigned int numRows = matrix_in.rows();
		unsigned int numCols = matrix_in.cols()-1;

		if( colToRemove < numCols )
			matrix_in.block(0,colToRemove,numRows,numCols-colToRemove) = matrix_in.block(0,colToRemove+1,numRows,numCols-colToRemove);
			
		matrix_in.conservativeResize(numRows,numCols);
	}
	
	void TorqueBasedPositionControllerGazebo::removeRow(Eigen::MatrixXd& matrix, unsigned int rowToRemove)
	{
		unsigned int numRows = matrix.rows()-1;
		unsigned int numCols = matrix.cols();

		if( rowToRemove < numRows )
			matrix.block(rowToRemove,0,numRows-rowToRemove,numCols) = matrix.block(rowToRemove+1,0,numRows-rowToRemove,numCols);

		matrix.conservativeResize(numRows,numCols);
	}
	
	void TorqueBasedPositionControllerGazebo::commandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
    {
		/*#if TRACE_Torque_Based_Position_ACTIVATED
			ROS_INFO("TorqueBasedPositionController: Start commandCB of robot %s!",robot_namespace_.c_str());
		#endif

		if(msg->data.size()!=joint_handles_.size())
		{ 
			ROS_ERROR_STREAM("TorqueBasedPositionController: Dimension (of robot " << robot_namespace_.c_str() << ") of command (" << msg->data.size() << ") does not match number of joints (" << joint_handles_.size() << ")! Not executing!");
			cmd_flag_ = 0;
			return; 
		}
		
		q_des_.resize(msg->data.size());
		for (std::size_t i=0; i<msg->data.size(); i++)
		{
			q_des_(i) = (double)msg->data[i]*0;
		}
		
		cmd_flag_ = 1;*/
		
	}
	
	
	void TorqueBasedPositionControllerGazebo::TrajPathPointCBInitial(const kuka_lwr_controllers::TrajPathPoint::ConstPtr & msg)
    {
			if (setInitialPositions_ == true)
			{
				for(size_t i=0; i<joint_handles_.size(); i++) 
				{
					initialPositions_(i) = joint_handles_[i].getPosition();
				}
				
				setInitialPositions_ = false;
			}
				
				
		
			IP_->CurrentPositionVector->VecData[0] = (double)P_current_.p.x(); 
			IP_->CurrentPositionVector->VecData[1] = (double)P_current_.p.y();
			IP_->CurrentPositionVector->VecData[2] = (double)P_current_.p.z();
			
			
			IP_->TargetPositionVector->VecData[0]	= (double)msg->pos.x; 
			IP_->TargetPositionVector->VecData[1]	= (double)msg->pos.y;
			IP_->TargetPositionVector->VecData[2]	= (double)msg->pos.z;
			
			
			IP_->MaxVelocityVector->VecData[0] = (double)msg->vel.x;
			IP_->MaxVelocityVector->VecData[1] = (double)msg->vel.y;
			IP_->MaxVelocityVector->VecData[2] = (double)msg->vel.z;
			
			
			IP_->MaxAccelerationVector->VecData[0] = (double)msg->acc.x;
			IP_->MaxAccelerationVector->VecData[1] = (double)msg->acc.y;
			IP_->MaxAccelerationVector->VecData[2] = (double)msg->acc.z;
			
			
			
			IP_->CurrentVelocityVector->VecData      [0] =   V_current_.p.v.x();
			IP_->CurrentVelocityVector->VecData      [1] =   V_current_.p.v.y();
			IP_->CurrentVelocityVector->VecData      [2] =   V_current_.p.v.z();

			/*IP_->CurrentAccelerationVector->VecData  [0] =   Acc_current_.p.dv.x();
			IP_->CurrentAccelerationVector->VecData  [1] =   Acc_current_.p.dv.y();
			IP_->CurrentAccelerationVector->VecData  [2] =   Acc_current_.p.dv.z();*/
			
			
			IP_->SelectionVector->VecData[0] = true;
			IP_->SelectionVector->VecData[1] = true;
			IP_->SelectionVector->VecData[2] = true;
		 

			resultValue_ = ReflexxesAPI::RML_WORKING;
		
			robotState_ = State::MoveInitial;
			
			count_pos_reached = 0;
			
			ROS_INFO("TorqueBasedPositionController: TrajPathPointCBInitial received  x = %f, y=%f, z=%f !",(double)msg->pos.x,(double)msg->pos.y,(double)msg->pos.z);
		
	}
	
	void TorqueBasedPositionControllerGazebo::TrajPathPointCB(const kuka_lwr_controllers::TrajPathPoint::ConstPtr & msg)
    {

		traj_des_.resize(6);

			traj_des_.q.data(0) = (double)msg->pos.x;
			traj_des_.q.data(1) = (double)msg->pos.y;
			traj_des_.q.data(2) = (double)msg->pos.z;
					
			traj_des_.qdot.data(0) = (double)msg->vel.x;
			traj_des_.qdot.data(1) = (double)msg->vel.y;
			traj_des_.qdot.data(2) = (double)msg->vel.z;
			
			traj_des_.qdotdot.data(0) = (double)msg->acc.x;
			traj_des_.qdotdot.data(1) = (double)msg->acc.y;
			traj_des_.qdotdot.data(2) = (double)msg->acc.z;
			
		//	ROS_INFO("TorqueBasedPositionController: TrajPathPointCB  msg->pos.x = %f !",(double)msg->pos.x);

		for (std::size_t i=3; i<6; i++)
		{
			traj_des_.q(i) = 0;
			traj_des_.qdot(i) = 0;
			traj_des_.qdotdot(i) = 0;
		}
		
		cmd_flag_ = 1;
		
		robotState_ = State::MoveTrajectory;
		
	}
	void TorqueBasedPositionControllerGazebo::ft_readingsCB(const geometry_msgs::WrenchStamped& msg)
    {		
		FT_sensor_(0) = -msg.wrench.force.x ;
		FT_sensor_(1) = msg.wrench.force.y ;
		FT_sensor_(2) = msg.wrench.force.z ;
		FT_sensor_(3) = msg.wrench.torque.x ;
		FT_sensor_(4) = msg.wrench.torque.y ;
		FT_sensor_(5) = msg.wrench.torque.z ;
		
		//cmd_flag_ = 1;

	}
	
	void TorqueBasedPositionControllerGazebo:: setjointsKp(const std_msgs::Float64MultiArrayConstPtr& msg)
	{
		#if TRACE_Torque_Based_Position_ACTIVATED
			ROS_INFO("TorqueBasedPositionController: Start setjointsKp of robot %s!",robot_namespace_.c_str());
		#endif

		if(msg->data.size()!=joint_handles_.size() )
		{ 
			ROS_ERROR_STREAM("TorqueBasedPositionController: setjointsKp Dimension (of robot " << robot_namespace_.c_str() << ") of command (" << msg->data.size() << ") does not match number of joints (" << joint_handles_.size() << ")! Not executing!");
			return; 
		}
		
		Kp_joints_.resize(joint_handles_.size());
		
		for (std::size_t i=0; i<msg->data.size(); i++)
		{
			Kp_joints_(i) = (double)msg->data[i];
		}
	}
	void TorqueBasedPositionControllerGazebo:: setjointsKd(const std_msgs::Float64MultiArrayConstPtr& msg)
	{
		#if TRACE_Torque_Based_Position_ACTIVATED
			ROS_INFO("TorqueBasedPositionController: setjointsKd setjointsKp of robot %s!",robot_namespace_.c_str());
		#endif

		if(msg->data.size()!=joint_handles_.size() )
		{ 
			ROS_ERROR_STREAM("TorqueBasedPositionController: setjointsKd Dimension (of robot " << robot_namespace_.c_str() << ") of command (" << msg->data.size() << ") does not match number of joints (" << joint_handles_.size() << ")! Not executing!");
			return; 
		}
		
		Kd_joints_.resize(joint_handles_.size());
		
		for (std::size_t i=0; i<msg->data.size(); i++)
		{
			Kd_joints_(i) = (double)msg->data[i];
		}
	}
	void TorqueBasedPositionControllerGazebo::setcartesianKp(const std_msgs::Float64MultiArrayConstPtr& msg)
    {
		#if TRACE_Torque_Based_Position_ACTIVATED
			ROS_INFO("TorqueBasedPositionController: Start setcartesianKp of robot %s!",robot_namespace_.c_str());
		#endif

		if(msg->data.size()!=6)
		{ 
			ROS_ERROR_STREAM("TorqueBasedPositionController: setcartesianKp Dimension (of robot " << robot_namespace_.c_str() << ") of command (" << msg->data.size() << ") does not match number of joints (" << joint_handles_.size() << ")! Not executing!");
			return; 
		}
		
		Kp_cartesian_.resize(6);
		
		for (std::size_t i=0; i<msg->data.size(); i++)
		{
			Kp_cartesian_(i) = (double)msg->data[i];
			KPv_(i,i) = (double)msg->data[i];
		}
		
	}
	
	void TorqueBasedPositionControllerGazebo::setcartesianKd(const std_msgs::Float64MultiArrayConstPtr& msg)
    {
		#if TRACE_Torque_Based_Position_ACTIVATED
			ROS_INFO("TorqueBasedPositionController: Start setcartesianKd of robot %s!",robot_namespace_.c_str());
		#endif

		if(msg->data.size()!=6)
		{ 
			ROS_ERROR_STREAM("TorqueBasedPositionController: setcartesianKd Dimension (of robot " << robot_namespace_.c_str() << ") of command (" << msg->data.size() << ") does not match number of joints (" << joint_handles_.size() << ")! Not executing!");
			return; 
		}
		
		Kd_cartesian_.resize(6);
		for (std::size_t i=0; i<msg->data.size(); i++)
		{
			Kd_cartesian_(i) = (double)msg->data[i];
			KDv_(i,i) = (double)msg->data[i];
		}
	}
	
	void TorqueBasedPositionControllerGazebo::setforceKp(const std_msgs::Float64MultiArrayConstPtr& msg)
    {
		#if TRACE_Torque_Based_Position_ACTIVATED
			ROS_INFO("TorqueBasedPositionController: Start setforceKp of robot %s!",robot_namespace_.c_str());
		#endif

		if(msg->data.size()!=6)
		{ 
			ROS_ERROR_STREAM("TorqueBasedPositionController: setforceKp Dimension (of robot " << robot_namespace_.c_str() << ") of command (" << msg->data.size() << ") does not match number of joints (" << joint_handles_.size() << ")! Not executing!");
			return; 
		}

		for (std::size_t i=0; i<msg->data.size(); i++)
		{
			KPf_(i,i) = (double)msg->data[i];
		}
	}	
	void TorqueBasedPositionControllerGazebo::setRollPitchYaw(const  geometry_msgs::PoseStamped& msg)
	{
		d_wall_ = msg.pose.position.z;
		//ROS_INFO("In setRollPitchYaw");
		double sinr = +2.0 * (msg.pose.orientation.w * msg.pose.orientation.x + msg.pose.orientation.y * msg.pose.orientation.z);
		double cosr = +1.0 - 2.0 * (msg.pose.orientation.x * msg.pose.orientation.x + msg.pose.orientation.y * msg.pose.orientation.y);
		roll_ = atan2(sinr, cosr);

		// pitch (y-axis rotation)
		double sinp = +2.0 * (msg.pose.orientation.w * msg.pose.orientation.y - msg.pose.orientation.z * msg.pose.orientation.x);
		if (fabs(sinp) >= 1)
			pitch_ = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
		else
		pitch_ = asin(sinp);

		// yaw (z-axis rotation)
		double siny = +2.0 * (msg.pose.orientation.w * msg.pose.orientation.z + msg.pose.orientation.x * msg.pose.orientation.y);
		double cosy = +1.0 - 2.0 * (msg.pose.orientation.y * msg.pose.orientation.y + msg.pose.orientation.z * msg.pose.orientation.z);  
		yaw_ = atan2(siny, cosy);
	}

void TorqueBasedPositionControllerGazebo::setStiffnessDamping(const kuka_lwr_controllers::StiffnessDamping::ConstPtr & msg){
		
		#if TRACE_Torque_Based_Position_ACTIVATED
			ROS_INFO("TorqueBasedPositionController: Start setStiffnessDamping of robot %s!",robot_namespace_.c_str());
		#endif
		
		
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
		cmd_flag_ = 1;
	}
}

PLUGINLIB_EXPORT_CLASS(kuka_lwr_controllers::TorqueBasedPositionControllerGazebo, controller_interface::ControllerBase)

