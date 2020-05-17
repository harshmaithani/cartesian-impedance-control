/*
 *  Kamal mohy el dine 
 *  Institut Pascal UMR6602
 *  kamal.mohy.el.dine@gmail.com
 * 
*/

#ifndef LWR_CONTROLLERS_TORQUE_BASED_POSITION_GAZEBO_H
#define LWR_CONTROLLERS_TORQUE_BASED_POSITION_GAZEBO_H

// Controller base
#include "kinematic_chain_controller_base.h"

// hardware_interface 
#include <hardware_interface/joint_command_interface.h> // contains definition of PositionJointInterface

// msgs 
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <kuka_lwr_controllers/TrajPathPoint.h>
#include <kuka_lwr_controllers/StiffnessDamping.h>

// KDL JntArray
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>

// STL
#include <string>

// KDL Solver
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

// Boost
#include <boost/scoped_ptr.hpp>

// hardware_interface 
#include <kuka_lwr_hw/lwr_kuka_interface.h> // contains definition of KUKAJointInterface

// FRI Type IRML II
#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>

// PID
#include "pid/pid.h"

#define TRACE_Torque_Based_Position_ACTIVATED 1

namespace kuka_lwr_controllers
{
	
	enum State { NoMove, MoveInitial, MoveTrajectory };
	
	class TorqueBasedPositionControllerGazebo: 
	public controller_interface::KinematicChainControllerBase<hardware_interface::KUKAJointInterface>
	//public controller_interface::KinematicChainControllerBase<hardware_interface::EffortJointInterface>
	{
		public:
			TorqueBasedPositionControllerGazebo();
			~TorqueBasedPositionControllerGazebo();

			bool init(hardware_interface::KUKAJointInterface *robot, ros::NodeHandle &n); // Init the controller
			void starting(const ros::Time& time);  // Start the controller
			void stopping(const ros::Time& time);  // Stop the controller
			void update(const ros::Time& time, const ros::Duration& period);  // Update the controller
			//void calculate3x3jacobian(KDL::JntArray & joint_resp, KDL::Jacobian & jacobian);
			void SelectionMatricesTransitionControl();
		private:
			void commandCB(const std_msgs::Float64MultiArrayConstPtr& msg);		 			// function associate to a subscribe joint command topic
			void setcartesianKp(const std_msgs::Float64MultiArrayConstPtr& msg); 			// function associate to a subscribe setcartesianKp topic
			void setcartesianKd(const std_msgs::Float64MultiArrayConstPtr& msg); 			// function associate to a subscribe setcartesianKd topic
			void setjointsKp(const std_msgs::Float64MultiArrayConstPtr& msg); 				// function associate to a subscribe setjointsKp topic
			void setjointsKd(const std_msgs::Float64MultiArrayConstPtr& msg); 				// function associate to a subscribe setjointsKd topic
			void setforceKp(const std_msgs::Float64MultiArrayConstPtr& msg); 				// function associate to a subscribe setforceKp topic
			void TrajPathPointCB(const kuka_lwr_controllers::TrajPathPoint::ConstPtr & msg);// function associate to a subscribe traj_cmd topic
			void TrajPathPointCBInitial(const kuka_lwr_controllers::TrajPathPoint::ConstPtr & msg);// function associate to a subscribe traj_cmd_initial topic
			void ft_readingsCB(const geometry_msgs::WrenchStamped& msg);
			void setRollPitchYaw(const  geometry_msgs::PoseStamped& msg);
			void setStiffnessDamping(const kuka_lwr_controllers::StiffnessDamping::ConstPtr & msg);
			
			ros::Subscriber sub_command_  , sub_traj_      , sub_ft_, sub_traj_initial_;
			ros::Subscriber sub_kp_joints_, sub_kd_joints_ , sub_kp_cartesian_ , sub_kd_cartesian_, sub_kp_force_,sub_ee_pos_, sub_stiffness_damping_; // subscribers of gains
			ros::Publisher pub_traj_resp_,pub_tau_cmd_, pub_F_des_, pub_Sv_, pub_Sf_;              
			int cmd_flag_; // flag set only to 1 when the controller receive a message to the command topic	
			
			std::string robot_namespace_;
			
			KDL::JntArray 				Kp_joints_, Kd_joints_, Kp_cartesian_ , Kd_cartesian_;	// gain arrays
			KDL::JntArray				initialPositions_;
			KDL::JntArray 				q_des_ , stiff_, damp_;			// desired joint value
			KDL::Jacobian 				Jkdl_;
			KDL::JntSpaceInertiaMatrix 	M_; 				//Inertia matrix
			KDL::JntArray 				C_, G_;        		//Coriolis and Gravitational matrices
			KDL::JntArrayAcc 			traj_des_;			// desired trajecory
			
			KDL::Frame    				P_current_;			// current end-effector pose
			KDL::Frame    				P_initial_;			// current end-effector pose
			
			KDL::FrameAcc    			Acc_current_;			// current end-effector acc
			
			KDL::FrameVel 				V_current_; 		// current end-effector velocity
			KDL::JntArrayVel 			Jnt_vel_;
			
			Eigen::MatrixXd 			J6x6_, J_inv_, J_trans_, J_inv_trans_, I6x6_, LAMDA_, S_v_,S_f_, Alpha_v_, KDv_, KPv_,KPf_, F_cmd_;
			Eigen::VectorXd				FT_sensor_, F_des_,F_des_max_,Tau_cmd_;

			int count =0;
			int count_pos_reached = 0;
			double roll_, pitch_, yaw_, d_wall_;
			bool setInitialPositions_,switchControl_;
			
			State robotState_;
			
			boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;	
			boost::scoped_ptr<KDL::ChainDynParam> id_solver_;		
			boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
			boost::scoped_ptr<KDL::ChainFkSolverVel_recursive> fk_vel_solver_;
			
			
			void removeColumn(Eigen::MatrixXd& matrix_in, unsigned int colToRemove);
			void removeRow(Eigen::MatrixXd& matrix, unsigned int rowToRemove);
			void transformKDLToEigen_(const KDL::Frame  & frame, Eigen::MatrixXd & matrix) const;
			void calculatePsuedoInertia(const Eigen::MatrixXd  & j6x6, const Eigen::MatrixXd & i6x6, Eigen::MatrixXd & lamda );
			void calculateAccelerationCommand(const KDL::Frame  & p_current, const KDL::FrameVel & v_current, const KDL::JntArrayAcc & traj_des, Eigen::MatrixXd & alpha_v );
			void calculateForceCommand(const Eigen::VectorXd & FT_sensor, const Eigen::VectorXd & f_des, Eigen::MatrixXd & F_cmd );
			void calculateJointTorques(const Eigen::MatrixXd & j6x6, const Eigen::MatrixXd & alpha_v,Eigen::VectorXd & Tau_cmd_ );
			void checkTorqueMax_(std_msgs::Float64MultiArray& torqueArray);
			
			
			
			ReflexxesAPI  *RML_, *RML_Q_;
	
			RMLPositionInputParameters *IP_, *IP_Q_;
			RMLPositionOutputParameters *OP_, *OP_Q_;
			RMLPositionFlags            Flags_, Flags_Q_;
			
			double cycleTime_, cycleTime_Q_;
			int resultValue_, resultValue_Q_;
			PID pid_fx_ = PID(0.002/*dt*/,100/*max*/,-100/*min*/,0.0/*Kp*/,0.05/*Kd*/, 0.0/*Ki*/);
			//PID *pid_fx_;
			
			ros::Time previous_, current_;
			kuka_lwr_controllers::TrajPathPoint traj_resp_msg_;  // traj response msg
			std_msgs::Float64MultiArray tau_cmd_msg_; // tau command msg
			geometry_msgs::WrenchStamped  F_des_msg_; // F desired msg
			std_msgs::Float64 Sv_msg_, Sf_msg_; // selections matrix data msg
			

	};
}


#endif

