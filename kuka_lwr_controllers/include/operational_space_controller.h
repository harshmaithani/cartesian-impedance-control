/*
 *  Kamal mohy el dine 
 *  Institut Pascal UMR6602
 *  kamal.mohy.el.dine@gmail.com
 * 
*/

#ifndef LWR_CONTROLLERS_OPERATIONAL_SPACE_H
#define LWR_CONTROLLERS_OPERATIONAL_SPACE_H

// Controller base
#include "kinematic_chain_controller_base.h"

// hardware_interface 
#include <hardware_interface/joint_command_interface.h> // contains definition of PositionJointInterface

// hardware_interface 
#include <kuka_lwr_hw/lwr_kuka_interface.h> // contains definition of KUKAJointInterface

// msgs 
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <kuka_lwr_controllers/PathPointCaracteristics.h>

// KDL JntArray
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>

// KDL Solver
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

// Boost
#include <boost/scoped_ptr.hpp>

// STL
#include <string>

#define TRACE_OperationalSpaceController_ACTIVATED 0

namespace kuka_lwr_controllers
{
	class OperationalSpaceController: 
	public controller_interface::KinematicChainControllerBase<hardware_interface::KUKAJointInterface>
	{
		public:
			OperationalSpaceController();
			~OperationalSpaceController();

			bool init(hardware_interface::KUKAJointInterface *robot, ros::NodeHandle &n); // Init the controller
			void starting(const ros::Time& time);  // Start the controller
			void stopping(const ros::Time& time);  // Stop the controller
			void update(const ros::Time& time, const ros::Duration& period);  // Update the controller

		private:
			
			ros::Publisher			pub_robot_data_,pub_endeffector_position_; 
			ros::Subscriber			sub_imu_,sub_ft_,sub_command_, sub_kp_cartesian_, sub_kd_cartesian_,sub_traj_; 
             
			std::string 			robot_namespace_;
			KDL::Frame    			P_current_;			// current end-effector pose
			KDL::Frame    			P_initial_;			// current end-effector pose
			KDL::FrameVel 			V_current_; 		// current end-effector velocity
			KDL::JntArrayVel 		Jnt_vel_;
			KDL::JntArray			stiff_, damp_,q_des_,Kp_cartesian_ , Kd_cartesian_;	// joinst stiffeness and damping
			KDL::Jacobian 			Jkdl_;
			KDL::JntArray 			C_, G_;        			//Coriolis and Gravitational matrices
			KDL::JntSpaceInertiaMatrix 	M_; 				//Inertia matrix
			KDL::JntArrayAcc 			traj_des_;			// desired trajecory

			Eigen::VectorXd			FT_sensor_,Tau_cmd_,force_des_,Kdf_force_ , F_lamda_,Hc_ ;
			Eigen::MatrixXd			LAMDA_,p_resp_,v_resp_,KDv_, KPv_,KPl_,Alpha_v_,Sv_, Sf_;
			
		
			double orientation_x_,orientation_y_,orientation_z_,orientation_w_,roll_,pitch_,yaw_ ;  // quaternions orientations
			int count =0;

			geometry_msgs::PointStamped  end_effector_pos_msg_; 
			geometry_msgs::AccelStamped  IMU_       ;
			std_msgs::Float64MultiArray  robot_data_; 

			boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;	
			boost::scoped_ptr<KDL::ChainDynParam> id_solver_;		
			boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
			boost::scoped_ptr<KDL::ChainFkSolverVel_recursive> fk_vel_solver_;
			// callback functions
			void imuCB(const geometry_msgs::AccelStamped & msg);		 			     	// function associate to subscribe the arduino imu data
			void ft_readingsCB(const geometry_msgs::WrenchStamped& msg);
			void qcommandCB(const std_msgs::Float64MultiArrayConstPtr& msg);   				// function associate to subscribe to the desired q
			void setcartesianKp(const std_msgs::Float64MultiArrayConstPtr& msg); 			// function associate to subscribe setcartesianKp topic
			void setcartesianKd(const std_msgs::Float64MultiArrayConstPtr& msg); 			// function associate to subscribe setcartesianKd topic
			void PathPointDesiredCB(const kuka_lwr_controllers::PathPointCaracteristics::ConstPtr & msg);// function associate to a subscribe traj_cmd topic

			// control functions
			void CheckTorqueMax_(Eigen::VectorXd & TorqueCommandVector);
			void CalculatePsuedoInertia(const Eigen::MatrixXd  & J, const Eigen::MatrixXd & M, Eigen::MatrixXd & lamda );
			void CalculateAccelerationCommand(const KDL::Frame  & p_current, const KDL::FrameVel & v_current, const KDL::JntArrayAcc & traj_des, Eigen::MatrixXd & alpha_v );
			void CalculateForce(Eigen::VectorXd	 & FT_sensor_,Eigen::MatrixXd KPl_ ,Eigen::VectorXd & force_des_, Eigen::VectorXd	 & F_lamda_ ) ;
			void controlWrench_h_c( Eigen::MatrixXd	& Sv_, Eigen::MatrixXd & Sf_ , Eigen::MatrixXd & lamda ,  Eigen::MatrixXd & alpha_v , Eigen::VectorXd	 & F_lamda_,Eigen::VectorXd & Hc_ )	;
			void CalculateJointTorques(const Eigen::MatrixXd & j, const Eigen::MatrixXd & alpha_v,Eigen::VectorXd & Tau_cmd_ );


	};
}

#endif

