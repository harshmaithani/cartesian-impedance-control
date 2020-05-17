/*
*Harsh Maithani
*Universite Clermont Auvergne
 * 
*/

#ifndef LWR_CONTROLLERS_HARSH_TRUST_IMPEDANCE_CONTROLLER_H
#define LWR_CONTROLLERS_HARSH_TRUST_IMPEDANCE_CONTROLLER_H

// Controller base
#include "kinematic_chain_controller_base.h"

// Hardware_interface 
#include <kuka_lwr_hw/lwr_kuka_cartesian_interface.h> // contains definition of KUKACartesianInterface

// hardware_interface 
#include <kuka_lwr_hw/lwr_kuka_interface.h> // contains definition of KUKAJointInterface

// msgs 
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/WrenchStamped.h>

// KDL JntArray
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>

// KDL Solver
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

// msgs Float64MultiArray and PoseStamped
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>

// KDL added
#include <kdl/stiffness.hpp>
#include <kdl/trajectory.hpp>
#include <kdl_conversions/kdl_msg.h>

// ROS Realtime Publisher Tools
#include <realtime_tools/realtime_publisher.h>

// BOOST added
#include <boost/scoped_ptr.hpp>

// TF conversions
#include <tf_conversions/tf_kdl.h>

// STL
#include <string>

// FRI Type IRML
#include <TypeIRML.h>

#define TRACE_CARTESIAN_IMPEDANCE_CONTROLLER_ACTIVATED 1

namespace kuka_lwr_controllers
{
	class HarshTrustImpedanceController: 
	public controller_interface::KinematicChainControllerBase<hardware_interface::KUKACartesianInterface>
	{
		public:
			HarshTrustImpedanceController();
			~HarshTrustImpedanceController();

			bool init(hardware_interface::KUKACartesianInterface *robot, ros::NodeHandle &n); // Init the controller
			void starting(const ros::Time& time);  // Start the controller
			void stopping(const ros::Time& time);  // Stop the controller
			void update(const ros::Time& time, const ros::Duration& period);  // Update the controller
			void setCartesianStiffness(const std_msgs::Float64MultiArrayConstPtr& msg);
			void setCartesianDamping(const std_msgs::Float64MultiArrayConstPtr& msg);
			
			void setCartesianPose(const std_msgs::Float64MultiArrayConstPtr& msg);
			void setCartesianWrench(const std_msgs::Float64MultiArrayConstPtr& msg);
			void publishCurrentPose(const KDL::Frame& f);
			
			
		private:
			ros::Subscriber sub_cart_stiffness_command_, sub_cart_damping_command_, sub_cart_pose_command_, sub_cart_wrench_command_;
			ros::Publisher pub_cart_imp_data_;
			ros::Publisher pub_cart_imp_pose_meters_;
			ros::Publisher pub_cart_imp_pose_mm_;
			ros::Publisher pub_cart_imp_pose_meters_rpy_rad_;
			ros::Publisher pub_cart_imp_pose_mm_rpy_deg_;
			ros::Publisher pub_cart_imp_joint_states_rad_;
			ros::Publisher pub_cart_imp_joint_states_deg_;
			ros::Publisher pub_cart_imp_joint_velocity_rads_;
			ros::Publisher pub_cart_imp_joint_velocity_degs_;
			ros::Publisher pub_cart_imp_kdl_cart_velocity_m_rad_;
			ros::Publisher pub_cart_imp_kdl_cart_velocity_mm_deg_;
			ros::Publisher pub_cart_imp_kdl_joint_motor_forces_;
			ros::Publisher pub_cart_imp_joint_motor_torques_;
			ros::Publisher pub_cart_imp_kdl_jacobian_;
			ros::Publisher pub_cart_imp_estimated_forces_;
			ros::Publisher pub_cart_imp_commanded_forces_;
			ros::Publisher pub_cart_imp_stiffness_;
			ros::Publisher pub_cart_imp_damping_;
			
		
		
			std::string robot_namespace_;
			hardware_interface::KUKACartesianStiffnessHandle kuka_cart_stiff_handle_;
			hardware_interface::KUKACartesianDampingHandle kuka_cart_damp_handle_;
			hardware_interface::KUKACartesianPoseHandle kuka_cart_pose_handle_;
			hardware_interface::KUKACartesianWrenchHandle kuka_cart_wrench_handle_;
			
			KDL::Frame    			    pose_current_;			// Current end-effector pose
			KDL::Frame    			    pose_initial_;			// Current end-effector pose
			KDL::FrameVel 			    cartesian_velocity_current_; 		// Current end-effector velocity
			KDL::Jacobian 			    jacobian_;              // Current Jacobian 
			KDL::JntArray				joint_positions_;       // Current Joint Positions for computing Jacobian	
			KDL::JntArrayVel			joint_velocities_;      // Current Joint Velocities for computing Cartesian Velocities
		
			Eigen::VectorXd			    FT_sensor_,Tau_cmd_ ;
			Eigen::VectorXd			    Jacobian_pinv_;		   // Jacobian Inverse
			
			Eigen::VectorXd				robot_forces_;         // Robot commanded forces
			Eigen::VectorXd				joint_motor_torques_eigen_; // Jacobian Transpose
			
			Eigen::MatrixXd 			jacobian_eigen_; // Current Jacobian in eigen format
			Eigen::MatrixXd				jacobian_transpose_eigen_; // Jacobian Transpose
			Eigen::MatrixXd 			jacobian_transpose_inverse_eigen_; // Current Jacobian in eigen format
			
			
			int trace_count_ = 0;
            int counter2=0;
			
			boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;	
			boost::scoped_ptr<KDL::ChainDynParam> id_solver_;		
			boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
			boost::scoped_ptr<KDL::ChainFkSolverVel_recursive> fk_vel_solver_;
			
			// Utility function to get the current pose
			void getCurrentPose_(KDL::Frame& frame);
			
			// Utility function to get the external estimated forces and torques
			void getExternalEstimatedForces_(KDL::Wrench& wrench);
			
			// FRI<->KDL conversion
			void fromKDLtoFRI_(const KDL::Frame& frame_in, std::vector<double>& vect_Pose_FRI_out);
			
			// Utility function to forward commands to FRI
			void forwardCmdFRI_(const KDL::Frame& frame, const KDL::Stiffness& stiffness, const KDL::Stiffness& damping, const KDL::Wrench& wrench);
			
			int cmd_flag_;  // flag set only to 1 when the controller receive a message to the command topic

			KDL::Frame pose_cur_;
			KDL::Frame pose_init_;
			KDL::Frame pose_des_;

			KDL::Stiffness stiff_cur_;
			KDL::Stiffness damp_cur_;
			KDL::Wrench wrench_cur_;
			KDL::Wrench wrench_external_estimated_;
			
			std_msgs::Float64MultiArray  robot_data_; 	 // local variable for the publishers 
			std_msgs::Float64MultiArray  robot_pose_meters_;
			std_msgs::Float64MultiArray  robot_pose_mm_;
			std_msgs::Float64MultiArray  robot_pose_meters_rpy_rad_;
			std_msgs::Float64MultiArray  robot_pose_mm_rpy_deg_;
			std_msgs::Float64MultiArray  robot_joint_states_rad_;
			std_msgs::Float64MultiArray  robot_joint_states_deg_;
			std_msgs::Float64MultiArray  robot_joint_velocity_rads_;
			std_msgs::Float64MultiArray  robot_joint_velocity_degs_;
			std_msgs::Float64MultiArray  robot_cartesian_velocity_m_rad_;
			std_msgs::Float64MultiArray  robot_cartesian_velocity_mm_deg_;
		    
		    std_msgs::Float64MultiArray  robot_joint_motor_forces_; 
			std_msgs::Float64MultiArray  robot_joint_motor_torques_;  
			std_msgs::Float64MultiArray  robot_estimated_forces_;
			std_msgs::Float64MultiArray  robot_commanded_forces_;
			std_msgs::Float64MultiArray  robot_jacobian_;
			std_msgs::Float64MultiArray  robot_stiffness_; 
			std_msgs::Float64MultiArray  robot_damping_;
			 
			std::vector<double> cur_Pose_FRI_;
			std::vector<double> des_Pose_FRI_;
			
			double roll_,pitch_,yaw_; // For roll pitch and yaw
			
			TypeIRML *RML_;
			TypeIRMLInputParameters *IP_;
			TypeIRMLOutputParameters *OP_;
			double cycleTime_;
			int resultValue_;
			
			static constexpr int NUMBER_OF_CART_DOFS = 6;
			static constexpr int NUMBER_OF_FRAME_ELEMENTS = 12;
			
			boost::shared_ptr< realtime_tools::RealtimePublisher< geometry_msgs::PoseStamped > > realtime_pose_pub_;
			
	};
	
}

#endif
