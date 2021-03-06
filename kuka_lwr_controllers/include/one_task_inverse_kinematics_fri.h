#ifndef LWR_CONTROLLERS_ONE_TASK_INVERSE_KINEMATICS_H
#define LWR_CONTROLLERS_ONE_TASK_INVERSE_KINEMATICS_H

// Controller base
#include "kinematic_chain_controller_base.h"

// KDL
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

// Boost
#include <boost/scoped_ptr.hpp>

// hardware_interface 
#include <hardware_interface/joint_command_interface.h> // contains definition of PositionJointInterface

// Ros messages generated
#include <kuka_lwr_controllers/PoseRPY.h>

// FRI Type IRML
#include <TypeIRML.h>

#include <string>

#ifndef PI
	#define PI 3.1415926535897932384626433832795
#endif

#ifndef RAD
	#define RAD(A)	((A) * PI / 180.0 )
#endif

#ifndef DEG
	#define DEG(A)	((A) * 180.0 / PI )
#endif

namespace kuka_lwr_controllers
{
	class OneTaskInverseKinematicsFRI: 
	public controller_interface::KinematicChainControllerBase<hardware_interface::PositionJointInterface>
	{
		public:
			OneTaskInverseKinematicsFRI();
			~OneTaskInverseKinematicsFRI();

			bool init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n);
			void starting(const ros::Time& time);
			void update(const ros::Time& time, const ros::Duration& period);
			void command(const kuka_lwr_controllers::PoseRPY::ConstPtr &msg);

		private:
			ros::Subscriber sub_command_;
			ros::Subscriber sub_gains_;

			KDL::Frame x_;		//current pose
			KDL::Frame x_des_;	//desired pose

			KDL::Twist x_err_;

			KDL::JntArray q_cmd_; // computed set points

			KDL::Jacobian J_;	//Jacobian

			Eigen::MatrixXd J_pinv_;
			Eigen::Matrix<double,3,3> skew_;

			struct quaternion_
			{
				KDL::Vector v;
				double a;
			} quat_curr_, quat_des_;

			KDL::Vector v_temp_;
	
			int cmd_flag_;
	
			boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
			boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
			boost::scoped_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
			boost::scoped_ptr<KDL::ChainIkSolverPos_NR_JL> ik_pos_solver_;

			TypeIRML *RML_;
			TypeIRMLInputParameters *IP_;
			TypeIRMLOutputParameters *OP_;
			double cycleTime_;
			int resultValue_;
	};
}

#endif
