#include <one_task_inverse_kinematics.h>

// Utils for pseudo inverse and skew_symmetric
#include <utils/pseudo_inversion.h>
#include <utils/skew_symmetric.h>

// For plugin
#include <pluginlib/class_list_macros.h>

namespace kuka_lwr_controllers 
{
    OneTaskInverseKinematics::OneTaskInverseKinematics() {}
    OneTaskInverseKinematics::~OneTaskInverseKinematics() {}

    bool OneTaskInverseKinematics::init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n)
    {
		ROS_INFO("***** START OneTaskInverseKinematics::init ************");

        if( !(KinematicChainControllerBase<hardware_interface::PositionJointInterface>::init(robot, n)) )
        {
            ROS_ERROR("Couldn't initilize OneTaskInverseKinematics controller.");
            return false;
        }

        jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
        fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
        ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));
        ik_pos_solver_.reset(new KDL::ChainIkSolverPos_NR_JL(kdl_chain_,joint_limits_.min,joint_limits_.max,*fk_pos_solver_,*ik_vel_solver_));

        q_cmd_.resize(kdl_chain_.getNrOfJoints());
        J_.resize(kdl_chain_.getNrOfJoints());

        // get joint positions
        for(int i=0; i < joint_handles_.size(); i++)
        {
            joint_msr_states_.q(i) = joint_handles_[i].getPosition();
            joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
            joint_des_states_.q(i) = joint_msr_states_.q(i);
        }

        // computing forward kinematics
        fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);

        //Desired posture is the current one
        x_des_ = x_;

        cmd_flag_ = 0;

        sub_command_ = nh_.subscribe("command", 1, &OneTaskInverseKinematics::command, this);
        
        ROS_INFO("***** FINISH OneTaskInverseKinematics::init ************");

        return true;
    }

    void OneTaskInverseKinematics::starting(const ros::Time& time)
    {
		ROS_INFO("***** OneTaskInverseKinematics::starting ************");
		cmd_flag_ = 0;  // set this flag to 0 to not run the update method
    }

    void OneTaskInverseKinematics::update(const ros::Time& time, const ros::Duration& period)
    {
	//ROS_INFO("***** OneTaskInverseKinematics::update debut ************");

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

            // end-effector position error
            x_err_.vel = x_des_.p - x_.p;

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

            // joint limits saturation
            for (int i =0;  i < joint_handles_.size(); i++)
            {
                if (joint_des_states_.q(i) < joint_limits_.min(i))
                    joint_des_states_.q(i) = joint_limits_.min(i);
                if (joint_des_states_.q(i) > joint_limits_.max(i))
                    joint_des_states_.q(i) = joint_limits_.max(i);
            }

            if (Equal(x_, x_des_, 0.015))
            {
                ROS_INFO("On target");
                cmd_flag_ = 0;
            }
            
           /*
            ROS_INFO("update position desired -> x = %f, y = %f, z = %f", x_des_.p.x(), x_des_.p.y(), x_des_.p.z());
            ROS_INFO("update position calculated -> x = %f, y = %f, z = %f", x_.p.x(), x_.p.y(), x_.p.z());
            ROS_INFO("error translation (des-cal) -> x = %f, y = %f, z = %f", x_err_.vel.x(), x_err_.vel.y(), x_err_.vel.z());
            ROS_INFO("update rotation calculated ->  O = %f, 1 = %f, 2 = %f, 3 = %f, 4 = %f, 5 = %f, 6 = %f, 7 = %f, 8 = %f", x_.M.data[0],x_.M.data[1],x_.M.data[2],x_.M.data[3],x_.M.data[4],x_.M.data[5],x_.M.data[6],x_.M.data[7],x_.M.data[8]);
            ROS_INFO("update rotation desired ->  O = %f, 1 = %f, 2 = %f, 3 = %f, 4 = %f, 5 = %f, 6 = %f, 7 = %f, 8 = %f", x_des_.M.data[0],x_des_.M.data[1],x_des_.M.data[2],x_des_.M.data[3],x_des_.M.data[4],x_des_.M.data[5],x_des_.M.data[6],x_des_.M.data[7],x_des_.M.data[8]);
           */

	    // set controls for joints
            for (int i = 0; i < joint_handles_.size(); i++)
            {
               joint_handles_[i].setCommand(joint_des_states_.q(i));
            }

        }
        
      /*  for (int i=0; i<joint_handles_.size(); i++)
        {
			if (joint_des_states_.q(i)-joint_msr_states_.q(i) > 0.001)
			{
				 ROS_INFO("joint n°%d -> limit position !!!",i);
				 ROS_INFO("joint n°%d -> %f !!!",i,joint_des_states_.q(i)-joint_msr_states_.q(i));
				 ROS_INFO("joint limit of n°%d -> min = %f, max = %f !!!",i,joint_limits_.min(i), joint_limits_.max(i));
				 joint_handles_[i].setCommand(joint_msr_states_.q(i)+0.001);
			 }
			 else
				if (joint_des_states_.q(i)-joint_msr_states_.q(i) < -0.001)
				{
					 ROS_INFO("joint n°%d -> limit position !!!",i);
					 ROS_INFO("joint n°%d -> %f !!!",i,joint_des_states_.q(i)-joint_msr_states_.q(i));
					 ROS_INFO("joint limit of n°%d -> min = %f, max = %f !!!",i,joint_limits_.min(i), joint_limits_.max(i));
					 joint_handles_[i].setCommand(joint_msr_states_.q(i)-0.001);
				 }
				 else
					joint_handles_[i].setCommand(joint_des_states_.q(i));
				 
		}*/

	//ROS_INFO("***** OneTaskInverseKinematics::update fin ************");
    }

    void OneTaskInverseKinematics::command(const kuka_lwr_controllers::PoseRPY::ConstPtr &msg)
    {
		ROS_INFO("***** START OneTaskInverseKinematics::command ************");

        KDL::Frame frame_des_;

        switch(msg->id)
        {
            case 0:
			ROS_INFO("***** OneTaskInverseKinematics::command position and orientation ************");
			//ROS_INFO("position desired -> x = %f, y = %f, z = %f", msg->position.x, msg->position.y, msg->position.z);
				frame_des_ = KDL::Frame(
                    KDL::Rotation::RPY(msg->orientation.roll,
                                      msg->orientation.pitch,
                                      msg->orientation.yaw),
                    KDL::Vector(msg->position.x,
                                msg->position.y,
                                msg->position.z));
            break;

            case 1: // position only
		ROS_INFO("***** OneTaskInverseKinematics::command position only ************");
				frame_des_ = KDL::Frame(
                KDL::Vector(msg->position.x,
                            msg->position.y,
                            msg->position.z));
            break;

            case 2: // orientation only
		ROS_INFO("***** OneTaskInverseKinematics::command orientation only ************");
				frame_des_ = KDL::Frame(
                KDL::Rotation::RPY(msg->orientation.roll,
                                   msg->orientation.pitch,
                                   msg->orientation.yaw));
            break;

            default:
				ROS_INFO("Wrong message ID");
            return;
        }

        x_des_ = frame_des_;
        cmd_flag_ = 1;
        
        ROS_INFO("***** FINISH OneTaskInverseKinematics::command ************");
    }

}

PLUGINLIB_EXPORT_CLASS(kuka_lwr_controllers::OneTaskInverseKinematics, controller_interface::ControllerBase)

