#ifndef LWR_HW__LWR_HW_REAL_H
#define LWR_HW__LWR_HW_REAL_H

// Laurent - Comment / Uncomment + Compile to enable / disable the fri_jacobian publisher.
#define PUBLISH_FRI_JACOBIAN_MATRIX 1

// lwr hw definition
#include "kuka_lwr_hw/lwr_hw.h"

// FRIL remote hooks
#include <FastResearchInterface.h>

#ifdef PUBLISH_FRI_JACOBIAN_MATRIX
	// Laurent 
	#include <std_msgs/Float64MultiArray.h>
#endif

#define NUMBER_OF_CYCLES_FOR_QUALITY_CHECK   2000
#define EOK 0

#define FRI_CONN_TIMEOUT_SEC	30

// Laurent
#define JACOBIAN_FRI_CART_DOF  6
#define JACOBIAN_FRI_NB_JOINTS  7

namespace lwr_hw
{

	class LWRHWFRIL : public LWRHW
	{
		
		private:

		  // Parameters
		  std::string init_file_;
		  bool file_set_ = false;

		  // low-level interface
		  boost::shared_ptr<FastResearchInterface> device_;
		  int resultValue_ = 0;
		  bool hasSwitched_;
		  
		  #ifdef PUBLISH_FRI_JACOBIAN_MATRIX
			  // Laurent
			  float ** jacobian_matrix_fri_ = NULL;
			  
			  void new_jacobian_matrix_() 
			  {
				  jacobian_matrix_fri_ = (float **)malloc(JACOBIAN_FRI_CART_DOF*sizeof(float *));
				  for (int i=0; i<JACOBIAN_FRI_CART_DOF; i++)
				  {
					  jacobian_matrix_fri_[i] = (float *)malloc(JACOBIAN_FRI_NB_JOINTS*sizeof(float));
				  }
			  }
			  
			  void free_jacobian_matrix_()
			  {
				  for (int i=0; i<JACOBIAN_FRI_CART_DOF; i++)
				  {
					  free(jacobian_matrix_fri_[i]);
				  }
				  
				  jacobian_matrix_fri_=NULL;
			  }
			  
			  ros::Publisher pub_fri_jacobian_;
			  std_msgs::Float64MultiArray  robot_fri_jacobian_;
			  ros::NodeHandle lwr_nh_;
		 #endif
		  
		public:

		  LWRHWFRIL() : LWRHW() {}
		  ~LWRHWFRIL() {
			  #ifdef PUBLISH_FRI_JACOBIAN_MATRIX
				// Laurent
				free_jacobian_matrix_();  
			  #endif
			  }

		  void stop(){return;};
		  void set_mode(){return;};

		  void setInitFile(std::string init_file){init_file_ = init_file; file_set_ = true;};

		  // Init, read, and write, with FRI hooks
		  bool init()
		  {
			if( !(file_set_) )
			{
			  std::cout << "Did you forget to set the init file?" << std::endl
						<< "You must do that before init()" << std::endl
						<< "Exiting..." << std::endl;
			  return false;
			}

			std::cout << "Before new FastResearchInterface(" << init_file_.c_str() << ")" << std::endl;

			// construct a low-level lwr
			device_.reset( new FastResearchInterface( init_file_.c_str() ) );

			std::cout << "After new FastResearchInterface(" << init_file_.c_str() << ")" << std::endl;
			
			setControlStrategy(NONE);
			
			hasSwitched_ = false;
			
			#ifdef PUBLISH_FRI_JACOBIAN_MATRIX
				// Laurent
				new_jacobian_matrix_();
				robot_fri_jacobian_.data.resize(42);
			
				pub_fri_jacobian_ = lwr_nh_.advertise<std_msgs::Float64MultiArray>("/fri_jacobian", 1000);
			#endif


			return true;
		  }

		  void read(ros::Time time, ros::Duration period)
		  {
			float msrJntPos[n_joints_];
			float msrJntTrq[n_joints_];
			float msrCartPose[NUMBER_OF_FRAME_ELEMENTS];
			float msrCartWrench[NUMBER_OF_CART_DOFS];

			device_->GetMeasuredJointPositions( msrJntPos );
			device_->GetMeasuredJointTorques( msrJntTrq );
			device_->GetMeasuredCartPose( msrCartPose );
			device_->GetEstimatedExternalCartForcesAndTorques( msrCartWrench );
			
			#if PUBLISH_FRI_JACOBIAN_MATRIX == 1
				// Laurent
				device_->GetCurrentJacobianMatrix(jacobian_matrix_fri_);
			#endif

			for (int j = 0; j < n_joints_; j++)
			{
			  joint_position_prev_[j] = joint_position_[j];
			  joint_position_[j] = (double)msrJntPos[j];
			  joint_position_kdl_(j) = joint_position_[j];
			  joint_effort_[j] = (double)msrJntTrq[j];
			  joint_velocity_prev_[j] = joint_velocity_[j];
			  joint_velocity_[j] = filters::exponentialSmoothing((joint_position_[j]-joint_position_prev_[j])/period.toSec(), joint_velocity_[j], 0.2);
			  
			  joint_acceleration_[j] = filters::exponentialSmoothing((joint_velocity_[j]-joint_velocity_prev_[j])/period.toSec(), joint_acceleration_[j], 0.2);
			  joint_stiffness_[j] = joint_stiffness_command_[j];
			  joint_damping_[j] = joint_damping_command_[j];
			}
			
			for (int i=0; i<NUMBER_OF_CART_DOFS; i++)
			{
				cart_stiff_[i] = cart_stiff_command_[i];
			    cart_damp_[i] = cart_damp_command_[i];
			    cart_wrench_[i] = (double)msrCartWrench[i];
			}
			
			for (int i=0; i < NUMBER_OF_FRAME_ELEMENTS; ++i)
			{
				cart_pose_[i] = (double)msrCartPose[i];
			}
			
			#ifdef PUBLISH_FRI_JACOBIAN_MATRIX
			
				//Laurent - Comment this loop to deactivate the Jacobian calculation
				for (int i=0; i<JACOBIAN_FRI_CART_DOF; i++)
				{
					for (int j=0; j<JACOBIAN_FRI_NB_JOINTS; j++)
					{ 
						robot_fri_jacobian_.data[(i*JACOBIAN_FRI_NB_JOINTS)+j]=jacobian_matrix_fri_[i][j];
					}
			
				}
				pub_fri_jacobian_.publish(robot_fri_jacobian_);
				
			#endif
			
			return;
		  }

		  void write(ros::Time time, ros::Duration period)
		  {
			enforceLimits(period);

			// ensure the robot is powered and it is in control mode, almost like the isMachineOk() of Standford
			if ( device_->IsMachineOK() )
			{
			 
			  if (hasSwitched_)
			  {
				   device_->WaitForKRCTick();
			  }
			 
			  switch (getControlStrategy())
			  {
				case JOINT_POSITION:

				  // Ensure the robot is in this mode
				  if (device_->GetCurrentControlScheme() == FastResearchInterface::JOINT_POSITION_CONTROL)
				  {
					 float newJntPosition[n_joints_];
					 
					 for (int j = 0; j < n_joints_; j++)
					 {
					   newJntPosition[j] = (float)joint_position_command_[j];
					 }
					 
					 device_->SetCommandedJointPositions(newJntPosition);
					// ROS_INFO("lwr_hw_fril::write -> j0=%f, j1=%f, j2=%f, j3=%f, j4=%f, j5=%f, j6=%f", newJntPosition[0], newJntPosition[1],newJntPosition[2],newJntPosition[3],newJntPosition[4],newJntPosition[5],newJntPosition[6]);
				  }
				  break;

				case CARTESIAN_IMPEDANCE:
				  // Ensure the robot is in this mode
				  if (device_->GetCurrentControlScheme() == FastResearchInterface::CART_IMPEDANCE_CONTROL)
				  {
					  float newCartStiff[NUMBER_OF_CART_DOFS];
					  float newCartDamp[NUMBER_OF_CART_DOFS];
					  float newCartWrench[NUMBER_OF_CART_DOFS];
					  float newCartPose[NUMBER_OF_FRAME_ELEMENTS];
					  
					  for (int j=0; j<NUMBER_OF_CART_DOFS; j++)
					  {
						  newCartStiff[j] = (float)cart_stiff_command_[j];
						  newCartDamp[j] = (float)cart_damp_command_[j];
						  newCartWrench[j] = (float)cart_wrench_command_[j];
					  }
					  
					  for (int i=0; i<NUMBER_OF_FRAME_ELEMENTS; i++)
					  {
						  newCartPose[i] = (float)cart_pose_command_[i];
					  }
					  
					  device_->SetCommandedCartStiffness(newCartStiff);
					  device_->SetCommandedCartDamping(newCartDamp);
					  device_->SetCommandedCartForcesAndTorques(newCartWrench);
					  device_->SetCommandedCartPose(newCartPose);
					  
				  }
				  break;

				 case JOINT_IMPEDANCE:

				  // Ensure the robot is in this mode
				  if (device_->GetCurrentControlScheme() == FastResearchInterface::JOINT_IMPEDANCE_CONTROL)
				  {
					   float newJntPosition[n_joints_];
					   float newJntStiff[n_joints_];
					   float newJntDamp[n_joints_];
					   float newJntAddTorque[n_joints_];

					   // WHEN THE URDF MODEL IS PRECISE
					   // 1. compute the gracity term
					   // f_dyn_solver_->JntToGravity(joint_position_kdl_, gravity_effort_);

					   // 2. read gravity term from FRI and add it with opposite sign and add the URDF gravity term
					   // newJntAddTorque = gravity_effort_  - device_->getF_DYN??

						for(int j=0; j < n_joints_; j++)
						{
						  newJntPosition[j] = (float)joint_position_command_[j];
						  newJntAddTorque[j] = (float)joint_effort_command_[j];
						  newJntStiff[j] = (float)joint_stiffness_command_[j];
						  newJntDamp[j] = (float)joint_damping_command_[j];
						}
						
						device_->SetCommandedJointStiffness(newJntStiff);
						device_->SetCommandedJointPositions(newJntPosition);
						device_->SetCommandedJointDamping(newJntDamp);
						device_->SetCommandedJointTorques(newJntAddTorque);
				  }
				  break;

				 case GRAVITY_COMPENSATION:
				   break;
			   }
			}
			return;
		  }
		  
		  
		  void printInterfaces(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list)
			{
				
				ROS_INFO("\nStart Interfaces :");
				
				for ( std::list<hardware_interface::ControllerInfo>::const_iterator it = start_list.begin(); it != start_list.end(); ++it )
				{
					ROS_INFO("\n%s",it->hardware_interface.c_str());
				}
				
				ROS_INFO("\nStop Interfaces :");
				
				for ( std::list<hardware_interface::ControllerInfo>::const_iterator it = stop_list.begin(); it != stop_list.end(); ++it )
				{
					ROS_INFO("\n%s",it->hardware_interface.c_str());
				}
				
			}
		  
		  
			ControlStrategy getNewControlStrategy(const std::list< hardware_interface::ControllerInfo >& start_list, const std::list< hardware_interface::ControllerInfo >& stop_list, ControlStrategy default_control_strategy)
			{
				ControlStrategy desired_strategy = default_control_strategy;

				// If any of the controllers in the start list works on a velocity interface, the switch can't be done.
				for ( std::list<hardware_interface::ControllerInfo>::const_iterator it = start_list.begin(); it != start_list.end(); ++it )
				{
					if( it->hardware_interface.compare( std::string("hardware_interface::PositionJointInterface") ) == 0 )
					{
						std::cout << "Request to switch to hardware_interface::PositionJointInterface (JOINT_POSITION)" << std::endl;
						desired_strategy = JOINT_POSITION;
						break;
					}
					else if( it->hardware_interface.compare( std::string("hardware_interface::KUKAJointInterface") ) == 0 )
					{
						std::cout << "Request to switch to hardware_interface::KUKAJointInterface (JOINT_IMPEDANCE)" << std::endl;
						desired_strategy = JOINT_IMPEDANCE;
						break;
					}
					else if( it->hardware_interface.compare( std::string("hardware_interface::KUKACartesianInterface") ) == 0 )
					{
						std::cout << "Request to switch to hardware_interface::KUKACartesianInterface (CARTESIAN_IMPEDANCE)" << std::endl;
						desired_strategy = CARTESIAN_IMPEDANCE;
						break;
					}
				}

				return desired_strategy;
			}
		  
		  void doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list)
		  {
				ROS_INFO("doSwitch !");
				
				printInterfaces(start_list,stop_list);
				
				hasSwitched_=false;
				
				ControlStrategy desired_strategy;
				
				desired_strategy = getNewControlStrategy(start_list,stop_list,NONE);

				if (desired_strategy != NONE)
				{
				
					ROS_INFO("doSwitch current strategy=%d, desired_strategy=%d", current_strategy_ , desired_strategy);

					for (int j = 0; j < n_joints_; ++j)
					{
					  ///semantic Zero
					  joint_position_command_[j] = joint_position_[j];
					  joint_effort_command_[j] = 0.0;

					  ///call setCommand once so that the JointLimitsInterface receive the correct value on their getCommand()!
					  try{  position_interface_.getHandle(joint_names_[j]).setCommand(joint_position_command_[j]);  }
					  catch(const hardware_interface::HardwareInterfaceException&){}
					  try{  effort_interface_.getHandle(joint_names_[j]).setCommand(joint_effort_command_[j]);  }
					  catch(const hardware_interface::HardwareInterfaceException&){}

					  ///reset joint_limit_interfaces
					  pj_sat_interface_.reset();
					  pj_limits_interface_.reset();
					}

					device_->StopRobot();
			
					switch( desired_strategy )
					{
						case JOINT_POSITION:
						  resultValue_ = device_->StartRobot( FastResearchInterface::JOINT_POSITION_CONTROL,FRI_CONN_TIMEOUT_SEC);
						  if (resultValue_ != EOK)
						  {
							std::cout << "An error occurred during starting the robot, couldn't switch to JOINT_POSITION...\n" << std::endl;
							return;
						  }
						  break;
						 case JOINT_IMPEDANCE:
						  resultValue_ = device_->StartRobot( FastResearchInterface::JOINT_IMPEDANCE_CONTROL,FRI_CONN_TIMEOUT_SEC);
						  if (resultValue_ != EOK)
						  {
							std::cout << "An error occurred during starting the robot, couldn't switch to JOINT_IMPEDANCE...\n" << std::endl;
							return;
						  }
						  break;
						  
						 case CARTESIAN_IMPEDANCE:
						  resultValue_ = device_->StartRobot( FastResearchInterface::CART_IMPEDANCE_CONTROL,FRI_CONN_TIMEOUT_SEC);
						  if (resultValue_ != EOK)
						  {
							std::cout << "An error occurred during starting the robot, couldn't switch to CARTESIAN_IMPEDANCE...\n" << std::endl;
							return;
						  }
						 break; 
					}

					// if sucess during the switch in FRI, set the ROS strategy
					setControlStrategy(desired_strategy);

					hasSwitched_=true;

					std::cout << "The ControlStrategy changed to: " << getControlStrategy() << std::endl;
				}
		  }
	};
}

#endif
