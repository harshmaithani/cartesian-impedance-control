#include "kuka_lwr_hw/lwr_hw_gazebo.h"

namespace lwr_hw
{
	
		void LWRHWGazebo::setParentModel(gazebo::physics::ModelPtr parent_model)
			{
			  parent_model_ = parent_model; parent_set_ = true;
			}
			
			// Init, read, and write, with Gazebo hooks
			bool LWRHWGazebo::init()
			{
				ROS_INFO("LWRHWGazebo  --> init()");
				
				if( !(parent_set_) )
				{
					ROS_INFO("Did you forget to set the parent model? You must do that before init(). Exiting...");
					return false;
				}

				gazebo::physics::JointPtr joint;
				for(int j=0; j < n_joints_; j++)
				{
					joint = parent_model_->GetJoint(joint_names_[j]);
					if (!joint)
					{
						ROS_INFO("This robot has a joint named %s , which is not in the gazebo model.",joint_names_[j].c_str());
						return false;
					}
					sim_joints_.push_back(joint);
				}
				
				setControlStrategy(NONE);
			
				hasSwitched_ = false;

				return true;
			}
			
			void LWRHWGazebo::read(ros::Time time, ros::Duration period)
			{
				#if (DEBUG)
					ROS_INFO("LWRHWGazebo  --> read()");
				#endif
				
				//ROS_INFO("LWRHWGazebo -> read,  sim_joint[0]=%f",sim_joints_[0]->GetAngle(0).Radian());
				
				for(int j=0; j < n_joints_; ++j)
				{
				  joint_position_prev_[j] = joint_position_[j];
				  joint_position_[j] += angles::shortest_angular_distance(joint_position_[j],sim_joints_[j]->GetAngle(0).Radian());
				  
				
				  joint_position_kdl_(j) = joint_position_[j];

				  joint_velocity_prev_[j] = joint_velocity_[j];
				  // derivate velocity as in the real hardware instead of reading it from simulation
				  joint_velocity_[j] = filters::exponentialSmoothing((joint_position_[j] - joint_position_prev_[j])/period.toSec(), joint_velocity_[j], 0.2);
				  joint_acceleration_[j] = filters::exponentialSmoothing((joint_velocity_[j]-joint_velocity_prev_[j])/period.toSec(), joint_acceleration_[j], 0.2);
				  joint_effort_[j] = sim_joints_[j]->GetForce((int)(0));
				  joint_stiffness_[j] = joint_stiffness_command_[j];
				  joint_damping_[j] = joint_damping_command_[j];
				  
				}
				
				for (int i=0; i<NUMBER_OF_CART_DOFS; i++)
				{
					cart_stiff_[i] = cart_stiff_command_[i];
					cart_damp_[i] = cart_damp_command_[i];
				}
				
				//ROS_INFO("LWRHWGazebo -> read,  joint_position_[0]=%f",joint_position_[0]);
			}
			
			void LWRHWGazebo::write(ros::Time time, ros::Duration period)
			{
				#if (DEBUG)
					ROS_INFO("LWRHWGazebo  --> write()");
				#endif
				
				enforceLimits(period);

				switch (getControlStrategy())
				{
						case JOINT_POSITION:
							#if (DEBUG)
								ROS_INFO("LWRHWGazebo  --> write() JOINT_POSITION strategy");
							#endif
							for(int j=0; j < n_joints_; j++)
							{
								// according to the gazebo_ros_control plugin, this must *not* be called if SetForce is going to be called
								// but should be called when SetPostion is going to be called
								// so enable this when I find the SetMaxForce reset.
								// sim_joints_[j]->SetMaxForce(0, joint_effort_limits_[j]);
								#if GAZEBO_MAJOR_VERSION >= 4
									  sim_joints_[j]->SetPosition(0, joint_position_command_[j]);
								#else
									  sim_joints_[j]->SetAngle(0, joint_position_command_[j]);
								#endif
							}
							//ROS_INFO("LWRHWGazebo -> write,  joint_position_command_[0]=%f",joint_position_command_[0]);
							break;

						case CARTESIAN_IMPEDANCE:
							//ROS_WARN("CARTESIAN IMPEDANCE NOT IMPLEMENTED");
							break;

						case JOINT_IMPEDANCE:
							// compute the gracity term
							f_dyn_solver_->JntToGravity(joint_position_kdl_, gravity_effort_);

							for(int j=0; j < n_joints_; j++)
							{
							  // replicate the joint impedance control strategy
							  // tau = k (q_FRI - q_msr) + tau_FRI + D(q_msr) + f_dyn(q_msr)
							  //const double stiffness_effort = 0.0;//10.0*( joint_position_command_[j] - joint_position_[j] ); // joint_stiffness_command_[j]*( joint_position_command_[j] - joint_position_[j] );
							  //double damping_effort = joint_damping_command_[j]*( joint_velocity_[j] );
							  //const double effort = stiffness_effort + joint_effort_command_[j] + gravity_effort_(j);
							  const double effort = joint_effort_command_[j];
							  sim_joints_[j]->SetForce(0, effort);
							  
							  //ROS_INFO("LWRHWGazebo -> write,  JOINT_IMPEDANCE effort =%f",effort);
							  #if GAZEBO_MAJOR_VERSION >= 4
									  sim_joints_[j]->SetPosition(0, joint_position_command_[j]);
							  #else
									  sim_joints_[j]->SetAngle(0, joint_position_command_[j]);
							  #endif
							}
							break;

						case GRAVITY_COMPENSATION:
							ROS_WARN("CARTESIAN IMPEDANCE NOT IMPLEMENTED");
							break;
				}
			}
			
			void LWRHWGazebo::printInterfaces(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list)
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
			
			LWRHW::ControlStrategy LWRHWGazebo::getNewControlStrategy(const std::list< hardware_interface::ControllerInfo >& start_list, const std::list< hardware_interface::ControllerInfo >& stop_list, ControlStrategy default_control_strategy)
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
					else if( (it->hardware_interface.compare( std::string("hardware_interface::KUKAJointInterface") ) == 0) || (it->hardware_interface.compare( std::string("hardware_interface::EffortJointInterface") ) == 0) )
					{
						std::cout << "Request to switch to hardware_interface::KUKAJointInterface (JOINT_IMPEDANCE)" << std::endl;
						desired_strategy = JOINT_IMPEDANCE;
						break;
					}
					else if( it->hardware_interface.compare( std::string("hardware_interface::PositionCartesianInterface") ) == 0 )
					{
						std::cout << "Request to switch to hardware_interface::PositionCartesianInterface (CARTESIAN_IMPEDANCE)" << std::endl;
						desired_strategy = CARTESIAN_IMPEDANCE;
						break;
					}
				}

				return desired_strategy;
			}
			
		  void LWRHWGazebo::doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list)
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

					// if sucess during the switch in FRI, set the ROS strategy
					setControlStrategy(desired_strategy);

					hasSwitched_=true;

					std::cout << "The ControlStrategy changed to: " << getControlStrategy() << std::endl;
				}
		  }
	
}
