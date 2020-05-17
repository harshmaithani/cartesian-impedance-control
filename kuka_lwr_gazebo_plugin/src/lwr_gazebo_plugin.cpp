// Boost
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

// ROS
#include <ros/ros.h>
#include <urdf/model.h>
#include <controller_manager/controller_manager.h>

// LWR sim class
#include "kuka_lwr_hw/lwr_hw_gazebo.h"

#ifndef DEBUG
	#define DEBUG 0
#endif

namespace lwr_plugin
{
	class LWRPlugin : public gazebo::ModelPlugin
	{
	 public:
		LWRPlugin() : gazebo::ModelPlugin() 
		{
			ROS_INFO("Constructor of LWRPlugin");
		}
		
  		virtual ~LWRPlugin()
  		{
			ROS_INFO("Destructor of LWRPlugin");
    			// Disconnect from gazebo events
    			gazebo::event::Events::DisconnectWorldUpdateBegin(update_connection_);
  		}
  		
		// Overloaded Gazebo entry point
	  	virtual void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf)
	  	{
			ROS_INFO("Load of LWRPlugin");
			
			// Save pointers to the model
			parent_model_ = parent;
			sdf_ = sdf;
			
			// Error message if the model couldn't be found
			if (!parent_model_)
			{
				ROS_ERROR_STREAM_NAMED("loadThread","parent model is NULL");
				return;
			}

			// Check that ROS has been initialized
			if(!ros::isInitialized())
			{
				ROS_FATAL_STREAM_NAMED("LWRPlugin","A ROS node for Gazebo has not been initialized, unable to load plugin. "
				<< "Load the Gazebo system plugin 'liblwr_gazebo_plugin.so' in the gazebo_ros package)");
				return;
			}
			
			// Get namespace for nodehandle
			if(sdf_->HasElement("robotNamespace"))
			{
			  robot_namespace_ = sdf_->GetElement("robotNamespace")->Get<std::string>();
			}
			else
			{
			  robot_namespace_ = parent_model_->GetName(); // default
			}
			
			ROS_INFO("LWRPlugin -> robot_namespace_ = %s",robot_namespace_.c_str());
			
			// Here the variable [robot_namespace_] contains the string 'kuka_lwr'. No element 'robotNamespace' defined in the xacro file robot model !
			
			#if (DEBUG)
				// For debug ***************************************************
				ROS_INFO("robot_namespace_ = %s",robot_namespace_.c_str());
				// *************************************************************
			#endif
			
			// Get robot_description ROS param name
			if (sdf_->HasElement("robotParam"))
			{
			  robot_description_ = sdf_->GetElement("robotParam")->Get<std::string>();
			}
			else
			{
			  robot_description_ = "robot_description"; // default
			}
			
			// Here the variable [robot_description_] contains the string 'robot_description'. No element 'robotParam' defined in the xacro file robot model !
			
			#if (DEBUG)
				// For debug ***************************************************
				ROS_INFO("robot_description_ = %s",robot_description_.c_str());
				// *************************************************************
			#endif
			
			// Get the Gazebo simulation period
			ros::Duration gazebo_period(parent_model_->GetWorld()->GetPhysicsEngine()->GetMaxStepSize());
			
			#if (DEBUG)
				// For debug ***************************************************
				ROS_INFO("gazebo_period = %f seconds",gazebo_period.toSec());
				// *************************************************************
			#endif
			
			// Decide the plugin control period
			if(sdf_->HasElement("controlPeriod"))
			{
				control_period_ = ros::Duration(sdf_->Get<double>("controlPeriod"));

				// Check the period against the simulation period
				if( control_period_ < gazebo_period )
				{
					ROS_ERROR_STREAM_NAMED("LWRPlugin","Desired controller update period ("<<control_period_
					<<" s) is faster than the gazebo simulation period ("<<gazebo_period<<" s).");
				}
				else if( control_period_ > gazebo_period )
				{
					ROS_WARN_STREAM_NAMED("LWRPlugin","Desired controller update period ("<<control_period_
					<<" s) is slower than the gazebo simulation period ("<<gazebo_period<<" s).");
				}
			}
			else
			{
				control_period_ = gazebo_period;
				ROS_DEBUG_STREAM_NAMED("LWRPlugin","Control period not found in URDF/SDF, defaulting to Gazebo period of "
				<< control_period_);
			}
			
			#if (DEBUG)
				// For debug ***************************************************
				ROS_INFO("control_period_ = %f seconds",control_period_.toSec());
				// *************************************************************
			#endif

			// Get parameters/settings for controllers from ROS param server
			// Create a Node with the namespace defined in the variable [robot_namespace_]. Here it is 'kuka_lwr'.
			model_nh_ = ros::NodeHandle(robot_namespace_);
			ROS_INFO_NAMED("LWRPlugin", "Starting LWRPlugin plugin in namespace: %s", robot_namespace_.c_str());

			// Read urdf from ros parameter server then
			// setup actuators and mechanism control node.
			// This call will block if ROS is not properly initialized.
			const std::string urdf_string = getURDF(robot_description_);
			
			#if (DEBUG)
				// For debug ***************************************************
				ROS_INFO("\n****************************\n");
				//ROS_INFO("urdf_string = %s",urdf_string.c_str());
				ROS_INFO("\n****************************\n");
				// *************************************************************
			#endif
			
			// Load the LWRHWsim abstraction to interface the controllers with the gazebo model
			ROS_INFO("\n****************************\n");
			ROS_INFO("Create Robot Sim ! namespace = %s", robot_namespace_.c_str());
			ROS_INFO("\n****************************\n");
			
			robot_hw_sim_.reset( new lwr_hw::LWRHWGazebo() );
			ROS_INFO("Robot Sim after new !");
			robot_hw_sim_->create(robot_namespace_, urdf_string);
			ROS_INFO("Robot Sim after create !");
			robot_hw_sim_->setParentModel(parent_model_);
			if(!robot_hw_sim_->init())
			{
				ROS_FATAL_NAMED("lwr_hw","Could not initialize robot simulation interface");
				return;
			}
			
			ROS_INFO("\n****************************\n");
			ROS_INFO("Robot Sim created and initialized !");
			ROS_INFO("\n****************************\n");
			
			// Create the controller manager
			ROS_INFO_STREAM_NAMED("ros_control_plugin","Loading controller_manager");
			controller_manager_.reset(
							new controller_manager::ControllerManager(robot_hw_sim_.get(), model_nh_)
									);
			
			
			// Listen to the update event. This event is broadcast every simulation iteration.
			update_connection_ =
			gazebo::event::Events::ConnectWorldUpdateBegin
			(boost::bind(&LWRPlugin::Update, this));
			
			ROS_INFO_NAMED("KukaLWRPlugin", "Loaded KukaLWRPlugin.");
		}
		
		// Called by the world update start event
  		void Update()
  		{
			// Get the simulation time and period
			gazebo::common::Time gz_time_now = parent_model_->GetWorld()->GetSimTime();
			
			// convert gazebo time to ros time
			ros::Time sim_time_ros(gz_time_now.sec, gz_time_now.nsec);
			
			// calculate the time elapsed between 2 Update. 
			ros::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;

			// Check if we should update the controllers
			if(sim_period >= control_period_) 
			{
				// Store this simulation time
				last_update_sim_time_ros_ = sim_time_ros;

				// Update the robot simulation with the state of the gazebo model
				robot_hw_sim_->read(sim_time_ros, sim_period);

				// Compute the controller commands
				controller_manager_->update(sim_time_ros, sim_period);
			}

			// Update the gazebo model with the result of the controller computation
			robot_hw_sim_->write(sim_time_ros, sim_time_ros - last_write_sim_time_ros_);
			
			last_write_sim_time_ros_ = sim_time_ros;
		}
		
		// Called on world reset
  		virtual void Reset()
  		{
			// Reset timing variables to not pass negative update periods to controllers on world reset
			last_update_sim_time_ros_ = ros::Time();
			last_write_sim_time_ros_ = ros::Time();
			
			ROS_INFO("Reset of KukaLWRPlugin");
		}
		
	 private:
	 
		// Get the URDF XML from the parameter server. Here the param_name is 'robot_description' defined in the xacro model file of the robot.
		std::string getURDF(std::string param_name) const
		{
			std::string urdf_string;

			// search and wait for robot_description on param server
			while (urdf_string.empty())
			{
				std::string search_param_name;
				
				if (model_nh_.searchParam(param_name, search_param_name)) // bool ros::NodeHandle::searchParam(const std::string& key,std::string& result) 	
				{
					ROS_INFO_ONCE_NAMED("LWRPlugin", "KukaLWRPlugin plugin is waiting for model"
					  " URDF in parameter [%s] on the ROS param server.", search_param_name.c_str());

					model_nh_.getParam(search_param_name, urdf_string); // bool ros::NodeHandle::getParam(const std::string& key,std::string& s) 	
				}
				else
				{
					ROS_INFO_ONCE_NAMED("LWRPlugin", "KukaLWRPlugin plugin is waiting for model"
					  " URDF in parameter [%s] on the ROS param server.", robot_description_.c_str());

					model_nh_.getParam(param_name, urdf_string);
				}

				usleep(100000);
			}
			ROS_DEBUG_STREAM_NAMED("LWRPlugin", "Recieved urdf from param server, parsing...");

			return urdf_string;
		}
		
		// Pointer to the update event connection
  		gazebo::event::ConnectionPtr update_connection_;
  		
  		// Pointer to the model
		gazebo::physics::ModelPtr parent_model_;
		sdf::ElementPtr sdf_;
		
		// Node Handles
		ros::NodeHandle model_nh_;
		
		// Controller manager
		boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
		
		// Robot simulator interface
		boost::shared_ptr<lwr_hw::LWRHWGazebo> robot_hw_sim_;
		
		// Strings
		std::string robot_namespace_;
		std::string robot_description_;
		
		// Timing
		ros::Duration control_period_;
		ros::Time last_update_sim_time_ros_;
		ros::Time last_write_sim_time_ros_;
		
	};
	
	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(LWRPlugin);
	
	
}
