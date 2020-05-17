/*
 *  Laurent LEQUIEVRE
 *  Institut Pascal UMR6602
 *  laurent.lequievre@univ-bpclermont.fr
 * 
*/

// C++ headers
#include <iostream>

// ROS headers
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include <signal.h>

#include <unistd.h>

#include "pan_tilt_hw.h"

#define TRACE_NODE_ACTIVATED 0

bool g_quit = false;

void quitRequested(int sig)
{
  g_quit = true;
}

// Get the URDF XML from the parameter server
std::string getURDF(ros::NodeHandle &model_nh_, std::string param_name)
{
  std::string urdf_string;
  std::string robot_description = "/robot_description";
  
  #if TRACE_NODE_ACTIVATED
	ROS_INFO("pan_tilt_hw_node -> getURDF, param_name='%s'",param_name.c_str());
  #endif

  // search and wait for robot_description on param server
  while (urdf_string.empty())
  {
    std::string search_param_name;
    if (model_nh_.searchParam(param_name, search_param_name))
    {
	  #if TRACE_NODE_ACTIVATED
		ROS_INFO("pan_tilt_hw_node -> getURDF -> found param_name = '%s'", search_param_name.c_str());
	  #endif

      model_nh_.getParam(search_param_name, urdf_string);
      
      #if TRACE_NODE_ACTIVATED
		ROS_INFO("pan_tilt_hw_node -> getURDF -> urdf_string=%s",urdf_string.c_str());
	  #endif
    }
    else
    {
      ROS_INFO_ONCE_NAMED("pan_tilt_hw_node -> getURDF -> kuka_lwr_hw_rt_node", "kuka_lwr_hw_rt_node node is waiting for model"
        " URDF in parameter [%s] on the ROS param server.", robot_description.c_str());

      model_nh_.getParam(param_name, urdf_string);
    }

    usleep(100000);
  }
  #if TRACE_NODE_ACTIVATED
	ROS_INFO("pan_tilt_hw_node -> GET URDF_STRING size=%d", (int)urdf_string.size());
  #endif

  return urdf_string;
}

int main( int argc, char** argv )
{
	// initialize ROS
	ros::init(argc, argv, "pan_tilt_hw_node", ros::init_options::NoSigintHandler);

	// ros spinner
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// custom signal handlers
	signal(SIGTERM, quitRequested);
	signal(SIGINT, quitRequested);
	signal(SIGHUP, quitRequested);

	// create a node
	ros::NodeHandle pan_tilt_nh;
	std::string robot_namespace = pan_tilt_nh.getNamespace();
	
	#if TRACE_NODE_ACTIVATED
		ROS_INFO("pan_tilt_hw_node -> **** node handle namespace = '%s'",robot_namespace.c_str());
	#endif

	// get the general robot description, the lwr class will take care of parsing what's useful to itself
	// ATTENTION Ici si dans le launch (exemple platform.launch) le param 'robot_description' se trouve dans un <group>, c'est a dire un namespace, il ne
	// faut pas oublier de concatener celui-ci a la chaine de caractere "/robot_description" !!!.
	std::string urdf_string = getURDF(pan_tilt_nh, robot_namespace + "/robot_description");
	
	// construct and start the real pan_tilt robot
	
	pan_tilt_hw::PanTiltHW pan_tilt_robot;
	
	pan_tilt_robot.create(urdf_string, pan_tilt_nh);
	
	if (!pan_tilt_robot.init())
	{
		ROS_FATAL_NAMED("pan_tilt_hw::PanTiltHW","pan_tilt_hw_node -> Could not initialize robot real interface");
		return -1;
	}
	
	#if TRACE_NODE_ACTIVATED
		ROS_INFO("pan_tilt_hw_node -> pan_tilt_hw::PanTiltHW Initialized !");
	#endif
	
	// timer variables
	struct timespec ts = {0, 0};
	ros::Time last(ts.tv_sec, ts.tv_nsec), now(ts.tv_sec, ts.tv_nsec), last_control(ts.tv_sec, ts.tv_nsec);
	ros::Duration period(1.0);
	ros::Duration period_control(1.0);

	//the controller manager
	controller_manager::ControllerManager manager(&pan_tilt_robot, pan_tilt_nh);
	
	// run as fast as possible
	while( !g_quit ) 
	{
		//usleep(300);
		
		// get the time / period
		if (!clock_gettime(CLOCK_MONOTONIC, &ts)) 
		{
		  now.sec = ts.tv_sec;
		  now.nsec = ts.tv_nsec;
		  period = now - last;
		  period_control = now - last_control;
		  last = now;
		} 
		else 
		{
		  ROS_FATAL("pan_tilt_hw_node -> Failed to poll realtime clock!");
		  break;
		} 
		
		// read the state from the lwr robot
		pan_tilt_robot.read(now, period);
		
		last_control = now;
		
		#if TRACE_NODE_ACTIVATED
			ROS_INFO("pan_tilt_hw_node-> Controller period_control.toNSec()= %ld",period_control.toNSec());
		#endif

		// update the controllers
		manager.update(now, period);
		
		// write the command to the lwr
		pan_tilt_robot.write(now, period);
		
	}

	#if TRACE_NODE_ACTIVATED
		ROS_INFO("Stopping spinner...");
	#endif
	
	spinner.stop();
	#if TRACE_NODE_ACTIVATED
		ROS_INFO("This node was killed!");
	#endif

	return 0;
}


