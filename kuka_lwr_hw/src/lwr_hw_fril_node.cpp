// SYS
#include <sys/mman.h>
#include <cmath>
#include <time.h>
#include <signal.h>
#include <stdexcept>

// ROS headers
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

// the lwr hw fri interface
#include "kuka_lwr_hw/lwr_hw_fril.hpp"

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

  // search and wait for robot_description on param server
  while (urdf_string.empty())
  {
    std::string search_param_name;
    if (model_nh_.searchParam(param_name, search_param_name))
    {
      ROS_INFO_ONCE_NAMED("LWRHWFRIL", "LWRHWFRIL node is waiting for model"
        " URDF in parameter [%s] on the ROS param server.", search_param_name.c_str());

      model_nh_.getParam(search_param_name, urdf_string);
    }
    else
    {
      ROS_INFO_ONCE_NAMED("LWRHWFRIL", "LWRHWFRIL node is waiting for model"
        " URDF in parameter [%s] on the ROS param server.", robot_description.c_str());

      model_nh_.getParam(param_name, urdf_string);
    }

    usleep(100000);
  }
  ROS_DEBUG_STREAM_NAMED("LWRHWFRIL", "Recieved urdf from param server, parsing...");

  return urdf_string;
}

int main( int argc, char** argv )
{
  // initialize ROS
  ros::init(argc, argv, "lwr_hw_interface", ros::init_options::NoSigintHandler);

  // ros spinner
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // custom signal handlers
  signal(SIGTERM, quitRequested);
  signal(SIGINT, quitRequested);
  signal(SIGHUP, quitRequested);

  // create a node
  ros::NodeHandle lwr_nh;
  std::string robot_namespace = lwr_nh.getNamespace();
  
  ROS_INFO("kuka_lwr_hw_rt_node -> **** node handle namespace = '%s'",robot_namespace.c_str());

  // Remove the first two characters.
  std::string robot_namespace_param;
  int nb_char_to_remove = 2;
  robot_namespace_param = robot_namespace.substr(nb_char_to_remove,robot_namespace.size()-nb_char_to_remove);

  ROS_INFO("kuka_lwr_hw_rt_node -> **** node handle robot_namespace_param = '%s'",robot_namespace_param.c_str());


  // If robot_namespace is "kuka_lwr_left" the Robot Hardware need to open the fri driver file contains in ros param : fri_driver_file_left
  // If robot_namespace is "kuka_lwr_right" the Robot Hardware need to open the fri driver file contains in ros param : fri_driver_file_right
 std::string file;
 if (robot_namespace.compare("//kuka_lwr_left") == 0)
	lwr_nh.param("/fri_driver_file_left", file, std::string(""));
 else
	lwr_nh.param("/fri_driver_file_right", file, std::string(""));

  ROS_INFO("fril node -> driver file = %s, namespace = %s",file.c_str(),robot_namespace_param.c_str());

  // Get the the general robot description URDF XML from the parameter server
  // Be careful ! the 'robot_description' parameter is set inside a 'group' (a namespace) in the launch file.
  // so don't forget to concatenate the namespace !
  std::string urdf_string = getURDF(lwr_nh, robot_namespace + "/robot_description");

  // construct and start the real lwr
  lwr_hw::LWRHWFRIL lwr_robot;
  lwr_robot.create(robot_namespace_param, urdf_string);
  lwr_robot.setInitFile(file);
  if(!lwr_robot.init())
  {
    ROS_FATAL_NAMED("lwr_hw","Could not initialize robot real interface");
    return -1;
  }

  // timer variables
  struct timespec ts = {0, 0};
  ros::Time last(ts.tv_sec, ts.tv_nsec), now(ts.tv_sec, ts.tv_nsec);
  ros::Duration period(1.0);

  ROS_INFO("Before controller_manager::ControllerManager !");

  //the controller manager
  controller_manager::ControllerManager manager(&lwr_robot, lwr_nh);

  // run as fast as possible
  while( !g_quit ) 
  {
    // get the time / period
    if (!clock_gettime(CLOCK_REALTIME, &ts)) 
    {
      now.sec = ts.tv_sec;
      now.nsec = ts.tv_nsec;
      period = now - last;
      last = now;
    } 
    else 
    {
      ROS_FATAL("Failed to poll realtime clock!");
      break;
    } 

    // read the state from the lwr
    lwr_robot.read(now, period);

    // update the controllers
    manager.update(now, period);

    // write the command to the lwr
    lwr_robot.write(now, period);
  }

  std::cerr<<"Stopping spinner..."<<std::endl;
  spinner.stop();

  std::cerr<<"Stopping LWR..."<<std::endl;
  lwr_robot.stop();

  std::cerr<<"This node was killed!"<<std::endl;

  return 0;
}
