#
#  Laurent LEQUIEVRE
#  Institut Pascal UMR6602
#  laurent.lequievre@univ-bpclermont.fr
# 
#

cmake_minimum_required(VERSION 2.8.3)
project(kuka_lwr_controllers)

# We need to then specify which other CMake packages that need to be found to build our project.
# message_generation # need for message generation
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  control_toolbox
  urdf
  kdl_parser
  geometry_msgs
  message_generation
  cmake_modules
  kuka_lwr_hw
  #trac_ik_lib
  tf
  tf_conversions
  realtime_tools
  
  roscpp
  rospy
  std_msgs
)

# After find_package(catkin REQUIRED COMPONENTS....) :
# -> the variable ${catkin_INCLUDE_DIRS} contains the include paths.
# -> the variable ${catkin_LIBRARY_DIRS} contains the path of the libraries.
# -> the variable ${catkin_LIBRARIES} contains the libraries.
# not only for catkin but also for all the components required.

set(FRIL_DEFS -D__LINUX__)
add_definitions(${FRIL_DEFS} -std=c++11) #The -std=c++11 tells the compiler to consider C++11

# Add the given directories to those the compiler uses to search for include files.
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${CATKIN_DEVEL_PREFIX}/include ${CATKIN_DEVEL_PREFIX}/include/fril ${CATKIN_DEVEL_PREFIX}/include/RML  ${CATKIN_DEVEL_PREFIX}/include/reflexxes)

# Specify the paths in which the linker should search for libraries.
link_directories(${catkin_LIBRARY_DIRS} ${CATKIN_DEVEL_PREFIX}/lib/x86_64-linux-gnu  ${CATKIN_DEVEL_PREFIX}/lib/reflexxes/release/shared)

# Add messages files defined in the 'msg' folder.
# il faut ajouter ceci dans le fichier package.xml :
# <build_depend>message_generation</build_depend>
# <run_depend>message_runtime</run_depend>
add_message_files(
   FILES
   PoseRPY.msg
   RPY.msg
   MultiPriorityTask.msg
   TrajPathPoint.msg
   StiffnessDamping.msg
   
#  Nouradine's Message file added on 15-6-2018
   PathPointCaracteristics.msg  
)

# Add service files
add_service_files(
  FILES
 GetJointVelocity.srv
)

# Generate source files from your msg definition.
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# catkin_package() is a catkin-provided CMake macro. 
# This is required to specify catkin-specific information to the build system which in turn is used to generate pkg-config and CMake files. 
# CATKIN_DEPENDS  define the catkin packages that this project depend.
# INCLUDE_DIRS include # The exported include paths for the package.
# LIBRARIES ${PROJECT_NAME} # The exported libraries from the project.
catkin_package(
  CATKIN_DEPENDS 
    controller_interface
    control_toolbox
    urdf
    kdl_parser
    message_runtime
    std_msgs
    geometry_msgs
    kuka_lwr_hw
    realtime_tools
    #trac_ik_lib
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} TypeIRML ReflexxesTypeII
)

add_library(${PROJECT_NAME}_pid
  src/pid/pid.cpp
)



# The add_library() CMake function is used to specify libraries to build. By default catkin builds shared libraries.
add_library(${PROJECT_NAME}
  src/group_command_controller_fri.cpp
  src/gravity_compensation_controller.cpp
  src/simple_cartesian_impedance_controller.cpp
  src/one_task_inverse_kinematics.cpp
  src/cartesian_velocity_control.cpp
  src/one_task_inverse_kinematics_fri.cpp
  src/torque_based_position.cpp
  src/torque_based_position_gazebo.cpp
  src/computed_torque_controller.cpp
  src/cartesian_computed_torque_controller.cpp
  src/basic_torque_controller.cpp
  src/operational_space_controller.cpp

# Harsh Controllers added on 5-7-2018
  src/harsh_cartesian_impedance_controller.cpp
  src/harsh_admittance_controller.cpp

# Harsh Controllers added on 10-9-2018
  src/harsh_trust_impedance_controller.cpp

# Harsh Controller added on 19-9-2018  
  src/harsh_admittance_controller_fri.cpp

# Harsh Controller added on 20-12-2018
  src/harsh_admittance_controller_ros.cpp  
  
# Nouradine's controller added on 15-6-2018
#  src/hybrid_position_force.cpp
)

# Link a target to given libraries.
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} TypeIRML ReflexxesTypeII ${PROJECT_NAME}_pid)

 
# Add a dependencies of generation message, to be sure that all headers files of all messages are generated before !
add_dependencies(kuka_lwr_controllers ${PROJECT_NAME}_generate_messages_cpp  FRILibrary reflexxes)


add_executable(node_test_irml  src/node_test_irml.cpp)

target_link_libraries(node_test_irml
  ${catkin_LIBRARIES}
  TypeIRML ReflexxesTypeII
)

add_dependencies(node_test_irml reflexxes)

add_executable(admittance_node  src/admittance_node.cpp)
add_executable(impedance_node  src/impedance_node.cpp)

add_executable(admittance_node_global_frame  src/admittance_node_global_frame.cpp)

add_executable(admittance_node_fri  src/admittance_node_fri.cpp)

target_link_libraries(admittance_node
  ${catkin_LIBRARIES}
  TypeIRML ReflexxesTypeII
)

target_link_libraries(impedance_node
  ${catkin_LIBRARIES}
  TypeIRML ReflexxesTypeII
)


target_link_libraries(admittance_node_global_frame
  ${catkin_LIBRARIES}
  TypeIRML ReflexxesTypeII
)


target_link_libraries(admittance_node_fri
  ${catkin_LIBRARIES}
  TypeIRML ReflexxesTypeII
)


#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#install(TARGETS ${PROJECT_NAME}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#  )

#install(FILES lwr_controllers_plugins.xml
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
