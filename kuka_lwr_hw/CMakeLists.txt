cmake_minimum_required(VERSION 2.8.3)
project(kuka_lwr_hw)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  control_toolbox
  controller_interface
  controller_manager
  hardware_interface
  realtime_tools
  joint_limits_interface
  roscpp
  tf
  urdf
  cmake_modules
  pluginlib
  kdl_parser
  transmission_interface
  gazebo_ros
  std_msgs
)

find_package(gazebo REQUIRED)

add_definitions (-fpermissive -std=c++11)

catkin_package(
  CATKIN_DEPENDS
    control_toolbox
    controller_interface
    controller_manager
    hardware_interface
    realtime_tools
    joint_limits_interface
    roscpp
    tf
    urdf
    cmake_modules
    pluginlib
    kdl_parser
    transmission_interface
    gazebo_ros
    std_msgs
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########
MESSAGE( STATUS "--> GAZEBO_INCLUDE_DIRS : " ${GAZEBO_INCLUDE_DIRS} )
MESSAGE( STATUS "--> GAZEBO_LIBRARY_DIRS : " ${GAZEBO_LIBRARY_DIRS} )
MESSAGE( STATUS "--> GAZEBO_LIBRARIES : " ${GAZEBO_LIBRARIES} )

include_directories(include
  ${GAZEBO_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

# LWR HW GAZEBO BASE CLASS 
add_library(${PROJECT_NAME}_gazebo
 src/lwr_hw.cpp
 src/lwr_hw_gazebo.cpp
)

## LWR HW BASE CLASS TO RULE'EM ALL
add_library(${PROJECT_NAME}
  #include/${PROJECT_NAME}/lwr_hw.h
  #include/${PROJECT_NAME}/lwr_hw_fril.hpp
  #include/${PROJECT_NAME}/lwr_kuka_interface.h
  src/lwr_hw.cpp
)

link_directories(${GAZEBO_LIBRARY_DIRS})
#list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_gazebo
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)


include(ExternalProject)

# Reflexxes type 2 library ----------------------------------------------------------------

message(WARNING "ATTENTION: This project supports the fork of the reflexxes type 2 motion library in https://github.com/lequievre/reflexxes_type2, and install the libraries in your catkin workspace, not in the system")

set(RML_HEADER_DIR "${CMAKE_BINARY_DIR}/${PROJECT_NAME}/reflexxes-prefix/src/reflexxes/include/")
set(RML_RELEASE_SHARED_LIBRARY_DIR "${CMAKE_BINARY_DIR}/${PROJECT_NAME}/reflexxes-prefix/src/reflexxes/Linux/x64/release/lib/shared/")
set(RML_DEBUG_SHARED_LIBRARY_DIR "${CMAKE_BINARY_DIR}/${PROJECT_NAME}/reflexxes-prefix/src/reflexxes/Linux/x64/debug/lib/shared/")
set(RML_RELEASE_STATIC_LIBRARY_DIR "${CMAKE_BINARY_DIR}/${PROJECT_NAME}/reflexxes-prefix/src/reflexxes/Linux/x64/release/lib/static/")
set(RML_DEBUG_STATIC_LIBRARY_DIR "${CMAKE_BINARY_DIR}/${PROJECT_NAME}/reflexxes-prefix/src/reflexxes/Linux/x64/debug/lib/static/")

message(STATUS "RML_HEADER_DIR = ${RML_HEADER_DIR}")
message(STATUS "RML_RELEASE_LIBRARY_DIR = ${RML_RELEASE_LIBRARY_DIR}")
message(STATUS "RML_DEBUG_LIBRARY_DIR = ${RML_DEBUG_LIBRARY_DIR}")
message(STATUS "copy dir = ${CATKIN_DEVEL_PREFIX}/include/reflexxes")


ExternalProject_Add(
  reflexxes
  URL "https://github.com/lequievre/reflexxes_type2/blob/master/ReflexxesTypeII.zip?raw=true"
  DOWNLOAD_NAME ReflexxesTypeII.zip
  CONFIGURE_COMMAND ""
  BINARY_DIR reflexxes-prefix/src/reflexxes/Linux
  BUILD_COMMAND make clean64 all64
  #INSTALL_COMMAND 
#	${CMAKE_COMMAND} -E copy_directory ${RML_HEADER_DIR} ${CATKIN_DEVEL_PREFIX}/include/reflexxes 
#	COMMAND 
#  	${CMAKE_COMMAND} -E copy ${RML_RELEASE_SHARED_LIBRARY_DIR}/libReflexxesTypeII.so ${CATKIN_DEVEL_PREFIX}/lib/reflexxes/release/shared/libReflexxesTypeII.so
# 	COMMAND
#	${CMAKE_COMMAND} -E copy ${RML_RELEASE_STATIC_LIBRARY_DIR}/libReflexxesTypeII.a ${CATKIN_DEVEL_PREFIX}/lib/reflexxes/release/static/libReflexxesTypeII.a
# 	COMMAND
#	${CMAKE_COMMAND} -E copy ${RML_DEBUG_SHARED_LIBRARY_DIR}/libReflexxesTypeII.so ${CATKIN_DEVEL_PREFIX}/lib/reflexxes/debug/shared/libReflexxesTypeII.so
#	COMMAND
#	${CMAKE_COMMAND} -E copy ${RML_DEBUG_STATIC_LIBRARY_DIR}/libReflexxesTypeII.a ${CATKIN_DEVEL_PREFIX}/lib/reflexxes/debug/static/libReflexxesTypeII.a
  )
  
ExternalProject_Add_Step(
  reflexxes CopyToCatkinDevelDirectory
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${RML_HEADER_DIR} ${CATKIN_DEVEL_PREFIX}/include/reflexxes 
  COMMAND ${CMAKE_COMMAND} -E copy ${RML_RELEASE_SHARED_LIBRARY_DIR}/libReflexxesTypeII.so ${CATKIN_DEVEL_PREFIX}/lib/reflexxes/release/shared/libReflexxesTypeII.so
  COMMAND ${CMAKE_COMMAND} -E copy ${RML_RELEASE_STATIC_LIBRARY_DIR}/libReflexxesTypeII.a ${CATKIN_DEVEL_PREFIX}/lib/reflexxes/release/static/libReflexxesTypeII.a
  COMMAND ${CMAKE_COMMAND} -E copy ${RML_DEBUG_SHARED_LIBRARY_DIR}/libReflexxesTypeII.so ${CATKIN_DEVEL_PREFIX}/lib/reflexxes/debug/shared/libReflexxesTypeII.so
  COMMAND ${CMAKE_COMMAND} -E copy ${RML_DEBUG_STATIC_LIBRARY_DIR}/libReflexxesTypeII.a ${CATKIN_DEVEL_PREFIX}/lib/reflexxes/debug/static/libReflexxesTypeII.a
  DEPENDEES build install
)  

#INSTALL (
#    DIRECTORY ${RML_HEADER_DIR}
#    DESTINATION ${CATKIN_DEVEL_PREFIX}/include/reflexxes
#    FILES_MATCHING PATTERN "*.h")

# -----------------------------------------------------------------------------------------

# Standford FRILibrary support in PROGRESS ------------------------------------------------

message(WARNING "ATTENTION: This project supports the fork of the Standford library found in https://github.com/lequievre/FRI_IP, and install the libraries in your catkin workspace, not in the system")

ExternalProject_Add(FRILibrary
	GIT_REPOSITORY https://github.com/lequievre/FRI_IP.git
	GIT_TAG master
	BUILD_COMMAND make
	BUILD_IN_SOURCE 1
	CONFIGURE_COMMAND cmake -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}
	INSTALL_COMMAND make install
)

set(FRIL_DEFS -D__LINUX__)
add_definitions(${FRIL_DEFS})

# ------------------------------------------------------------------------------------------

# lwr hw fril node
include_directories(${CATKIN_DEVEL_PREFIX}/include/fril ${CATKIN_DEVEL_PREFIX}/include/RML)
link_directories(${CATKIN_DEVEL_PREFIX}/lib/x86_64-linux-gnu)
add_executable(lwr_hw_fril_node  src/lwr_hw_fril_node.cpp
  include/${PROJECT_NAME}/lwr_hw_fril.hpp
)

add_dependencies(lwr_hw_fril_node
  FRILibrary
  ${PROJECT_NAME}
)

target_link_libraries(lwr_hw_fril_node
  FastResearchInterfaceLibrary
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#target_link_libraries(lwr_hw_fril_node
#  FastResearchInterfaceLibrary_rt.a
#  ${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#)

 #set_target_properties( lwr_hw_fril_node PROPERTIES
 #                    COMPILE_FLAGS "-DRTNET -D_GNU_SOURCE -D_REENTRANT -D__XENO__" 
 #                    LINK_FLAGS "-Wl,@/usr/xenomai_2_6_5/lib/posix.wrappers")

# test fril move node
#add_executable(test_move_fril_node  src/test_move_fril_node.cpp)
#target_link_libraries(test_move_fril_node
#  FastResearchInterfaceLibrary_rt
#  TypeIRML
#  ${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#)

#set_target_properties( test_move_fril_node PROPERTIES
#                     COMPILE_FLAGS "-DRTNET -D_GNU_SOURCE -D_REENTRANT -D__XENO__" 
#                     LINK_FLAGS "-Wl,@/usr/xenomai_2_6_5/lib/posix.wrappers")


