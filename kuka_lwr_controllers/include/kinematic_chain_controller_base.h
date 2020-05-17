/*
 *  Laurent LEQUIEVRE
 *  Institut Pascal UMR6602
 *  laurent.lequievre@univ-bpclermont.fr
 * 
*/

#ifndef KINEMATIC_CHAIN_CONTROLLER_BASE_H
#define KINEMATIC_CHAIN_CONTROLLER_BASE_H

// STL
#include <vector>

// ROS
#include <controller_interface/controller.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <urdf/model.h>

// KDL
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarrayacc.hpp>
#include <kdl_parser/kdl_parser.hpp>

#define TRACE_ACTIVATED 0

namespace controller_interface
{
	template<typename T>
	class KinematicChainControllerBase: public Controller<T>
	{
		public:
			KinematicChainControllerBase() {}
			~KinematicChainControllerBase() {}

			bool init(T *robot, ros::NodeHandle &n);

		protected:
			ros::NodeHandle nh_;

			KDL::Chain kdl_chain_;	// KDL chain construct from URDF
			KDL::Vector gravity_; 	// KDL Vector for gravity
			
			KDL::JntArrayAcc joint_msr_states_, joint_des_states_;  // joint states (measured and desired)
			KDL::JntArray torque_msr_states_, torque_des_states_;  // torque states (measured and desired)

			struct limits_
			{
				KDL::JntArray min;
				KDL::JntArray max;
				KDL::JntArray center;
			} joint_limits_; // KDL structures to store limits min, max, center of each joint

			std::vector<typename T::ResourceHandleType> joint_handles_;
	};

	template <typename T>
	bool KinematicChainControllerBase<T>::init(T *robot, ros::NodeHandle &n)
	{
		#if TRACE_ACTIVATED
			ROS_INFO("KinematicChainControllerBase -> Start KinematicChainControllerBase<T>::init !");
		#endif
		
		nh_ = n;
		
		#if TRACE_ACTIVATED
			ROS_INFO("KinematicChainControllerBase -> KinematicChainControllerBase<T>::init ros::NodeHandle namespace = %s", n.getNamespace().c_str());
		#endif

		// get URDF and name of root and tip from the parameter server
		std::string robot_description, root_name, tip_name;

		if (!ros::param::search(n.getNamespace(),"robot_description", robot_description))
		{
		    ROS_ERROR_STREAM("KinematicChainControllerBase: No robot description (URDF) found on parameter server (" << n.getNamespace() << "/robot_description)");
		    return false;
		}

		#if TRACE_ACTIVATED
			ROS_INFO("KinematicChainControllerBase -> KinematicChainControllerBase<T>::init -> robot_description = %s", robot_description.c_str());
		#endif

		if (!nh_.getParam("root_name", root_name))
		{
		    ROS_ERROR_STREAM("KinematicChainControllerBase: No root name found on parameter server (" << n.getNamespace() << "/root_name)");
		    return false;
		}

		#if TRACE_ACTIVATED
			ROS_INFO("KinematicChainControllerBase -> KinematicChainControllerBase<T>::init -> root_name = %s", root_name.c_str());
		#endif

		if (!nh_.getParam("tip_name", tip_name))
		{
		    ROS_ERROR_STREAM("KinematicChainControllerBase: No tip name found on parameter server (" << n.getNamespace() << "/tip_name)");
		    return false;
		}

		#if TRACE_ACTIVATED
			ROS_INFO("KinematicChainControllerBase -> KinematicChainControllerBase<T>::init -> tip_name = %s", tip_name.c_str());
		#endif

		// Get the gravity vector (direction and magnitude)
		gravity_ = KDL::Vector::Zero();
		gravity_(2) = -9.81;

		// Construct an URDF model from the xml string
		std::string xml_string;

		if (n.hasParam(robot_description))
		    n.getParam(robot_description.c_str(), xml_string);
		else
		{
		    ROS_ERROR("KinematicChainControllerBase -> (init) Parameter %s not set, shutting down node...",robot_description.c_str());
		    n.shutdown();
		    return false;
		}

		if (xml_string.size() == 0)
		{
		    ROS_ERROR("KinematicChainControllerBase -> (init) Unable to load robot model from parameter %s",robot_description.c_str());
		    n.shutdown();
		    return false;
		}
		
		// Get urdf model out of robot_description
		urdf::Model model;
		if (!model.initString(xml_string))
		{
		    ROS_ERROR("KinematicChainControllerBase -> (init) Failed to parse urdf file");
		    n.shutdown();
		    return false;
		}
		
		#if TRACE_ACTIVATED
			ROS_INFO("KinematicChainControllerBase -> (init) Successfully parsed urdf file");
		#endif
		
		KDL::Tree kdl_tree_;
		// Parse URDF to get the KDL tree
		if (!kdl_parser::treeFromUrdfModel(model, kdl_tree_))
		{
		    ROS_ERROR("KinematicChainControllerBase -> (init) Failed to construct kdl tree");
		    n.shutdown();
		    return false;
		}

		// Populate the KDL chain
		if(!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
		{
		    ROS_ERROR_STREAM("KinematicChainControllerBase -> (init) Failed to get KDL chain from tree: ");
		    ROS_ERROR_STREAM("  " << root_name << " --> " << tip_name);
		    ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfJoints() << " joints");
		    ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfSegments() << " segments");
		    ROS_ERROR_STREAM("  The segments are:");

		    KDL::SegmentMap segment_map = kdl_tree_.getSegments();
		    KDL::SegmentMap::iterator it;

		    for( it=segment_map.begin(); it != segment_map.end(); it++ )
		      ROS_ERROR_STREAM( "    " << (*it).first);

		    return false;
		}

		#if TRACE_ACTIVATED
			ROS_INFO("KinematicChainControllerBase -> (init) Number of segments: %d", kdl_chain_.getNrOfSegments());
			ROS_INFO("KinematicChainControllerBase -> (init) Number of joints in chain: %d", kdl_chain_.getNrOfJoints());
		#endif
		
		// Parsing joint limits from urdf model along kdl chain
		boost::shared_ptr<const urdf::Link> link_ = model.getLink(tip_name);	// A Link defined in a URDF structure
		boost::shared_ptr<const urdf::Joint> joint_;  // A Joint defined in a URDF structure
		
		// Resize joints limits arrays with the "KDL chain" number of joints
		joint_limits_.min.resize(kdl_chain_.getNrOfJoints());
		joint_limits_.max.resize(kdl_chain_.getNrOfJoints());
		joint_limits_.center.resize(kdl_chain_.getNrOfJoints());

		int index;
		
		for (int i = 0; i < kdl_chain_.getNrOfJoints() && link_; i++)
		{
		    joint_ = model.getJoint(link_->parent_joint->name);
 
			#if TRACE_ACTIVATED
				ROS_INFO("KinematicChainControllerBase -> (init) Getting limits for joint: %s", joint_->name.c_str());
			#endif

		    index = kdl_chain_.getNrOfJoints() - i - 1;

		    joint_limits_.min(index) = joint_->limits->lower;
		    joint_limits_.max(index) = joint_->limits->upper;
		    joint_limits_.center(index) = (joint_limits_.min(index) + joint_limits_.max(index))/2;

		    link_ = model.getLink(link_->getParent()->name);
		}

		// Get joint handles for all of the joints in the chain
		for (std::vector<KDL::Segment>::const_iterator it = kdl_chain_.segments.begin(); it != kdl_chain_.segments.end(); ++it)
		{
		    joint_handles_.push_back(robot->getHandle(it->getJoint().getName()));
		    
		    #if TRACE_ACTIVATED
				ROS_INFO("KinematicChainControllerBase -> (init) : %s", it->getJoint().getName().c_str() );
			#endif
		}

		#if TRACE_ACTIVATED
			ROS_INFO("KinematicChainControllerBase -> (init) Number of joints in handle = %lu", joint_handles_.size() );
		#endif

		joint_msr_states_.resize(kdl_chain_.getNrOfJoints());
		joint_des_states_.resize(kdl_chain_.getNrOfJoints());
		
		#if TRACE_ACTIVATED
			ROS_INFO("KinematicChainControllerBase -> (init) Finish KinematicChainControllerBase<T>::init");
		#endif

		return true;
	}
}

#endif
