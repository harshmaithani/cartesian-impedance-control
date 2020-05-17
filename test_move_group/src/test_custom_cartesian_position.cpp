#include <moveit/move_group_interface/move_group.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_custom_node");
	
	// start a ROS spinning thread
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();
	
	std::string my_test_group_name;
	if (node_handle.getParam("/test_group_name", my_test_group_name))
	{
	  ROS_INFO("Got param value of test_group_name : %s", my_test_group_name.c_str());
	}
	else
	{
	  ROS_ERROR("Failed to get param 'my_test_group_name'");
	}
	
	
	// setup the group you would like to control and plan for.
	moveit::planning_interface::MoveGroup group(my_test_group_name);
	
	// retrieve current position and orientation
	geometry_msgs::PoseStamped robot_pose;
	robot_pose = group.getCurrentPose();
	geometry_msgs::Pose current_position;
	current_position = robot_pose.pose;
	geometry_msgs::Point exact_pose = current_position.position;
	geometry_msgs::Quaternion exact_orientation = current_position.orientation;
	ROS_INFO("Current position : x=%f, y=%f, z=%f", exact_pose.x, exact_pose.y, exact_pose.z);
	ROS_INFO("Current orientation : x=%f, y=%f, z=%f, w=%f", exact_orientation.x, exact_orientation.y, exact_orientation.z,
	exact_orientation.w);
	// We can print the name of the reference frame for this robot.
	ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
	// We can also print the name of the end-effector link for this group.
	ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());
	
	// Planning to a Pose goal
	// =======================
	// We can plan a motion for this group to a desired pose for the
	// end-effector.
	geometry_msgs::Pose target_goal;
	target_goal.orientation.w = 1.0;
	target_goal.orientation.x = 0.0;
	target_goal.orientation.y = 0.0;
	target_goal.orientation.z = 0.0;
	target_goal.position.x = 0.3;
	target_goal.position.y = 0.3;
	target_goal.position.z = 1.0;
	group.setPoseTarget(target_goal);
	
	// call the planner to compute the plan and visualize it.
	moveit::planning_interface::MoveGroup::Plan my_plan;
	bool success = group.plan(my_plan);
	ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
	// then move the group to the sampled target
	group.move();
	// wait for shutdown
	ros::waitForShutdown();
	return 0;
}
