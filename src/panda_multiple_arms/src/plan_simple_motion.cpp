/* 
   Author: Mohamed Raessa (mohamed@avatarin, mohamed.s.raessa@gmail.com
   Desc: Planning simple motions for multiple arms and hands using MoveIt move group interface
*/

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "plan_simple_motion");
    ros::NodeHandle node_handle;

    // ROS spinner is required for the MoveGroupInterface to get the robot state
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Set the arms and hands planning groups names
    static const std::string right_arm_group = "right_arm";
    static const std::string right_hand_group = "right_hand";

    static const std::string left_arm_group = "left_arm";
    static const std::string left_hand_group = "left_hand";

    // Declar MoveGroupInterface for each arm and hand
    moveit::planning_interface::MoveGroupInterface right_arm_move_group_interface(right_arm_group);
    moveit::planning_interface::MoveGroupInterface right_hand_move_group_interface(right_hand_group);

    moveit::planning_interface::MoveGroupInterface left_arm_move_group_interface(left_arm_group);
    moveit::planning_interface::MoveGroupInterface left_hand_move_group_interface(left_hand_group);

    // Set the goals to be the pre-defined/named "ready" pose
    right_arm_move_group_interface.setNamedTarget("ready");
    left_arm_move_group_interface.setNamedTarget("ready");

    // Declar Plan objects for each arm
    moveit::planning_interface::MoveGroupInterface::Plan right_arm_plan;
    moveit::planning_interface::MoveGroupInterface::Plan left_arm_plan;

    // Do the planning with the specific target poses
    bool rgt_success = (right_arm_move_group_interface.plan(right_arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    bool lft_success = (left_arm_move_group_interface.plan(left_arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // If the planning is successful, execute the arms motions (move the robot) and open the gripper
    if (rgt_success)
    {
        right_arm_move_group_interface.execute(right_arm_plan);

        right_hand_move_group_interface.setNamedTarget("open");
        right_hand_move_group_interface.move();
    }

    if (lft_success)
    {
        left_arm_move_group_interface.execute(left_arm_plan);

        left_hand_move_group_interface.setNamedTarget("open");
        left_hand_move_group_interface.move();
    }

    // Plan arbitrary poses for the arms
    // Move the right arm 0.10 meters up with respect to its current pose
    geometry_msgs::PoseStamped current_right_arm_pose = right_arm_move_group_interface.getCurrentPose();
    geometry_msgs::PoseStamped target_right_arm_pose = current_right_arm_pose;

    target_right_arm_pose.pose.position.z += 0.10;

    right_arm_move_group_interface.setPoseTarget(target_right_arm_pose);
    rgt_success = (right_arm_move_group_interface.plan(right_arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (rgt_success)
    {
        right_arm_move_group_interface.execute(right_arm_plan);
    }

    // Move the left arm 0.10 meters front with respect to its current pose
    geometry_msgs::PoseStamped current_left_arm_pose = left_arm_move_group_interface.getCurrentPose();
    geometry_msgs::PoseStamped target_left_arm_pose = current_left_arm_pose;

    target_left_arm_pose.pose.position.x += 0.10;
    left_arm_move_group_interface.setPoseTarget(target_left_arm_pose);

    lft_success = (left_arm_move_group_interface.plan(left_arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (lft_success)
    {
        left_arm_move_group_interface.execute(left_arm_plan);
    }

    ros::shutdown();

    return 0;
}