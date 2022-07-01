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
    static const std::string rgt_arm_group = "rgt_arm";
    static const std::string rgt_hand_group = "rgt_hand";

    static const std::string lft_arm_group = "lft_arm";
    static const std::string lft_hand_group = "lft_hand";

    // Declar MoveGroupInterface for each arm and hand
    moveit::planning_interface::MoveGroupInterface rgt_arm_move_group_interface(rgt_arm_group);
    moveit::planning_interface::MoveGroupInterface rgt_hand_move_group_interface(rgt_hand_group);

    moveit::planning_interface::MoveGroupInterface lft_arm_move_group_interface(lft_arm_group);
    moveit::planning_interface::MoveGroupInterface lft_hand_move_group_interface(lft_hand_group);

    // Set the goals to be the pre-defined/named "ready" pose
    rgt_arm_move_group_interface.setNamedTarget("ready");
    lft_arm_move_group_interface.setNamedTarget("ready");

    // Declar Plan objects for each arm
    moveit::planning_interface::MoveGroupInterface::Plan rgt_arm_plan;
    moveit::planning_interface::MoveGroupInterface::Plan lft_arm_plan;

    // Do the planning with the specific target poses
    bool rgt_success = (rgt_arm_move_group_interface.plan(rgt_arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    bool lft_success = (lft_arm_move_group_interface.plan(lft_arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // If the planning is successful, execute the arms motions (move the robot) and open the gripper
    if (rgt_success)
    {
        rgt_arm_move_group_interface.execute(rgt_arm_plan);

        rgt_hand_move_group_interface.setNamedTarget("open");
        rgt_hand_move_group_interface.move();
    }

    if (lft_success)
    {
        lft_arm_move_group_interface.execute(lft_arm_plan);

        lft_hand_move_group_interface.setNamedTarget("open");
        lft_hand_move_group_interface.move();
    }

    // Plan arbitrary poses for the arms
    // Move the right arm 0.10 meters up with respect to its current pose
    geometry_msgs::PoseStamped current_rgt_arm_pose = rgt_arm_move_group_interface.getCurrentPose();
    geometry_msgs::PoseStamped target_rgt_arm_pose = current_rgt_arm_pose;

    target_rgt_arm_pose.pose.position.z += 0.10;

    rgt_arm_move_group_interface.setPoseTarget(target_rgt_arm_pose);
    rgt_success = (rgt_arm_move_group_interface.plan(rgt_arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (rgt_success)
    {
        rgt_arm_move_group_interface.execute(rgt_arm_plan);
    }

    // Move the left arm 0.10 meters front with respect to its current pose
    geometry_msgs::PoseStamped current_lft_arm_pose = lft_arm_move_group_interface.getCurrentPose();
    geometry_msgs::PoseStamped target_lft_arm_pose = current_lft_arm_pose;

    target_lft_arm_pose.pose.position.x += 0.10;
    lft_arm_move_group_interface.setPoseTarget(target_lft_arm_pose);

    lft_success = (lft_arm_move_group_interface.plan(lft_arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (lft_success)
    {
        lft_arm_move_group_interface.execute(lft_arm_plan);
    }

    ros::shutdown();

    return 0;
}