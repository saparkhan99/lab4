#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <math.h>

// Main moveit libraries are included
int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(0);
    spinner.start(); // For moveit implementation we need AsyncSpinner, we cant use ros::spinOnce()

    static const std::string PLANNING_GROUP = "group1_controller"; /* Now we specify with what group we want work, here group1 is the name of my group controller*/
    moveit::planning_interface::MoveGroupInterface

    move_group(PLANNING_GROUP); // loading move_group
    const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP); //For joint control

    geometry_msgs::PoseStamped current_pose;
    geometry_msgs::PoseStamped target_pose; // Pose in ROS is implemented using geometry_msgs::PoseStamped, google what is the type of this msg
    current_pose = move_group.getCurrentPose(); /* Retrieving the information about the current position and orientation of the end effector*/
    target_pose = current_pose;
    ros::Rate loop_rate(50); // Loop frequency


    float radius = 0.4; // circle radius 
    float pi = 3.1415;
    float center_x = current_pose.pose.position.x-radius;
    // center of y is zero, so we do not need it

    
    while (ros::ok())
    {
        for (int i=0; i<37; i++)
        {
            // move by 10 degrees in a circle in each itersation 
            target_pose.pose.position.y = radius*sin(i*10*pi/180); // calculate next y
            target_pose.pose.position.x = center_x+radius*cos(i*10*pi/180); // calculate next x
            move_group.setApproximateJointValueTarget(target_pose); // To calculate the trajectory
            move_group.setMaxVelocityScalingFactor(1.0); // fo the simalation to be faster.
            move_group.move(); // Move the robot
            
        }

        current_pose = move_group.getCurrentPose();
        if (abs(current_pose.pose.position.x - target_pose.pose.position.x) < 0.01  and abs(current_pose.pose.position.y- target_pose.pose.position.y)){
            break; // Basically, check if we reached the desired position
        }
        loop_rate.sleep();
    }

    ROS_INFO("%f", current_pose.pose.position.x);
    ROS_INFO("%f", current_pose.pose.position.y);
    ROS_INFO("%f", current_pose.pose.position.z);

    ROS_INFO("Done");
    ros::shutdown();
    return 0;
}
