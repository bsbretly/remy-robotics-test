#ifndef PICK_AND_PLACE_H
#define PICK_AND_PLACE_H

// ROS
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <gazebo_msgs/GetModelState.h> // message to get block state
#include <std_msgs/Float64.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/CollisionObject.h>

class PickAndPlace {
public:
    PickAndPlace(ros::NodeHandle &nh);
    void getBlockPoses();
    void actuateGripper();
    void addCollisionObjects();
    bool loadParams();
    void initManipulator();
    void pickNPlace();
    void moveArm(geometry_msgs::Pose block_pose);
    void moveHome();
    
private:
    //** planning_scene _interface: planning scene of a robot, including collision environment
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_ {}; 
    //** move_group_interface: provides an API for controlling the robot's movement, and the 
    //** planning_scene message, which is used to communicate the current state of the planning
    //** scene to other nodes 
    moveit::planning_interface::MoveGroupInterface move_arm_{"ur5e_arm"};
    
    ros::NodeHandle nh_;
    // vector of block pose (position and orientation)
    std::vector<geometry_msgs::Pose> block_poses_ {};
    std::vector<moveit_msgs::CollisionObject> collision_objects_{};
    
    ros::ServiceClient block_pose_client_;
    std::string robot_pose_service_, gripper_command_topic_, robot_name_, block_name_;
    ros::Publisher gripper_command_pub_;  
    tf2::Quaternion arm_q_{}; // arm orientation as as qauternion
    int number_blocks_;
    double planning_time_, column_length_, pre_grip_z_, grip_z_;
    bool pick_; // true if picking, false if placing
};

#endif //PICK_AND_PLACE_H