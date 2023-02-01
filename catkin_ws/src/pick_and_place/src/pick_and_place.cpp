#include "pick_and_place/pick_and_place.hpp"

PickAndPlace::PickAndPlace(ros::NodeHandle &nh): nh_(nh) {
    // Load topic/service names from parameter server
    if (!loadParams()) {
       ROS_ERROR("Could not load pick_and_place services");
       ros::requestShutdown();
    }
    move_arm_.setPlanningTime(planning_time_);
    block_pose_client_ = nh_.serviceClient<gazebo_msgs::GetModelState>(robot_pose_service_);
    gripper_command_pub_ = nh_.advertise<std_msgs::Float64>(gripper_command_topic_, 1);
    pick_ = false;
}

bool PickAndPlace::loadParams() {
    if (!nh_.getParam("robot_pose_service", robot_pose_service_)) {
        ROS_INFO("robot_pose_service not loaded");
        return false;
    } 
    if (!nh_.getParam("gripper_command_topic", gripper_command_topic_)) {
        ROS_INFO("gripper_command_topic not loaded");
        return false;
    } 
    if (!nh_.getParam("robot_name", robot_name_)) {
        ROS_INFO("robot_name not loaded");
        return false;
    } 
    if (!nh_.getParam("block_name", block_name_)) {
        ROS_INFO("block_name not loaded");
        return false;
    }
    if (!nh_.param("number_blocks", number_blocks_, 5)) {
        ROS_INFO("number_blocks not loaded");
        return false;
    }
    if (!nh_.param("planning_time", planning_time_, 10.)) {
        ROS_INFO("planning_time not loaded");
        return false;
    }
    if (!nh_.param("column_length", column_length_, 0.7)) {
        ROS_INFO("column_length not loaded");
        return false;
    }
    if (!nh_.param("pre_grip_z", pre_grip_z_, 0.25)) {
        ROS_INFO("pre_grip_z not loaded");
        return false;
    }
    if (!nh_.param("grip_z", grip_z_, 0.05)) {
        ROS_INFO("grip_z not loaded");
        return false;
    }
    return true;
}

void PickAndPlace::initManipulator() {
  getBlockPoses();
  addCollisionObjects();
  moveHome();
  actuateGripper();
  // get arm ortientation
  auto arm_pose = move_arm_.getCurrentPose().pose;
  tf2::fromMsg(arm_pose.orientation, arm_q_);
}

void PickAndPlace::moveHome() {
  move_arm_.setNamedTarget("home");
  move_arm_.move();
}

void PickAndPlace::getBlockPoses() {
  block_poses_.resize(number_blocks_);
  gazebo_msgs::GetModelState srv;
  srv.request.relative_entity_name = robot_name_;
  for (int i=0; i<number_blocks_; ++i) {
    srv.request.model_name = block_name_ + "_" + std::to_string(i+1);
    while(!block_pose_client_.call(srv)) {
      ROS_INFO_STREAM_THROTTLE(1.0, "Waiting for the pose of block " << i+1);
      ros::WallDuration(0.5).sleep();
    };
    block_poses_[i] = std::move(srv.response.pose);
  }
}

void PickAndPlace::pickNPlace() {
  initManipulator();  
  for (auto block_pose : block_poses_) {
    moveArm(block_pose); // pick
    ros::WallDuration(1.0).sleep();
    moveArm(block_pose); // place
  }
  moveHome();
  actuateGripper();
}

void PickAndPlace::moveArm(geometry_msgs::Pose block_pose) {

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  auto pose_target = block_pose;
  pose_target.position.z += pre_grip_z_;
  pose_target.orientation = tf2::toMsg(arm_q_);

  // if placing, place block on the other table (opposite y-dimension to pick location)
  if (!pick_) pose_target.position.y *= -1;

  // move to pre-pick/place pose
  move_arm_.setPoseTarget(pose_target);
  if(move_arm_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS){
    move_arm_.execute(my_plan);
  } else {
    ROS_WARN("No pre-pick/place trajectory was found");
  }

  pose_target.position.z -= grip_z_;

  // move to pick/place pose
  move_arm_.setPoseTarget(pose_target);
  if(move_arm_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS){
    move_arm_.execute(my_plan);
  } else {
    ROS_WARN("No pick/place trajectory was found");
  }
  
  actuateGripper();
  pose_target.position.z += grip_z_;

  // move to post-pick/place pose
  move_arm_.setPoseTarget(pose_target);
  if(move_arm_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS){
    move_arm_.execute(my_plan);
  } else {
      ROS_WARN("No post-pick/place trajectory was found");
  }
}

void PickAndPlace::actuateGripper() {
  auto gripper_command_msg = std_msgs::Float64();
  if (pick_) {
    gripper_command_msg.data = -0.45; // close gripper 
    pick_ = false;
  }
  else {
    gripper_command_msg.data = 0.2; // open gripper
    pick_ = true;
  }
  ros::Time start = ros::Time::now();
  ros::Duration duration = ros::Duration(1); 
  ros::Time endTime = start + duration;
  while(ros::Time::now() < endTime ) {
    gripper_command_pub_.publish(gripper_command_msg);
  }
}

void PickAndPlace::addCollisionObjects() {
  // 8 objects total: column, 2 tables and 5 blocks
  collision_objects_.resize(8);

  // column holding the base of the arm
  collision_objects_[0].id = "column";
  collision_objects_[0].header.frame_id = "base_link_inertia";

  collision_objects_[0].primitives.resize(1);
  // in simple_scene.world, column is input as a box collision 
  collision_objects_[0].primitives[0].type = collision_objects_[0].primitives[0].BOX;
  collision_objects_[0].primitives[0].dimensions.resize(3);
  // column geometry in radius, length
  collision_objects_[0].primitives[0].dimensions[0] = 0.042; // x
  collision_objects_[0].primitives[0].dimensions[1] = 0.042; // y
  collision_objects_[0].primitives[0].dimensions[2] = 0.74; // z
  //column pose
  collision_objects_[0].primitive_poses.resize(1);
  collision_objects_[0].primitive_poses[0].position.x = 0.;
  collision_objects_[0].primitive_poses[0].position.y = 0.;
  collision_objects_[0].primitive_poses[0].position.z = 0.37 - column_length_;
  collision_objects_[0].operation = collision_objects_[0].ADD;

  // first table surface
  collision_objects_[1].id = "cafe_table_1";
  collision_objects_[1].header.frame_id = "base_link_inertia";
  collision_objects_[1].primitives.resize(1);
  collision_objects_[1].primitives[0].type = collision_objects_[1].primitives[0].BOX;
  collision_objects_[1].primitives[0].dimensions.resize(3);
  // table geometry
  collision_objects_[1].primitives[0].dimensions[0] = 0.913; // x 
  collision_objects_[1].primitives[0].dimensions[1] = 0.913; // y
  collision_objects_[1].primitives[0].dimensions[2] = 0.04; // z
  // table pose
  collision_objects_[1].primitive_poses.resize(1);
  collision_objects_[1].primitive_poses[0].position.x = 0.;
  collision_objects_[1].primitive_poses[0].position.y = -0.6;
  collision_objects_[1].primitive_poses[0].position.z = 0.755 - column_length_;
  collision_objects_[1].operation = collision_objects_[1].ADD;

  // second table surface, same geometry as table 1, same pose except for y-dimension
  collision_objects_[2].id = "cafe_table_2";
  collision_objects_[2].header.frame_id = "base_link_inertia";
  collision_objects_[2].primitives.resize(1);
  collision_objects_[2].primitives[0].type = collision_objects_[1].primitives[0].BOX;
  collision_objects_[2].primitives[0].dimensions.resize(3);
  // table geometry
  collision_objects_[2].primitives[0].dimensions[0] = 0.913; // x 
  collision_objects_[2].primitives[0].dimensions[1] = 0.913; // y
  collision_objects_[2].primitives[0].dimensions[2] = 0.04; // z
  // table pose
  collision_objects_[2].primitive_poses.resize(1);
  collision_objects_[2].primitive_poses[0].position.x = 0.;
  collision_objects_[2].primitive_poses[0].position.y = 0.6;
  collision_objects_[2].primitive_poses[0].position.z = 0.755 - column_length_;
  collision_objects_[2].operation = collision_objects_[2].ADD;

  // add blocks as collision objects
  for (int i=0; i < block_poses_.size(); i++) {
    collision_objects_[i+3].id = block_name_ + "_" + std::to_string(i+1);
    collision_objects_[i+3].header.frame_id = "base_link_inertia";
    collision_objects_[i+3].primitives.resize(1);
    collision_objects_[i+3].primitives[0].type = collision_objects_[i+3].primitives[0].BOX;
    collision_objects_[i+3].primitives[0].dimensions.resize(3);
    // block geometry is the same for all blocks
    collision_objects_[i+3].primitives[0].dimensions[0] = 0.1; // x 
    collision_objects_[i+3].primitives[0].dimensions[1] = 0.03; // y
    collision_objects_[i+3].primitives[0].dimensions[2] = 0.03 ; // z
    // block pose
    collision_objects_[i+3].primitive_poses.resize(1);
    collision_objects_[i+3].primitive_poses[0].position.x = block_poses_[i].position.x;
    collision_objects_[i+3].primitive_poses[0].position.y = block_poses_[i].position.y;
    collision_objects_[i+3].primitive_poses[0].position.z = block_poses_[i].position.z - column_length_;
    collision_objects_[i+3].operation = collision_objects_[i+3].ADD;
  }
  planning_scene_interface_.applyCollisionObjects(collision_objects_);
}