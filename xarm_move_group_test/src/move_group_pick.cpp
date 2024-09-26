// Derived from move group interface sample of moveit2 tutorials

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/trajectory_constraints.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <math.h>

#include <json.hpp>

#define TO_RAD(x) ((x)/180.0*M_PI)

static const rclcpp::Logger LOGGER = rclcpp::get_logger("xarm_move_group_test");

using json = nlohmann::json;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("xarm_move_group_test", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, "xarm");
  
  // Add object to the planning scene that represents the platform where 
  // picked object will be placed.
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object.id = "platform";

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.150;
  primitive.dimensions[primitive.BOX_Y] = 0.280;
  primitive.dimensions[primitive.BOX_Z] = 0.100;

  // Define a pose for the box (specified relative to frame_id).
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x =  0.160;
  box_pose.position.y =  0.0;
  box_pose.position.z =  0.190/2;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  planning_scene_interface.addCollisionObjects(collision_objects);
  
  // Get Basic Information
  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
  
  geometry_msgs::msg::PoseStamped pose = move_group.getCurrentPose();
  RCLCPP_INFO(LOGGER, "Starting pose: x,y,z = %f, %f, %f, orientation = %f, %f, %f, %f",
      pose.pose.position.x,
      pose.pose.position.y,
      pose.pose.position.z,
      pose.pose.orientation.x,
      pose.pose.orientation.y,
      pose.pose.orientation.z,
      pose.pose.orientation.w);

  move_group.setMaxVelocityScalingFactor(1.0);
  move_group.setMaxAccelerationScalingFactor(0.10);
  move_group.setNumPlanningAttempts(10);
  move_group.setPlanningTime(5);
  move_group.setGoalTolerance(0.010);

  move_group.setJointValueTarget(move_group.getNamedTargetValues("home"));
  move_group.move();

  tf2::Quaternion q;
  geometry_msgs::msg::Pose target;

  // Move to above pick position
  q.setRPY(TO_RAD(180.0), TO_RAD(0.0), TO_RAD(0.0));
  target.orientation = tf2::toMsg(q);

  target.position.x = -0.0;
  target.position.y = -0.157;
  target.position.z =  0.085;

  RCLCPP_INFO(LOGGER, "Move to above object");
  move_group.setPoseTarget(target);
  move_group.move();

  // Open gripper
  RCLCPP_INFO(LOGGER, "Open gripper");
  moveit::planning_interface::MoveGroupInterface move_group_eff(move_group_node, "arm_end_effector");
  move_group_eff.setJointValueTarget(move_group_eff.getNamedTargetValues("open"));
  move_group_eff.move();
  
  // Move downward so that gripper is around object
  RCLCPP_INFO(LOGGER, "Move down to object");
  geometry_msgs::msg::Pose target_grab = target;
  target_grab.position.z = target.position.z - 0.14;
  move_group.setPoseTarget(target_grab);
  move_group.move();

  // Close gripper
  RCLCPP_INFO(LOGGER, "Close gripper");
  move_group_eff.setJointValueTarget(move_group_eff.getNamedTargetValues("closed"));
  move_group_eff.move();

  // Move to above drop point
  q.setRPY(TO_RAD(90.0), TO_RAD(0.0), TO_RAD(90.0));
  target.orientation = tf2::toMsg(q);

  target.position.x =  0.010;
  target.position.y =  0.000;
  target.position.z =  0.260;

  move_group.setPoseTarget(target);
  move_group.move();

  // Move to drop point
  RCLCPP_INFO(LOGGER, "Move to drop location");

  target.position.x =  0.030;
  target.position.y =  0.000;
  target.position.z =  0.210;

  move_group.setPoseTarget(target);
  move_group.move();

  // Open gripper
  RCLCPP_INFO(LOGGER, "Open gripper");
  move_group_eff.setJointValueTarget(move_group_eff.getNamedTargetValues("open"));
  move_group_eff.move();

  // Move to home  
  RCLCPP_INFO(LOGGER, "Move to home location");
  move_group.setJointValueTarget(move_group.getNamedTargetValues("home"));
  move_group.move();

  return 0;
}
