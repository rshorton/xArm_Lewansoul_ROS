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

#include <tf2/utils.h>
#include <math.h>

#include <json.hpp>

// Define this to step thru the poses using the RvizVisualToolsGui 'Next' button
#undef MANUAL_MODE

#define TO_RAD(x) ((x)/180.0*M_PI)

static const rclcpp::Logger LOGGER = rclcpp::get_logger("xarm_move_group_test");

static const std::string POSE_FILE_NAME = "test_poses.json";
// Magic 'x' values that can be used in the test poses to loop or prematurely stop processing the list
static const int LOOP_MAGIC_VALUE = 99999.0;
static const int FINISH_MAGIC_VALUE = 99998.0;

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

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, "xarm");
  
  // Getting Basic Information
  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());
  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));
  
  {
	  geometry_msgs::msg::PoseStamped pose = move_group.getCurrentPose();
	  RCLCPP_INFO(LOGGER, "Starting pose: x,y,z = %f, %f, %f, orientation = %f, %f, %f, %f",
			  pose.pose.position.x,
			  pose.pose.position.y,
			  pose.pose.position.z,
			  pose.pose.orientation.x,
			  pose.pose.orientation.y,
			  pose.pose.orientation.z,
			  pose.pose.orientation.w);
  }

  move_group.setMaxVelocityScalingFactor(1.0);
  move_group.setMaxAccelerationScalingFactor(0.10);
  move_group.setNumPlanningAttempts(10);
  move_group.setPlanningTime(5);
  move_group.setGoalTolerance(0.010);

  move_group.setJointValueTarget(move_group.getNamedTargetValues("home"));
  move_group.move();

  // Move to above pick position

  tf2::Quaternion q;
  geometry_msgs::msg::Pose target;

  q.setRPY(TO_RAD(180.0), TO_RAD(0.0), TO_RAD(0.0));
  target.orientation = tf2::toMsg(q);

  target.position.x = -0.0;
  target.position.y = -0.157;
  target.position.z =  0.085;

  move_group.setPoseTarget(target);
  move_group.move();

  // Open gripper
  moveit::planning_interface::MoveGroupInterface move_group_eff(move_group_node, "arm_end_effector");
  move_group_eff.setJointValueTarget(move_group_eff.getNamedTargetValues("open"));
  move_group_eff.move();
  
  // Move downward so that gripper is around object
  geometry_msgs::msg::Pose target_grab = target;
  target_grab.position.z = target.position.z - 0.14;
  move_group.setPoseTarget(target_grab);
  move_group.move();

  // Close gripper
  move_group_eff.setJointValueTarget(move_group_eff.getNamedTargetValues("closed"));
  move_group_eff.move();

  // Move to above drop point
  target.position.x = -0.084;
  target.position.y = -0.090;
  target.position.z =  0.150;

  move_group.setPoseTarget(target);
  move_group.move();

  // Move to drop point
  target.position.x = -0.084;
  target.position.y = -0.069;
  target.position.z =  0.118;

  move_group.setPoseTarget(target);
  move_group.move();

  // Open gripper
  move_group_eff.setJointValueTarget(move_group_eff.getNamedTargetValues("open"));
  move_group_eff.move();

  // Move to home  
  move_group.setJointValueTarget(move_group.getNamedTargetValues("home"));
  move_group.move();

  return 0;
}
