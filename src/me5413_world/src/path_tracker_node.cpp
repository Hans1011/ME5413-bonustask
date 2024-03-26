/** path_tracker_node.cpp
 * 
 * Copyright (C) 2024 Shuo SUN & Advanced Robotics Center, National University of Singapore
 * 
 * MIT License
 * 
 * ROS Node for robot to track a given path
 */

#include "angles/angles.h"
#include "me5413_world/math_utils.hpp"
#include "me5413_world/path_tracker_node.hpp"
#include <cmath> 
namespace me5413_world 
{

// Dynamic Parameters
double SPEED_TARGET;
double PID_Kp, PID_Ki, PID_Kd;
double ahead_distance; // Pure pursuit look ahead distance
bool PARAMS_UPDATED;

// A dynamic parameter callback function to set dynamic parameters upon changes
void dynamicParamCallback(const me5413_world::path_trackerConfig& config, uint32_t level)
{
  // Common Params
  SPEED_TARGET = config.speed_target;
  // PID 
  PID_Kp = config.PID_Kp;
  PID_Ki = config.PID_Ki;
  PID_Kd = config.PID_Kd;
  // Load ahead distance
  ahead_distance = 1.5;
  PARAMS_UPDATED = true;
}

// Class constructor
PathTrackerNode::PathTrackerNode() : tf2_listener_(tf2_buffer_)
{
  f = boost::bind(&dynamicParamCallback, _1, _2);
  server.setCallback(f);

  this->sub_robot_odom_ = nh_.subscribe("/gazebo/ground_truth/state", 1, &PathTrackerNode::robotOdomCallback, this);
  this->sub_local_path_ = nh_.subscribe("/me5413_world/planning/local_path", 1, &PathTrackerNode::localPathCallback, this);
  this->pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/jackal_velocity_controller/cmd_vel", 1);

  // Initialization
  this->robot_frame_ = "base_link";
  this->world_frame_ = "world";

  this->pid_ = control::PID(0.1, 1.0, -1.0, PID_Kp, PID_Ki, PID_Kd);
}


// Callback function for local path subscription
void PathTrackerNode::localPathCallback(const nav_msgs::Path::ConstPtr& path)
{
  this->pub_cmd_vel_.publish(computeControlOutputs(this->odom_world_robot_, path));
  return;
}

// Callback function for robot odometry subscription
void PathTrackerNode::robotOdomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
  this->world_frame_ = odom->header.frame_id;
  this->robot_frame_ = odom->child_frame_id;
  this->odom_world_robot_ = *odom.get();

  return;
}

geometry_msgs::Point PathTrackerNode::calculateTargetPoint(const tf2::Vector3& currentPosition, const nav_msgs::Path::ConstPtr& navigationPath, double lookaheadDist)  {
  auto lookaheadPoint = std::find_if(navigationPath->poses.begin(), navigationPath->poses.end(), [&](const geometry_msgs::PoseStamped& poseStamped) {
    tf2::Vector3 pointPos;
    tf2::fromMsg(poseStamped.pose.position, pointPos);

    // Compute the distance from the current position to the point on the path
    double distToCurrentPos = sqrt(pow(pointPos.x() - currentPosition.x(), 2) +
                                   pow(pointPos.y() - currentPosition.y(), 2) +
                                   pow(pointPos.z() - currentPosition.z(), 2));

    return distToCurrentPos >= lookaheadDist;
  });

  if (lookaheadPoint != navigationPath->poses.end()) {
    return lookaheadPoint->pose.position;
  } else {
    // Use the final point if no suitable lookahead point is found
    return navigationPath->poses.back().pose.position;
  }
}


// Function to calculate the control output
geometry_msgs::Twist PathTrackerNode::computeControlOutputs(const nav_msgs::Odometry& odom_robot, const nav_msgs::Path::ConstPtr& path)
{
  // Extract robot's orientation and position
  tf2::Quaternion q_robot;
  tf2::fromMsg(odom_robot.pose.pose.orientation, q_robot);
  const tf2::Matrix3x3 m_robot = tf2::Matrix3x3(q_robot);
  double roll, pitch, yaw_robot;
  m_robot.getRPY(roll, pitch, yaw_robot);
  tf2::Vector3 point_robot, point_goal;
  tf2::fromMsg(odom_robot.pose.pose.position, point_robot);

  // Change velocity based on PID controller
  tf2::Vector3 robot_vel;
  tf2::fromMsg(this->odom_world_robot_.twist.twist.linear, robot_vel);
  const double velocity = robot_vel.length();
  geometry_msgs::Twist cmd_vel;
  if (PARAMS_UPDATED)
  {
    this->pid_.updateSettings(PID_Kp, PID_Ki, PID_Kd);
    PARAMS_UPDATED = false;
  }
  // Controller output linear speed control
  cmd_vel.linear.x = this->pid_.calculate(SPEED_TARGET, velocity);

  // Find the goal point and calculate the error
  geometry_msgs::Point goal_point = calculateTargetPoint(point_robot, path, ahead_distance);
  double yaw_goal = atan2(goal_point.y - point_robot.y(), goal_point.x - point_robot.x());
  double yaw_error = angles::normalize_angle(yaw_goal - yaw_robot); // normalize it to [-pi, pi)

  // Controller output angle control
  cmd_vel.angular.z = 1.9 * yaw_error;    

  return cmd_vel;
}

} // namespace me5413_world

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_tracker_node");
  me5413_world::PathTrackerNode path_tracker_node;
  ros::spin();  // spin the ros node.
  return 0;
}
