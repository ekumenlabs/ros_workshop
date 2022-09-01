#include <control_robot/proportional_controller.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>

#include <cmath>

namespace control_robot {

geometry_msgs::Twist ProportionalController::computeVelocityTowardsGoal(
    const geometry_msgs::Pose2D& current_pose,
    const geometry_msgs::Pose2D& goal_pose) {
  geometry_msgs::Twist velocity = getZeroVelocity();
  if (isAtGoalPosition(current_pose, goal_pose)) {
    return velocity;
  }

  velocity.linear.x = computeLinearSpeedToGoal(current_pose, goal_pose);
  velocity.angular.z = computeAngularSpeedToLookAtGoal(current_pose, goal_pose);
  return velocity;
}

geometry_msgs::Twist ProportionalController::getZeroVelocity() {
  geometry_msgs::Twist velocity;
  velocity.linear.x = 0.0;
  velocity.linear.y = 0.0;
  velocity.linear.z = 0.0;
  velocity.angular.x = 0.0;
  velocity.angular.y = 0.0;
  velocity.angular.z = 0.0;
  return velocity;
}

bool ProportionalController::isAtGoalPosition(
    const geometry_msgs::Pose2D& current_pose,
    const geometry_msgs::Pose2D& goal_pose) {
  return computeEuclideanDistanceToGoal(current_pose, goal_pose) <
         kEuclideanDistanceToGoalTolerance;
}

float ProportionalController::computeLinearSpeedToGoal(
    const geometry_msgs::Pose2D& current_pose,
    const geometry_msgs::Pose2D& goal_pose) {
  return kLinearSpeedConstant *
         computeEuclideanDistanceToGoal(current_pose, goal_pose);
}

float ProportionalController::computeEuclideanDistanceToGoal(
    const geometry_msgs::Pose2D& current_pose,
    const geometry_msgs::Pose2D& goal_pose) {
  const float delta_x = goal_pose.x - current_pose.x;
  const float delta_y = goal_pose.y - current_pose.y;
  return std::sqrt(delta_x * delta_x + delta_y * delta_y);
}

float ProportionalController::computeAngularSpeedToLookAtGoal(
    const geometry_msgs::Pose2D& current_pose,
    const geometry_msgs::Pose2D& goal_pose) {
  return kAngularSpeedConstant *
         computeAngularDistanceToLookAtGoal(current_pose, goal_pose);
}

float ProportionalController::computeAngularDistanceToLookAtGoal(
    const geometry_msgs::Pose2D& current_pose,
    const geometry_msgs::Pose2D& goal_pose) {
  const float steering_angle =
      std::atan2(goal_pose.y - current_pose.y, goal_pose.x - current_pose.x);
  return steering_angle - current_pose.theta;
}

}  // namespace control_robot
