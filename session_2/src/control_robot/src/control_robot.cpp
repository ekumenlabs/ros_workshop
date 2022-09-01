#include <control_robot/proportional_controller.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/tf.h>

#include <cmath>

/**
 * Controla un robot a lazo cerrado utilizando su odometría.
 */
class ControlRobot {
 public:
  /**
   * Constructor.
   */
  ControlRobot(const geometry_msgs::Pose2D& goal_pose) : goal_pose_(goal_pose) {
    velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    odometry_sub_ =
        nh_.subscribe("/odom", 10, &ControlRobot::odometryCallback, this);
  }

  /**
   * Método de devolución de llamada (callback) del tópico `/odom`.
   */
  void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    ROS_INFO("Posicion actual: (%lf;%lf)", msg->pose.pose.position.x,
             msg->pose.pose.position.y);

    geometry_msgs::Pose2D current_pose = poseToPose2D(msg->pose.pose);
    velocity_pub_.publish(proportional_controller_.computeVelocityTowardsGoal(
        current_pose, goal_pose_));
  }

 private:
  /**
   * Convierte un mensaje `Pose` en uno `Pose2D`.
   */
  geometry_msgs::Pose2D poseToPose2D(const geometry_msgs::Pose& pose) {
    geometry_msgs::Pose2D pose2d;
    pose2d.x = pose.position.x;
    pose2d.y = pose.position.y;
    pose2d.theta = tf::getYaw(pose.orientation);
    return pose2d;
  }

  ros::NodeHandle nh_;

  ros::Subscriber odometry_sub_;
  ros::Publisher velocity_pub_;

  geometry_msgs::Pose2D goal_pose_;

  control_robot::ProportionalController proportional_controller_;
};

/**
 * Este nodo envía a un robot a una posición objetivo.
 */
int main(int argc, char** argv) {
  ros::init(argc, argv, "control_robot");

  geometry_msgs::Pose2D goal_pose;
  goal_pose.x = 2.0f;
  goal_pose.y = 2.0f;
  ControlRobot control_robot(goal_pose);

  ros::spin();

  return 0;
}
