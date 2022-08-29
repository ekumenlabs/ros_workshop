#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

/**
 * Este nodo mueve a un robot utilizando ROS.
 */
int main(int argc, char** argv) {
  ros::init(argc, argv, "move_robot");
  ros::NodeHandle nh;

  ros::Publisher velocity_pub =
      nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

  geometry_msgs::Twist velocity_msg;
  velocity_msg.linear.x = 0.0f;
  velocity_msg.linear.y = 0.0f;
  velocity_msg.linear.z = 0.0f;
  velocity_msg.angular.x = 0.0f;
  velocity_msg.angular.y = 0.0f;
  velocity_msg.angular.z = 0.0f;

  ros::Rate loop_rate(10);

  while (ros::ok()) {
    velocity_msg.linear.x += 0.01f;
    ROS_INFO("Velocidad lineal en X: %f", velocity_msg.linear.x);

    velocity_pub.publish(velocity_msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
