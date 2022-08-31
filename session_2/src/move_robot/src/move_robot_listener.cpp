#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

/**
 * Función de devolución de llamada (callback) del tópico `/cmd_vel`.
 */
void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  ROS_INFO("Velocidad lineal en X: [%f]", msg->linear.x);
}

/**
 * Este nodo escucha los mensajes enviados al robot.
 */
int main(int argc, char** argv) {
  ros::init(argc, argv, "move_robot_listener");
  ros::NodeHandle nh;

  ros::Subscriber chatter_sub =
      nh.subscribe("/cmd_vel", 1000, velocityCallback);

  ros::spin();

  return 0;
}
