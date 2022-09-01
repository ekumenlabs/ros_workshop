#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

/**
 * Función de devolución de llamada (callback) del tópico `/cmd_vel`.
 */
void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  // TODO: Imprimir velocidad lineal en x.
}

/**
 * Este nodo escucha los mensajes enviados al robot.
 */
int main(int argc, char** argv) {
  ros::init(argc, argv, "move_robot_listener");
  ros::NodeHandle nh;
  
  // TODO: Implementar nodo para escuchar mensajes del nodo move_robot.
  
  return 0;
}
