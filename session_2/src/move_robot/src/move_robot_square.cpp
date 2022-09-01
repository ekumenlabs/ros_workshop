#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#include <cmath>

constexpr double kLinearSpeedConstant = 0.1f;
constexpr double kAngularSpeedConstant = 0.1f;

constexpr double kSquareSideLength = 1.0f;

/**
 * Mueve a un robot de forma lineal.
 */
void move(const ros::Publisher& velocity_pub, double linear_speed,
          double linear_distance) {
  geometry_msgs::Twist velocity_msg;


  ROS_INFO("Moviendo robot una distancia de %lf", linear_distance);

  // TODO: Implementar usando la implementacion de rotate como inspiracion.

}

/**
 * Mueve a un robot de forma angular.
 */
void rotate(const ros::Publisher& velocity_pub, double angular_speed,
            double angular_distance) {
  geometry_msgs::Twist velocity_msg;
  velocity_msg.linear.x = 0.0f;
  velocity_msg.linear.y = 0.0f;
  velocity_msg.linear.z = 0.0f;
  velocity_msg.angular.x = 0.0f;
  velocity_msg.angular.y = 0.0f;
  velocity_msg.angular.z = angular_speed;

  ROS_INFO("Rotando robot un angulo de %lf", angular_distance);

  ros::Rate loop_rate(10);
  double rotated_angle = 0.0f;
  const double start_time = ros::Time::now().toSec();
  while (ros::ok() && rotated_angle < angular_distance) {
    velocity_pub.publish(velocity_msg);

    ros::spinOnce();
    loop_rate.sleep();

    /**
     * El ángulo se estima a lazo abierto a partir de la velocidad angular y el
     * tiempo trascurrido (theta(t) = w*t).
     */
    rotated_angle = angular_speed * (ros::Time::now().toSec() - start_time);
  }

  /**
   * Detengo el robot.
   */
  ROS_INFO("Deteniendo robot");
  velocity_msg.angular.z = 0.0f;
  velocity_pub.publish(velocity_msg);
}

/**
 * Mueve a un robot describiendo una trayectoria cuadrada.
 */
void drawSquare(const ros::Publisher& velocity_pub, double side_length) {
  for (int side = 0; side < 4; ++side) {
    move(velocity_pub, kLinearSpeedConstant, side_length);
    rotate(velocity_pub, kAngularSpeedConstant, M_PI_2);
  }
}

/**
 * Este nodo mueve a un robot describiendo una trayectoria cuadrada.
 */
int main(int argc, char** argv) {
  ros::init(argc, argv, "move_robot_square");
  ros::NodeHandle nh;

  ros::Publisher velocity_pub =
      nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  ros::Duration(1.0).sleep();  // Demora para finalizar handshakes de tópicos.

  drawSquare(velocity_pub, kSquareSideLength);

  return 0;
}
