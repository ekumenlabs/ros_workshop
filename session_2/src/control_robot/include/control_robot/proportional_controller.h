#ifndef CONTROL_ROBOT_PROPORTIONAL_CONTROLLER_H_
#define CONTROL_ROBOT_PROPORTIONAL_CONTROLLER_H_

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>

namespace control_robot {

/**
 * Controla un robot a lazo cerrado a partir de su posición actual y la posición
 * objetivo, utilizando un controlador proporcional.
 */
class ProportionalController {
 public:
  /**
   * Computa la velocidad a aplicar al robot para alcanzar el objetivo.
   */
  geometry_msgs::Twist computeVelocityTowardsGoal(
      const geometry_msgs::Pose2D& current_pose,
      const geometry_msgs::Pose2D& goal_pose);

 private:
  /**
   * Devuelve un mensaje `geometry_msgs::Twist` con todos sus campos
   * inicializados en cero.
   */
  geometry_msgs::Twist getZeroVelocity();

  /**
   * Devuelve `true` si el robot se encuentra en la posición objetivo.
   */
  bool isAtGoalPosition(const geometry_msgs::Pose2D& current_pose,
                        const geometry_msgs::Pose2D& goal_pose);

  /**
   * Computa la componente lineal de la velocidad hacia el objetivo.
   */
  float computeLinearSpeedToGoal(const geometry_msgs::Pose2D& current_pose,
                                 const geometry_msgs::Pose2D& goal_pose);

  /**
   * Computa la distancia euclideana (en línea recta) hacia el objetivo.
   */
  float computeEuclideanDistanceToGoal(
      const geometry_msgs::Pose2D& current_pose,
      const geometry_msgs::Pose2D& goal_pose);

  /**
   * Computa la componente angular de la velocidad hacia el objetivo.
   */
  float computeAngularSpeedToLookAtGoal(
      const geometry_msgs::Pose2D& current_pose,
      const geometry_msgs::Pose2D& goal_pose);

  /**
   * Computa la distancia angular para ubicar al robot mirando hacia el
   * objetivo.
   */
  float computeAngularDistanceToLookAtGoal(
      const geometry_msgs::Pose2D& current_pose,
      const geometry_msgs::Pose2D& goal_pose);

  const float kLinearSpeedConstant{0.1f};
  const float kAngularSpeedConstant{0.2f};
  const float kEuclideanDistanceToGoalTolerance{0.1f};
};

}  // namespace control_robot

#endif  // CONTROL_ROBOT_PROPORTIONAL_CONTROLLER_H_
