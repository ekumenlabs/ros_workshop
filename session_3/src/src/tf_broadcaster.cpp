#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

std::string frame_name;
float theta = 0;


void timerCallback(const ros::TimerEvent& event){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0, 0, 0.0) );
  tf::Quaternion q;
  theta = theta + 1;
  if( theta == 3) {
    theta = 0;
  }
  q.setRPY(0, 0, theta);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", frame_name));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  if (argc != 2){ROS_ERROR("need frame name as argument"); return -1;};
  frame_name = argv[1];

  ros::NodeHandle node;
  ros::Timer timer = node.createTimer(ros::Duration(0.1), timerCallback);
 

  ros::spin();
  return 0;
};