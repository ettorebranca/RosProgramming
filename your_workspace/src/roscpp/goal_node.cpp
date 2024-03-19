#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <iostream>

using namespace std;

int main(int argc, char** argv){
  cerr<<"goal node starting";

  ros::init(argc, argv, "goal_node");
  ros::NodeHandle node;

  tf::TransformBroadcaster br;
  tf::Transform transform;

  ros::Rate rate(10.0);
  
  int x,y;
  cout << "goal x: ";
  cin >> x;
  cout << "goal y: ";
  cin >> y;
  
  while (node.ok()){
    transform.setOrigin( tf::Vector3(y, x, 0.0) ); // Set initial position to (10,10)
    transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "goal"));
    rate.sleep();
  }
  return 0;
};