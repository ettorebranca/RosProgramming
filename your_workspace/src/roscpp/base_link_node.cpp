#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <iostream>

using namespace std;

int main(int argc, char** argv){
  cerr<<"base link node starting";
  ros::init(argc, argv, "base_link_node");
  ros::NodeHandle node;

  tf::TransformBroadcaster br;
  tf::Transform transform;
  int x,y;
  cout << "start x: ";
  cin >> x;
  cout << "start y: ";
  cin >> y;
  ros::Rate rate(10.0);
  while (node.ok()){
    
    transform.setOrigin( tf::Vector3(y,x, 0.0) ); // Set initial position to (10,10)
    transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
    rate.sleep();
  }
  return 0;
};