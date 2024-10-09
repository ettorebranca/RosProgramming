#include "RViz.h"

RV_Pos::RV_Pos(int x, int y, int w, int h) {
  this->x = x - w / 2 + (1 - w % 2) * .5;
  this->y = h / 2 - y - (1 - h % 2) * .5;
  this->w = w;
  this->h = h;
}

RV_Map::RV_Map(ros::NodeHandle node_handle) {
  ROS_INFO("RViz node ready");
  rviz_pub = node_handle.advertise<visualization_msgs::Marker>(
      "visualization_marker", 0);
  robot_marker.header.frame_id = "map";
  robot_marker.header.stamp = ros::Time();
  robot_marker.ns = "robot_pose";
  robot_marker.id = 0;
  robot_marker.type = visualization_msgs::Marker::SPHERE;
  robot_marker.action = visualization_msgs::Marker::ADD;
  robot_marker.pose.position.z = 0;
  robot_marker.pose.orientation.x = 0.0;
  robot_marker.pose.orientation.y = 0.0;
  robot_marker.pose.orientation.z = 0.0;
  robot_marker.pose.orientation.w = 1.0;
  robot_marker.scale.x = .3;
  robot_marker.scale.y = .3;
  robot_marker.scale.z = .3;
  robot_marker.color.a = 1; 
  robot_marker.color.r = 0.0;
  robot_marker.color.g = 1.0;
  robot_marker.color.b = 0.0;
  robot_marker.lifetime = ros::Duration();

  initial_pos_sub = node_handle.subscribe("initial_pose", 1, &RV_Map::initial_position_callback, this);
}

void RV_Map::initial_position_callback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  geometry_msgs::PoseWithCovarianceStamped initial_pose = *msg;
  ROS_INFO("initial pose received: %0.2f, %0.2f",
           initial_pose.pose.pose.position.x,
           initial_pose.pose.pose.position.y);
  x_in = initial_pose.pose.pose.position.x < 0
                      ? floor(initial_pose.pose.pose.position.x) + .5
                      : ceil(initial_pose.pose.pose.position.x) - .5;
  y_in = ceil(initial_pose.pose.pose.position.y) - .5;
  std::cout << x_in << ", " << y_in << std::endl;
  initial_position_received = true;
}
void RV_Map::update_robot_position(RV_Pos coord) {
  robot_marker.pose.position.x = coord.x;
  robot_marker.pose.position.y = coord.y;
  rviz_pub.publish(robot_marker);
}

RV_Path::RV_Path(ros::NodeHandle node_handle) {
  // publisher to visualize the path
  path_pub = node_handle.advertise<nav_msgs::Path>("/path", 1);
  // populate header values
  path.header.stamp = ros::Time();
  path.header.frame_id = "map";
}

void RV_Path::update_path(RV_Pos coord) {
  // add each new visited position as a new pose
  geometry_msgs::PoseStamped pose;
  // populate pose header.
  pose.header.stamp = ros::Time();
  pose.header.frame_id = "map";
  geometry_msgs::Point position;
  position.x = coord.x;
  position.y = coord.y;
  position.z = 0;
  geometry_msgs::Quaternion orientation;
  orientation.x = 0;
  orientation.y = 0;
  orientation.z = 0;
  orientation.w = 0;
  pose.pose.orientation = orientation;
  pose.pose.position = position;
  path.poses.push_back(pose);
  // pass the new path vector to RViz
  path_pub.publish(path);
}