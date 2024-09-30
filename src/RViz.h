#pragma once
#include <vector>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>

class RV_Pos {
 public:
  float x;
  float y;
  int w;
  int h;
  RV_Pos(int x, int y, int w, int h);
};
class RV_Map {
 public:
  visualization_msgs::Marker robot_marker;
  ros::Publisher rviz_pub;
  ros::Subscriber initial_pos_sub;
  float initial_pos_x;
  float initial_pos_y;
  bool initial_pos_recieved;

  RV_Map(ros::NodeHandle node_handle);
  void update_robot_position(RV_Pos coord);
  void initial_pos_callback(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void set_goal(RV_Pos coord);
};

class RV_Path {
 public:
  ros::Publisher path_pub;
  nav_msgs::Path path;

  RV_Path(ros::NodeHandle node_handle);
  void update_path(RV_Pos coord);
};