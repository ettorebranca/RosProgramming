#pragma once
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <stdio.h>

enum Cell { Obst, Valid, Arrive, NotVal };

class Position {
 public:
  int x;
  int y;
  int idx;
 Position(int x, int y, int w, int h);
};


class Map {
 private:
  nav_msgs::OccupancyGrid occupancy_grid;
  ros::Subscriber map_sub;

 public:
  bool get_message = false;
  Map(ros::NodeHandle node_handle);
  void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  std::vector<int8_t, std::allocator<int8_t>> get_data();
  int get_w();
  int get_h();
  Cell get_element_at(Position coords);
};