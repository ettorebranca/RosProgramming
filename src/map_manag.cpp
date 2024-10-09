#include "map_manag.h"

Position::Position(int x, int y, int w, int h) {
  int x_map = x;
  int y_map = (h - 1) - y;
  this->idx = x_map + (y_map * w);
  this->x = x;
  this->y = y;
}

Map::Map(ros::NodeHandle node_handle) {
  ROS_INFO("Subscribed to map_server_node");
  map_sub = node_handle.subscribe("map", 100, &Map::map_callback, this);
}
void Map::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  occupancy_grid = *msg;
  get_message = true;
}
std::vector<int8_t, std::allocator<int8_t>> Map::get_data() {
  return occupancy_grid.data;
}
int Map::get_w() { return occupancy_grid.info.width; }
int Map::get_h() { return occupancy_grid.info.height; }
Cell Map::get_element_at(Position coords) {
  switch (occupancy_grid.data[coords.idx]) {
    case 0:
      return Cell::Valid;
    case -1:
      return Cell::Goal;
    case 100:
      return Cell::Obst;
    default:
      return Cell::NotVal;
  }
}