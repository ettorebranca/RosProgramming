#include "path_finder.h"
#include "RViz.h"
#include "map_manag.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "simple_planner_rp_node");
  ros::NodeHandle node_handle;
  ros::Rate r(3);
  Map map(node_handle);
  std::vector<RV_Pos> path;
  while (ros::ok()) {
    while (!map.get_message) {
      ros::spinOnce();
      ROS_INFO_ONCE("Waiting to get data");
    }
    ROS_INFO("Data received");
    int w = map.get_w();
    int h = map.get_h();
    if (w % 2 != 0 || h % 2 != 0) {
      ROS_ERROR("Even number of rows and cols required");
      return 1;
    }
    RV_Map rviz_map(node_handle);
    RV_Path rviz_path(node_handle);

    while (rviz_map.initial_pos_sub.getNumPublishers() < 1 ||
           !rviz_map.initial_position_received) {
      ros::spinOnce();
      ROS_INFO_ONCE("Waiting to get initial position from RViz");
    }
    int in_x =
        rviz_map.x_in + w / 2 - (1 - w % 2) * .5;  
    int in_y =
        h / 2 - rviz_map.y_in - (1 - h % 2) * .5;  
    Position initial_position(in_x, in_y, w, h);
    while (rviz_map.rviz_pub.getNumSubscribers() < 1 &&
           rviz_path.path_pub.getNumSubscribers() < 1) {
      ROS_INFO_ONCE("Waiting for RViz to subscribe");
    }
    rviz_map.update_robot_position(RV_Pos(in_x, in_y, w, h));
    if (map.get_element_at(initial_position) == Cell::Obst) {
      ROS_ERROR("Invalid starting position (obstacle)");
      return 1;
    }
    if (in_x >= w || in_x < 0 || in_y >= h || in_y < 0) {
      ROS_ERROR("Invalid starting position");
      return 1;
    }
    Planner planner(map);
    ROS_INFO("Planning...");
    path = planner.plan_path(initial_position);

    ROS_INFO("Press to start");
    std::cin.get();
    if (path.size() == 0) {
      ROS_WARN("Path not found");
      return 0;
    }
    for (size_t i = 0; i < path.size(); i++) {
      rviz_map.update_robot_position(path[i]);
      rviz_path.update_path(path[i]);
      r.sleep();
    }
    ROS_INFO("Press to stop");
    std::cin.get();
    return 0;
  }
}