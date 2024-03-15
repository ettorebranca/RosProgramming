#include <ros/ros.h>
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/TransformStamped.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "base/grid_map.h"
#include <vector>
#include <iostream>


using namespace std;

bool gridMapFlag = false;
bool baseLinkFlag = false;
bool goalFlag = false;
bool baseLingAndGoalReceived = false;
bool RowsAndColsFlag = false;
GridMap gridMap;
pair<int, int> start, goal;
vector<uint8_t> gridMapMsg;
vector<int>RowsAndColsMsg;




void gridMapCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg) {
    // Print the received matrix data
    /*
    for (size_t i = 0; i < msg->data.size(); i++) {
        std::cerr << static_cast<int>(msg->data[i]) << " ";
    }
    std::cerr << std::endl;
    */
    if (gridMapFlag==false){
        ROS_INFO("grid_map received");
    }
    gridMapFlag=true;
    gridMapMsg = msg->data;
}

void rowsAndColsCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    if (RowsAndColsFlag==false){
        ROS_INFO("rows_and_cols received");
    }
    RowsAndColsFlag = true;
    RowsAndColsMsg = msg->data;
}

void printPath(const std::vector<std::pair<int, int>>& path) {
    for (const auto& point : path) {
        std::cout << "(" << point.first << ", " << point.second << ") ";
    }
    std::cout << std::endl;
}


int main(int argc, char** argv)
{
  std::cerr<<"planner node starting"<<endl;

  ros::init(argc, argv, "planner_node");
  ros::NodeHandle node;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  // Create a subscriber for UInt8MultiArray messages
  ros::Subscriber sub = node.subscribe("grid_map", 10, gridMapCallback);
  ros::Subscriber sub2 = node.subscribe("rowsAndCols", 10, rowsAndColsCallback);

  geometry_msgs::TransformStamped goalTransform;
  geometry_msgs::TransformStamped baseLinkTransform;

  ros::Rate rate(10.0);

  while (node.ok()) {
    ros::spinOnce();
    
    
    try {
      goalTransform = tfBuffer.lookupTransform("map", "goal", ros::Time(0));
      goal = {goalTransform.transform.translation.x,goalTransform.transform.translation.y};
      goalFlag=true;
      
    }
    catch (tf2::TransformException& ex) {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    try {
      baseLinkTransform = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
      start = {baseLinkTransform.transform.translation.x,baseLinkTransform.transform.translation.y};
      baseLinkFlag = true;
    }
    catch (tf2::TransformException& ex) {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    
    if (baseLinkFlag==true && goalFlag==true && baseLingAndGoalReceived==false){
        ROS_INFO("goal and baselink received");
        baseLingAndGoalReceived = true;};
    if (gridMapFlag==true && baseLinkFlag==true && goalFlag==true && RowsAndColsFlag){
        gridMap.loadFromVec(gridMapMsg,RowsAndColsMsg[0],RowsAndColsMsg[1]);
        gridMap.computeDistanceMap();
        vector<pair<int, int>> path;
        path = gridMap.findPath(start,goal);
        //printPath(path);
        gridMap.displayPath(path);
        ROS_INFO("completato");
    return 0;
    }

    

    rate.sleep();
  }
  return 0;
}