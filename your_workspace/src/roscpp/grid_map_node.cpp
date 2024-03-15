#include <ros/ros.h>
#include "base/grid_map.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include <iostream>
#include <vector>

int main(int argc, char** argv) {
   
    // Create a GridMap object
    GridMap gridMap;
    gridMap.loadFromImage("put_image_here");

    vector<int> rowsAndCols(2);
    rowsAndCols[0]= gridMap.rows;
    rowsAndCols[1]= gridMap.cols;

    // Initialize the ROS node
    ros::init(argc, argv, "grid_map_node");
    ros::NodeHandle nh;

    // Define the topic name
    std::string topic_name = "grid_map";

    // Create a publisher for UInt8MultiArray messages
    ros::Publisher pub = nh.advertise<std_msgs::UInt8MultiArray>(topic_name, 10);
    ros::Publisher pub2 = nh.advertise<std_msgs::Int32MultiArray>("rowsAndCols", 10);

    // Set the publishing rate
    ros::Rate rate(1); // 1 Hz

    

    while (ros::ok())
    {   

        // Create the message object
        std_msgs::UInt8MultiArray matrix_msg;
        matrix_msg.data = gridMap.gridMapArray;

        // Create the message object
        std_msgs::Int32MultiArray rowsAndColsMsg;
        rowsAndColsMsg.data = rowsAndCols;
        

        // Publish the message
        pub.publish(matrix_msg);
        pub2.publish(rowsAndColsMsg);

        //ROS_INFO("Matrix sent:");

        /*
        // Print the matrix for debugging
        for (size_t i = 0; i < gridMap.gridMapArray.size(); 3)
        {
            ROS_INFO("%3d %3d %3d", gridMap.gridMapArray[i], gridMap.gridMapArray[i + 1], gridMap.gridMapArray[i + 2]);
        }
        */

        // Sleep to maintain the publishing rate
        rate.sleep();
    }

    return 0;
}