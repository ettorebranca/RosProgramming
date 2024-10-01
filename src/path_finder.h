#pragma once
#include <iostream>
#include <queue>
#include <unordered_set>
#include <vector>
#include "RViz.h"
#include "map_manag.h"

class Node : public Position {
 public:
  Node* parent;  
  Node(const Position& coord, Node* p);
};

class Planner {    
  Map map;
 public:
  Planner(Map map);
  std::vector<RV_Pos> rec_path(Node* goalNode);   //to trace back the path from the goal node
  std::vector<RV_Pos> plan_path(Position start);   //to implement the A* algorithm starting from a given coordinate.
};