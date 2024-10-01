#include "path_finder.h"


Node::Node(const Position &coord, Node *p = nullptr)
    : Position(coord), parent(p) {}



Planner::Planner(Map map) : map(map) {}
std::vector<RV_Pos> Planner::rec_path(Node *goal_node) {
  std::vector<RV_Pos> path;
  Node *current_node = goal_node;
  while (current_node != nullptr) {
    path.push_back(RV_Pos(current_node->x, current_node->y, map.get_w(), map.get_h()));
    current_node = current_node->parent;
  }
  reverse(path.begin(), path.end());
  return path;
}
std::vector<RV_Pos> Planner::plan_path(Position start) {
  if (map.get_element_at(start) == Cell::Goal) {
    return std::vector<RV_Pos>(
        1, RV_Pos(start.x, start.y, map.get_w(), map.get_h()));
  }
  std::queue<Node *> frontier;   // FIFO queue
  std::vector<std::vector<bool>> visited(
      map.get_w(), std::vector<bool>(map.get_h(), false));
  visited[start.x][start.y] = true;
  frontier.push(new Node(start));
  std::vector<std::pair<int, int>> directions = {{1, 0},{-1, 0},{0, 1},{0, -1}};
  while (!frontier.empty()) {
    Node *current_node = frontier.front();
    frontier.pop();
    if (map.get_element_at(*current_node) == Cell::Goal) {
      return rec_path(current_node);
    }
    for (const auto &direction : directions) {
      int x = current_node->x + direction.first;
      int y = current_node->y + direction.second;
      // Check if the new cell is valid and not visited yet
      if (x >= 0 && x < map.get_w() && y >= 0 && y < map.get_h() &&
          !visited[x][y] &&
          map.get_element_at(*current_node) != Cell::Obst) {
        visited[x][y] = true; 
        frontier.push(
            new Node(Position(x, y, map.get_w(), map.get_h()), current_node));  
      }
    }
  }
  return {};
}