#include "route_planner.h"

#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y,
                           float end_x, float end_y)
    : m_Model(model) {
  // Convert inputs to percentage:
  start_x *= 0.01;
  start_y *= 0.01;
  end_x *= 0.01;
  end_y *= 0.01;
  start_node = &m_Model.FindClosestNode(start_x, start_y);
  end_node = &m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  return std::sqrt(std::pow(node->x - end_node->x, 2) +
                   std::pow(node->y - end_node->y, 2));
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  current_node->FindNeighbors();
  for (auto neighbor : current_node->neighbors) {
    neighbor->parent = current_node;
    neighbor->h_value = CalculateHValue(neighbor);
    neighbor->g_value =
        current_node->g_value + current_node->distance(*neighbor);
    open_list.push_back(neighbor);
    neighbor->visited = true;
  }
}

RouteModel::Node *RoutePlanner::NextNode() {
  std::sort(
      open_list.front(), open_list.back(),
      [](const RouteModel::Node &left, const RouteModel::Node &right) -> bool {
        auto FValue = [](const RouteModel::Node &node) -> double {
          return node.g_value + node.h_value;
        };
        return FValue(left) > FValue(right);
      });
  RouteModel::Node *next_node = open_list.back();
  open_list.pop_back();
  return next_node;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(
    RouteModel::Node *current_node) {
  distance = 0.0f;
  std::vector<RouteModel::Node> path_found;

  auto parent = current_node->parent;
  do {
    path_found.push_back(*current_node);
    distance += current_node->distance(*parent);
    current_node = parent;
    parent = current_node->parent;
  } while (current_node != start_node);
  path_found.push_back(*current_node);
  distance *= m_Model.MetricScale();
  std::reverse(path_found.begin(), path_found.end());
  return path_found;
}

// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node
// to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method
// to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits.
// This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
  RouteModel::Node *current_node = nullptr;

  // TODO: Implement your solution here.
}