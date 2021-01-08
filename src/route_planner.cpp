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
      open_list.begin(), open_list.end(),
      [](const RouteModel::Node *left, const RouteModel::Node *right) -> bool {
        auto FValue = [](const RouteModel::Node *node) -> double {
          return node->g_value + node->h_value;
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
  RouteModel::Node *next_node;
  next_node = current_node;
  auto parent = next_node->parent;
  do {
    path_found.push_back(*next_node);
    distance += next_node->distance(*parent);
    next_node = parent;
    parent = next_node->parent;
  } while (next_node != start_node);
  path_found.push_back(*next_node);
  distance *= m_Model.MetricScale();
  std::reverse(path_found.begin(), path_found.end());
  return path_found;
}

void RoutePlanner::AStarSearch() {
  open_list.push_back(start_node);
  start_node->visited = true;
  while (!open_list.empty()) {
    RouteModel::Node *current_node = NextNode();
    if (current_node == end_node) {
      m_Model.path = ConstructFinalPath(current_node);
      return;
    }
    AddNeighbors(current_node);
  }
}