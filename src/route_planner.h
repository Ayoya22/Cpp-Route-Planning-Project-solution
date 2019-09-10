#pragma once

#include <iostream>
#include <vector>
#include <string>
#include "route_model.h"

// contain methods and data to perform the A* search on the RouteModel data.
class RoutePlanner {
  public:
    RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y);
    float GetDistance() const {return distance;}
  void AStarSearch();

  private:
    void AddNeighbors(RouteModel::Node *current_node);
    float CalculateHValue(RouteModel::Node *node); // calculates the h-value for a given node.
    std::vector<RouteModel::Node> ConstructFinalPath(RouteModel::Node *);
    RouteModel::Node *NextNode();
    RouteModel &m_Model;
   /* 
   the floats are used to construct the start_node and end_node of the class.
  These will point to the nodes in the model which are closest to our starting and ending points.
  */
    RouteModel::Node *start_node;
    RouteModel::Node *end_node;
    float distance;
    std::vector<RouteModel::Node*> open_list;
};
