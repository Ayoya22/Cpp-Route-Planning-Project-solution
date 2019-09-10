#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
 /*
 To conduct  search, Routeplanner has to initialize the starting and ending coordinate
  the floats are used to construct the start_node and end_node of your class
  Scale the floats to percentages by multiplying each float by 0.01 and storing the result in the float variable.
  */
  start_x *= 0.01;
  start_y *= 0.01;
  end_x *= 0.01;
  end_y *= 0.01;
  
  //method to find the closest nodes to (start_x, start_y) and (end_x, end_y). 
  start_node = &m_Model.FindClosestNode(start_x, start_y);
  end_node = &m_Model.FindClosestNode(end_x, end_y);
  
}
// searches the closest node to the end node using the A* algorithm
void RoutePlanner::AStarSearch() {
   start_node->visited = true;
   open_list.push_back(start_node);
   RouteModel::Node *current_node = nullptr;
   while (open_list.size() > 0) {
       current_node = NextNode();
     
       if (current_node->distance(*end_node) == 0) {
           m_Model.path = ConstructFinalPath(current_node);
           return;
       }
     AddNeighbors(current_node);
   }
   
}

/*
This method will take each neighbor of the current node in the A* search, set the neighbor's g-value, h-value, and parent, and add the neighbor to the open list.
*/
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
   current_node->FindNeighbors();
  
  for (auto neighbor : current_node->neighbors){
      neighbor->parent = current_node;
      neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
      neighbor->h_value = CalculateHValue(neighbor); 
    
      open_list.push_back(neighbor);
      neighbor->visited = true;
  }
}

//sort the list of open nodes in the A* search, and remove the node from the list of open nodes. 
RouteModel::Node *RoutePlanner::NextNode() {
   std::sort(open_list.begin(), open_list.end(), [](const auto &_1st, const auto &_2nd) {
     return _1st->h_value + _1st->g_value < _2nd->h_value + _2nd->g_value; 
   });

  RouteModel::Node *lowest_node = open_list.front();
  open_list.erase(open_list.begin());
  return lowest_node;// return node with lowest f-value
}
 // Calculating the h value for a given node
float RoutePlanner::CalculateHValue(RouteModel::Node *node){
   return node->distance(*end_node);
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
   // creating path_found vector now
  distance = 0.0f;
  std::vector<RouteModel::Node> path_found;
  RouteModel::Node parent;
   while (current_node->parent != nullptr) {
       path_found.push_back(*current_node);
       parent = *(current_node->parent);
       distance += current_node->distance(parent);
       current_node = current_node->parent;
   }
   path_found.push_back(*current_node);
   distance *= m_Model.MetricScale();
   return path_found;
}


