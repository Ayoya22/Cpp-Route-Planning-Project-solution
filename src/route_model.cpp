#include "route_model.h"
#include <iostream>

RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) {
   
  int counter = 0;
  for (Model::Node node : this->Nodes()) {
      m_Nodes.push_back(Node(counter, this, node));
      counter++;
  }
  
  CreateNodeToRoadHashmap();
  
}
/* a function that creates a hash table of Node index values to a vector of Road pointers that those nodes belong to.
   This method will operate only on the node_to_road variable declared above
 */
void RouteModel::CreateNodeToRoadHashmap() {
    for (const Model::Road &road : Roads()) {
        if (road.type != Model::Road::Type::Footway) {// checking that the type is not a footway, for each reference "&road"in the vector
            for (int node_idx : Ways()[road.way].nodes) {
                if (node_to_road.find(node_idx) == node_to_road.end()) {
                    node_to_road[node_idx] = std::vector<const Model::Road *> ();
                }
              node_to_road[node_idx].push_back(&road);
            }
        }
    }
}

/* return a pointer to the closest unvisited node from a vector of node indices, where the distance is measured to the current node (this).     This method will be used later to help find all of the possible next steps in the A* search.
*/
RouteModel::Node *RouteModel::Node::FindNeighbor(std::vector<int> node_indices) {
    Node *closest_node = nullptr;
    Node node;
  
    for (int node_index : node_indices) {
        node = parent_model->SNodes()[node_index];
        if (this->distance(node) != 0 && !node.visited) {
            if (closest_node == nullptr || this->distance(node) < this->distance(*closest_node)) {
                closest_node = &parent_model->SNodes()[node_index]; //  update closest_node as you find closer nodes in the loop
            }
        }
    }
   return closest_node;
}


// populate the neighbors vector of the current Node object (the vector this->neighbors).
void RouteModel::Node::FindNeighbors() {
   for (auto & road : parent_model->node_to_road[this->index]) {
       RouteModel::Node *new_neighbor = this->FindNeighbor(parent_model->Ways()[road->way].nodes);
       if (new_neighbor) {
           this->neighbors.emplace_back(new_neighbor);
       }
   }
}
/* The starting points are generally given in the form of floats and the doesn,t always correspond to 
 a given location on the map so we have to find the closest node representing a location to our inputed
 coordinates 
*/
RouteModel::Node &RouteModel::FindClosestNode(float x, float y) {
   Node input;
   input.x = x;
   input.y = y;
   
   float min_dist = std::numeric_limits<float>::max();
   float dist;
   int closest_idx;
  
   for (const Model::Road &road : Roads()) {
       if (road.type != Model::Road::Type::Footway) {
           for (int node_idx : Ways()[road.way].nodes) {
               dist = input.distance(SNodes()[node_idx]);
               if (dist < min_dist) {
                   closest_idx = node_idx;
                   min_dist = dist;
               }
           }
       }
   }
     return SNodes()[closest_idx];
}