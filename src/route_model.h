#pragma once

#include <limits>
#include <cmath>
#include <unordered_map>
#include "model.h"
#include <iostream>

//in order to perform the A* search, you will need to use RouteModel::Node objects
class RouteModel : public Model { 

  public:
  // contains information about a node
    class Node : public Model::Node {
      public:
        
        Node * parent = nullptr;
        float h_value = std::numeric_limits<float>::max();
        float g_value = 0.0;
        bool visited = false;
        std::vector<Node *> neighbors;
         
      // populate the neighbors vector of the current Node object (the vector this->neighbors)
        void FindNeighbors();//error located!!
      
      // This function will allow the search algorithm to find the closest node to the current node.
        float distance(Node other) const {
           return std::sqrt(std::pow((x - other.x), 2) + std::pow((y - other.y), 2));
        }
        
        Node(){}
        Node(int idx, RouteModel * search_model, Model::Node node) : Model::Node(node), parent_model(search_model), index(idx) {}
      
      private:
        // Add private Node variables and methods here.
        int index;
        Node * FindNeighbor(std::vector<int> node_indices);
        RouteModel * parent_model = nullptr;
    };
    
    // Add public RouteModel variables and methods here.
    RouteModel(const std::vector<std::byte> &xml); 
    Node &FindClosestNode(float x, float y);
    auto &SNodes() {return m_Nodes;}
    auto &GetNodeToRoadMap() {return node_to_road;}
    std::vector<Node> path; // This variable will eventually store the path that is found by the A* search.

  private:
    // Add private RouteModel variables and methods here.
  void CreateNodeToRoadHashmap(); 
  /* 
  a function that creates a hash table of Node index values to a vector of Road pointers that those   nodes belong to.
  */
  std::unordered_map<int, std::vector<const Model::Road*>> node_to_road;
  std::vector<Node> m_Nodes;

};
