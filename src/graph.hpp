#pragma once
#include <vector>
#include <map>
#include "collision_detection.hpp"

// #include <iostream>

namespace Graph
{

    // struct cell
    // {
    //     std::vector<int> nodes;
    // };

    struct node
    {
        bool visited = false; // Have we searched this node before?
        bool obstacle = false;
        bool removed = false;
        float fGlobalGoal;    // Distance to goal so far
        float fLocalGoal;     // Distance to goal if we took the alternative route
        float x;              // Nodes position in 2D space
        float y;
        std::vector<int> neighbours; // Connections to neighbours
        node *parent; // Node connecting to this node that offers shortest parent
    };

    bool Solve_AStar();

    typedef std::vector<node> Graph;
    // std::vector<cell> *cells_ = nullptr;
};
void closestPoint(Graph::Graph &graph,const float x, const float y, const std::vector<Polygon> &obstacle_list);
