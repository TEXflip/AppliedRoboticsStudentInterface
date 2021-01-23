#pragma once
#include <vector>

class Graph
{
public:
    struct cell
    {
        std::vector<int> nodes;
    };

    struct node
    {
        bool visited = false; // Have we searched this node before?
        float fGlobalGoal;    // Distance to goal so far
        float fLocalGoal;     // Distance to goal if we took the alternative route
        float x;              // Nodes position in 2D space
        float y;
        std::vector<int> neighbours; // Connections to neighbours
        std::vector<int> neighboursCells;
        node *parent; // Node connecting to this node that offers shortest parent
    };

    Graph() = default;
    bool Solve_AStar();
    void setNodes(std::vector<node> *nodes);
    void setCells(std::vector<cell> *cells);
    std::vector<node>* nodes();
    std::vector<cell>* cells();

private:
    std::vector<node> *nodes_ = nullptr;
    std::vector<cell> *cells_ = nullptr;
};