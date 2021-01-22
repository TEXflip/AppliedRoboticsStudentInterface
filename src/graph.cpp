#include "graph.hpp"

void Graph::setNodes(std::vector<Graph::node> *nodes){
    this->nodes_ = nodes;
}

void Graph::setCells(std::vector<Graph::cell> *cells){
    this->cells_ = cells;
}

std::vector<Graph::node>* Graph::nodes(){
    return this->nodes_;
}

std::vector<Graph::cell>* Graph::cells(){
    return this->cells_;
}