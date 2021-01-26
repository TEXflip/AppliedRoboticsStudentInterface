#pragma once
#include "utils.hpp"
#include "graph.hpp"
#include <vector>


void buildGridGraph(Graph::Graph &graph, const std::vector<Polygon> &obstacle_list, const Polygon &borders, float sideLength);
static std::vector<Polygon> scale(const std::vector<Polygon>& p, float scale);