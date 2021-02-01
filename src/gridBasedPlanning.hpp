#pragma once
#include "utils.hpp"
#include "graph.hpp"
#include <vector>

void buildGridGraph(Graph::Graph &graph, const std::vector<Polygon> &obstacle_list,float margin, int nVert, int nOriz, float sideLength);
// static std::vector<Polygon> extend(const std::vector<Polygon>& p, float scale);
