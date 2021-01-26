#pragma once
#include "utils.hpp"
#include "graph.hpp"
#include "clipper/clipper.hpp"
#include <vector>

void buildGridGraph(Graph::Graph &graph, const std::vector<Polygon> &obstacle_list, const Polygon &borders, float sideLength);
static std::vector<Polygon> extend(const std::vector<Polygon>& p, float scale);
std::vector<Polygon> offsetPolygon(const std::vector<Polygon> &polygons, float offset);