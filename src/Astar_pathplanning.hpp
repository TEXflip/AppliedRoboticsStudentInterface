#pragma once

#include <string>
#include <algorithm>
#include <vector>
#include "graph.hpp"

using namespace std;
using namespace Graph;

class Astar
{
private:
	int nodeStart = 0;
	int nodeEnd = 0;

	static float distance(Graph::Graph &graph, int a, int b);
	static float heuristic(Graph::Graph &graph, int a, int b);

public:
	static vector<int> Solve_AStar(Graph::Graph &graph, int start, int end);
};