#pragma once

#include <iostream>
#include <string>
#include <algorithm>
#include <vector>
using namespace std;
#include "graph.hpp"
using namespace Graph;

class Astar
{
private:
	int nodeStart = 0;
	int nodeEnd = 0;

public:
	static vector<int> Solve_AStar(Graph::Graph &graph, int start, int end);
};