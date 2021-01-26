#ifndef ASTAR_PATHPLANNING
#define ASTAR_PATHPLANNING

#include <iostream>
#include <string>
#include <algorithm>
using namespace std;
#include "graph.hpp"
using namespace Graph;


class Astar 
{
public:


private:
	node *nodes = nullptr;
	int nMapWidth = 16;
	int nMapHeight = 16;

	int nodeStart =0;
	int nodeEnd= 0;
	

protected:
    bool Graph();//creates
	bool Solve_AStar(Graph::Graph &graph);
};
class Test
{
	protected:
	bool gama();
};


#endif