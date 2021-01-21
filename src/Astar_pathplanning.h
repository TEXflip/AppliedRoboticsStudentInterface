#ifndef ASTAR_PATHPLANNING
#define ASTAR_PATHPLANNING

#include <iostream>
#include <string>
#include <algorithm>
using namespace std;

struct sNode
	{
		bool bObstacle = false;			// Is the node an obstruction?
		bool bVisited = false;			// Have we searched this node before?
		float fGlobalGoal;				// Distance to goal so far
		float fLocalGoal;				// Distance to goal if we took the alternative route
		int x;							// Nodes position in 2D space
		int y;
		vector<sNode*> vecNeighbours;	// Connections to neighbours
		sNode* parent;					// Node connecting to this node that offers shortest parent
	};

class Astar 
{
public:


private:
	sNode *nodes = nullptr;
	int nMapWidth = 16;
	int nMapHeight = 16;

	sNode *nodeStart = nullptr;
	sNode *nodeEnd = nullptr;
	

protected:
    bool Graph();//creates
    bool Solve_AStar();
};
class Test
{
	protected:
	bool gama();
};


#endif