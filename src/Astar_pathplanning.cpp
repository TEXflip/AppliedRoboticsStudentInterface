#include <iostream>
#include <string>
#include <algorithm>
#include<list>

#include"Astar_pathplanning.h"

using namespace std;
using namespace Graph;


bool Test:: gama(){
		return false;
	}





float distance (Graph::Graph &graph, int a, int b)
    {
      return sqrtf((graph[a].x - graph[b].x)*(graph[a].x - graph[b].x) + (graph[a].y - graph[b].y)*(graph[a].y - graph[b].y)); //Sqrt(dx^2+dy^2)
    };

float heuristic (Graph::Graph &graph,int a, int b) // So we can experiment with heuristic
		{
			return distance(graph,a, b);
		};


	bool Astar :: Solve_AStar(Graph::Graph &graph)
	{
		// Reset Navigation Graph - default all node states
		for (int x = 0; x < graph.size(); x++)
			{
				graph[x].visited = false;
				graph[x].fGlobalGoal = INFINITY;
				graph[x].fLocalGoal = INFINITY;
				graph[x].parent = -1;	// No parents
			}



		// Setup starting conditions
		int nodeCurrent = nodeStart;
		graph[nodeStart].fLocalGoal = 0.0f;
		graph[nodeStart].fGlobalGoal = heuristic(graph,nodeStart, nodeEnd);

		// Add start node to not tested map - this will ensure it gets tested.
		// As the algorithm progresses, newly discovered nodes get added to this
		// map, and will themselves be tested later(soted by global)
		multimap<float,int> mapNotTestedNodes;
		mapNotTestedNodes.insert(std::pair<float,int>(graph[nodeStart].fGlobalGoal,nodeStart));
    	std::multimap<float, int>::const_iterator it_NotTestedNodes = mapNotTestedNodes.begin();
	

		// if the not tested map contains nodes, there may be better paths
		// which have not yet been explored. However, we will also stop 
		// searching when we reaach the target - there may well be better
		// paths but this one will do - it wont be the longest.
		while (!mapNotTestedNodes.empty() && nodeCurrent != nodeEnd)// Find absolutely shortest path // && nodeCurrent != nodeEnd)
		{
			it_NotTestedNodes = mapNotTestedNodes.begin();
			// Front of mapNotTestedNodes is potentially the lowest distance node. Our
			// map may also contain nodes that have been visited, so ditch these...
			while(!mapNotTestedNodes.empty() && graph[it_NotTestedNodes->second].visited)
				mapNotTestedNodes.erase(it_NotTestedNodes);

			// ...or abort because there are no valid nodes left to test
			if (mapNotTestedNodes.empty())
				break;
			

			it_NotTestedNodes = mapNotTestedNodes.begin();

			nodeCurrent = it_NotTestedNodes->second;
			graph[nodeCurrent].visited = true; // We only explore a node once
			
					
			// Check each of this node's neighbours...
			for (int i = 0; i < graph[nodeCurrent].neighbours.size(); i++)
			{
				int currNode = graph[nodeCurrent].neighbours[i];
			
				// ... and only if the neighbour is not visited and is 
				// not an obstacle, add it to NotTested List
				if (graph[currNode].visited && graph[currNode].obstacle == 0)
					mapNotTestedNodes.insert(std::pair<float,int>(graph[currNode].fGlobalGoal,currNode));

				// Calculate the neighbours potential lowest parent distance
				float fPossiblyLowerGoal = 	graph[nodeCurrent].fLocalGoal + distance(graph,nodeCurrent, currNode);

				// If choosing to path through this node is a lower distance than what 
				// the neighbour currently has set, update the neighbour to use this node
				// as the path source, and set its distance scores as necessary
				if (fPossiblyLowerGoal < graph[currNode].fLocalGoal)
				{
					graph[currNode].parent = nodeCurrent;
					graph[currNode].fLocalGoal = fPossiblyLowerGoal;

					// The best path length to the neighbour being tested has changed, so
					// update the neighbour's score. The heuristic is used to globally bias
					// the path algorithm, so it knows if its getting better or worse. At some
					// point the algo will realise this path is worse and abandon it, and then go
					// and search along the next best path.
					graph[currNode].fGlobalGoal = graph[currNode].fGlobalGoal + heuristic(graph,currNode, nodeEnd);
				}
			}	
		}

		return true;
	}

	

int main()
{
	Astar game;

	return 0;
}