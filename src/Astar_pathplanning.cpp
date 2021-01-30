#include <iostream>
#include <string>
#include <algorithm>
#include <list>

#include "collision_detection.hpp"
#include "Astar_pathplanning.hpp"
#include "utils.hpp"

using namespace std;
using namespace Graph;

float Astar::distance(Graph::Graph &graph, int a, int b)
{
	float dx = (graph[a].x - graph[b].x);
	float dy = (graph[a].y - graph[b].y);
	return sqrtf(dx * dx + dy * dy); //Sqrt(dx^2+dy^2)
};

float Astar::heuristic(Graph::Graph &graph, int a, int b) // So we can experiment with heuristic
{
	return Astar::distance(graph, a, b);
};

vector<int> Astar::Solve_AStar(Graph::Graph &graph, int nodeStart, int nodeEnd)
{
	// Reset Navigation Graph - default all node states
	for (int x = 0; x < graph.size(); x++)
	{
		graph[x].visited = false;
		graph[x].fGlobalGoal = INFINITY;
		graph[x].fLocalGoal = INFINITY;
		graph[x].parent = -1; // No parents
	}

	// Setup starting conditions
	int nodeCurrent = nodeStart;
	graph[nodeStart].fLocalGoal = 0.0f;
	graph[nodeStart].fGlobalGoal = Astar::heuristic(graph, nodeStart, nodeEnd);

	// Add start node to not tested map - this will ensure it gets tested.
	// As the algorithm progresses, newly discovered nodes get added to this
	// map, and will themselves be tested later(soted by global)
	// list<int> listNotTestedNodes;
	multimap<float, int> mapNotTestedNodes;
	mapNotTestedNodes.insert(std::pair<float, int>(graph[nodeStart].fGlobalGoal, nodeStart));
	std::multimap<float, int>::const_iterator it_NotTestedNodes = mapNotTestedNodes.begin();

	// if the not tested map contains nodes, there may be better paths
	// which have not yet been explored. However, we will also stop
	// searching when we reaach the target - there may well be better
	// paths but this one will do - it wont be the longest.
	while (!mapNotTestedNodes.empty() && nodeCurrent != nodeEnd) // Find absolutely shortest path // && nodeCurrent != nodeEnd)
	{
		// cout << nodeCurrent << endl;
		it_NotTestedNodes = mapNotTestedNodes.begin();
		// Front of mapNotTestedNodes is potentially the lowest distance node. Our
		// map may also contain nodes that have been visited, so ditch these...
		while (!mapNotTestedNodes.empty() && graph[it_NotTestedNodes->second].visited)
		{
			mapNotTestedNodes.erase(it_NotTestedNodes);
			it_NotTestedNodes = mapNotTestedNodes.begin();
		}

		// ...or abort because there are no valid nodes left to test
		if (mapNotTestedNodes.empty())
			break;

		it_NotTestedNodes = mapNotTestedNodes.begin();

		nodeCurrent = it_NotTestedNodes->second;
		graph[nodeCurrent].visited = true; // We only explore a node once

		// Check each of this node's neighbours...
		for (int i = 0; i < graph[nodeCurrent].neighbours.size(); i++)
		{
			int neighNode = graph[nodeCurrent].neighbours[i];

			// ... and only if the neighbour is not visited and is
			// not an obstacle, add it to NotTested List
			if (!graph[neighNode].visited && !graph[neighNode].obstacle)
				mapNotTestedNodes.insert(std::pair<float, int>(graph[neighNode].fGlobalGoal, neighNode));

			// Calculate the neighbours potential lowest parent distance
			float fPossiblyLowerGoal = graph[nodeCurrent].fLocalGoal + Astar::distance(graph, nodeCurrent, neighNode);

			// If choosing to path through this node is a lower distance than what
			// the neighbour currently has set, update the neighbour to use this node
			// as the path source, and set its distance scores as necessary
			if (fPossiblyLowerGoal < graph[neighNode].fLocalGoal)
			{
				graph[neighNode].parent = nodeCurrent;
				graph[neighNode].fLocalGoal = fPossiblyLowerGoal;

				// The best path length to the neighbour being tested has changed, so
				// update the neighbour's score. The heuristic is used to globally bias
				// the path algorithm, so it knows if its getting better or worse. At some
				// point the algo will realise this path is worse and abandon it, and then go
				// and search along the next best path.
				graph[neighNode].fGlobalGoal = graph[neighNode].fGlobalGoal + Astar::heuristic(graph, neighNode, nodeEnd);
			}
		}
	}

	vector<int> optimalPath;
	int curr = nodeEnd;
	optimalPath.emplace_back(curr);

	while (curr != nodeStart)
	{
		curr = graph[curr].parent;
		optimalPath.emplace_back(curr);
	}

	for (int i = 0; i < (int)(optimalPath.size() / 2); i++)
	{
		int temp = optimalPath[i];
		optimalPath[i] = optimalPath[optimalPath.size() - (i + 1)];
		optimalPath[optimalPath.size() - (i + 1)] = temp;
	}
	return optimalPath;
}

void Astar::smoothPath(Graph::Graph &graph, vector<int> &path, vector<int> &newPath, const std::vector<Polygon> &obstacle_list)
{
	list<pair<int, int>> selected;
	selected.push_back(pair<int, int>(0, path.size() - 1));

	while (!selected.empty())
	{
		pair<int, int> segment = selected.back();
		selected.pop_back();

		Point p0(graph[path[segment.first]].x, graph[path[segment.first]].y);
		Point p1(graph[path[segment.second]].x, graph[path[segment.second]].y);

		bool hit = intersect_Global(p0, p1, obstacle_list);

		if (hit)
		{
			int mid = (int)((segment.second + segment.first) / 2);
			if (mid != segment.first && mid != segment.second)
			{
				selected.push_back(pair<int, int>(mid, segment.second));
				selected.push_back(pair<int, int>(segment.first, mid));
			}
			std::cout << "intersection in: " << p0.x << " , " << p0.y << "\tmid: " << graph[path[mid]].x << " , " << graph[path[mid]].y << std::endl;
		}
		else
		{
			newPath.push_back(path[segment.first]);
		}
	}
	newPath.push_back(path[path.size() - 1]);
}