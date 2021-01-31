#pragma once
#include <vector>
#include <set>
#include "utils.hpp"
#include "graph.hpp"

using namespace std;
class MissionPlanning
{
private:
    float bonusTime;
    float velocity = 1;
    vector<Polygon> obstacle_list;
    vector<Polygon> victim_list;
    Polygon gate;
    Point start;

    struct decision
    {
        float reward;
        bool isGate;
        float x, y;
    };
    Point avgPoint(const Polygon &polygon);
    float pathLength(Graph::Graph &graph, vector<int> path);
    pair<float, vector<int>> pickDecision(float **costs, vector<decision> &decisions, set<int> remaining, float currCost, int next);
    void initDecisions(vector<decision> &decisions);

public:
    explicit MissionPlanning(float bonusTime, const float x, const float y, vector<Polygon> &obstacle_list,const vector<pair<int, Polygon>> &victim_list, const Polygon &gate);
    void buildDecisionTree(Graph::Graph &graph, int nVert, int nOriz, float sideLength);
};