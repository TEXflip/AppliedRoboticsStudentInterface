#include "missionPlanning.hpp"
#include "DubinsCurves.hpp"
#include "Astar_pathplanning.hpp"

#include <list>
#include <tuple>
#include <iostream>

Point MissionPlanning::avgPoint(const Polygon &polygon)
{
    float avgX = 0, avgY = 0;
    for (int i = 0; i < polygon.size(); i++)
    {
        avgX += polygon[i].x;
        avgY += polygon[i].y;
    }
    avgX /= polygon.size();
    avgY /= polygon.size();
    return Point(avgX, avgY);
}

float MissionPlanning::pathLength(Graph::Graph &graph, vector<int> path)
{
    float x1, y1, x2, y2, dx, dy, length = 0;
    for (int i = 1; i < path.size(); i++)
    {
        x1 = graph[path[i - 1]].x;
        y1 = graph[path[i - 1]].y;
        x2 = graph[path[i]].x;
        y2 = graph[path[i]].y;
        dx = x2 - x1;
        dy = y2 - y1;
        length += sqrt(dx * dx + dy * dy);
    }
    return length;
}

pair<float, vector<int>> MissionPlanning::pickDecision(float **costs, vector<decision> &decisions, set<int> remaining, float currCost, int curr)
{
    // cout << curr << "\t" << remaining.size() << endl;
    if (decisions[curr].isGate){
        vector<int> bestPath{curr};
        return pair<float, vector<int>>(currCost, bestPath);
    }

    remaining.erase(curr);

    float min = INFINITY;
    pair<float, vector<int>> currDec;
    vector<int> bestPath;

    for (int i : remaining)
    {
        float cost = currCost + costs[curr][i] - decisions[i].reward;
        currDec = pickDecision(costs, decisions, remaining, cost, i);

        if (min > currDec.first)
        {
            min = currDec.first;
            bestPath = currDec.second;
        }
    }

    bestPath.push_back(curr);

    return pair<float, vector<int>>(min, bestPath);
}

MissionPlanning::MissionPlanning(float bonusTime, const float x, const float y, vector<Polygon> &obstacle_list, const vector<pair<int, Polygon>> &victim_list, const Polygon &gate)
{
    this->bonusTime = bonusTime;
    this->start = Point(x, y);
    this->obstacle_list = obstacle_list;
    this->victim_list = victim_list;
    this->gate = gate;
}

void MissionPlanning::initDecisions(vector<decision> &decisions)
{
    decision d;
    d.reward = 0;
    d.isGate = false;
    d.x = this->start.x;
    d.y = this->start.y;
    decisions.push_back(d);
    for (pair<int, Polygon> p : this->victim_list)
    {
        d.reward = this->bonusTime;
        d.isGate = false;
        Point avg = avgPoint(p.second);
        d.x = avg.x;
        d.y = avg.y;
        decisions.push_back(d);
    }
    d.reward = 0;
    d.isGate = true;
    Point avg = avgPoint(this->gate);
    d.x = avg.x;
    d.y = avg.y;
    decisions.push_back(d);
}

vector<Pose> MissionPlanning::buildDecisionPath(Graph::Graph &graph, int nVert, int nOriz, float sideLength)
{
    vector<decision> decisions;
    initDecisions(decisions);

    int N = decisions.size();
    float **costs = new float *[N];
    for (int i = 0; i < N; ++i)
        costs[i] = new float[N];

    auto toGraphCoord = [sideLength](float coord) {
        return (int)((coord + sideLength * 0.5) / sideLength);
    };

    vector<int> path_segment, smoothed_path;
    int x1, y1, x2, y2;
    float length;

    for (int i = 0; i < N - 1; i++)
    {
        for (int j = (i + 1); j < N; j++)
        {
            x1 = toGraphCoord(decisions[i].x);
            y1 = toGraphCoord(decisions[i].y);
            x2 = toGraphCoord(decisions[j].x);
            y2 = toGraphCoord(decisions[j].y);
            path_segment = Astar::Solve_AStar(graph, (y1 * nOriz + x1), (y2 * nOriz + x2));
            Astar::smoothPath(graph, path_segment, smoothed_path, this->obstacle_list);
            length = this->pathLength(graph, smoothed_path);
            path_segment.clear();
            smoothed_path.clear();

            costs[i][j] = costs[j][i] = length / this->velocity;

            // cout << "\t" << costs[i][j];
        }
        // cout<<endl;
    }

    set<int> remaining;
    for (int i = 0; i < N; i++)
        remaining.insert(i);

    pair<float, vector<int>> best = this->pickDecision(costs, decisions, remaining, 0, 0);

    // cout << "min Cost: " << best.first << endl;

    // for (int i = 1; i < best.second.size()-1; i++)
    // {
    //     cout << best.second[i] << endl;
    // }

    vector<Pose> finalPath;
    finalPath.resize(best.second.size());
    int j = best.second.size()-1;
    for (int i : best.second)
    {
        Pose p;
        p.x = decisions[i].x;
        p.y = decisions[i].y;
        p.theta = 0;
        finalPath[j--] = p;
    }

    return finalPath;
}
