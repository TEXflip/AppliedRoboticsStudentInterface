#include "missionPlanning.hpp"
#include "DubinsCurves.hpp"
#include "Astar_pathplanning.hpp"
#include "graph.hpp"

#include <list>
#include <tuple>

class MissionPlanning
{
private:
    float bonusTime;
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

    Point avgPoint(const Polygon &polygon)
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

    float pathLength(Graph::Graph &graph, vector<int> path)
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

    void pickDecision(float** costs, vector<decision>& decisions, vector<int> remaining, float currCost)
    {
        // if (remaining.empty())
        //     return;
        // else{
        //     int curr = remaining.back();
        //     remaining.pop_back();

        //     for (int i = 0; i < remaining.size(); i++)
        //     {
        //         float cost = currCost + costs[curr][remaining[i]];

        //     }
            
        // }
    }

public:
    MissionPlanning(float bonusTime, const float x, const float y, vector<Polygon> &obstacle_list, vector<pair<int, Polygon>> &victim_list, const Polygon &gate)
    {
        this->bonusTime = bonusTime;
        this->start = Point(x, y);
        this->obstacle_list = obstacle_list;
        for (int i = 0; i < victim_list.size(); i++)
        {
            this->victim_list.push_back(victim_list[i].second);
        }
        this->gate = gate;
    }

    void initDecisions(vector<decision> &decisions)
    {
        decision d;
        d.reward = 0;
        d.isGate = false;
        d.x = this->start.x;
        d.y = this->start.y;
        decisions.push_back(d);
        for (Polygon p : this->victim_list)
        {
            d.reward = this->bonusTime;
            d.isGate = false;
            Point avg = avgPoint(p);
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

    void buildDecisionTree(Graph::Graph &graph, int nVert, int nOriz, float sideLength)
    {
        vector<decision> decisions;
        initDecisions(decisions);

        int N = decisions.size();
        float costs[N][N];

        auto toGraphCoord = [sideLength](float coord) {
            return (int)((coord + sideLength * 0.5) / sideLength);
        };

        vector<int> path_segment, smoothed_path;
        int x1, y1, x2, y2;
        float length;

        for (int i = 0; i < N - 1; i++)
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

                costs[i][j] = costs[j][i] = length;
            }

        // list<int> decisionStack;
        // vector<int> currDecision, best;
        // decisionStack.push_back(0);

        // while(!decisionStack.empty()){

        //     int curr = decisionStack.back();
        //     decisionStack.pop_back();

        //     for (decision d : decisions)
        //     {

        //     }

        // }
    }
};