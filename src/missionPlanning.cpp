#include "missionPlanning.hpp"
#include "DubinsCurves.hpp"
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
public:
    MissionPlanning(float bonusTime, const float x, const float y, vector<Polygon> &obstacle_list, vector<pair<int, Polygon>> &victim_list, const Polygon &gate){
        this->bonusTime = bonusTime;
        this->start = Point(x, y);
        this->obstacle_list = obstacle_list;
        for (int i = 0; i < victim_list.size(); i++)
        {
            this->victim_list.push_back(victim_list[i].second);
        }
        this->gate = gate;
    }

    struct decision
    {
        float reward;
        bool isGate;
        float x, y;
    };

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

    void buildDecisionTree(Graph::Graph graph, int nVert, int nOriz)
    {
        vector<decision> decisions;
        initDecisions(decisions);

        int N = victim_list.size() + 2;
        float costs[N][N];


    }
};