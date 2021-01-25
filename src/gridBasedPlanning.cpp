#include "gridBasedPlanning.hpp"
#include "collision_detection.hpp"
#include "debug.hpp"

#include <cmath>

#define footprint_width 0.09

void buildGridGraph(Graph::Graph &graph, const std::vector<Polygon> &obstacle_list, const Polygon &borders, float sideLength)
{
    int nOriz = max(max(borders[0].x, borders[1].x), max(borders[2].x, borders[3].x)) / sideLength;
    int nVert = max(max(borders[0].y, borders[1].y), max(borders[2].y, borders[3].y)) / sideLength;
    graph.resize(nOriz * nVert);

    vector<Polygon> rescaled_ob_list = scale(obstacle_list, footprint_width / 1.5);

    for (int i = 0; i < nVert; i++)
        for (int j = 0; j < nOriz; j++)
        {
            Graph::node newNode;
            newNode.x = sideLength * j;
            newNode.y = sideLength * i;
            if (isInside_Global(Point(newNode.x, newNode.y), rescaled_ob_list)){
                newNode.obstacle = true;
                newNode.removed = true;
            }
            graph[i * nOriz + j] = newNode;
        }

    for (int i = 0; i < nVert; i++)
        for (int j = 0; j < nOriz; j++)
        {
            if (i + 1 != nVert)
                graph[i * nOriz + j].neighbours.emplace_back((i + 1) * nOriz + j);
            if (i - 1 != -1)
                graph[i * nOriz + j].neighbours.emplace_back((i - 1) * nOriz + j);
            if (j + 1 != nOriz)
                graph[i * nOriz + j].neighbours.emplace_back(i * nOriz + j + 1);
            if (j - 1 != -1)
                graph[i * nOriz + j].neighbours.emplace_back(i * nOriz + j - 1);
        }
    
    showGraphAndPolygons(graph, rescaled_ob_list);
}

std::vector<Polygon> scale(const std::vector<Polygon> &polygons, float scale)
{
    vector<Polygon> resized;
    resized.resize(polygons.size());
    for (int i = 0; i < polygons.size(); i++)
    {
        Polygon poly = polygons[i];
        float x = 0, y = 0;
        for (Point p : poly)
        {
            x += p.x;
            y += p.y;
        }
        x /= poly.size();
        y /= poly.size();

        Polygon newP;

        for (auto p : poly)
        {
            // float diffX = p.x - x;
            // float diffY = p.y - y;
            float theta = atan((p.y - y) / (p.x - x));
            float sign = (p.x - x) > 0 ? 1 : -1;
            newP.emplace_back(p.x + cos(theta) * scale * sign, p.y + sin(theta) * scale * sign);
        }
        resized[i] = newP;
    }
    return resized;
}