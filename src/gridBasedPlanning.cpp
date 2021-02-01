#include "gridBasedPlanning.hpp"
#include "collision_detection.hpp"
#include "debug.hpp"

#include <cmath>
#include <iostream>

void buildGridGraph(Graph::Graph &graph, const std::vector<Polygon> &obstacle_list, float margin, int nVert, int nOriz, float sideLength)
{
    graph.resize(nOriz * nVert);

    for (int i = 0; i < nVert; i++)
        for (int j = 0; j < nOriz; j++)
        {
            Graph::node newNode;
            newNode.x = sideLength * j;
            newNode.y = sideLength * i;
            if (isInside_Global(Point(newNode.x, newNode.y), obstacle_list))
            {
                newNode.obstacle = true;
                newNode.removed = true;
            }
            if (newNode.x < margin || newNode.x > nOriz * sideLength - margin || newNode.y < margin || newNode.y > nVert * sideLength - margin)
            {
                newNode.obstacle = true;
                newNode.removed = true;
            }
            graph[i * nOriz + j] = newNode;
        }

    for (int i = 0; i < nVert; i++)
        for (int j = 0; j < nOriz; j++)
        {
            if (i + 1 != nVert)                                                    //           Connect to:
                graph[i * nOriz + j].neighbours.emplace_back((i + 1) * nOriz + j); //       bottom cell
            if (i - 1 != -1)
                graph[i * nOriz + j].neighbours.emplace_back((i - 1) * nOriz + j); //       upper cell
            if (j + 1 != nOriz)
                graph[i * nOriz + j].neighbours.emplace_back(i * nOriz + j + 1); //         right cell
            if (j - 1 != -1)
                graph[i * nOriz + j].neighbours.emplace_back(i * nOriz + j - 1); //         left cell
            if (i + 1 != nVert && j + 1 != nOriz)
                graph[i * nOriz + j].neighbours.emplace_back((i + 1) * nOriz + j + 1); //   bottom right cell
            if (i + 1 != nVert && j - 1 != -1)
                graph[i * nOriz + j].neighbours.emplace_back((i + 1) * nOriz + j - 1); //   bottom left cell
            if (i - 1 != -1 && j + 1 != nOriz)
                graph[i * nOriz + j].neighbours.emplace_back((i - 1) * nOriz + j + 1); //   up right cell
            if (i - 1 != -1 && j - 1 != -1)
                graph[i * nOriz + j].neighbours.emplace_back((i - 1) * nOriz + j - 1); //   up left cell
        }
}

/*std::vector<Polygon> extend(const std::vector<Polygon> &polygons, float summedLength)
{
    vector<Polygon> resized;
    resized.resize(polygons.size());
    for (int i = 0; i < polygons.size(); i++)
    {
        // calcolo punto centrale del poligono
        Polygon poly = polygons[i];
        float x = 0, y = 0;
        for (Point p : poly)
        {
            x += p.x;
            y += p.y;
        }
        x /= poly.size();
        y /= poly.size();

        // estendo i punti del poligono

        Polygon newP;
        int p, prec, nPoints = poly.size();
        for (p = 0, prec = nPoints - 1; p < nPoints; prec = p++)
        {
            float Xavg = (poly[p].x + poly[prec].x) / 2 - x;
            float Yavg = (poly[p].y + poly[prec].y) / 2 - y;   // vettore tra centro e punto medio del segmento
            float avgLength = sqrt(Xavg * Xavg + Yavg * Yavg); // lunghezza del vettore del punto medio del segmento

            float dpx = (poly[p].x - x);
            float dpy = (poly[p].y - y);                     // vettore tra centro e punto del poligono
            float pointLength = sqrt(dpx * dpx + dpy * dpy); // lunghezza del vettore tra centro e punto del poligono

            float cosAngle = (Xavg * dpx + Yavg * dpy) / (sqrt(dpx * dpx + dpy * dpy) * avgLength); // cos(alpha) = prodotto scalare / prodotto dei moduli
            // float cosAngle = cos( 2 * M_PI /nPoints);
            float extension = (avgLength + summedLength) / cosAngle - pointLength; // lunghezza tra centro e nuovo punto

            float theta = M_PI_2, sign = 1;
            if (dpx != 0)
            {
                theta = atan(dpy / dpx); // angolo sull'asse X del punto
                sign = dpx > 0 ? 1 : -1; // salvo l'orientamento su X
            }
            newP.emplace_back(poly[p].x + cos(theta) * extension * sign, poly[p].y + sin(theta) * extension * sign); //
        }
        resized[i] = newP;
    }
    return resized;
}

*/