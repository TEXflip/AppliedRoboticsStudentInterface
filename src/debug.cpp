#include "debug.hpp"
#include <iostream>

void getImage(cv::Mat &image, float &scale)
{
    image = cv::imread("graph.jpg", cv::IMREAD_COLOR);
    std::ifstream infile;
    infile.open(config_folder + "/scale.txt");
    char s[20];
    char *p_;
    infile >> s;
    scale = (float)std::strtod(s, &p_);
}

void showGraph(const Graph::Graph &graph, bool printPoints)
{
    cv::Mat image;
    float scale;
    getImage(image, scale);

    for (int v = 0; v < graph.size(); v++)
    {
        for (int i = 0; i < graph[v].neighbours.size(); i++)
        {
            int removed = graph[v].removed ? 1 : 0;
            if (!printPoints)
                cv::line(image, cv::Point(scale * graph[v].x, scale * graph[v].y), cv::Point(scale * graph[graph[v].neighbours[i]].x, scale * graph[graph[v].neighbours[i]].y), cv::Scalar(255 * (1 - removed), 131 * removed, 255 * removed), 1, cv::LINE_AA);
            else
                cv::circle(image, cv::Point(scale * graph[v].x, scale * graph[v].y), scale * 0.004, cv::Scalar(255 * (1 - removed), 131 * removed, 255 * removed), cv::FILLED, cv::LINE_AA);
            // std::cout<< "\tx0: " << out[i].p0.x << "\ty0: " << out[i].p0.y << "\tx1: " << out[i].p1.x << "\ty1: " << out[i].p1.y << std::endl;
        }
    }

    cv::imshow("graph", image);
    cv::waitKey(0);
}

void showPolygons(const vector<Polygon> &p)
{
    cv::Mat image;
    float scale;
    getImage(image, scale);

    for (Polygon poly : p)
    {
        for (int i = 0; i < poly.size(); i++)
        {
            int next = (i + 1) % poly.size();
            cv::line(image, cv::Point(scale * poly[i].x, scale * poly[i].y), cv::Point(scale * poly[next].x, scale * poly[next].y), cv::Scalar(255, 0, 255), 1, cv::LINE_AA);
        }
    }

    cv::imshow("polygons", image);
    cv::waitKey(0);
}

void showGraphAndPolygons(const Graph::Graph &graph, const vector<Polygon> &p)
{
    cv::Mat image;
    float scale;
    getImage(image, scale);

    for (Polygon poly : p)
    {
        for (int i = 0; i < poly.size(); i++)
        {
            int next = (i + 1) % poly.size();
            cv::line(image, cv::Point(scale * poly[i].x, scale * poly[i].y), cv::Point(scale * poly[next].x, scale * poly[next].y), cv::Scalar(255, 0, 255), 1, cv::LINE_AA);
        }
    }

    for (int v = 0; v < graph.size(); v++)
    {
        for (int i = 0; i < graph[v].neighbours.size(); i++)
        {
            int removed = graph[v].removed ? 1 : 0;
            //   if (!printPoints)
            cv::line(image, cv::Point(scale * graph[v].x, scale * graph[v].y), cv::Point(scale * graph[graph[v].neighbours[i]].x, scale * graph[graph[v].neighbours[i]].y), cv::Scalar(255 * (1 - removed), 131 * removed, 255 * removed), 1, cv::LINE_AA);
            //   else
            //     cv::circle(image, cv::Point(scale * graph[v].x, scale * graph[v].y), scale*0.004, cv::Scalar(255*(1-removed), 131*removed, 255*removed), cv::FILLED, cv::LINE_AA);
        }
    }

    cv::imshow("graph and polygons", image);
    cv::waitKey(0);
}

void showPath(const Graph::Graph &graph, const vector<int> &path, bool showGraph)
{
    vector<Point> points;
    showPath(graph, path, points, showGraph);
}

void showPath(const Graph::Graph &graph, const vector<int> &path, const vector<Point> &points, bool showGraph)
{   
    cv::Mat image;
    float scale;
    getImage(image, scale);

    if (showGraph)
        for (int v = 0; v < graph.size(); v++)
        {
            for (int i = 0; i < graph[v].neighbours.size(); i++)
            {
                int removed = graph[v].removed ? 1 : 0;
                //   if (!printPoints)
                cv::line(image, cv::Point(scale * graph[v].x, scale * graph[v].y), cv::Point(scale * graph[graph[v].neighbours[i]].x, scale * graph[graph[v].neighbours[i]].y), cv::Scalar(255 * (1 - removed), 131 * removed, 255 * removed), 1, cv::LINE_AA);
                //   else
                //     cv::circle(image, cv::Point(scale * graph[v].x, scale * graph[v].y), scale*0.004, cv::Scalar(255*(1-removed), 131*removed, 255*removed), cv::FILLED, cv::LINE_AA);
            }
        }

    for (int i = 1, j = 0; i < path.size(); j = i++)
    {
        cv::line(image, cv::Point(scale * graph[path[i]].x, scale * graph[path[i]].y), cv::Point(scale * graph[path[j]].x, scale * graph[path[j]].y), cv::Scalar(255, 0, 255), 1, cv::LINE_AA);
        // std::cout << "\t" << path[i] << std::endl;
    }

    for (Point p : points)
    {
        cv::circle(image, cv::Point(scale * p.x, scale * p.y), scale * 0.005, cv::Scalar(0, 0, 0), cv::FILLED, cv::LINE_AA);
    }

    cv::imshow("path", image);
    cv::waitKey(0);
}

void showPath(const Graph::Graph &graph, const vector<int> &path, const vector<Pose> &dubins, bool showGraph)
{
    cv::Mat image;
    float scale;
    getImage(image, scale);

    if (showGraph)
        for (int v = 0; v < graph.size(); v++)
        {
            for (int i = 0; i < graph[v].neighbours.size(); i++)
            {
                int removed = graph[v].removed ? 1 : 0;
                //   if (!printPoints)
                cv::line(image, cv::Point(scale * graph[v].x, scale * graph[v].y), cv::Point(scale * graph[graph[v].neighbours[i]].x, scale * graph[graph[v].neighbours[i]].y), cv::Scalar(255 * (1 - removed), 131 * removed, 255 * removed), 0.2, cv::LINE_AA);
                //   else
                //     cv::circle(image, cv::Point(scale * graph[v].x, scale * graph[v].y), scale*0.004, cv::Scalar(255*(1-removed), 131*removed, 255*removed), cv::FILLED, cv::LINE_AA);
            }
        }

    for (int i = 1, j = 0; i < path.size(); j = i++)
    {
        cv::line(image, cv::Point(scale * graph[path[i]].x, scale * graph[path[i]].y), cv::Point(scale * graph[path[j]].x, scale * graph[path[j]].y), cv::Scalar(255, 0, 255), 2, cv::LINE_AA);
        // std::cout << "\t" << path[i] << std::endl;
    }

    for (int i = 1; i < dubins.size()-1; i++)
    {
        cv::line(image, cv::Point(scale * dubins[i].x, scale * dubins[i].y), cv::Point(scale * dubins[i+1].x, scale * dubins[i+1].y), cv::Scalar(0,131,255), 2, cv::LINE_AA);
    }

    cv::imshow("path", image);
    cv::waitKey(0);
}