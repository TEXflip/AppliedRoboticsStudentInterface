#include "debug.hpp"

void getImage(cv::Mat& image, float& scale){
    image = cv::imread("graph.jpg", cv::IMREAD_COLOR);
    std::ifstream infile;
    infile.open(config_folder + "/scale.txt");
    char s[20];
    char *p_;
    infile >> s;
    scale = (float) std::strtod(s, &p_);
}

void showGraph(const Graph::Graph& graph, bool printPoints){
    cv::Mat image;
    float scale;
    getImage(image, scale);

    for (int v = 0; v < graph.size(); v++)
    {
        for (int i = 0; i < graph[v].neighbours.size(); i++)
        {
          int removed = graph[v].removed ? 1 : 0;
          if (!printPoints)
            cv::line(image, cv::Point(scale * graph[v].x, scale * graph[v].y), cv::Point(scale * graph[graph[v].neighbours[i]].x, scale * graph[graph[v].neighbours[i]].y), cv::Scalar(255*(1-removed), 131*removed, 255*removed), 1, cv::LINE_AA);
          else
            cv::circle(image, cv::Point(scale * graph[v].x, scale * graph[v].y), scale*0.004, cv::Scalar(255*(1-removed), 131*removed, 255*removed), cv::FILLED, cv::LINE_AA);
          // std::cout<< "\tx0: " << out[i].p0.x << "\ty0: " << out[i].p0.y << "\tx1: " << out[i].p1.x << "\ty1: " << out[i].p1.y << std::endl;
        }
    }

    cv::imshow("graph", image);
    cv::waitKey(0);
}

void showPolygons(const vector<Polygon>& p){
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