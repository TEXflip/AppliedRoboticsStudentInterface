#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>
#include <string>
#include <fstream>
#include <vector>
#include "graph.hpp"
#include "utils.hpp"

using namespace std;

const string config_folder = "/tmp";

void getImage(cv::Mat& image, float& scale);

void showGraph(const Graph::Graph& graph, bool printPoints = false);

void showPolygons(const vector<Polygon>& p);

void showGraphAndPolygons(const Graph::Graph& graph, const vector<Polygon>& p);