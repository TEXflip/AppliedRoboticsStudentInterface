#pragma once
#include <vector> 
#include "utils.hpp" 

using namespace std;

struct CustomPose{
    float x, y, theta;
    std::vector<float> thetas;
};