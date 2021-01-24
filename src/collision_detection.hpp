#pragma once
#include "utils.hpp"

bool collide(Polygon a, Polygon b);
bool isInside_Global(const Point& p, const std::vector<Polygon> &obstacle_list);
bool isInside(const Point& point, const Polygon& polygon);
