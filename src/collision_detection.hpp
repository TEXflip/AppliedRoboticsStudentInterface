#pragma once
#include "utils.hpp"
#include "clipper/clipper.hpp"
#include <vector>

bool collide(Polygon a, Polygon b);
bool isInside_Global(const Point& p, const std::vector<Polygon> &obstacle_list);
bool isInside(const Point& point, const Polygon& polygon);
bool intersect(const Point& a0, const Point& a1, const Point& b0, const Point& b1);
bool intersectPolygon(const Point& a0, const Point& a1, const Polygon& p);
bool intersect_Global(const Point& a0, const Point& a1, const std::vector<Polygon> &obstacle_list);
bool intersectCircle_Global(float a, float b, float r, const std::vector<Polygon> &obstacle_list);
bool intersectCircleLine(float a, float b,float r, float x1, float x2, float y1, float y2);
std::vector<Polygon> offsetPolygon(const std::vector<Polygon> &polygons, float offset);
