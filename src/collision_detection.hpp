#include "utils.hpp"

bool collide(Polygon a, Polygon b);
bool isInside_Global(Point p, const std::vector<Polygon> &obstacle_list);
bool isInside(Polygon &pol, int n, Point p);
bool doIntersect(Point p1, Point q1, Point p2, Point q2);
int orientation(Point p, Point q, Point r);
bool onSegment(Point p, Point q, Point r);




