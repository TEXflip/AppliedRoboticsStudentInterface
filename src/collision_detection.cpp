#include "collision_detection.hpp"
#include <cmath>

float *boundingBoxcaracteristics(Polygon &x)
{
    //create bounding  Box around Polygon x
    float *bound_x = new float[4];
    int max = 0;
    int min = 0;

    //select min / max  from Polygon

    //max
    for (int i = 0; i < x.size(); i++)
    {
        if (x[i].x > max)
        {
            max = x[i].x;
        }
    }

    //min
    min = max;
    for (int i = 0; i < x.size(); i++)
    {
        if (x[i].x < min)
        {
            min = x[i].x;
        }
    }
    bound_x[0] = min; //min_x

    //determine width
    bound_x[2] = max - min;

    //select min / max  y from Polygon
    //max
    max = 0;
    for (int i = 0; i < x.size(); i++)
    {
        if (x[i].y > max)
        {
            max = x[i].y;
        }
    }

    //min
    min = max;
    for (int i = 0; i < x.size(); i++)
    {
        if (x[i].y < min)
        {
            min = x[i].y;
        }
    }

    bound_x[1] = min;

    //determine height
    bound_x[3] = max - min;

    return bound_x;
}

bool collide(Polygon a, Polygon b)
{

    /*
    broad fase implementaion
    
    1.Load the two desired Polygons in here
    2.Create a bounding box around them and extract their caracteristics (boundingBoxcaracteristics() )
        a.boundingBoxcaracteristics()
            1.select the highest and lowest x value of the Point vector of the Polygon
            2.calculate the width of the bounding box
            3.select the highest and lowest y value of the Point vector of the Polygon
            4.calculate the height of the bounding box
    3.Run both bounding boxcahracteristics throug the controll 
    4. The controll checks if the projection of both polygons overlap on BOTH axis
        a.if both overlap there is a collision
        b.if one axis has no overlap of the projection there is no collision
    
    */
    bool collided = false;

    float *bound_a = boundingBoxcaracteristics(a);
    float *bound_b = boundingBoxcaracteristics(b);
    /*

    std::cout << "minx a: " << bound_a[0] << std::endl;
    std::cout << "miny a:" << bound_a [1]<< std::endl;
    std::cout << " width a: " << bound_a[2] << std::endl;
    std::cout << " height a: " << bound_a[3] << std::endl;
    
    std::cout << "minx b: " << bound_b[0] << std::endl;
    std::cout << "miny b:" << bound_b [1]<< std::endl;
    std::cout << " width b: " << bound_b[2] << std::endl;
    std::cout << " height b: " << bound_b[3] << std::endl;
*/

    // Colition detection of the two bounding boxes
    if (bound_a[0] < bound_b[0] + bound_b[2] &&
        bound_a[0] + bound_a[2] > bound_b[0] &&
        bound_a[1] < bound_b[1] + bound_b[3] &&
        bound_a[1] + bound_a[3] > bound_b[1])
    {
        collided = true;
    }

    //////////////////////// implement narrow phase////////////////////////////////////7
    /* 
        does work for convex polygons
        if it is concave does not create problems but the result is as if the concave one is a convex one

        1.  define pointers to both Polygons in order to swap them
            this helps to shorten the code because
            we check one shape agains the other and then swap
        2.  create a segment of the first Polygon
        3.  create a normal to this segment
        3.  project all points of the first Polygon on this normal
        4.  project all points of the second Polygon on this normal
        5.  Check if there is an overlap
            a. Yes  -> continue
            b. No   -> stop imediately. No overlap
        6.  repeat for every obstacle and for every segment

    */

    if (collided == true)
    {
        Polygon *poly1 = &a;
        Polygon *poly2 = &b;

        float overlap = INFINITY;

        for (int shape = 0; shape < 2; shape++)
        {
            if (shape == 1)
            {
                poly1 = &a;
                poly2 = &b;
            }

            for (int a = 0; a < poly1->size(); a++)
            {
                int b = (a + 1) % poly1->size(); //create segments out of the points inside a Polygon vector

                //creating a normal to the segment by puting the deltax in the y place and vica versa (Transpose)
                Point axisProj = {-((*poly1)[b].y - (*poly1)[a].y), (*poly1)[b].x - (*poly1)[a].x};

                // Work out min and max  of all points of the poly1, projected on the normal of the segment axisProj
                float min_poly1 = INFINITY, max_poly1 = -INFINITY;
                for (int p = 0; p < poly1->size(); p++)
                {
                    float q = ((*poly1)[p].x * axisProj.x + (*poly1)[p].y * axisProj.y);
                    min_poly1 = std::min(min_poly1, q);
                    max_poly1 = std::max(max_poly1, q);
                }

                // Work out min and max  of all points of the poly2, projected on the normal of the segment axisProj
                float min_poly2 = INFINITY, max_poly2 = -INFINITY;
                for (int p = 0; p < poly2->size(); p++)
                {
                    float q = ((*poly2)[p].x * axisProj.x + (*poly2)[p].y * axisProj.y);
                    min_poly2 = std::min(min_poly2, q);
                    max_poly2 = std::max(max_poly2, q);
                }

                // Calculate actual overlap along projected axis, and store the minimum
                overlap = std::min(std::min(max_poly1, max_poly2) - std::max(min_poly1, min_poly2), overlap);
                //if one of the projected axis has no overlapp there is no collition
                if (!(max_poly2 >= min_poly1 && max_poly1 >= min_poly2))
                    return false;
            }
        }
    }

    return collided;
}
//determins if a point is inside a polygon
bool isInside(const Point &point, const Polygon &polygon)
{
    int i, j, n = polygon.size();
    bool c = false;

    for (i = 0, j = n - 1; i < n; j = i++)
    {
        if (((polygon[i].y >= point.y) != (polygon[j].y >= point.y)) &&
            (point.x <= (polygon[j].x - polygon[i].x) * (point.y - polygon[i].y) / (polygon[j].y - polygon[i].y) + polygon[i].x))
            c = !c;
    }

    return c;
}
//determins if a point is inside any of the arenas polygons
bool isInside_Global(const Point &p, const std::vector<Polygon> &obstacle_list)
{
    for (int i = 0; i < obstacle_list.size(); i++)
    {
        Polygon obstacle = obstacle_list[i];
        // int n = sizeof(obstacle) / sizeof(obstacle[1]);
        if (isInside(p, obstacle))
            return true;
    }
    return false;
}
//determins if 2 segments intersect
bool intersect(const Point &a0, const Point &a1, const Point &b0, const Point &b1)
{
    float dax = a1.x - a0.x;
    float day = a1.y - a0.y;

    float dbx = b0.x - b1.x;
    float dby = b0.y - b1.y;

    float dx = b0.x - a0.x;
    float dy = b0.y - a0.y;

    float det = dax * dby - day * dbx;

    if (fabs(det) < 1e-6)
        return false;

    float r = (dx * dby - dy * dbx) / det;
    float s = (dax * dy - day * dx) / det;

    return !(r < 0 || r > 1 || s < 0 || s > 1);
}
// determins if a segment intersects with a polygon
bool intersectPolygon(const Point &a0, const Point &a1, const Polygon &p)
{
    int i, j, n = p.size();
    for (i = 0, j = n - 1; i < n; j = i++)
        if (intersect(a0, a1, p[i], p[j]))
            return true;
    return false;
}

bool intersect_Global(const Point &a0, const Point &a1, const std::vector<Polygon> &obstacle_list)
{
    for (Polygon p : obstacle_list)
        if (intersectPolygon(a0, a1, p))
            return true;
    return false;
}
// calculates if a circle or an arc intersects with a segment
bool intersectCircleLine(float a, float b, float r, float x1, float y1, float x2, float y2)
{
    /*
        a,b is the cicrcle center
        r is its radius
    */

    float p1 = 2 * x1 * x2;
    float p2 = 2 * y1 * y2;
    float p3 = 2 * a * x1;
    float p4 = 2 * a * x2;
    float p5 = 2 * b * y1;
    float p6 = 2 * b * y2;

    float c1 = x1 * x1 + x2 * x2 - p1 + y1 * y1 + y2 * y2 - p2;
    float c2 = -2 * x2 * x2 + p1 - p3 + p4 - 2 * y2 * y2 + p2 - p5 + p6;
    float c3 = x2 * x2 - p4 + a * a + y2 * y2 - p6 + b * b - r * r;

    float t1;
    float x;
    float y;
    float t2;
    float deltaSq;
    float delta = c2 * c2 - 4 * c1 * c3;
    // std::vector<Point> pts;
    // std::vector<float> t;
    if (delta < 0)
        return false;

    if (delta > 0)
    {
        deltaSq = sqrt(delta);
        t1 = (-c2 + deltaSq) / (2 * c1);
        t2 = (-c2 - deltaSq) / (2 * c1);
    }
    else
    {
        t1 = -c2 / (2 * c1);
        t2 = t1;
    }

    if (t1 >= 0 && t1 <= 1)
    {
        // x = x1 * t1 + x2 * (1 - t1);
        // y = y1 * t1 + y2 * (1 - t1);
        // pts.emplace_back(x, y);
        // t.emplace_back(t);
        // t.emplace_back(t1);
        return true;
    }

    if (t2 >= 0 && t2 <= 1 && t2 != t1)
    {
        // x = x1 * t2 + x2 * (1 - t2);
        // y = y1 * t2 + y2 * (1 - t2);
        // pts.emplace_back(x, y);
        return true;
    }

    return false;
}

bool intersectCircle_Global(float a, float b, float r, const std::vector<Polygon> &obstacle_list)
{
    for (Polygon p : obstacle_list)
    {
        int i, j, n = p.size();
        for (i = 0, j = n - 1; i < n; j = i++)
            if (intersectCircleLine(a, b, r, p[i].x, p[i].y, p[j].x, p[j].y))
                return true;
    }
    return false;
}

std::vector<Polygon> offsetPolygon(const std::vector<Polygon> &polygons, float offset)
{
    float INT_ROUND = 1e8, i = 0;
    std::vector<Polygon> resized;
    resized.resize(polygons.size());
    for (const Polygon &poly : polygons)
    {
        ClipperLib::Path srcPoly;
        ClipperLib::Paths newPoly;

        for (const Point &p : poly)
            srcPoly << ClipperLib::IntPoint((p.x * INT_ROUND), (p.y * INT_ROUND));

        ClipperLib::ClipperOffset co;
        co.ArcTolerance = 0.0015 * INT_ROUND;
        co.AddPath(srcPoly, ClipperLib::jtRound, ClipperLib::etClosedPolygon);
        co.Execute(newPoly, offset * INT_ROUND);

        Polygon myPoly;

        for (const ClipperLib::IntPoint &p : newPoly[0])
            myPoly.emplace_back(p.X / INT_ROUND, p.Y / INT_ROUND);

        resized[i++] = myPoly;
    }
    return resized;
}