#include "collision_detection.hpp"

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

// Given three colinear points p, q, r, the function checks if
// point q lies on line segment 'pr'
bool onSegment(Point p, Point q, Point r)
{
    if (q.x <= std ::max(p.x, r.x) && q.x >= std ::min(p.x, r.x) &&
        q.y <= std ::max(p.y, r.y) && q.y >= std ::min(p.y, r.y))
        return true;
    return false;
}

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation(Point p, Point q, Point r)
{
    int val = (q.y - p.y) * (r.x - q.x) -
              (q.x - p.x) * (r.y - q.y);

    if (val == 0)
        return 0;             // colinear
    return (val > 0) ? 1 : 2; // clock or counterclock wise
}

// The function that returns true if line segment 'p1q1'
// and 'p2q2' intersect.
bool doIntersect(Point p1, Point q1, Point p2, Point q2)
{
    // Find the four orientations needed for general and
    // special cases
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    // General case
    if (o1 != o2 && o3 != o4)
        return true;

    // Special Cases
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1
    if (o1 == 0 && onSegment(p1, p2, q1))
        return true;

    // p1, q1 and p2 are colinear and q2 lies on segment p1q1
    if (o2 == 0 && onSegment(p1, q2, q1))
        return true;

    // p2, q2 and p1 are colinear and p1 lies on segment p2q2
    if (o3 == 0 && onSegment(p2, p1, q2))
        return true;

    // p2, q2 and q1 are colinear and q1 lies on segment p2q2
    if (o4 == 0 && onSegment(p2, q1, q2))
        return true;

    return false; // Doesn't fall in any of the above cases
}

// Returns true if the point p lies inside the polygon[] with n vertices
bool isInside(Polygon &pol, int n, Point p)
{
    float inf = 100000;
    // There must be at least 3 vertices in polygon[]
    if (n < 3)
        return false;

    // Create a point for line segment from p to infinite
    Point extreme = {inf, p.y};

    // Count intersections of the above line with sides of polygon
    int count = 0, i = 0;
    do
    {
        int next = (i + 1) % n;

        // Check if the line segment from 'p' to 'extreme' intersects
        // with the line segment from 'polygon[i]' to 'polygon[next]'
        if (doIntersect(pol[i], pol[next], p, extreme))
        {
            // If the point 'p' is colinear with line segment 'i-next',
            // then check if it lies on segment. If it lies, return true,
            // otherwise false
            if (orientation(pol[i], p, pol[next]) == 0)
                return onSegment(pol[i], p, pol[next]);

            count++;
        }
        i = next;
    } while (i != 0);

    // Return true if count is odd, false otherwise
    return count % 2 == 1; // Same as (count%2 == 1)
}

bool isInside_Global(Point p, const std::vector<Polygon> &obstacle_list)
{
    bool inside = false;
    for (int i = 0; i < obstacle_list.size(); i++)
    {
        Polygon obstacle = obstacle_list[i];
        int n = sizeof(obstacle) / sizeof(obstacle[1]);
        inside = isInside(obstacle, n, p);
        if (inside == true)
        {
            return true;
        }
    }
    return false;
}
