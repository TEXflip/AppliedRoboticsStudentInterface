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

bool isInside(const Point& point,const Polygon& polygon)
{
    int i, j, nvert = polygon.size();
    bool c = false;

    for (i = 0, j = nvert - 1; i < nvert; j = i++)
    {
        if (((polygon[i].y >= point.y) != (polygon[j].y >= point.y)) &&
            (point.x <= (polygon[j].x - polygon[i].x) * (point.y - polygon[i].y) / (polygon[j].y - polygon[i].y) + polygon[i].x))
            c = !c;
    }

    return c;
}

bool isInside_Global(const Point& p, const std::vector<Polygon> &obstacle_list)
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
