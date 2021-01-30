// color_space_rgb.cpp
// Adapted from OpenCV sample: samples/cpp/tutorial_code/ImgProc/Threshold_inRange.cpp

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include "utils.hpp"

using namespace std;
using namespace cv;

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
    /*
    t = roots([ c1, c2, c3 ]);
    % t = t(find(imag(t) == 0));

    */
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

int main()
{

    /*
x: 0.16	y: 0.22
x: 0.8	y: 0.2
x: 1.3	y: 0.2
x: 1.32	y: 0.46
x: 1.32	y: 0.76
x: 1.32	y: 1.02

     */

    cout << "do intersect circel and line? " << intersectCircleLine(0, 0, 1, 0, 0, 4, 4) << endl;

    vector<Point2d> p;
    vector<Pose> c;

    p.emplace_back(0.16, 0.22);
    p.emplace_back(0.8, 0.2);
    p.emplace_back(1.3, 0.2);
    p.emplace_back(1.32, 0.46);
    p.emplace_back(1.32, 0.76);
    p.emplace_back(1.32, 1.02);

    float x1 = 0.8;
    float x2 = 1.3;
    float y1 = 0.2;
    float y2 = 0.2;
    float b;  //cross product
    int sign; // sign del cross product
    float av; // angle between exit direction and x - axis
    float a;  //angle between the 2 segments (entry and exit direction)
    float ah; // angle for the robot to assume in the certain location (theta f for the dubins)
    float angle;
    float mag;
    float dot;
    Pose pose;

    for (int i = 0; i < p.size() - 2; i++)
    {
        x1 = p[i + 1].x - p[i].x;
        x2 = p[i + 2].x - p[i + 1].x;
        y1 = p[i + 1].y - p[i].y;
        y2 = p[i + 2].y - p[i + 1].y;

        dot = x1 * x2 + y1 * y2; // dot product between [x1, y1] and [x2, y2]
        mag = (sqrt((x1 * x1) + (y1 * y1))) * (sqrt((x2 * x2) + (y2 * y2)));

        b = ((x1 * y2) - (x2 * y1));       //cros product
        sign = b > 0 ? 1 : -1;             // sign of cross product
        av = atan2(y2, x2);                //angle between exit direction and x -axis
        a = M_PI - acos((dot) / (mag));    // angle between entrz and exit direction
        ah = sign * a / 2 + av + M_PI / 2; // theta f
        if (sign > 0)
        {
            ah = ah - M_PI;
        }
        //cout << "angle " << ah * 180/M_PI  << " x " << p[i+1].x << " y " << p[i+1].y<<endl;
    }
}