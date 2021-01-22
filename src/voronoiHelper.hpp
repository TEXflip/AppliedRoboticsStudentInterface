#include <vector>
#include <iostream>
#include "student_image_elab_interface.hpp"
#include <boost/polygon/voronoi.hpp>
using boost::polygon::high;
using boost::polygon::low;
using boost::polygon::voronoi_builder;
using boost::polygon::voronoi_diagram;
using boost::polygon::x;
using boost::polygon::y;

struct PointInt {
  float x;
  float y;
  PointInt(float x, float y) : x(x), y(y) {}
};

struct SegmentInt
{
    PointInt p0;
    PointInt p1;
    SegmentInt(float x1, float y1, float x2, float y2) : p0(x1, y1), p1(x2, y2) {}
    SegmentInt(PointInt p0_, PointInt p1_) : p0(p0_), p1(p1_) {}
};

struct Segment
{
    Point p0;
    Point p1;
    Segment(float x1, float y1, float x2, float y2) : p0(x1, y1), p1(x2, y2) {}
    Segment(Point p0_, Point p1_) : p0(p0_), p1(p1_) {}
};

namespace boost
{
    namespace polygon
    {

        template <>
        struct geometry_concept<PointInt>
        {
            typedef point_concept type;
        };

        template <>
        struct point_traits<PointInt>
        {
            typedef int coordinate_type;

            static inline coordinate_type get(
                const PointInt &point, orientation_2d orient)
            {
                return (orient == HORIZONTAL) ? point.x : point.y;
            }
        };

        template <>
        struct geometry_concept<SegmentInt>
        {
            typedef segment_concept type;
        };

        template <>
        struct segment_traits<SegmentInt>
        {
            typedef int coordinate_type;
            typedef PointInt point_type;

            static inline point_type get(const SegmentInt &segment, direction_1d dir)
            {
                return dir.to_int() ? segment.p1 : segment.p0;
            }
        };
    } // namespace polygon
} // namespace boost

class VoronoiHelper
{
public:
    static void buildVoronoi(std::vector<Point> &points, std::vector<Segment> &segments, std::vector<Segment> &out)
    {
        float scale = 10000000;
        std::vector<PointInt> pointInts;
        std::vector<SegmentInt> segmentInts;
        for (Point p : points)
        {
            pointInts.emplace_back((int)(p.x*scale),(int)(p.y*scale));
            // std::cout<< "\tx " << (int)(p.x*scale) << "\ty " << (int)(p.y*scale) << std::endl;
        }
        for (Segment s : segments)
        {
            segmentInts.emplace_back((int)(s.p0.x*scale),(int)(s.p0.y*scale),(int)(s.p1.x*scale),(int)(s.p1.y*scale));
        }
        

        voronoi_diagram<double> vd;
        construct_voronoi(pointInts.begin(), pointInts.end(), &vd);

        std::cout << "Edges: " << vd.num_edges() << std::endl;

        for (voronoi_diagram<double>::const_edge_iterator it = vd.edges().begin();
             it != vd.edges().end(); ++it)
        {
            if (it->is_primary() && it->is_finite())
                out.emplace_back(Segment(it->vertex0()->x()/scale, it->vertex0()->y()/scale, it->vertex1()->x()/scale, it->vertex1()->y()/scale));
        }
    }
};