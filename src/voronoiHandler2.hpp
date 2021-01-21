#include "boost/polygon/voronoi.hpp"
#include "boost/polygon/polygon.hpp"
#include "boost/polygon/isotropy.hpp"
// #include "boost/polygon/point_concept.hpp"
// #include "boost/polygon/segment_concept.hpp"
#include "boost/polygon/rectangle_concept.hpp"

#include <vector>
#include <stack>

// struct Point
// {
//     float x, y;

//     Point(float x, float y) : x(x), y(y)
//     {
//     }

//     Point() : Point(0, 0)
//     {
//     }
// };

struct Segment
{
    Point p0;
    Point p1;
    Segment(double x1, double y1, double x2, double y2) : p0(x1, y1), p1(x2, y2) {}
    Segment(Point p_0, Point p_1) : p0(p_0), p1(p_1) {}
};

using boost::polygon::voronoi_diagram;
typedef voronoi_diagram<double>::cell_type cell;

namespace boost
{
    namespace polygon
    {

        template <>
        struct geometry_concept<Point>
        {
            typedef point_concept type;
        };

        template <>
        struct point_traits<Point>
        {
            typedef int coordinate_type;

            static inline coordinate_type get(
                const Point &point, orientation_2d orient)
            {
                return (orient == HORIZONTAL) ? point.x : point.y;
            }
        };

        template <>
        struct geometry_concept<Segment>
        {
            typedef segment_concept type;
        };

        template <typename CT>
        class VoronoiHandler
        {
        private:
            static std::vector<Segment> segment_data_;
            static CT parabola_y(CT x, CT a, CT b)
            {
                return ((x - a) * (x - a) + b * b) / (b + b);
            }
            template <class InCT1, class InCT2,
                      template <class> class Point,
                      template <class> class Segment>
            static
                typename enable_if<
                    typename gtl_and<
                        typename gtl_if<
                            typename is_point_concept<
                                typename geometry_concept<Point<InCT1>>::type>::type>::type,
                        typename gtl_if<
                            typename is_segment_concept<
                                typename geometry_concept<Segment<InCT2>>::type>::type>::type>::type,
                    void>::type
                discretize(
                    const Point<InCT1> &point,
                    const Segment<InCT2> &segment,
                    const CT max_dist,
                    std::vector<Point<CT>> *discretization)
            {
                // Apply the linear transformation to move start point of the segment to
                // the point with coordinates (0, 0) and the direction of the segment to
                // coincide the positive direction of the x-axis.
                CT segm_vec_x = cast(x(high(segment))) - cast(x(low(segment)));
                CT segm_vec_y = cast(y(high(segment))) - cast(y(low(segment)));
                CT sqr_segment_length = segm_vec_x * segm_vec_x + segm_vec_y * segm_vec_y;

                // Compute x-coordinates of the endpoints of the edge
                // in the transformed space.
                CT projection_start = sqr_segment_length *
                                      get_point_projection((*discretization)[0], segment);
                CT projection_end = sqr_segment_length *
                                    get_point_projection((*discretization)[1], segment);

                // Compute parabola parameters in the transformed space.
                // Parabola has next representation:
                // f(x) = ((x-rot_x)^2 + rot_y^2) / (2.0*rot_y).
                CT point_vec_x = cast(x(point)) - cast(x(low(segment)));
                CT point_vec_y = cast(y(point)) - cast(y(low(segment)));
                CT rot_x = segm_vec_x * point_vec_x + segm_vec_y * point_vec_y;
                CT rot_y = segm_vec_x * point_vec_y - segm_vec_y * point_vec_x;

                // Save the last point.
                Point<CT> last_point = (*discretization)[1];
                discretization->pop_back();

                // Use stack to avoid recursion.
                std::stack<CT> point_stack;
                point_stack.push(projection_end);
                CT cur_x = projection_start;
                CT cur_y = parabola_y(cur_x, rot_x, rot_y);

                // Adjust max_dist parameter in the transformed space.
                const CT max_dist_transformed = max_dist * max_dist * sqr_segment_length;
                while (!point_stack.empty())
                {
                    CT new_x = point_stack.top();
                    CT new_y = parabola_y(new_x, rot_x, rot_y);

                    // Compute coordinates of the point of the parabola that is
                    // furthest from the current line segment.
                    CT mid_x = (new_y - cur_y) / (new_x - cur_x) * rot_y + rot_x;
                    CT mid_y = parabola_y(mid_x, rot_x, rot_y);

                    // Compute maximum distance between the given parabolic arc
                    // and line segment that discretize it.
                    CT dist = (new_y - cur_y) * (mid_x - cur_x) -
                              (new_x - cur_x) * (mid_y - cur_y);
                    dist = dist * dist / ((new_y - cur_y) * (new_y - cur_y) + (new_x - cur_x) * (new_x - cur_x));
                    if (dist <= max_dist_transformed)
                    {
                        // Distance between parabola and line segment is less than max_dist.
                        point_stack.pop();
                        CT inter_x = (segm_vec_x * new_x - segm_vec_y * new_y) /
                                         sqr_segment_length +
                                     cast(x(low(segment)));
                        CT inter_y = (segm_vec_x * new_y + segm_vec_y * new_x) /
                                         sqr_segment_length +
                                     cast(y(low(segment)));
                        discretization->push_back(Point<CT>(inter_x, inter_y));
                        cur_x = new_x;
                        cur_y = new_y;
                    }
                    else
                    {
                        point_stack.push(mid_x);
                    }
                }

                // Update last point.
                discretization->back() = last_point;
            }

            template <class InCT,
                      template <class> class Point,
                      template <class> class Segment>
            static
                typename enable_if<
                    typename gtl_and<
                        typename gtl_if<
                            typename is_point_concept<
                                typename geometry_concept<Point<int>>::type>::type>::type,
                        typename gtl_if<
                            typename is_segment_concept<
                                typename geometry_concept<Segment<long>>::type>::type>::type>::type,
                    CT>::type
                get_point_projection(
                    const Point<CT> &point, const Segment<InCT> &segment)
            {
                CT segment_vec_x = cast(x(high(segment))) - cast(x(low(segment)));
                CT segment_vec_y = cast(y(high(segment))) - cast(y(low(segment)));
                CT point_vec_x = x(point) - cast(x(low(segment)));
                CT point_vec_y = y(point) - cast(y(low(segment)));
                CT sqr_segment_length =
                    segment_vec_x * segment_vec_x + segment_vec_y * segment_vec_y;
                CT vec_dot = segment_vec_x * point_vec_x + segment_vec_y * point_vec_y;
                return vec_dot / sqr_segment_length;
            }

            template <typename InCT>
            static CT cast(const InCT &value)
            {
                return static_cast<CT>(value);
            }

            static Point retrieve_point(const cell &cell)
            {
                cell::source_index_type index = cell.source_index();
                cell::source_category_type category = cell.source_category();
                // if (category == SOURCE_CATEGORY_SINGLE_POINT) {
                //   return point_data_[index];
                // }
                // index -= point_data_.size();
                if (category == SOURCE_CATEGORY_SEGMENT_START_POINT)
                {
                    return low(segment_data_[index]);
                }
                else
                {
                    return high(segment_data_[index]);
                }
            }
            static Segment retrieve_segment(const cell &cell)
            {
                cell::source_index_type index = cell.source_index() /* - point_data_.size()*/;
                return segment_data_[index];
            }

        public:
            static void buildVoronoi(std::vector<Segment> &segments, std::vector<Segment> &out, double discretizationSize)
            {
                segment_data_ = segments;
                voronoi_diagram<double> vd;
                construct_voronoi(segments.begin(), segments.end(), &vd);

                for (voronoi_diagram<double>::const_edge_iterator it = vd.edges().begin(); it != vd.edges().end(); ++it)
                {
                    if (!it->is_primary() || !it->is_finite()) // ingora edges secondari e infiniti
                        continue;

                    std::vector<Point> samples;
                    Point vertex0(it->vertex0()->x(), it->vertex0()->y());
                    samples.push_back(vertex0);
                    Point vertex1(it->vertex1()->x(), it->vertex1()->y());
                    samples.push_back(vertex1);

                    double max_dist = 0.02;
                    Point point = it->cell()->contains_point() ? retrieve_point(*it->cell()) : retrieve_point(*it->twin()->cell());
                    Segment segment = it->cell()->contains_point() ? retrieve_segment(*it->twin()->cell()) : retrieve_segment(*it->cell());
                    discretize(point, segment, max_dist, samples);

                    for (int i = 0; i < samples.size() - 1; i++)
                        out.emplace_back(Segment(samples[i], samples[i + 1]));
                }
            }
        };
    } // namespace polygon
} // namespace boost