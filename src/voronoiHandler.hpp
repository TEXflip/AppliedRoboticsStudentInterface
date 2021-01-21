#include <stack>
#include <vector>

#include "boost/polygon/isotropy.hpp"
#include "boost/polygon/point_concept.hpp"
#include "boost/polygon/segment_concept.hpp"
#include "boost/polygon/rectangle_concept.hpp"

#include "student_image_elab_interface.hpp"

struct Segment
{
    Point p0;
    Point p1;
    Segment(float x1, float y1, float x2, float y2) : p0(x1, y1), p1(x2, y2) {}
    Segment(Point p_0, Point p_1) : p0(p_0), p1(p_1) {}
};

namespace boost
{
    namespace polygon
    {
        template <typename CT>
        class VoronoiHelper
        {
        public:
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

        private:
            // Compute y(x) = ((x - a) * (x - a) + b * b) / (2 * b).
            static CT parabola_y(CT x, CT a, CT b)
            {
                return ((x - a) * (x - a) + b * b) / (b + b);
            }

            // Get normalized length of the distance between:
            //   1) point projection onto the segment
            //   2) start point of the segment
            // Return this length divided by the segment length. This is made to avoid
            // sqrt computation during transformation from the initial space to the
            // transformed one and vice versa. The assumption is made that projection of
            // the point lies between the start-point and endpoint of the segment.
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
        };
    } // namespace polygon
} // namespace boost

#include "boost/polygon/polygon.hpp"
#include "boost/polygon/voronoi.hpp"

using namespace boost::polygon;
// using boost::polygon::voronoi_diagram;



class VoronoiHandler
{
public:
    typedef float coordinate_type;
    typedef point_data<coordinate_type> point_type;
    typedef segment_data<coordinate_type> segment_type;
    typedef voronoi_diagram<double>::cell_type cell;
    static void buildVoronoi(std::vector<segment_type> &segments, std::vector<Segment> &out, double discretizationSize);

private:
    static std::vector<segment_type> segment_data_;
    static point_type retrieve_point(const cell &cell);
    static segment_type retrieve_segment(const cell &cell);
};