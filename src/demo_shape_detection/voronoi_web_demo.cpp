
// Boost.Polygon library voronoi_basic_tutorial.cpp file

//          Copyright Andrii Sydorchuk 2010-2012.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

// See http://www.boost.org for updates, documentation, and revision history.

#include <cstdio>
#include <vector>
#include <stack>

#include "boost/polygon/voronoi.hpp"
using boost::polygon::high;
using boost::polygon::low;
using boost::polygon::voronoi_builder;
using boost::polygon::voronoi_diagram;
using boost::polygon::x;
using boost::polygon::y;

//#include "voronoi_visual_utils.hpp"

struct Point
{
  int a;
  int b;
  Point(int x, int y) : a(x), b(y) {}
};

struct Segment
{
  Point p0;
  Point p1;
  Segment(int x1, int y1, int x2, int y2) : p0(x1, y1), p1(x2, y2) {}
};

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
      typedef float coordinate_type;

      static inline coordinate_type get(
          const Point &point, orientation_2d orient)
      {
        return (orient == HORIZONTAL) ? point.a : point.b;
      }
    };

    template <>
    struct geometry_concept<Segment>
    {
      typedef segment_concept type;
    };

    template <>
    struct segment_traits<Segment>
    {
      typedef float coordinate_type;
      typedef Point point_type;

      static inline point_type get(const Segment &segment, direction_1d dir)
      {
        return dir.to_int() ? segment.p1 : segment.p0;
      }
    };

    // class VoronoiHelper
    // {
    // public:
    //   static void discretize(
    //       const Point &point,
    //       const Segment &segment,
    //       const float max_dist,
    //       std::vector<Point> *discretization)
    //   {
    //     // Apply the linear transformation to move start point of the segment to
    //     // the point with coordinates (0, 0) and the direction of the segment to
    //     // coincide the positive direction of the x-axis.
    //     float segm_vec_x = cast(x(high(segment))) - cast(x(low(segment)));
    //     float segm_vec_y = cast(y(high(segment))) - cast(y(low(segment)));
    //     float sqr_segment_length = segm_vec_x * segm_vec_x + segm_vec_y * segm_vec_y;

    //     // Compute x-coordinates of the endpoints of the edge
    //     // in the transformed space.
    //     float projection_start = sqr_segment_length *
    //                              get_point_projection((*discretization)[0], segment);
    //     float projection_end = sqr_segment_length *
    //                            get_point_projection((*discretization)[1], segment);

    //     // Compute parabola parameters in the transformed space.
    //     // Parabola has next representation:
    //     // f(x) = ((x-rot_x)^2 + rot_y^2) / (2.0*rot_y).
    //     float point_vec_x = cast(x(point)) - cast(x(low(segment)));
    //     float point_vec_y = cast(y(point)) - cast(y(low(segment)));
    //     float rot_x = segm_vec_x * point_vec_x + segm_vec_y * point_vec_y;
    //     float rot_y = segm_vec_x * point_vec_y - segm_vec_y * point_vec_x;

    //     // Save the last point.
    //     Point last_point = (*discretization)[1];
    //     discretization->pop_back();

    //     // Use stack to avoid recursion.
    //     std::stack<float> point_stack;
    //     point_stack.push(projection_end);
    //     float cur_x = projection_start;
    //     float cur_y = parabola_y(cur_x, rot_x, rot_y);

    //     // Adjust max_dist parameter in the transformed space.
    //     const float max_dist_transformed = max_dist * max_dist * sqr_segment_length;
    //     while (!point_stack.empty())
    //     {
    //       float new_x = point_stack.top();
    //       float new_y = parabola_y(new_x, rot_x, rot_y);

    //       // Compute coordinates of the point of the parabola that is
    //       // furthest from the current line segment.
    //       float mid_x = (new_y - cur_y) / (new_x - cur_x) * rot_y + rot_x;
    //       float mid_y = parabola_y(mid_x, rot_x, rot_y);

    //       // Compute maximum distance between the given parabolic arc
    //       // and line segment that discretize it.
    //       float dist = (new_y - cur_y) * (mid_x - cur_x) -
    //                    (new_x - cur_x) * (mid_y - cur_y);
    //       dist = dist * dist / ((new_y - cur_y) * (new_y - cur_y) + (new_x - cur_x) * (new_x - cur_x));
    //       if (dist <= max_dist_transformed)
    //       {
    //         // Distance between parabola and line segment is less than max_dist.
    //         point_stack.pop();
    //         float inter_x = (segm_vec_x * new_x - segm_vec_y * new_y) /
    //                             sqr_segment_length +
    //                         cast(x(low(segment)));
    //         float inter_y = (segm_vec_x * new_y + segm_vec_y * new_x) /
    //                             sqr_segment_length +
    //                         cast(y(low(segment)));
    //         discretization->push_back(Point(inter_x, inter_y));
    //         cur_x = new_x;
    //         cur_y = new_y;
    //       }
    //       else
    //       {
    //         point_stack.push(mid_x);
    //       }
    //     }

    //     // Update last point.
    //     discretization->back() = last_point;
    //   }

    // private:
    //   // Compute y(x) = ((x - a) * (x - a) + b * b) / (2 * b).
    //   static float parabola_y(float x, float a, float b)
    //   {
    //     return ((x - a) * (x - a) + b * b) / (b + b);
    //   }

    //   // Get normalized length of the distance between:
    //   //   1) point projection onto the segment
    //   //   2) start point of the segment
    //   // Return this length divided by the segment length. This is made to avoid
    //   // sqrt computation during transformation from the initial space to the
    //   // transformed one and vice versa. The assumption is made that projection of
    //   // the point lies between the start-point and endpoint of the segment.
    //   static float get_point_projection(
    //       const Point &point, const Segment &segment)
    //   {
    //     float segment_vec_x = cast(x(high(segment))) - cast(x(low(segment)));
    //     float segment_vec_y = cast(y(high(segment))) - cast(y(low(segment)));
    //     float point_vec_x = x(point) - cast(x(low(segment)));
    //     float point_vec_y = y(point) - cast(y(low(segment)));
    //     float sqr_segment_length =
    //         segment_vec_x * segment_vec_x + segment_vec_y * segment_vec_y;
    //     float vec_dot = segment_vec_x * point_vec_x + segment_vec_y * point_vec_y;
    //     return vec_dot / sqr_segment_length;
    //   }

    //   static float cast(const float &value)
    //   {
    //     return static_cast<float>(value);
    //   }
    // };

  } // namespace polygon
} // namespace boost

// Traversing Voronoi edges using edge iterator.
int iterate_primary_edges1(const voronoi_diagram<double> &vd)
{
  int result = 0;
  for (voronoi_diagram<double>::const_edge_iterator it = vd.edges().begin();
       it != vd.edges().end(); ++it)
  {
    if (it->is_primary())
      ++result;
  }
  return result;
}

// Traversing Voronoi edges using cell iterator.
int iterate_primary_edges2(const voronoi_diagram<double> &vd)
{
  int result = 0;
  for (voronoi_diagram<double>::const_cell_iterator it = vd.cells().begin();
       it != vd.cells().end(); ++it)
  {
    const voronoi_diagram<double>::cell_type &cell = *it;
    const voronoi_diagram<double>::edge_type *edge = cell.incident_edge();
    // This is convenient way to iterate edges around Voronoi cell.
    do
    {
      if (edge->is_primary())
        ++result;
      edge = edge->next();
    } while (edge != cell.incident_edge());
  }
  return result;
}

// Traversing Voronoi edges using vertex iterator.
// As opposite to the above two functions this one will not iterate through
// edges without finite endpoints and will iterate only once through edges
// with single finite endpoint.
int iterate_primary_edges3(const voronoi_diagram<double> &vd)
{
  int result = 0;
  for (voronoi_diagram<double>::const_vertex_iterator it =
           vd.vertices().begin();
       it != vd.vertices().end(); ++it)
  {
    const voronoi_diagram<double>::vertex_type &vertex = *it;
    const voronoi_diagram<double>::edge_type *edge = vertex.incident_edge();
    // This is convenient way to iterate edges around Voronoi vertex.
    do
    {
      if (edge->is_primary())
        ++result;
      edge = edge->rot_next();
    } while (edge != vertex.incident_edge());
  }
  return result;
}

// typedef voronoi_diagram<double>::cell_type cell;

// Point retrieve_point(const cell &cell)
// {
//     cell::source_index_type index = cell.source_index();
//     cell::source_category_type category = cell.source_category();
//     // if (category == SOURCE_CATEGORY_SINGLE_POINT) {
//     //   return point_data_[index];
//     // }
//     // index -= point_data_.size();
//     if (category == 0x1)
//     {
//         return low(segment_data_[index]);
//     }
//     else
//     {
//         return high(segment_data_[index]);
//     }
// }

// void curveDiscretization(voronoi_diagram<double> &vd)
// {
//   for (voronoi_diagram<double>::const_edge_iterator it = vd.edges().begin(); it != vd.edges().end(); ++it)
//   {
//     if (!it->is_primary() || !it->is_finite()) // ingora edges secondari e infiniti
//       continue;

//     std::vector<Point> samples;
//     Point vertex0(it->vertex0()->x(), it->vertex0()->y());
//     samples.push_back(vertex0);
//     Point vertex1(it->vertex1()->x(), it->vertex1()->y());
//     samples.push_back(vertex1);

//     float max_dist = 0.02;
//     Point point = it->cell()->contains_point() ? retrieve_point(*it->cell()) : retrieve_point(*it->twin()->cell());
//     Segment segment = it->cell()->contains_point() ? retrieve_segment(*it->twin()->cell()) : retrieve_segment(*it->cell());
//     boost::polygon::VoronoiHelper::discretize(point, segment, max_dist, &samples);

//     for (int i = 0; i < samples.size() - 1; i++)
//       out.emplace_back(Segment((float)samples[i].x(), (float)samples[i].y(), (float)samples[i + 1].x(), (float)samples[i + 1].y()));
//   }
// }

int main()
{
  // Preparing Input Geometries.
  std::vector<Point> points;
  points.push_back(Point(0, 0));
  points.push_back(Point(1, 6));
  std::vector<Segment> segments;
  segments.push_back(Segment(-4, 5, 5, -1));
  segments.push_back(Segment(3, -11, 13, -1));

  // Construction of the Voronoi Diagram.
  voronoi_diagram<double> vd;
  construct_voronoi(points.begin(), points.end(),
                    segments.begin(), segments.end(),
                    &vd);

  // curveDiscretization(vd);

  // Traversing Voronoi Graph.
  {
    printf("Traversing Voronoi graph.\n");
    printf("Number of visited primary edges using edge iterator: %d\n",
           iterate_primary_edges1(vd));
    printf("Number of visited primary edges using cell iterator: %d\n",
           iterate_primary_edges2(vd));
    printf("Number of visited primary edges using vertex iterator: %d\n",
           iterate_primary_edges3(vd));
    printf("\n");
  }

  // Using color member of the Voronoi primitives to store the average number
  // of edges around each cell (including secondary edges).
  {
    printf("Number of edges (including secondary) around the Voronoi cells:\n");
    for (voronoi_diagram<double>::const_edge_iterator it = vd.edges().begin();
         it != vd.edges().end(); ++it)
    {
      std::size_t cnt = it->cell()->color();
      it->cell()->color(cnt + 1);
    }
    for (voronoi_diagram<double>::const_cell_iterator it = vd.cells().begin();
         it != vd.cells().end(); ++it)
    {
      printf("%lu ", it->color());
    }
    printf("\n");
    printf("\n");
  }

  // Linking Voronoi cells with input geometries.
  {
    unsigned int cell_index = 0;
    for (voronoi_diagram<double>::const_cell_iterator it = vd.cells().begin();
         it != vd.cells().end(); ++it)
    {
      if (it->contains_point())
      {
        std::size_t index = it->source_index();
        Point p = points[index];
        printf("Cell #%ud contains a point: (%f, %f).\n",
               cell_index, x(p), y(p));
      }
      else
      {
        std::size_t index = it->source_index() - points.size();
        Point p0 = low(segments[index]);
        Point p1 = high(segments[index]);
        if (it->source_category() ==
            boost::polygon::SOURCE_CATEGORY_SEGMENT_START_POINT)
        {
          printf("Cell #%ud contains segment start point: (%f, %f).\n",
                 cell_index, x(p0), y(p0));
        }
        else if (it->source_category() ==
                 boost::polygon::SOURCE_CATEGORY_SEGMENT_END_POINT)
        {
          printf("Cell #%ud contains segment end point: (%f, %f).\n",
                 cell_index, x(p0), y(p0));
        }
        else
        {
          printf("Cell #%ud contains a segment: ((%f, %f), (%f, %f)). \n",
                 cell_index, x(p0), y(p0), x(p1), y(p1));
        }
      }
      ++cell_index;
    }
  }
  return 0;
}