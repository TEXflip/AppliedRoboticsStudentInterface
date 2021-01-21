#include "voronoiHandler.hpp"

void VoronoiHandler::buildVoronoi(std::vector<VoronoiHandler::segment_type> &segments, std::vector<Segment> &out, double discretizationSize)
{
    segment_data_ = segments;
    voronoi_diagram<double> vd;
    construct_voronoi(segments.begin(), segments.end(), &vd);

    for (voronoi_diagram<double>::const_edge_iterator it = vd.edges().begin(); it != vd.edges().end(); ++it)
    {
        if (!it->is_primary() || !it->is_finite()) // ingora edges secondari e infiniti
            continue;

        std::vector<point_type> samples;
        point_type vertex0(it->vertex0()->x(), it->vertex0()->y());
        samples.push_back(vertex0);
        point_type vertex1(it->vertex1()->x(), it->vertex1()->y());
        samples.push_back(vertex1);

        coordinate_type max_dist = 0.02;
        point_type point = it->cell()->contains_point() ? retrieve_point(*it->cell()) : retrieve_point(*it->twin()->cell());
        segment_type segment = it->cell()->contains_point() ? retrieve_segment(*it->twin()->cell()) : retrieve_segment(*it->cell());
        VoronoiHelper<coordinate_type>::discretize(point, segment, max_dist, &samples);

        for (int i = 0; i < samples.size() - 1; i++)
            out.emplace_back(Segment((float)samples[i].x(), (float)samples[i].y(), (float)samples[i + 1].x(), (float)samples[i + 1].y()));
    }
}

VoronoiHandler::point_type VoronoiHandler::retrieve_point(const VoronoiHandler::cell &cell)
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

VoronoiHandler::segment_type VoronoiHandler::retrieve_segment(const VoronoiHandler::cell &cell)
{
    cell::source_index_type index = cell.source_index() /* - point_data_.size()*/;
    return segment_data_[index];
}