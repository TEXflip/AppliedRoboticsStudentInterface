#include "voronoiHandler.hpp"
#include <iostream>

void VoronoiHandler::buildVoronoi(std::vector<VoronoiHandler::segment_type> &segments, std::vector<Segment> &out, double discretizationSize)
{
    voronoi_diagram<double> vd;
    construct_voronoi(segments.begin(), segments.end(), &vd);

    std::cout << "Number of Edges: " << vd.edges().size() << std::endl;

    for (voronoi_diagram<double>::const_edge_iterator it = vd.edges().begin(); it != vd.edges().end(); ++it)
    {
        if (!it->is_primary() || !it->is_finite()) // ingora edges secondari e infiniti
            continue;

        std::vector<point_type> samples;
        point_type vertex0(it->vertex0()->x(), it->vertex0()->y());
        samples.push_back(vertex0);
        point_type vertex1(it->vertex1()->x(), it->vertex1()->y());
        samples.push_back(vertex1);

        if (it->is_curved()){
            coordinate_type max_dist = static_cast<coordinate_type>(discretizationSize);
            point_type point = it->cell()->contains_point() ? retrieve_point(*it->cell(), segments) : retrieve_point(*it->twin()->cell(), segments);
            segment_type segment = it->cell()->contains_point() ? retrieve_segment(*it->twin()->cell(), segments) : retrieve_segment(*it->cell(), segments);
            VoronoiHelper<coordinate_type>::discretize(point, segment, max_dist, &samples);

            std::cout << "Discretized" << std::endl;
        }

        for (int i = 0; i < samples.size() - 1; i++)
            out.emplace_back(Segment((float)samples[i].x(), (float)samples[i].y(), (float)samples[i + 1].x(), (float)samples[i + 1].y()));
    }
}

VoronoiHandler::point_type VoronoiHandler::retrieve_point(const VoronoiHandler::cell &cell, std::vector<VoronoiHandler::segment_type> &segments)
{
    cell::source_index_type index = cell.source_index();
    cell::source_category_type category = cell.source_category();
    // if (category == SOURCE_CATEGORY_SINGLE_POINT) {
    //   return point_data_[index];
    // }
    // index -= point_data_.size();
    if (category == SOURCE_CATEGORY_SEGMENT_START_POINT)
    {
        return low(segments[index]);
    }
    else
    {
        return high(segments[index]);
    }
}

VoronoiHandler::segment_type VoronoiHandler::retrieve_segment(const VoronoiHandler::cell &cell, std::vector<VoronoiHandler::segment_type> &segments)
{
    cell::source_index_type index = cell.source_index() /* - point_data_.size()*/;
    return segments[index];
}