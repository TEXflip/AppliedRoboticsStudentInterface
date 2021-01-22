#include "voronoiHandler.hpp"
#include <iostream>

void VoronoiHandler::buildVoronoi(const Polygon &borders,const std::vector<Polygon> &obstacle_list, std::vector<Segment> &out, double discretizationSize, float precision)
{
    std::vector<VoronoiHandler::segment_type> segments;
    for (int ob = 0; ob < obstacle_list.size(); ob++)
    {
        Polygon v = obstacle_list[ob];
        for (int i = 0, next; i < v.size(); i++)
        {
            next = (i + 1) % v.size();
            VoronoiHandler::point_type p1((int)(v[i].x * precision), (int)(v[i].y * precision));
            VoronoiHandler::point_type p2((int)(v[next].x * precision), (int)(v[next].y * precision));
            segments.emplace_back(VoronoiHandler::segment_type(p1, p2));
            // std::cout<< "\tx " << v[i].x << "\ty " << v[i].y << std::endl;
        }
    }
    for (int i = 0, next; i < borders.size(); i++)
    {
        next = (i + 1) % borders.size();
        VoronoiHandler::point_type p1((int)(borders[i].x * precision), (int)(borders[i].y * precision));
        VoronoiHandler::point_type p2((int)(borders[next].x * precision), (int)(borders[next].y * precision));
        segments.emplace_back(VoronoiHandler::segment_type(p1, p2));
        // std::cout<< "\tx " << borders[i].x << "\ty " << borders[i].y << std::endl;
    }

    std::cout<< "Total input segments: " << segments.size() << std::endl;

    voronoi_diagram<double> vd;
    construct_voronoi(segments.begin(), segments.end(), &vd);

    std::cout << "Number of Edges: " << vd.edges().size() << std::endl;

    for (voronoi_diagram<double>::const_edge_iterator it = vd.edges().begin(); it != vd.edges().end(); ++it)
    {
        if (it->is_secondary() || it->is_infinite()) // ingora edges secondari e infiniti
            continue;

        std::vector<point_type> samples;
        point_type vertex0(it->vertex0()->x(), it->vertex0()->y());
        samples.push_back(vertex0);
        point_type vertex1(it->vertex1()->x(), it->vertex1()->y());
        samples.push_back(vertex1);

        if (it->is_curved())
        {
            coordinate_type max_dist = static_cast<coordinate_type>(discretizationSize*precision);
            point_type point = it->cell()->contains_point() ? retrieve_point(*it->cell(), segments) : retrieve_point(*it->twin()->cell(), segments);
            segment_type segment = it->cell()->contains_point() ? retrieve_segment(*it->twin()->cell(), segments) : retrieve_segment(*it->cell(), segments);
            VoronoiHelper<coordinate_type>::discretize(point, segment, max_dist, &samples);

            std::cout << "Discretized" << std::endl;
        }

        for (int i = 0; i < samples.size() - 1; i++)
            out.emplace_back(Segment(samples[i].x() / precision, samples[i].y() / precision, samples[i + 1].x() / precision, samples[i + 1].y() / precision));
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