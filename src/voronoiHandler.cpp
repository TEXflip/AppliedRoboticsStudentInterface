#include "voronoiHandler.hpp"
#include <iostream>
#include "collision_detection.hpp"
#include "debug.hpp"

#define footprint_width 0.09
#define footprint_height 0.02
#define footprint_length 0.19

void VoronoiHandler::buildVoronoi(const Polygon &borders, const std::vector<Polygon> &obstacle_list, Graph::Graph &outGraph, double discretizationSize, float precision)
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

    std::cout << "Total input segments: " << segments.size() << std::endl;

    voronoi_diagram<double> vd;
    construct_voronoi(segments.begin(), segments.end(), &vd);

    std::cout << "Edges: " << vd.edges().size() << "\tVertices: " << vd.vertices().size() << std::endl;

    vector<Polygon> rescaled_ob_list = VoronoiHandler::scale(obstacle_list, footprint_width/1.5);
    // showPolygons(rescaled_ob_list);

    int i = 0, rem = 0;
    for (voronoi_diagram<double>::const_vertex_iterator it = vd.vertices().begin();
         it != vd.vertices().end(); ++it)
    {
        float x = it->x() / precision;
        float y = it->y() / precision;
        if (!isInside_Global(Point(x, y), rescaled_ob_list))
        {
            Graph::node n;
            it->color(i);
            n.x = x;
            n.y = y;
            // if (isInside_Global(Point(x,y), rescaled_ob_list)){
            //     n.removed = true;
            //     rem++;
            // }
            outGraph.emplace_back(n);
            i++;
        }
        else
        {
            it->color(-1);
        }
    }

    // std::cout << "Vertices removed: " << rem << std::endl;
    // int i = 0;
    for (voronoi_diagram<double>::const_cell_iterator it = vd.cells().begin();
         it != vd.cells().end(); ++it)
    {
        const voronoi_diagram<double>::cell_type &cell = *it;
        const voronoi_diagram<double>::edge_type *edge = cell.incident_edge();

        do
        {
            if (edge->is_primary() && edge->is_finite())
            {
                int pos0 = edge->vertex0()->color();
                int pos1 = edge->vertex1()->color();
                if (pos0 >= 0 && pos1 >= 0)
                    outGraph[pos0].neighbours.emplace_back(pos1);
                // if (intersect_Global(Point(edge->vertex0()->x()/ precision,edge->vertex0()->y()/ precision), Point(edge->vertex1()->x()/ precision,edge->vertex1()->y()/ precision), rescaled_ob_list))
                // {
                //     outGraph[pos0].removed = true;
                //     outGraph[pos1].removed = true;
                // }
                // nodes[pos].neighboursCells.emplace_back(i);
                // cells[i].nodes.emplace_back(pos);
            }

            edge = edge->next();
        } while (edge != cell.incident_edge());
    }
    // showGraphAndPolygons(outGraph, rescaled_ob_list);
    // std::cout << "Tot Cells1: " << cells.size() << "\t" << &cells << std::endl;
    // outGraph.setCells(&cells);
    // outGraph.setNodes(&nodes);
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

vector<Polygon> VoronoiHandler::scale(const vector<Polygon>& polygons, float scale)
{
    vector<Polygon> resized;
    resized.resize(polygons.size());
    for (int i = 0; i < polygons.size(); i++)
    {
        Polygon poly = polygons[i];
        float x = 0, y = 0;
        for (Point p : poly)
        {
            x += p.x;
            y += p.y;
        }
        x /= poly.size();
        y /= poly.size();

        Polygon newP;

        for(auto p : poly){
            // float diffX = p.x - x;
            // float diffY = p.y - y;
            float theta = atan((p.y - y)/(p.x - x));
            float sign = (p.x - x) > 0 ? 1 : -1;
            newP.emplace_back(p.x + cos(theta)*scale*sign, p.y + sin(theta)*scale*sign);
        }
        resized[i] = newP;
    }
    return resized;
}