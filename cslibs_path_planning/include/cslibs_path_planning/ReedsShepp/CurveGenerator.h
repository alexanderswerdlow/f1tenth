/*
 * CurveGenerator.h
 *
 *  Created on: Aug 15, 2011
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef CURVEGENERATOR_H
#define CURVEGENERATOR_H

/// COMPONENT
#include "Curve.h"
#include "CurveSegment.h"

/// PROJECT
#include "../common/GridMap2d.h"

/// SYSTEM
#include <vector>
#include <string>
#include <ostream>

namespace lib_path
{

/**
 * The CurveGenerator class is the facade to this module.
 * It is responsible for managing different curve sequences and allows to plan a path between two points.
 */
class CurveGenerator
{
public:
    CurveGenerator();

    /**
     * Compiles a description of a Reeds-Shepp-Curve
     *
     * @param sequence string of segments
     *                   possible characters:
     *                   L - left curve
     *                   R - right curve
     *                   S - straight
     *                   | - change driving direction (forward <-> backward)
     * @returns true, iff the sequence was specified correctly
     */
    bool parse(const std::string& sequence, std::ostream& out=std::cerr);

    /**
     * Uses all parsed sequences to find the shortest path in a map
     *
     * @param start pose to start from in map coordinates
     * @param goal pose to end at in map coordinates
     * @param map MapInfo that is used to check for obstacles
     * @param curve_radius radius of circle elements, NOT the maximal steering angle
     * @param max_distance_between_waypoints Maximum distance between two connected waypoints
     * @param ignore_obstacles true <-> Find the shortest path, <b>ignoring</b> obstacles (default: false)
     *
     * @returns a Reeds-Shepp-Curve that describes the shortest path, or NULL if no free path exists
     */
    Curve* find_path(const Pose2d& start, const Pose2d& goal, const GridMap2d *map,
                     bool ignore_obstacles = false);

    /**
     * Setters
     */
    void set_circle_radius(double circle_radius);
    void set_max_waypoint_distance(double max_waypoint_distance);
    void set_use_map_cost(bool arg);
    void set_min_cell_cost(uint8_t cost);
    void set_cost_forwards(double cost_forwards);
    void set_cost_backwards(double cost_backwards);
    void set_cost_curve(double cost_curve);
    void set_cost_straight(double cost_straight);
    void set_trace(int value);
    void set_max_curve_arc(double arc);

private:
    std::vector< Curve > m_curves;

    bool m_use_map_cost;
    uint8_t m_min_cell_cost;
    double m_circle_radius;
    double m_max_waypoint_distance;
    double m_cost_forwards;
    double m_cost_backwards;
    double m_cost_curve;
    double m_cost_straight;
    double m_max_curve_arc;

    int m_trace;
};

}

#endif // CURVEGENERATOR_H
