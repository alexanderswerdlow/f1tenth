/*
 * CurveSegment.cpp
 *
 *  Created on: Aug 15, 2011
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

/// HEADER
#include <cslibs_path_planning/ReedsShepp/CurveSegment.h>

/// SYSTEM
#include <assert.h>

using namespace lib_path;

CurveSegment::CurveSegment(DIRECTION direction)
    : m_direction(direction),
      m_use_map_cost(false), m_min_cell_cost(10),
      m_cost_forwards(1.0), m_cost_backwards(1.0), m_cost_curve(1.0), m_cost_straight(1.0),
      m_trace(-1)
{
}

CurveSegment::~CurveSegment()
{

}

void CurveSegment::set_map(const GridMap2d* map)
{
    m_map = map;
}

void CurveSegment::set_trace(int value)
{
    m_trace = value;
}

CurveSegment::DIRECTION CurveSegment::direction() const
{
    return m_direction;
}

void CurveSegment::set_max_distance(float distance)
{
    assert(m_map != NULL);

    m_max_distance = distance;
}

void CurveSegment::set_use_map_cost(bool arg)
{
    m_use_map_cost = arg;
}

void CurveSegment::set_min_cell_cost(uint8_t cost)
{
    m_min_cell_cost = cost;
}

void CurveSegment::set_cost_backwards(double cost)
{
    m_cost_backwards = cost;
}

void CurveSegment::set_cost_forwards(double cost)
{
    m_cost_forwards = cost;
}

void CurveSegment::set_cost_curve(double cost)
{
    m_cost_curve = cost;
}

void CurveSegment::set_cost_straight(double cost)
{
    m_cost_straight = cost;
}
