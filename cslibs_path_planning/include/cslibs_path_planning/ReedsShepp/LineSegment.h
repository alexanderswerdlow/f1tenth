/*
 * LineSegment.h
 *
 *  Created on: Aug 15, 2011
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef LINESEGMENT_H
#define LINESEGMENT_H

/// COMPONENT
#include "CurveSegment.h"

/// PROJECT
#include "../common/Point2d.h"

namespace lib_path
{

/**
 * The LineSegment class represents a line that has a direction
 */
class LineSegment: public CurveSegment
{
    friend class CurveRenderer;

public:
    /**
     * Constructor, that sets the direction
     *
     * @param direction FORWARD or BACKWARD
     */
    LineSegment(DIRECTION direction);

    /**
     * Copy constructor
     */
    // not necessary, use default

    /**
     * Creates a new segment that is an exact copy of this one
     *
     * @return copy of this segment
     */
    CurveSegment* clone() const;

    /**
     * Sets the points that describe the line
     *
     * @param start start point
     * @param end end point
     */
    void set_points(Point2d start, Point2d end);

    /**
     * Start iterating over the points on this segment
     */
    virtual void reset_iteration();

    /**
     * Check, if another point exists
     *
     * @return true, iff there exists another point
     */
    virtual bool has_next();

    /**
     * Get the next point in this iteration
     */
    virtual Pose2d next();

    /**
     * Get the start point
     */
    virtual Pose2d start();

    /**
     * Get the end point
     */
    virtual Pose2d end();

    /**
     * Computes the weight of this segment
     */
    virtual float weight(bool ignore_obstacles);

private:
    Pose2d m_start;
    Pose2d m_end;

    bool m_iterating;
    int m_output;
    int m_segments;

    float m_theta;
};

}

#endif // LINESEGMENT_H
