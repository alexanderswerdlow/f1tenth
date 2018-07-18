#ifndef COLLISION_DETECTOR_POLYGON_H
#define COLLISION_DETECTOR_POLYGON_H

#include "collision_detector.h"
#include <opencv2/core/core.hpp>
#include <tf/transform_listener.h>

class CollisionDetectorPolygon : public CollisionDetector
{
protected:
    struct PolygonWithTfFrame
    {
        std::string frame;
        std::vector<cv::Point2f> polygon;
    };


    /**
     * @brief Check, if there is an obstacle within the polygon, given by getPolygon().
     * @see getPolygon()
     * @return True if there is an obstacle, false if not.
     */
    virtual bool checkOnCloud(std::shared_ptr<ObstacleCloud const> obstacles,
                              float width,
                              float length,
                              float course_angle,
                              float curve_enlarge_factor);


    /**
     * @brief Get the polygon which is checked for obstacles.
     * @return Collision polygon.
     */
    virtual PolygonWithTfFrame getPolygon(float width,
                                          float length,
                                          float course_angle,
                                          float curve_enlarge_factor) const = 0;


private:
    void visualize(PolygonWithTfFrame polygon, bool hasObstacle) const;
};

#endif // COLLISION_DETECTOR_POLYGON_H
