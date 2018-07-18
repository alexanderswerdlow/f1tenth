#include <path_follower/collision_avoidance/collision_detector_ackermann.h>

#include <Eigen/Core>
//#include <opencv2/opencv.hpp> // only for debugging
#include <path_follower/utils/visualizer.h>

using namespace Eigen;



bool CollisionDetectorAckermann::avoid(MoveCommand * const cmd,
                                    const CollisionAvoider::State &state)
{
    velocity_ = cmd->getVelocity();

    return CollisionDetectorPolygon::avoid(cmd, state);
}

CollisionDetectorPolygon::PolygonWithTfFrame CollisionDetectorAckermann::getPolygon(float width, float length, float course_angle, float curve_enlarge_factor) const
{
    /// Based on http://stackoverflow.com/questions/1217585/parallelogram-contains-point

    /*
     * The parallelogram is defined by four points p,q,r,s:
     * p and q are the corners near the robot, r and s are on the opposite site.
     *
     * For course_angle = 0, the parallelogram is a rectangle. If course_angle != 0, a and b
     * stay constant, but c is moved to the side, depending on course_angle:
     *
     * course_angle = 0:
     *          p------------------r
     *  ##   ## |                  | |
     *  ####### |                  | width
     *  ####### |                  | |
     *  ##   ## |                  |
     *          q------------------s
     *   ^robot    <-  length  ->
     *
     *
     * course_angle < 0:
     *          p---+
     *  ##   ## |    +--+
     *  ####### |        +--+
     *  ####### |            +--r
     *  ##   ## |               |
     *          |               |
     *          |               |
     *          q---+           |
     *               +--+       |
     *                   +--+   |
     *                       +--s
     *
     * The Box is enlarged toward the inside curve. In the above example, q is moved away from
     * the robot, while p is fixed. For course_angle > 0 it is vice verca.
     * The amount of this enlarging is controlled by the argument 'curve_enlarge_factor'.
     */


    Vector2f p(0.0f,  width/2.0f);
    Vector2f q(0.0f, -width/2.0f);

    float dir = velocity_ < 0.f ? -1.f : 1.f;
    float sin_angle = std::sin(course_angle);
    float cos_angle = std::cos(course_angle);

    // stretch box sidewards, depending on course angle
    if (course_angle > 0) {
        p(1) += curve_enlarge_factor * sin_angle;
    } else if (course_angle < 0) {
        q(1) += curve_enlarge_factor * sin_angle;
    }

    Vector2f r = p + dir * length * Vector2f(cos_angle, sin_angle);
    Vector2f s = q + dir * length * Vector2f(cos_angle, sin_angle);

    PolygonWithTfFrame pwf;
    pwf.frame = robot_frame_;

    pwf.polygon.push_back( cv::Point2f(p[0], p[1]) );
    pwf.polygon.push_back( cv::Point2f(q[0], q[1]) );
    pwf.polygon.push_back( cv::Point2f(s[0], s[1]) );
    pwf.polygon.push_back( cv::Point2f(r[0], r[1]) );

    return pwf;
}
