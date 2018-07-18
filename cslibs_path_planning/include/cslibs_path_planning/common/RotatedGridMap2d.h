#ifndef ROTATED_GRID_MAP_2D_H
#define ROTATED_GRID_MAP_2D_H

/// COMPONENT
#include "SimpleGridMap2d.h"

/// SYSTEM
#include <stdint.h>
#include <vector>


namespace lib_path {

/**
 * @class RotatedGridMap2d
 * @brief An implementation of a 2d grid map that can be rotated.
 */
class RotatedGridMap2d : public SimpleGridMap2d {
public:

    /**
     * @brief Create a map with the given size and resolution.
     *
     * The origin will be (0,0) and the lower threshold 50, the upper threshold 200.
     *
     * @param w Number of cells in x-direction.
     * @param h Number of cells in y-direction.
     * @param r Size of one cell in meter.
     */
    RotatedGridMap2d( const unsigned int w, const unsigned int h, const double yaw, const double r );

    virtual ~RotatedGridMap2d() { /* Nothing to do */ }

    /**
     * @brief Set new map data.
     * @param data The new map data. Size should be >= w * h
     * @param w New map width (number of cells in x-direction)
     * @param h new map height (number of cells in y-direction)
     */
    void set( const std::vector<uint8_t>& data, const unsigned int w, const unsigned int h, const double yaw ) {
        width_ = w;
        height_ = h;
        yaw_ = yaw;
        data_.assign( data.begin(), data.end());
    }


    Point2d getOrigin() const
        { return origin_; }

    void setOrigin( const Point2d& p )
    {
        origin_ = p.rotate(-yaw_);
    }

    void cell2point( const unsigned int x, const unsigned int y, float& px, float& py ) const {
        float nx = res_*(double)(x+0.5);
        float ny = res_*(double)(y+0.5);

        float c = std::cos(yaw_);
        float s = std::sin(yaw_);

        float rx = nx + origin_.x;
        float ry = ny + origin_.y;

        px = c * rx - s * ry;
        py = s * rx + c * ry;
    }


    bool point2cell( const double px, const double py, unsigned int& x, unsigned int& y ) const {
        double c = std::cos(-yaw_);
        double s = std::sin(-yaw_);
       
        double nx = px;// - origin_.x;
        double ny = py;// - origin_.y;
 
        double rx = c * nx - s * ny;
        double ry = s * nx + c * ny;

        x = (rx - origin_.x)/res_;
        y = (ry - origin_.y)/res_;

        if ( !isInMap( (int)x, (int)y ))
            return false;
        return true;
    }

    void cell2point( const unsigned int x, const unsigned int y, double& px, double& py ) const {
        double nx = res_*(double)(x+0.5);
        double ny = res_*(double)(y+0.5);

        double c = std::cos(yaw_);
        double s = std::sin(yaw_);
        
        double rx = nx + origin_.x;
        double ry = ny + origin_.y;

        px = c * rx - s * ry;
        py = s * rx + c * ry;
    }

    virtual void cell2pointSubPixel( const double x, const double y, double& px, double& py ) const {
        double nx = res_ * x;
        double ny = res_ * y;

        double c = std::cos(yaw_);
        double s = std::sin(yaw_);
        
        double rx = nx + origin_.x;
        double ry = ny + origin_.y;

        px = c * rx - s * ry;
        py = s * rx + c * ry;
    }

    bool isInMap( int x, int y) const {
        return SimpleGridMap2d::isInMap(x, y);
    }
   
    bool isInMap( const double x, const double y ) const {
        double c = std::cos(-yaw_);
        double s = std::sin(-yaw_);
       
        double nx = x;// - origin_.x;
        double ny = y;// - origin_.y;
        
        int px = c * nx - s * ny;
        int py = s * nx + c * ny;

        return px >= 0 && py >= 0 && px < (int) width_ && py < (int) height_;
    }

    virtual double getRotation() const 
    {
        return yaw_;
    }

protected:
    /// Rotation of the map
    double yaw_;
};

} // namespace "lib_path"

#endif // ROTATED_GRID_MAP_2D_H
