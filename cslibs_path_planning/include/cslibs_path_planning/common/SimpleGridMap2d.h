/**
 * @file SimpleGridMap2d.h
 * @date Jan 2012
 * @author marks
 */

#ifndef SIMPLEGRIDMAP2D_H
#define SIMPLEGRIDMAP2D_H

// C/C++
#include <stdint.h>
#include <vector>

// Project
#include "GridMap2d.h"


namespace lib_path {

/**
 * @class SimpleGridMap2d
 * @brief A simple implementation of a 2d grid map.
 */
class SimpleGridMap2d : public GridMap2d {
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
    SimpleGridMap2d( const unsigned int w, const unsigned int h, const double r );

    virtual ~SimpleGridMap2d() { /* Nothing to do */ }

    /**
     * @brief Set the free cell  threshold.
     * Every cell with a value less or equal to this threshold is free.
     * @param thres The new threshold.
     */
    void setLowerThreshold( const uint8_t thres )
        { lower_thres_ = thres; }

    /**
     * @brief Set the occupied cell threshold.
     * Every cell with a value greater or equal to this threshold is occupied.
     * @param thres The new Threshold.
     */
    void setUpperThreshold( const uint8_t thres )
        { upper_thres_ = thres; }

    /**
     * @brief Set the no information value.
     * @param val The new value.
     */
    void setNoInformationValue( const int val )
        { no_information_ = val; }

    /**
     * @brief Set the size on one cell.
     * @param r Size of one cell in meter.
     */
    void setResolution( const double r )
        { res_ = r; }

    /**
     * @brief Set new map data.
     * @param data The new map data. Size should be >= w * h
     * @param w New map width (number of cells in x-direction)
     * @param h new map height (number of cells in y-direction)
     */
    void set( const std::vector<uint8_t>& data, const unsigned int w, const unsigned int h ) {
        width_ = w;
        height_ = h;
        data_.assign( data.begin(), data.end());
    }

    /**
     * @brief Set all cell values to a given value.
     * @param value New value of all cells.
     */
    void set( const uint8_t value ) {
        std::vector<uint8_t>::iterator cell_it = data_.begin();
        while ( cell_it != data_.end()) {
            *cell_it = value;
            cell_it++;
        }
    }

    /// @todo enervated hotfix
    void cell2point( const unsigned int x, const unsigned int y, float& px, float& py ) const {
        px = (float)(res_*(double)(x+0.5) + origin_.x);
        py = (float)(res_*(double)(y+0.5) + origin_.y);
    }

    /* Inherited from GridMap2d */

    uint8_t getValue( const unsigned int x, const unsigned int y ) const
        { return data_[y*width_ + x]; }

    void setValue( const unsigned int x, const unsigned int y, const uint8_t value )
        { data_[y*width_ + x] = value; }

    unsigned int getWidth() const
        { return width_; }

    unsigned int getHeight() const
        { return height_; }

    double getResolution() const
        { return res_; }

    Point2d getOrigin() const
        { return origin_; }

    void setOrigin( const Point2d& p )
        { origin_ = p; }

    bool isFree( const unsigned int x, const unsigned int y ) const {
        uint8_t val = getValue( x, y );
        if(no_information_ != -1 && val == no_information_) {
            return false;
        }
        return val <= lower_thres_;
    }

    bool isOccupied( const unsigned int x, const unsigned int y ) const {
        uint8_t val = getValue( x, y );
        if(no_information_ != -1 && val == no_information_) {
            return false;
        }
        return val >= upper_thres_;
    }

    bool isNoInformation( const unsigned int x, const unsigned int y ) const {
        uint8_t value = getValue( x, y );
        if(no_information_ != -1 && value == no_information_) {
            return true;
        }

        return  value > lower_thres_ && value < upper_thres_;
    }

    bool point2cell( const double px, const double py, unsigned int& x, unsigned int& y ) const {
        x = (unsigned int)((px - origin_.x)/res_);
        y = (unsigned int)((py - origin_.y)/res_);

        if ( !isInMap( (int)x, (int)y ))
            return false;
        return true;
    }
    virtual bool point2cellSubPixel( const double px, const double py, double& x, double& y ) const {
        x = (px - origin_.x)/res_;
        y = (py - origin_.y)/res_;

        if ( !isInMap( (int)x, (int)y ))
            return false;
        return true;
    }

    void cell2point( const unsigned int x, const unsigned int y, double& px, double& py ) const {
        px = res_*(double)(x+0.5) + origin_.x;
        py = res_*(double)(y+0.5) + origin_.y;
    }

    virtual void cell2pointSubPixel( const double x, const double y, double& px, double& py ) const {
        px = res_*x + origin_.x;
        py = res_*y + origin_.y;
    }

    bool isInMap( const int x, const int y ) const
        { return !(x < 0 || y < 0 || x >= (int)width_ || y >= (int)height_); }

    bool isInMap( const double x, const double y ) const
        { return (x - origin_.x)/res_ < width_ && (y - origin_.y)/res_ < height_; }

    unsigned char* getData()
    {
        return data_.data();
    }

protected:
    /// Number of cells in x-direction
    unsigned int width_;

    /// Number of cells in y-direction
    unsigned int height_;

    /// Size of one cell in meter
    double res_;

    /// Origin of the map
    Point2d origin_;

    /// The map data (row major order)
    std::vector<uint8_t> data_;

    /// Lower threshold. Every cell with a value equal or less to this threshold id free.
    uint8_t lower_thres_;

    /// Upper threshold. Every cell with an value greater or equal to this threshold is occupied.
    uint8_t upper_thres_;

    /// If exactly this value is encountered, interpret it as unknown
    int no_information_;
};

} // namespace "lib_path"

#endif // SIMPLEGRIDMAP2D_H
