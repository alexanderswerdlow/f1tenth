/*
 * path_interpolated.h
 *
 *  Created on: Apr 25, 2015
 */

#ifndef PATH_INTERPOLATED_H_
#define PATH_INTERPOLATED_H_

#include "path.h"
#include <nav_msgs/Path.h>

class PathInterpolated {
public:
	PathInterpolated();
	virtual ~PathInterpolated();

    void interpolatePath(const Path::Ptr path, const bool hack = true);

    void interpolatePath(const SubPath& path, const std::string& frame_id);

    inline double s(const unsigned int i) const {
        return s_.at(i);
    }

    inline double p(const unsigned int i) const {
        return p_.at(i);
    }
    inline double q(const unsigned int i) const {
        return q_.at(i);
    }

    inline double p_prim(const unsigned int i) const {
        return p_prim_.at(i);
    }
    inline double q_prim(const unsigned int i) const {
        return q_prim_.at(i);
    }

    inline double p_sek(const unsigned int i) const {
        return p_sek_.at(i);
    }
    inline double q_sek(const unsigned int i) const {
        return q_sek_.at(i);
    }

    inline double s_new() const {
        return s_new_;
    }

    inline void set_s_new(double s_new) {
        s_new_ = s_new;
    }

    inline double s_prim() const {
        return s_prim_;
    }

    inline void set_s_prim(double s_prim) {
        s_prim_ = s_prim;
    }

    inline double curvature(const unsigned int i) const {
        return curvature_.at(i);
	}

    inline std::size_t n() const {
        return N_;
    }

    double curvature_prim(const unsigned int i) const;
    double curvature_sek(const unsigned int i) const;

    inline double theta_p(const unsigned int i) const {
        return atan2(q_prim_.at(i), p_prim_.at(i));
    }

    inline std::string frame_id() const {
        return frame_id_;
    }

    inline void get_end(Waypoint &wp)
    {
        const unsigned int i = p_.size()-1;

        wp.x = p_[i];
        wp.y = q_[i];
        wp.orientation = std::atan2(q_prim_[i],p_prim_[i]);
        wp.s = s_[i];

    }

	operator nav_msgs::Path() const;
    operator SubPath() const;

    Path::Ptr getOriginalPath() const;

    std::string frame_id_;

private:
	void clearBuffers();

    void interpolatePath(const std::deque<Waypoint>& waypoints);

    //number of path elements
    uint N_;

    Path::Ptr original_path_;

	nav_msgs::Path interp_path;
    //curvilinear abscissa
    std::vector<double> s_;
    //x component of the interpolated path
	std::vector<double> p_;
    //y componenet of the interpolated path
    std::vector<double> q_;
    //first derivative of the x component w.r.t. path
    std::vector<double> p_prim_;
    //first derivative of the y component w.r.t. path
    std::vector<double> q_prim_;
    //second derivative of the x component w.r.t. path
    std::vector<double> p_sek_;
    //second derivative of the y component w.r.t. path
    std::vector<double> q_sek_;
    //curvature in path coordinates
	std::vector<double> curvature_;

    //next point
    double s_new_;
    //path variable derivative
    double s_prim_;
};

#endif /* PATH_INTERPOLATED_H_ */
