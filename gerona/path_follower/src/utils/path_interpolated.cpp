// HEADER
#include <path_follower/utils/path_interpolated.h>

// PROJECT
#include <path_follower/utils/cubic_spline_interpolation.h>
#include <cslibs_utils/MathHelper.h>
#include <path_follower/parameters/path_follower_parameters.h>

// SYSTEM
#include <deque>
#include <nav_msgs/Path.h>
#pragma GCC diagnostic ignored "-Wignored-qualifiers"
#include <interpolation.h>
#pragma GCC diagnostic pop

using namespace Eigen;

PathInterpolated::PathInterpolated()
    : frame_id_(PathFollowerParameters::getInstance()->world_frame()),
      N_(0),
      s_new_(0),
	  s_prim_(0)
{
}

PathInterpolated::~PathInterpolated() {
}

void PathInterpolated::interpolatePath(const Path::Ptr path, const bool hack) {

	clearBuffers();

    original_path_ = path;

    frame_id_ = path->getFrameId();

	std::deque<Waypoint> waypoints;
    while (!path->isDone()) {
        waypoints.insert(waypoints.end(), path->getCurrentSubPath().wps.begin(), path->getCurrentSubPath().wps.end());

        if(hack){
            int originalNumWaypoints = waypoints.size();
            // (messy) hack!!!!!
            // remove waypoints that are closer than 0.1 meters to the starting point
            Waypoint start = waypoints.front();
            while(!waypoints.empty()) {
                std::deque<Waypoint>::iterator it = waypoints.begin();
                const Waypoint& wp = *it;

                double dx = wp.x - start.x;
                double dy = wp.y - start.y;
                double distance = hypot(dx, dy);
                if(distance < 0.1) {
                    waypoints.pop_front();
                } else {
                    break;
                }
            }

            // eliminate subpaths containing only the same points
            if(waypoints.size() > 1)
                break;

            //special case where all waypoints are below 0.1m, keep at least last two waypoints
            if (originalNumWaypoints >= 2 && waypoints.size() < 2)
            {
                waypoints.insert(waypoints.end(), path->getCurrentSubPath().wps.begin(), path->getCurrentSubPath().wps.end());
                while (waypoints.size() > 2) waypoints.pop_front();
                break;
            }

            path->switchToNextSubPath();
        }else{
            break;
        }
    }
    // In case path->switchToNextSubPath(); was called a reset is required
    path->reset();

    try {
        interpolatePath(waypoints);
    } catch(const alglib::ap_error& error) {
        throw std::runtime_error(error.msg);
    }
}

void PathInterpolated::interpolatePath(const SubPath& path, const std::string& frame_id){

    clearBuffers();

    frame_id_ = frame_id;

    std::deque<Waypoint> waypoints;
    waypoints.insert(waypoints.end(),path.wps.begin(),path.wps.end());

    interpolatePath(waypoints);
}

void PathInterpolated::interpolatePath(const std::deque<Waypoint>& waypoints){
	//copy the waypoints to arrays X_arr and Y_arr, and introduce a new array l_arr_unif required for the interpolation
	//as an intermediate step, calculate the arclength of the curve, and do the reparameterization with respect to arclength

	N_ = waypoints.size();

	if(N_ < 2) {
        N_ = 0;
		return;
	}

	double X_arr[N_], Y_arr[N_], l_arr[N_], l_arr_unif[N_];
	double L = 0;

    X_arr[0] = waypoints[0].x;
    Y_arr[0] = waypoints[0].y;
	l_arr[0] = 0;

    for(std::size_t wp_index = 1, insert_index = 1, n = N_; wp_index < n; ++wp_index){
        const Waypoint& waypoint = waypoints[wp_index];

        auto dist = hypot(waypoint.x - X_arr[insert_index-1], waypoint.y - Y_arr[insert_index-1]);

        if(dist >= 1e-3) {
            X_arr[insert_index] = waypoint.x;
            Y_arr[insert_index] = waypoint.y;

            L += dist;
            l_arr[insert_index] = L;

            ++insert_index;

        } else {
            // two points were to close...
            ROS_WARN_STREAM("dropping point (" << waypoint.x << " / " << waypoint.y <<
                            ") because it is too close to the last point (" << X_arr[insert_index-1] << " / " << Y_arr[insert_index-1] << ")" );
            --N_;
        }

	}
//	ROS_INFO("Length of the path: %lf m", L);


	double f = std::max(0.0001, L / (double) (N_-1));

    for(std::size_t i = 0; i < N_; i++){

		l_arr_unif[i] = i * f;

	}

	//initialization before the interpolation
	alglib::real_1d_array X_alg, Y_alg, l_alg, l_alg_unif;
	alglib::real_1d_array x_s, y_s, x_s_prim, y_s_prim, x_s_sek, y_s_sek;

	X_alg.setcontent(N_, X_arr);
	Y_alg.setcontent(N_, Y_arr);
	l_alg.setcontent(N_, l_arr);
	l_alg_unif.setcontent(N_, l_arr_unif);


	//interpolate the path and find the derivatives
    try {
        alglib::spline1dconvdiff2cubic(l_alg, X_alg, l_alg_unif, x_s, x_s_prim, x_s_sek);
        alglib::spline1dconvdiff2cubic(l_alg, Y_alg, l_alg_unif, y_s, y_s_prim, y_s_sek);

    } catch(const alglib::ap_error& error) {
        ROS_FATAL_STREAM("alglib error: " << error.msg);
        throw std::runtime_error(error.msg);
    }

	//define path components, its derivatives, and curvilinear abscissa, then calculate the path curvature
	for(uint i = 0; i < N_; ++i) {

		s_.push_back(l_arr_unif[i]);

		p_.push_back(x_s[i]);
		q_.push_back(y_s[i]);

		p_prim_.push_back(x_s_prim[i]);
		q_prim_.push_back(y_s_prim[i]);

		p_sek_.push_back(x_s_sek[i]);
		q_sek_.push_back(y_s_sek[i]);

		curvature_.push_back((x_s_prim[i]*y_s_sek[i] - x_s_sek[i]*y_s_prim[i])/
									(sqrt(pow((x_s_prim[i]*x_s_prim[i] + y_s_prim[i]*y_s_prim[i]), 3))));

	}

	assert(p_prim_.size() == N_);
	assert(q_prim_.size() == N_);
	assert(p_.size() == N_);
	assert(q_.size() == N_);
	assert(p_sek_.size() == N_);
	assert(q_sek_.size() == N_);
	assert(n() == N_);
}

double PathInterpolated::curvature_prim(const unsigned int i) const {
	if(n() <= 1)
		return 0.;

	unsigned int i_1 = i == n() - 1 ? i : i + 1;
	unsigned int i_0 = i_1 - 1;

	// differential quotient
	return (curvature(i_1) - curvature(i_0)) / (s(i_1) - s(i_0));
}


double PathInterpolated::curvature_sek(const unsigned int i) const {
	if(n() <= 2)
		return 0.;

	unsigned int i_1 = i == n() - 1 ? i : i + 1;
	unsigned int i_0 = i_1 - 1;

	// differential quotient
	return (curvature_prim(i_1) - curvature_prim(i_0)) / (s(i_1) - s(i_0));
}

PathInterpolated::operator nav_msgs::Path() const {

	nav_msgs::Path path;
	const unsigned int length = p_.size();

	for (uint i = 0; i < length; ++i) {
		geometry_msgs::PoseStamped poza;
		poza.pose.position.x = p_[i];
		poza.pose.position.y = q_[i];
		path.poses.push_back(poza);
	}

    path.header.frame_id = frame_id_;

	return path;
}

PathInterpolated::operator SubPath() const {

    SubPath path(true);
    path.wps.resize(p_.size());
    const unsigned int length = p_.size();

    for (uint i = 0; i < length; ++i) {
        auto& wp = path.wps.at(i);
        wp.x = p_[i];
        wp.y = q_[i];
        wp.orientation = std::atan2(q_prim_[i],p_prim_[i]);
        wp.s = s_[i];
    }

    return path;
}

Path::Ptr PathInterpolated::getOriginalPath() const
{
    return original_path_;
}

void PathInterpolated::clearBuffers() {
	N_ = 0;

	s_.clear();

	p_.clear();
	q_.clear();

	p_prim_.clear();
	q_prim_.clear();

	p_sek_.clear();
	q_sek_.clear();

	s_new_ = 0;
	s_prim_ = 0;

	curvature_.clear();

	interp_path.poses.clear();
}
