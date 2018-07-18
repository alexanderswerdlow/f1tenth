#ifndef REEDSSHEPPEXPANSION_HPP
#define REEDSSHEPPEXPANSION_HPP

/// PROJECT
#include "../ReedsShepp/CurveGenerator.h"

/// SYSTEM
#include <boost/foreach.hpp>

namespace lib_path {

template <int N, bool exp_forward, bool exp_backward>
struct ReedsSheppExpansion
{
    ReedsSheppExpansion()
    {
        current = N;
        n = N;

        initial_cost = -1.0;

        if(exp_forward) {
            forwards_generator.parse("LSL");
            forwards_generator.parse("LSR");
            forwards_generator.parse("RSL");
            forwards_generator.parse("RSR");

            forwards_generator.set_max_curve_arc(M_PI);
        }

        if(exp_backward) {
            backwards_generator.parse("|LSL");
            backwards_generator.parse("|LSR");
            backwards_generator.parse("|RSL");
            backwards_generator.parse("|RSR");

            backwards_generator.set_max_curve_arc(M_PI);
        }
    }

    template <class T, class Map>
    bool expand(const T* start, const T* goal, const Map* map_ptr) {
        forward = start->forward;

        if(!exp_forward) {
            if(forward) {
                return false;
            }
        }
        if(!exp_backward) {
            if(!forward) {
                return false;
            }
        }


        ++current;

        double cost = start->h;
        if(initial_cost < 0) {
            initial_cost = cost;
            min_cost = cost;
            return false;
        }

        if(cost < min_cost) {
            min_cost = cost;

            n = N * (min_cost / initial_cost);
        }

        if(current < n) {
            return false;
        }

        current = 0;
        if(forward) {
            return find(forwards_generator, start, goal, map_ptr);
        } else {
            return false;
            //            return find(backwards_generator, start, goal, map_ptr);
        }
    }


    template <class T, class Map>
    bool find(CurveGenerator& generator, const T* start, const T* goal, const Map* map_ptr) {
        float radius = 1.3f /*meter*/;
        float max_dist = 0.15f /*meter*/;

        generator.set_circle_radius(radius);
        generator.set_max_waypoint_distance(max_dist);

        generator.set_cost_backwards(2.0);
        generator.set_cost_curve(1.3);

        lib_path::Curve* curve = generator.find_path(*start, *goal, map_ptr, false);

        if(!curve) {
            return false;
        }

        poses.clear();
        curve->reset_iteration();
        while(curve->has_next()) {
            Pose2d n = curve->next();
            poses.push_back(n);
        }

        bool result = curve->is_valid();
        delete curve;
        return result;
    }

    template <class PathT>
    void get(PathT* out)
    {
        typedef typename PathT::value_type NodeT;
        BOOST_FOREACH(const Pose2d& pose, poses) {
            NodeT node;
            NodeT::init(node, pose.x, pose.y, pose.theta, forward);
            out->push_back(node);
        }
    }

    static ReedsSheppExpansion& instance()
    {
        static ReedsSheppExpansion inst;
        return inst;
    }

    template <class T, class Map>
    static bool canExpand(const T* start, const T* goal, const Map* map_ptr)
    {
        return instance().expand(start,goal, map_ptr);
    }

    template <class PathT>
    static void getPath(PathT* out)
    {
        instance().get(out);
    }

private:
    CurveGenerator forwards_generator;
    CurveGenerator backwards_generator;
    std::vector<Pose2d> poses;

    double initial_cost;
    double min_cost;

    bool forward;
    int current;
    int n;
};

}

#endif // REEDSSHEPPEXPANSION_HPP
