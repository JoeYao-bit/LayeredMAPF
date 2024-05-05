//
// Created by yaozhuo on 2024/5/4.
//

#ifndef LAYEREDMAPF_SHAPED_AGENT_H
#define LAYEREDMAPF_SHAPED_AGENT_H
#include <boost/geometry.hpp>
#include "../freeNav-base/basic_elements/distance_map_update.h"
#include "../../freeNav-base/basic_elements/point.h"

namespace freeNav::LayeredMAPF::LA_MAPF {

    // distance from current position to obstacles
    // if inside obstacle, = 0
    // for accelerate collision check between obstacles and agents
    typedef std::vector<float> DistanceTable;

    template <Dimension N>
    struct Pose {
        Pose(Pointi<N> pt, int orient) : pt_(pt), orient_(orient) {}

        Pointi<N> pt_;

        // for a N dimensional space, there are 2*N orientation
        // e.g., 0,1,2,3 for 2D, 0,1,2,3,4,5,6,7 for 3D
        // int orient can be translated into a Orthogonal vector:
        // e.g., 2D: 0 -> (-1, 1),    1 -> (1, 0),    2 -> (0, -1),    3 -> (0, 1)
        //       3D: 0 -> (-1, 0, 0), 1 -> (1, 0, 0), 2 -> (0, -1, 0), 3 -> (0, 1, 0), 4 -> (0, 0, -1), 5 -> (0, 0, 1),
        int orient_ = 0;
    };

    template <Dimension N>
    using PosePtr = Pose<N>*;

    template <Dimension N>
    struct Agent {

        virtual bool isCollide(const Pose<N> pose, DimensionLength* dim, const IS_OCCUPIED_FUNC<N>& isoc, const DistanceMapUpdater<N>& distance_table) const = 0;

        virtual bool isCollide(const Pose<N>& edge_from, const Pose<N>& edge_to, DimensionLength* dim, const IS_OCCUPIED_FUNC<N>& isoc, const DistanceMapUpdater<N>& distance_table) const = 0;

    };

    template <Dimension N>
    using Agents = std::vector<Agent<N>* >;

    template <Dimension N>
    struct CircleAgent {

        CircleAgent(float radius):radius_(radius) {}

        virtual bool isCollide(const Pose<N>& pose, DimensionLength* dim,
                               const IS_OCCUPIED_FUNC<N>& isoc, const DistanceMapUpdater<N>& distance_table) const {
            if(isoc(pose.pt_)) { return true; }
            if(radius_ < 0.5) {
                return false;
            }
            Id pt_id = PointiToId(pose.pt_, dim);
            if(distance_table.getClosestDistance(pt_id) <= radius_) {
                return true;
            }
            return false;
        }

        // when add edges, assume agent can only change position or orientation, cannot change both of them
        // and orientation can only change 90 degree at one timestep, that means the offset between orient are 1, 2, 4, ... (2^0, 2^1, 2^2...)

        virtual bool isCollide(const Pose<N>& edge_from, const Pose<N>& edge_to,
                               DimensionLength* dim,
                               const IS_OCCUPIED_FUNC<N>& isoc, const DistanceMapUpdater<N>& distance_table) const {
            const Pointi<N> pt1 = edge_from.pt_, pt2 = edge_to.pt_;
            if(isoc(pt1) || isoc(pt1)) { return true; }
            if(radius_ < 0.5) { return false; }
            if(pt1 == pt2) { return true; } // any orientation change are permit, as this is a circle
            assert(edge_to.orient_ == edge_to.orient_); // if not the same position, can not change in orientation
            assert((pt1 - pt2).Norm() <= 1); // limit range of move ?
            Line<N> line(pt1, pt2);
            int check_step = line.step;
            Pointi<N> pt;
            for(int i=1; i<check_step; i++) {
                pt = line.GetPoint(i);
                Id pt_id = PointiToId(pt, dim);
                if(distance_table.getClosestDistance(pt_id) <= radius_) {
                    return true;
                }
            }
            return false;
        }

        const float radius_ = 1.;

    };

    // detect collision between agents

    // check whether two moving circle are collide with each other
    bool isCollide(const CircleAgent<2>& a1, const Pose<2>& s1, const Pose<2>& e1,
                   const CircleAgent<2>& a2, const Pose<2>& s2, const Pose<2>& e2) {

        namespace bg = boost::geometry;
        using bg_pt = bg::model::point<int, 2, bg::cs::cartesian>;
        using bg_seg = bg::model::segment<bg_pt>;
        // calculate the shortest distance between two segments
        bg_pt pt1(s1.pt_[0], s1.pt_[1]), pt2(e1.pt_[0], e1.pt_[1]),
              pt3(s2.pt_[0], s2.pt_[1]), pt4(e2.pt_[0], e2.pt_[1]);
        bg_seg seg1(pt1, pt2), seg2(pt3, pt4);
        std::cout << " bg::distance(seg1, seg2) = " << bg::distance(seg1, seg2) << std::endl;
        return bg::distance(seg1, seg2) <= (a1.radius_ + a2.radius_);
    }

    // check whether one moving circle are collide with one waiting circle
    bool isCollide(const CircleAgent<2>& a1, const Pose<2>& s1, const Pose<2>& e1,
                   const CircleAgent<2>& a2, const Pose<2>& s2) {

        namespace bg = boost::geometry;
        using bg_pt = bg::model::point<int, 2, bg::cs::cartesian>;
        using bg_seg = bg::model::segment<bg_pt>;
        // calculate the shortest distance between a point and a segment
        bg_pt pt1(s1.pt_[0], s1.pt_[1]), pt2(e1.pt_[0], e1.pt_[1]),
                pt3(s2.pt_[0], s2.pt_[1]);
        bg_seg seg1(pt1, pt2);
        std::cout << " bg::distance(seg1, pt3) = " << bg::distance(seg1, pt3) << std::endl;
        return bg::distance(seg1, pt3) <= (a1.radius_ + a2.radius_);
    }

    bool isCollide(const CircleAgent<2>& a1, const Pose<2>& s1,
                   const CircleAgent<2>& a2, const Pose<2>& s2, const Pose<2>& e2) {
        return isCollide(a2, s2, e2, a1, s1);
    }

    // boost seems have no distance calculation about 3D segments, compile failed
//    bool isCollide(const CircleAgent<3>& a1, const Pose<3>& s1, const Pose<3>& e1,
//                   const CircleAgent<3>& a2, const Pose<3>& s2, const Pose<3>& e2) {
//
//        namespace bg = boost::geometry;
//        using bg_pt = bg::model::point<int, 3, bg::cs::cartesian>;
//        using bg_seg = bg::model::segment<bg_pt>;
//        // calculate the shortest distance between two segments
//        bg_pt pt1(s1.pt_[0], s1.pt_[1], s1.pt_[2]), pt2(e1.pt_[0], e1.pt_[1], e1.pt_[2]),
//              pt3(s2.pt_[0], s2.pt_[1], s2.pt_[2]), pt4(e2.pt_[0], e2.pt_[1], e2.pt_[2]);
//        bg_seg seg1(pt1, pt2), seg2(pt3, pt4);
//        std::cout << " bg::distance(seg1, seg2) = " << bg::distance(seg1, seg2) << std::endl;
//        return bg::distance(seg1, seg2) <= (a1.radius_ + a2.radius_);
//    }

}

#endif //LAYEREDMAPF_SHAPED_AGENT_H
