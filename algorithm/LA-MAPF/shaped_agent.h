//
// Created by yaozhuo on 2024/5/4.
//

#ifndef LAYEREDMAPF_SHAPED_AGENT_H
#define LAYEREDMAPF_SHAPED_AGENT_H
#include "common.h"

namespace freeNav::LayeredMAPF::LA_MAPF {

#define MAX_TIMESTEP INT_MAX / 2
#define MAX_COST INT_MAX / 2
#define MAX_NODES INT_MAX / 2



    template <Dimension N>
    struct CircleAgent : public Agent<N> {

        CircleAgent(float radius, int id) : Agent<N>(radius, radius, id), radius_(radius) {}

        virtual bool isCollide(const Pose<N>& pose,
                               DimensionLength* dim,
                               const IS_OCCUPIED_FUNC<N>& isoc,
                               const DistanceMapUpdater<N>& distance_table) const {
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

        virtual bool isCollide(const Pose<N>& edge_from,
                               const Pose<N>& edge_to,
                               DimensionLength* dim,
                               const IS_OCCUPIED_FUNC<N>& isoc,
                               const DistanceMapUpdater<N>& distance_table) const {
            const Pointi<N> pt1 = edge_from.pt_, pt2 = edge_to.pt_;
            if(isoc(pt1) || isoc(pt2)) { return true; }
            if(radius_ < 0.5) { return false; }
            if(pt1 == pt2) {
                // the angle of orientation change must be minus than 90 degree
                if(edge_from.orient_ / 2 == edge_to.orient_ / 2) {
                    return true;
                } else {
                    return false;
                }
            } // any orientation change are permit, as this is a circle
            if(edge_to.orient_ != edge_from.orient_) { return true; }; // if not the same position, can not change in orientation
            assert((pt1 - pt2).Norm() <= 1); // limit range of move ?
            // can only move in current orient
            Pointi<N> mov_vector = pt2 - pt1;
//            std::cout << "pt1/pt2 orient = " << pt1 << " " << pt2 << " " << edge_from.orient_ << std::endl;
//            std::cout << "mov_vector[edge_to.orient_/2] " << mov_vector[edge_to.orient_/2] << std::endl;
//            if(mov_vector[edge_to.orient_/2] == 0) { return true; }

            if(edge_to.orient_ % 2 == 0) {
                if(mov_vector[edge_to.orient_/2] != -1) { return true;}
            } else {
                if(mov_vector[edge_to.orient_/2] != 1) { return true;}
            }

//            Line<N> line(pt1, pt2);
//            int check_step = line.step;
//            Pointi<N> pt;
//            for(int i=1; i<check_step; i++) {
//                pt = line.GetPoint(i);
//                Id pt_id = PointiToId(pt, dim);
//                if(distance_table.getClosestDistance(pt_id) <= radius_) {
//                    return true;
//                }
//            }
            return false;
        }

        const float radius_ = 1.;

    };

    template <Dimension N>
    using CircleAgents = std::vector<CircleAgent<N> >;

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
        //std::cout << " bg::distance(seg1, seg2) = " << bg::distance(seg1, seg2) << std::endl;
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
        //std::cout << " bg::distance(seg1, pt3) = " << bg::distance(seg1, pt3) << std::endl;
        return bg::distance(seg1, pt3) <= (a1.radius_ + a2.radius_);
    }

    bool isCollide(const CircleAgent<2>& a1, const Pose<2>& s1,
                   const CircleAgent<2>& a2, const Pose<2>& s2, const Pose<2>& e2) {
        return isCollide(a2, s2, e2, a1, s1);
    }


    template<Dimension N>
    std::vector<std::shared_ptr<Conflict> > detectFirstConflictBetweenPaths(const LAMAPF_Path& p1, const LAMAPF_Path& p2,
                                                          const CircleAgent<N>& a1, const CircleAgent<N>& a2,
                                                          const std::vector<Pose<N>*>& all_nodes) {
        int t1 = p1.size()-1, t2 = p2.size()-1;
        const auto& longer_agent  = p1.size() > p2.size() ? a1 : a2;
        const auto& shorter_agent = longer_agent.id_ == a1.id_ ? a2 : a1;
        const auto& longer_path   = longer_agent.id_ == a1.id_ ? p1 : p2;
        const auto& shorter_path  = longer_agent.id_ == a1.id_ ? p2 : p1;

        int common_part = std::min(t1, t2);
        for(int t=0; t<common_part-1; t++) {
            if(isCollide(a1, *all_nodes[p1[t]], *all_nodes[p1[t+1]], a2, *all_nodes[p2[t]], *all_nodes[p2[t+1]])) {
                auto c1 = std::make_shared<Constraint>(a1.id_, p1[t], p1[t+1], t, t+2);
                auto c2 = std::make_shared<Constraint>(a2.id_, p1[t], p1[t+1], t, t+2);
                auto cf = std::make_shared<Conflict>(a1.id_, a2.id_, Constraints{c1}, Constraints{c2});
                return { cf };
            }
        }
        for(int t=common_part-1; t<std::max(t1, t2) - 1; t++) {
            if(isCollide(longer_agent, *all_nodes[longer_path[t]], *all_nodes[longer_path[t+1]], shorter_agent, *all_nodes[shorter_path.back()])) {
                auto c1 = std::make_shared<Constraint>(longer_agent.id_,  longer_path[t], longer_path[t+1], t, t+2);
                auto c2 = std::make_shared<Constraint>(shorter_agent.id_, shorter_path.back(), MAX_NODES,   0, t+2);
                auto cf = std::make_shared<Conflict>(longer_agent.id_, shorter_agent.id_, Constraints{c1}, Constraints{c2});
                return { cf };
            }
        }
        return {};
    }

    template<Dimension N>
    std::vector<std::shared_ptr<Conflict> > detectAllConflictBetweenPaths(const LAMAPF_Path& p1, const LAMAPF_Path& p2,
                                                        const CircleAgent<N>& a1, const CircleAgent<N>& a2,
                                                        const std::vector<Pose<N>*>& all_nodes) {
        int t1 = p1.size()-1, t2 = p2.size()-1;
        const auto& longer_agent  = p1.size() > p2.size() ? a1 : a2;
        const auto& shorter_agent = longer_agent.id_ == a1.id_ ? a2 : a1;
        const auto& longer_path   = longer_agent.id_ == a1.id_ ? p1 : p2;
        const auto& shorter_path  = longer_agent.id_ == a1.id_ ? p2 : p1;

        int common_part = std::min(t1, t2);
        std::vector<std::shared_ptr<Conflict> > cfs;
        for(int t=0; t<common_part-1; t++) {
            if(isCollide(a1, *all_nodes[p1[t]], *all_nodes[p1[t+1]],
                         a2, *all_nodes[p2[t]], *all_nodes[p2[t+1]])) {

//                std::cout << "cs type 1 : " << *all_nodes[p1[t]] << "->" << *all_nodes[p1[t+1]] << ", "
//                                            << *all_nodes[p2[t]] << "->" << *all_nodes[p2[t+1]]
//                                            << "/t:{" << t << "," << t+1 << "}" << std::endl;

                auto c1 = std::make_shared<Constraint>(a1.id_, p1[t], p1[t+1], t, t+2);
                auto c2 = std::make_shared<Constraint>(a2.id_, p2[t], p2[t+1], t, t+2);
                auto cf = std::make_shared<Conflict>(a1.id_, a2.id_, Constraints{c1}, Constraints{c2});
                cfs.push_back(cf);
            }
        }
        for(int t=common_part-1; t<std::max(t1, t2) - 1; t++) {
            if(isCollide(longer_agent, *all_nodes[longer_path[t]], *all_nodes[longer_path[t+1]],
                         shorter_agent, *all_nodes[shorter_path.back()])) {

//                std::cout << "cs type 2 : " << *all_nodes[longer_path[t]] << "->" << *all_nodes[longer_path[t+1]] << ", "
//                                            << *all_nodes[shorter_path.back()]
//                                            << "/t:{" << t << "," << t+1 << "}"
//                                            << std::endl;

                auto c1 = std::make_shared<Constraint>(longer_agent.id_,  longer_path[t],      longer_path[t+1], t, t+2);
                auto c2 = std::make_shared<Constraint>(shorter_agent.id_, shorter_path.back(), MAX_NODES,        0, t+2);
                auto cf = std::make_shared<Conflict>(longer_agent.id_, shorter_agent.id_, Constraints{c1}, Constraints{c2});
                cfs.push_back(cf);
            }
        }
        return cfs;
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
