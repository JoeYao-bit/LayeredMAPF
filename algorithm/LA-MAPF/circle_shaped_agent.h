//
// Created by yaozhuo on 2024/5/4.
//

#ifndef LAYEREDMAPF_CIRCLE_SHAPED_AGENT_H
#define LAYEREDMAPF_CIRCLE_SHAPED_AGENT_H
#include "common.h"
#include <ostream>
namespace freeNav::LayeredMAPF::LA_MAPF {

    template<Dimension N>
    std::pair<Pointis<N>, Pointis<N> > getCircleCoverage(const float& radius) {
        assert(radius > 0.);
        Pointis<N> retv_full, retv_part;
        DimensionLength dim[N];
        Pointi<N> center_pt;
        for(int i=0; i<N; i++) {
            dim[i] = 2*ceil(radius) + 1;
            center_pt[i] = ceil(radius);
        }
//        std::cout << "radius " << radius << std::endl;
//        std::cout << "center_pt " << center_pt << std::endl;
        Pointi<N> temp_pt;
        Pointf<N> temp_ptf;
        Id total_index = getTotalIndexOfSpace<N>(dim);
        float temp_dist, max_dist, min_dist;
        const auto& corners = getAllCornerOfGrid<N>();
        for(int id=0; id<total_index; id++) {
            temp_pt = IdToPointi<N>(id, dim);
            temp_pt = temp_pt - center_pt;
//            std::cout << "temp_pt " << temp_pt << std::endl;
            max_dist = 0, min_dist = MAX<float>;
            for(const auto& offset : corners) {
                for(int i=0; i<N; i++) {
                    temp_ptf[i] = offset[i] + temp_pt[i];
                }
                temp_dist = temp_ptf.Norm();
                if(temp_dist > max_dist) {
                    max_dist = temp_dist;
                }
                if(temp_dist < min_dist) {
                    min_dist = temp_dist;
                }
            }
//            std::cout << " max_dist " << max_dist << ", min_dist " << min_dist << std::endl;
            if(min_dist > radius) {
                if(temp_pt == Pointi<N>()) {
                    // current agent may small than a grid, considering as part occupied
                    retv_part.push_back(temp_pt);
                }
                continue;
            }
            if(max_dist < radius) {
                retv_full.push_back(temp_pt);
            } else {
//                retv_part.push_back(temp_pt);
                retv_full.push_back(temp_pt); // current considering all grid as full occupied, for simplification
            }
        }
        return {retv_full, retv_part};
    }

    template <Dimension N>
    struct CircleAgent : public Agent<N> {

        CircleAgent(float radius, int id, DimensionLength* dim)
        : Agent<N>(radius, radius, id, dim, std::string("Circle")), radius_(radius) {

            occupied_grids_ = getCircleCoverage<N>(radius);

        }

        std::shared_ptr<Agent<N> > copy() const {
            return std::make_shared<CircleAgent<N> >(CircleAgent<N>(radius_, this->id_, this->dim_));
        }

        virtual bool isCollide(const Pose<int, N>& pose,
                               DimensionLength* dim,
                               const IS_OCCUPIED_FUNC<N>& isoc,
                               const DistanceMapUpdater<N>& distance_table) const {
            if(isoc(pose.pt_)) { return true; }
            // if the block is in a grid
            if(radius_ < 0.5) { return false; }
            Id pt_id = PointiToId(pose.pt_, dim);
            if(distance_table.getClosestDistance(pt_id) <= radius_) {
                return true;
            }
            return false;
        }

        // assume have pass pose collide check
        // when add edges, assume agent can only change position or orientation, cannot change both of them
        // and orientation can only change 90 degree at one timestep, that means the offset between orient are 1, 2, 4, ... (2^0, 2^1, 2^2...)

        virtual bool isCollide(const Pose<int, N>& edge_from,
                               const Pose<int, N>& edge_to,
                               DimensionLength* dim,
                               const IS_OCCUPIED_FUNC<N>& isoc,
                               const DistanceMapUpdater<N>& distance_table) const {
            const Pointi<2> pt1 = edge_from.pt_, pt2 = edge_to.pt_;
            if(isoc(pt1) || isoc(pt2)) { return true; }
            if(pt1 == pt2) {
                // the angle of orientation change must be equal to 90 degree
                if(edge_from.orient_ / 2 == edge_to.orient_ / 2) {
                    return true;
                } else {
                    return false;
                }
            }
            if(edge_to.orient_ != edge_from.orient_) { return true; }; // if not the same position, can not change in orientation
            assert((pt1 - pt2).Norm() <= 1); // TODO: limit range of move ?
            // can only move in current orient
            Pointi<2> mov_vector = pt2 - pt1;
//            std::cout << "pt1/pt2 orient = " << pt1 << " " << pt2 << " " << edge_from.orient_ << std::endl;
//            std::cout << "mov_vector[edge_to.orient_/2] " << mov_vector[edge_to.orient_/2] << std::endl;
//            if(mov_vector[edge_to.orient_/2] == 0) { return true; }
            // must move in current orientation
            if(mov_vector[edge_to.orient_/2] == 0) { return true; }
            // move forward
            if(edge_to.orient_ % 2 == 0) {
                if(mov_vector[edge_to.orient_/2] != 1) { return true;}
            } else {
                // move backward
                if(mov_vector[edge_to.orient_/2] != -1) { return true;}
            }

            // now do not considering any angle transformation
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

        Pointis<N> getTransferOccupiedGrid(const Pose<int, N>& edge_from,
                                           const Pose<int, N>& edge_to) const override {
            if(edge_from.orient_ == edge_to.orient_) {
                std::pair<Pointis<N>, Pointis<N>> pts1 = getPoseOccupiedGrid(edge_from),
                                                  pts2 = getPoseOccupiedGrid(edge_to);
                pts1.first.insert(pts1.first.end(), pts2.first.begin(), pts2.first.end());
                return pts1.first;
            } else {
                return getPoseOccupiedGrid(edge_from).first;
            }
        }

        std::pair<Pointis<N>, Pointis<N>> getPoseOccupiedGrid(const Pose<int, N>& pose) const override {
            // circle have no need to rotate
            Pointis<2> retv_full = {}, retv_part = {};
            for(const auto& grid : occupied_grids_.first) {
                retv_full.push_back(grid + pose.pt_);
            }
            for(const auto& grid : occupied_grids_.second) {
                retv_part.push_back(grid + pose.pt_);
            }
            return {retv_full, retv_part};
        }

        std::string serialize() const {
            std::stringstream ss;
            ss << this->type_ << " " << this->id_ << " "
               << radius_ << " ";
            return ss.str();
        }

        // serialize as a mapf problem
        std::string serialize(const Pose<int, N>& start_pose, const Pose<int, N>& target_pose) const {
            std::stringstream ss;
            ss << this->type_ << " " << this->id_ << " "
               << radius_ << " ";
            for(int i=0; i<N; i++) {
                ss << start_pose.pt_[i] << " ";
            }
            ss << start_pose.orient_ << " ";
            for(int i=0; i<N; i++) {
                ss << target_pose.pt_[i] << " ";
            }
            ss << target_pose.orient_;
            return ss.str();
        }

        static std::pair<AgentPtr<N>, InstanceOrient<N> > deserialize(const std::string& string, DimensionLength* dim) {
            std::vector<std::string> strs;
            boost::split(strs, string, boost::is_any_of(" "), boost::token_compress_on);
            assert(strs.size() == 5 + 2*N);
            int id = atoi(strs[1].c_str());
            Pointi<N> start_pt, target_pt;
            for(int i=0; i<N; i++) {
                start_pt[i]  = atoi(strs[i+3].c_str());
                target_pt[i] = atoi(strs[i+3 + N+1].c_str());
            }

            Pose<int, N> start_pose (start_pt,  atoi(strs[3 + N].c_str()));
            Pose<int, N> target_pose(target_pt, atoi(strs[4 + 2*N].c_str()));

            auto agent = std::make_shared<CircleAgent<N> >(atof(strs[2].c_str()), id, dim);
            return {agent, {start_pose, target_pose}};
        }

        float radius_ = 1.;

        // which grid the agent occupied, include full occupied and partially occupied
        std::pair<Pointis<N>, Pointis<N> > occupied_grids_;

    };



    // detect collision between agents

    // check whether two moving circle are collide with each other
    bool isCollide(const CircleAgent<2>& a1, const Pose<int, 2>& s1, const Pose<int, 2>& e1,
                   const CircleAgent<2>& a2, const Pose<int, 2>& s2, const Pose<int, 2>& e2);

    // check whether one moving circle are collide with one waiting circle
    bool isCollide(const CircleAgent<2>& a1, const Pose<int, 2>& s1, const Pose<int, 2>& e1,
                   const CircleAgent<2>& a2, const Pose<int, 2>& s2);

    // check whether one moving circle are collide with one waiting circle
    bool isCollide(const CircleAgent<2>& a1, const Pose<int, 2>& s1,
                   const CircleAgent<2>& a2, const Pose<int, 2>& s2, const Pose<int, 2>& e2);

    bool isCollide(const CircleAgent<2>& a1, const Pose<int, 2>& s1,
                   const CircleAgent<2>& a2, const Pose<int, 2>& s2);

    template <Dimension N>
    using CircleAgents = std::vector<CircleAgent<N> >;


}

#endif //LAYEREDMAPF_CIRCLE_SHAPED_AGENT_H
