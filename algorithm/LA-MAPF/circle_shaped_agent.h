//
// Created by yaozhuo on 2024/5/4.
//

#ifndef LAYEREDMAPF_CIRCLE_SHAPED_AGENT_H
#define LAYEREDMAPF_CIRCLE_SHAPED_AGENT_H
#include "common.h"
#include "../../freeNav-base/visualization/canvas/canvas.h"
#include <ostream>
namespace freeNav::LayeredMAPF::LA_MAPF {

    template <Dimension N>
    struct CircleAgent : public Agent<N> {

        CircleAgent(float radius, int id) : Agent<N>(radius, radius, id), radius_(radius) {}

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

        // serialize as a mapf problem
        std::string serialize(const Pose<int, N>& start_pose, const Pose<int, N>& target_pose) const {
            std::stringstream ss;
            ss << this->id_ << " "
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

        static std::pair<CircleAgent<N>, InstanceOrient<N> > deserialize(const std::string& string, DimensionLength* dim) {
            std::vector<std::string> strs;
            boost::split(strs, string, boost::is_any_of(" "), boost::token_compress_on);
            assert(strs.size() == 4 + 2*N);
            int id = atoi(strs[0].c_str());
            Pointi<N> start_pt, target_pt;
            for(int i=0; i<N; i++) {
                start_pt[i]  = atoi(strs[i+2].c_str());
                target_pt[i] = atoi(strs[i+2 + N+1].c_str());
            }

            Pose<int, N> start_pose (start_pt,  atoi(strs[2 + N].c_str()));
            Pose<int, N> target_pose(target_pt, atoi(strs[3 + 2*N].c_str()));

            CircleAgent<N> agent(atof(strs[1].c_str()), id);
            return {agent, {start_pose, target_pose}};
        }

        float radius_ = 1.;

    };

    template<Dimension N>
    std::ostream& operator << (std::ostream& os, const CircleAgent<N>& circle) {
        os << "CircleAgent " << circle.id_ << ": "<< circle.radius_ << " ";
        return os;
    }

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

    void DrawOnCanvas(const CircleAgent<2>& circle, const Pose<int, 2>& pose,
                      Canvas& canvas, const cv::Vec3b& color = cv::Vec3b::all(0));

}

#endif //LAYEREDMAPF_CIRCLE_SHAPED_AGENT_H
