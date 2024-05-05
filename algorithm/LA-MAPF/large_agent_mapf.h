//
// Created by yaozhuo on 2024/5/5.
//

#ifndef LAYEREDMAPF_LARGE_AGENT_MAPF_H
#define LAYEREDMAPF_LARGE_AGENT_MAPF_H

#include "shaped_agent.h"

namespace freeNav::LayeredMAPF::LA_MAPF {

//    template<Dimension N>
//    struct SubGraphNode;
//
//    template<Dimension N>
//    using SubGraphNodePtr = SubGraphNode<N>*;
//
//    template<Dimension N>
//    struct SubGraphNode {
//        PosePtr<N>* pose_;
//        std::vector<SubGraphNodePtr<N> > neighbor_;
//    };
//
    template<Dimension N>
    struct SubGraphOfAgent {
        std::vector<PosePtr<N> > all_poses_;
        std::vector<std::vector<Id> > all_edges_;
    };


    template<Dimension N>
    class LargeAgentMAPF {
    public:
        LargeAgentMAPF(const Instances<N> & instances,
                       const Agents<N>& agents,
                       DimensionLength* dim,
                       const IS_OCCUPIED_FUNC<N> & isoc) : instances_(instances), agents_(agents), dim_(dim), isoc_(isoc),
                                                           distance_map_updater_(DistanceMapUpdater<N>(this->isoc_, this->dim_)) {
            assert(instances.size() == agents.size());

            // 1, construct distance map for current map

            // 2, construct all possible poses
            Id total_index = getTotalIndexOfSpace<N>(dim_);
            all_poses_.resize(total_index*2*N, nullptr); // a position with 2*N orientation
            Pointi<N> pt;
            for(Id id=0; id<total_index; id++) {
                 pt = IdToPointi<N>(id, dim_);
                 if(!isoc_(pt)) {
                     for(int orient=0; orient<2*N; orient++) {
                         PosePtr<N> pose_ptr = new Pose<N>(pt, orient);
                         all_poses_[id*2*N + orient] = pose_ptr;
                     }
                 }
            }

            // 3, construct each agents subgraph (vertex: where it can stay, edges: whether can it move from one vertex to another)
            agent_sub_graphs_.reserve(instances_.size());
            for(const auto& agent : agents) {
                agent_sub_graphs_.push_back(constructSubGraphOfAgent(agent));
            }
        }

        virtual bool solve() = 0;

        virtual Path<N> getSolution() const {
            return solution_;
        }

        SubGraphOfAgent<N> constructSubGraphOfAgent(const Agent<N>* agent) const {
            Id total_index = getTotalIndexOfSpace<N>(dim_);
            assert(all_poses_.size() == total_index*2*N);
            SubGraphOfAgent<N> sub_graph;
            sub_graph.all_poses_.resize(total_index*2*N, nullptr);
            // initial nodes in subgraph
            for(Id id=0; id<total_index; id++) {
                if(all_poses_[id*2*N] != nullptr) {
                    // if current grid is free, then check all orientation
                    for(int orient=0; orient<2*N; orient ++) {
                        const auto& current_pose = all_poses_[id * 2 * N + orient];
                        if(!agent->isCollide(*current_pose, dim_, isoc_, distance_map_updater_)) {
                            sub_graph.all_poses_[id * 2 * N + orient] = current_pose;
                        }
                    }
                }
            }
            // when add edges, assume agent can only change position or orientation, cannot change both of them
            // and orientation can only change 90 degree at one timestep, that means the two orient must be orthogonal
            sub_graph.all_edges_.resize(total_index*2*N, {});
            Pointis<N> neighbors = GetNearestOffsetGrids<N>();
            for(int pose_id=0; pose_id < sub_graph.all_poses_.size(); pose_id++) {
                const auto& node_ptr = sub_graph.all_poses_[pose_id];
                if(node_ptr != nullptr) {
                    // add edges about position changing
                    int origin_orient = pose_id%(2*N);
                    Pointi<N> origin_pt = node_ptr->pt_;
                    Pointi<N> new_pt;
                    for(const auto& offset : neighbors) {
                        new_pt = origin_pt + offset;
                        if(isOutOfBoundary(new_pt, dim_)) { continue; }
                        Id another_node_id = PointiToId<N>(new_pt, dim_)*2*N + origin_orient;
                        PosePtr<N> another_node_ptr = sub_graph.all_poses_[another_node_id];
                        if(another_node_ptr == nullptr) { continue; }
                        if(!agent->isCollide(*node_ptr, *another_node_ptr, dim_, isoc_, distance_map_updater_)) {
                            sub_graph.all_edges_[pose_id].push_back(another_node_id);
                        }
                    }
                    // add edges about orientation changing
                    int base_id = pose_id / (2*N) * (2*N);
                    for(int orient=0; orient<2*N; orient++) {
                        // avoid repeat itself
                        if(node_ptr->orient_ == orient) { continue; }
                        // if another node in subgraph
                        Id another_node_id = base_id + orient;
                        PosePtr<N> another_node_ptr = sub_graph.all_poses_[another_node_id];
                        if(another_node_ptr == nullptr) { continue; }
                        // check whether can transfer to another node
                        if(!agent->isCollide(*node_ptr, *another_node_ptr, dim_, isoc_, distance_map_updater_)) {
                            sub_graph.all_edges_[pose_id].push_back(another_node_id);
                        }
                    }
                }
            }
            return sub_graph;
        }

        ~LargeAgentMAPF() {
            // release pose ptrs
            for(auto& pose_ptr : all_poses_) {
                if(pose_ptr != nullptr) {
                    delete pose_ptr;
                    pose_ptr = nullptr;
                }
            }
        }

    //protected:
        const Instances<N> instances_;
        const Agents<N>& agents_;
        DimensionLength* dim_;
        const IS_OCCUPIED_FUNC<N>& isoc_;

        std::vector<PosePtr<N> > all_poses_;
        DistanceMapUpdater<N> distance_map_updater_;

        std::vector<SubGraphOfAgent<N> > agent_sub_graphs_;

        Path<N> solution_;

    };


}

#endif //LAYEREDMAPF_LARGE_AGENT_MAPF_H
