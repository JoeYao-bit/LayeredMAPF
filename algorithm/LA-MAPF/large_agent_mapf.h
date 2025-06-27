//
// Created by yaozhuo on 2024/5/5.
//

#ifndef LAYEREDMAPF_LARGE_AGENT_MAPF_H
#define LAYEREDMAPF_LARGE_AGENT_MAPF_H

#include "common.h"

#include <boost/heap/pairing_heap.hpp>
#include <sys/time.h>

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
    std::string printPath(const LAMAPF_Path& path, const std::vector<PosePtr<int, N> >& all_poses) {
        std::stringstream ss;
        for(int t=0; t<path.size(); t++) {
            ss << *(all_poses[path[t]]) << "->";
        }
        return ss.str();
    }

    // state means agent's state
    template<Dimension N, typename State>
    class LargeAgentMAPF {
    public:
        LargeAgentMAPF(const std::vector<std::pair<State, State> > & instances,
                       const std::vector<AgentPtr<N> >& agents,
                       DimensionLength* dim,
                       const IS_OCCUPIED_FUNC<N> & isoc,

                       const std::vector<PosePtr<int, N> >& all_poses,
                       const DistanceMapUpdaterPtr<N>& distance_map_updater,
                       const std::vector<SubGraphOfAgent<N, State> >& agent_sub_graphs,
                       const std::vector<std::vector<int> >& agents_heuristic_tables,
                       const std::vector<std::vector<int> >& agents_heuristic_tables_ignore_rotate,

                       double time_limit = 60,
                       int num_of_CPU = 4
                       ) : instances_(instances), agents_(agents), dim_(dim), isoc_(isoc),
                           all_poses_(all_poses), distance_map_updater_(distance_map_updater),
                           agent_sub_graphs_(agent_sub_graphs),
                           agents_heuristic_tables_(agents_heuristic_tables),
                           agents_heuristic_tables_ignore_rotate_(agents_heuristic_tables_ignore_rotate),
                           time_limit_(time_limit),
                           remaining_time_(time_limit)
                       {

            assert(instances.size() == agents.size());

        }

        virtual bool solve(int cost_lowerbound = 0, int cost_upperbound = MAX_COST) = 0;

        virtual std::vector<LAMAPF_Path> getSolution() const {
            return solutions_;
        }

        PosePaths<N> transferToPosePaths(const std::vector<LAMAPF_Path>& paths) const {
            PosePaths<N> retv;
            for(const auto& path : paths) {
                retv.push_back(transferToPosePath(path));
            }
            return retv;
        }

        PosePath<N> transferToPosePath(const LAMAPF_Path& path) const {
            PosePath<N> retv;
            for(const auto& node_id : path) {
                retv.push_back(transferToPose());
            }
            return retv;
        }

        Pose<int, N> transferToPose(const size_t& node_id) const {
            assert(all_poses_[node_id] != nullptr);
            return *all_poses_[node_id];
        }

        bool solutionValidation() const {
            for(int a1=0; a1<this->agents_.size(); a1++) {
                for(int a2=a1+1; a2<this->agents_.size(); a2++) {

                    const auto& p1 = solutions_[a1], p2 = solutions_[a2];
                    const auto& all_nodes = all_poses_;
                    int t1 = p1.size()-1, t2 = p2.size()-1;
                    const auto& longer_agent  = p1.size() > p2.size() ? agents_[a1] : agents_[a2];
                    const auto& shorter_agent = longer_agent->id_ == agents_[a1]->id_ ? agents_[a2] : agents_[a1];
                    const auto& longer_path   = longer_agent->id_ == agents_[a1]->id_ ? p1 : p2;
                    const auto& shorter_path  = longer_agent->id_ == agents_[a1]->id_ ? p2 : p1;

                    int common_part = std::min(t1, t2);
                    std::vector<std::shared_ptr<Conflict> > cfs;
                    for(int t=0; t<common_part-1; t++) {
                        if(isCollide(agents_[a1], *all_nodes[p1[t]], *all_nodes[p1[t+1]],
                                     agents_[a2], *all_nodes[p2[t]], *all_nodes[p2[t+1]])) {
                            return false;
                        }
                    }
                    for(int t=common_part-1; t<std::max(t1, t2) - 1; t++) {
                        if(isCollide(longer_agent, *all_nodes[longer_path[t]], *all_nodes[longer_path[t+1]],
                                     shorter_agent, *all_nodes[shorter_path.back()])) {
                            return false;
                        }
                    }
                }
            }
            return true;
        }


        size_t getMakeSpan() const {
            if(!this->solvable) { return 0; }
            size_t mk = 0;
            for (size_t a1 = 0; a1 < this->instances_.size(); a1++) {
                mk = std::max(this->solutions_[a1].size(), mk); // yz: soc: sum of cost
            }
            return mk;
        }

        size_t getSOC() const {
            if(!this->solvable) { return 0; }
            size_t soc = 0;
            for (size_t a1 = 0; a1 < this->instances_.size(); a1++) {
                soc += this->solutions_[a1].size() - 1; // yz: soc: sum of cost
            }
            return soc;
        }

        std::vector<PosePtr<int, N> > getAllPoses() const {
            return all_poses_;
        }

        void removeStopAfterReachTarget(std::vector<LAMAPF_Path>& retv) const {
            // yz: remove way point when agent is stop
            for (int agent = 0; agent < this->instance_node_ids_.size(); agent++) {
                const size_t & target = retv[agent].back();
                auto &path = retv[agent];
//            std::cout << "before prune" << path << std::endl;
                for (auto iter = path.end(); iter != path.begin();) {
                    if (*(iter - 2) == target) {
                        iter = path.erase(iter - 1);
                    } else {
                        break;
                    }
                }
//            std::cout << "target " << target << " instance_sat[agent].second " << instance_sat[agent].second << std::endl;
//            std::cout << "start " << path.front() << " instance_sat[agent].first " << instance_sat[agent].first << std::endl;
                assert(path.front() == this->instance_node_ids_[agent].first);
                assert(path.back() == this->instance_node_ids_[agent].second);
//            std::cout << "after prune" << path << std::endl;
            }
        }

        ~LargeAgentMAPF() {
            // release pose ptrs
//            for(auto& pose_ptr : all_poses_) {
//                if(pose_ptr != nullptr) {
//                    delete pose_ptr;
//                    pose_ptr = nullptr;
//                }
//            }
        }

        bool solvable = true;

    //protected:
        // initial constant values
        std::vector<std::pair<State, State> > instances_;
        std::vector<AgentPtr<N> > agents_;
        DimensionLength* dim_;
        const IS_OCCUPIED_FUNC<N>& isoc_;

        std::vector<std::pair<size_t, size_t> > instance_node_ids_;

        // intermediate variables
        std::vector<PosePtr<int, N> > all_poses_;
        DistanceMapUpdaterPtr<N> distance_map_updater_;

        std::vector<SubGraphOfAgent<N, State> > agent_sub_graphs_;
        std::vector<std::vector<int> > agents_heuristic_tables_;
        std::vector<std::vector<int> > agents_heuristic_tables_ignore_rotate_;

        // solutions
        std::vector<LAMAPF_Path> solutions_, initial_solutions_;

        double subgraph_and_heuristic_time_cost_ = 0; // time cost of get subgraph and heuristic table for each agent

        double time_limit_ = 60; // s
        double remaining_time_ = 0; // s

        MSTimer mst_;

    };


}

#endif //LAYEREDMAPF_LARGE_AGENT_MAPF_H
