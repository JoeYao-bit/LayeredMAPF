//
// Created by yaozhuo on 2024/5/5.
//

#ifndef LAYEREDMAPF_LARGE_AGENT_MAPF_H
#define LAYEREDMAPF_LARGE_AGENT_MAPF_H

#include "common.h"
#include <boost/heap/pairing_heap.hpp>

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
        std::vector<PosePtr<int, N> > all_nodes_;
        std::vector<std::vector<size_t> > all_edges_;
        std::vector<std::vector<size_t> > all_backward_edges_;
    };

    template<Dimension N, typename AgentType>
    class LargeAgentMAPF {
    public:
        LargeAgentMAPF(const InstanceOrients<N> & instances,
                       const std::vector<AgentType>& agents,
                       DimensionLength* dim,
                       const IS_OCCUPIED_FUNC<N> & isoc) : instances_(instances), agents_(agents), dim_(dim), isoc_(isoc),
                                                           distance_map_updater_(DistanceMapUpdater<N>(this->isoc_, this->dim_)) {
            assert(instances.size() == agents.size());

            // 0, init and final state overlap check
            for(int i=0; i<instances_.size(); i++) {
                for(int j=i+1; j<instances_.size(); j++) {
                    if(isCollide(agents_[i], instances_[i].first, agents_[j], instances_[j].first)) {
                        std::cout << " agent " << i << ", " << j << "'s start overlap" << std::endl;
                        solvable = false;
                        break;
                    }
                    if(isCollide(agents_[i], instances_[i].second, agents_[j], instances_[j].second)) {
                        std::cout << " agent " << i << ", " << j << "'s target overlap" << std::endl;
                        solvable = false;
                        break;
                    }
                }
            }

            // 1, construct all possible poses
            Id total_index = getTotalIndexOfSpace<N>(dim_);
            all_poses_.resize(total_index*2*N, nullptr); // a position with 2*N orientation
            Pointi<N> pt;
            for(Id id=0; id<total_index; id++) {
                 pt = IdToPointi<N>(id, dim_);
                 if(!isoc_(pt)) {
                     for(int orient=0; orient<2*N; orient++) {
                         PosePtr<int, N> pose_ptr = new Pose<int, N>(pt, orient);
                         all_poses_[id*2*N + orient] = pose_ptr;
                     }
                 }
            }

            // 2, construct each agents subgraph (vertex: where it can stay, edges: whether can it move from one vertex to another)
            agent_sub_graphs_.reserve(instances_.size());
            for(const auto& agent : agents) {
                agent_sub_graphs_.push_back(constructSubGraphOfAgent(agent));
            }
            // 3, construct each agent's heuristic table, i.e., distance from each node to target
            agents_heuristic_tables_.reserve(instances_.size());
            instance_node_ids_.reserve(instances_.size());
            for(int agent=0; agent<instances_.size(); agent++) {
                // check start
                int start_node_id = PointiToId<N>(instances_[agent].first.pt_, dim_)*2*N + instances_[agent].first.orient_;
                if(agent_sub_graphs_[agent].all_nodes_[start_node_id] == nullptr) {
                    std::cerr << " agent " << agent << "'s start " << instances_[agent].first.pt_ << "^" << instances_[agent].first.orient_ << " is unavailable " << std::endl;
                    solvable = false;
                    break;
                }
                // check target
                int target_node_id = PointiToId<N>(instances_[agent].second.pt_, dim_)*2*N + instances_[agent].second.orient_;
                if(agent_sub_graphs_[agent].all_nodes_[target_node_id] == nullptr) {
                    std::cerr << " agent " << agent << "'s target " << instances_[agent].second.pt_ << "^" << instances_[agent].second.orient_ << " is unavailable " << std::endl;
                    solvable = false;
                    break;
                }
                instance_node_ids_.push_back(std::make_pair(start_node_id, target_node_id));
                agents_heuristic_tables_.push_back(constructHeuristicTable(agent_sub_graphs_[agent], target_node_id));
            }
        }

        virtual bool solve(double time_limit, int cost_lowerbound = 0, int cost_upperbound = MAX_COST) = 0;

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

        SubGraphOfAgent<N> constructSubGraphOfAgent(const AgentType& agent) const {
            Id total_index = getTotalIndexOfSpace<N>(dim_);
            assert(all_poses_.size() == total_index*2*N);
            SubGraphOfAgent<N> sub_graph;
            sub_graph.all_nodes_.resize(total_index * 2 * N, nullptr);
            // initial nodes in subgraph
            for(size_t id=0; id<total_index; id++) {
                if(all_poses_[id*2*N] != nullptr) {
                    // if current grid is free, then check all orientation
                    for(int orient=0; orient<2*N; orient ++) {
                        const auto& current_pose = all_poses_[id * 2 * N + orient];
                        if(!agent.isCollide(*current_pose, dim_, isoc_, distance_map_updater_)) {
                            sub_graph.all_nodes_[id * 2 * N + orient] = current_pose;
                        }
                    }
                }
            }
            // when add edges, assume agent can only change position or orientation, cannot change both of them
            // and orientation can only change 90 degree at one timestep, that means the two orient must be orthogonal
            sub_graph.all_edges_.resize(total_index*2*N, {});
            sub_graph.all_backward_edges_.resize(total_index*2*N, {});
            Pointis<N> neighbors = GetNearestOffsetGrids<N>();
            for(size_t pose_id=0; pose_id < sub_graph.all_nodes_.size(); pose_id++) {
                const auto& node_ptr = sub_graph.all_nodes_[pose_id];
                if(node_ptr != nullptr) {
                    // add edges about position changing
                    size_t origin_orient = pose_id%(2*N);
                    Pointi<N> origin_pt = node_ptr->pt_;
                    Pointi<N> new_pt;
                    for(const auto& offset : neighbors) {
                        new_pt = origin_pt + offset;
                        if(isOutOfBoundary(new_pt, dim_)) { continue; }
                        Id another_node_id = PointiToId<N>(new_pt, dim_)*2*N + origin_orient;
                        PosePtr<int, N> another_node_ptr = sub_graph.all_nodes_[another_node_id];
                        if(another_node_ptr == nullptr) { continue; }
                        if(!agent.isCollide(*node_ptr, *another_node_ptr, dim_, isoc_, distance_map_updater_)) {
                            sub_graph.all_edges_[pose_id].push_back(another_node_id);
                            sub_graph.all_backward_edges_[another_node_id].push_back(pose_id);
                        }
                    }
                    // add edges about orientation changing
                    size_t base_id = pose_id / (2*N) * (2*N);
                    for(size_t orient=0; orient<2*N; orient++) {
                        // avoid repeat itself
                        if(node_ptr->orient_ == orient) { continue; }
                        // if another node in subgraph
                        size_t another_node_id = base_id + orient;
                        PosePtr<int, N> another_node_ptr = sub_graph.all_nodes_[another_node_id];
                        if(another_node_ptr == nullptr) { continue; }
                        // check whether can transfer to another node
                        if(!agent.isCollide(*node_ptr, *another_node_ptr, dim_, isoc_, distance_map_updater_)) {
                            sub_graph.all_edges_[pose_id].push_back(another_node_id);
                            sub_graph.all_backward_edges_[another_node_id].push_back(pose_id);
                        }
                    }
                }
            }
            return sub_graph;
        }

        std::vector<int> constructHeuristicTable(const SubGraphOfAgent<N>& sub_graph, const size_t& goal_id) const {
            struct Node {
                int node_id;
                int value;

                Node() = default;

                Node(int node_id, int value) : node_id(node_id), value(value) {}

                // the following is used to compare nodes in the OPEN list
                struct compare_node {
                    // returns true if n1 > n2 (note -- this gives us *min*-heap).
                    bool operator()(const Node &n1, const Node &n2) const {
                        return n1.value >= n2.value;
                    }
                };  // used by OPEN (heap) to compare nodes (top of the heap has min f-val, and then highest g-val)
            };

            std::vector<int> agent_heuristic(sub_graph.all_nodes_.size(), MAX_TIMESTEP);

            // generate a heap that can save nodes (and an open_handle)
            boost::heap::pairing_heap<Node, boost::heap::compare<typename Node::compare_node> > heap;

            Node root(goal_id, 0);
            agent_heuristic[goal_id] = 0;
            heap.push(root);  // add root to heap
            // yz: compute heuristic from goal to visible grid via Breadth First Search
            //     search the static shortest path
            while (!heap.empty()) {
                Node curr = heap.top();
                heap.pop();
                for (int next_location : sub_graph.all_backward_edges_[curr.node_id]) {
                    if (agent_heuristic[next_location] > curr.value + 1) {
                        agent_heuristic[next_location] = curr.value + 1;
                        Node next(next_location, curr.value + 1);
                        heap.push(next);
                    }
                }
            }
            return agent_heuristic;
        }

        bool solutionValidation() const {
            for(int a1=0; a1<this->agents_.size(); a1++) {
                for(int a2=a1+1; a2<this->agents_.size(); a2++) {

                    const auto& p1 = solutions_[a1], p2 = solutions_[a2];
                    const auto& all_nodes = all_poses_;
                    int t1 = p1.size()-1, t2 = p2.size()-1;
                    const auto& longer_agent  = p1.size() > p2.size() ? agents_[a1] : agents_[a2];
                    const auto& shorter_agent = longer_agent.id_ == agents_[a1].id_ ? agents_[a2] : agents_[a1];
                    const auto& longer_path   = longer_agent.id_ == agents_[a1].id_ ? p1 : p2;
                    const auto& shorter_path  = longer_agent.id_ == agents_[a1].id_ ? p2 : p1;

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

        void printPath(int agent, const LAMAPF_Path& path) const {
            std::cout << "agent " << agent << ": " <<  this->instances_[agent].first << "->" << this->instances_[agent].second << std::endl;
            for(int t=0; t<path.size(); t++) {
                std::cout << *(this->all_poses_[path[t]]) << "->";
            }
            std::cout << std::endl;
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
            for(auto& pose_ptr : all_poses_) {
                if(pose_ptr != nullptr) {
                    delete pose_ptr;
                    pose_ptr = nullptr;
                }
            }
        }

        bool solvable = true;

    //protected:
        // initial constant values
        InstanceOrients<N> instances_;
        std::vector<AgentType> agents_;
        DimensionLength* dim_;
        const IS_OCCUPIED_FUNC<N>& isoc_;

        std::vector<std::pair<size_t, size_t> > instance_node_ids_;

        // intermediate variables
        std::vector<PosePtr<int, N> > all_poses_;
        DistanceMapUpdater<N> distance_map_updater_;
        std::vector<SubGraphOfAgent<N> > agent_sub_graphs_;
        std::vector<std::vector<int> > agents_heuristic_tables_;

        // solutions
        std::vector<LAMAPF_Path> solutions_, initial_solutions_;

    };


}

#endif //LAYEREDMAPF_LARGE_AGENT_MAPF_H
