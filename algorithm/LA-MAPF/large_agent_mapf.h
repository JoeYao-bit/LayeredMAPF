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

    template<Dimension N>
    class LargeAgentMAPF {
    public:
        LargeAgentMAPF(const InstanceOrients<N> & instances,
                       const std::vector<AgentPtr<N> >& agents,
                       DimensionLength* dim,
                       const IS_OCCUPIED_FUNC<N> & isoc,

                       const std::vector<PosePtr<int, N> >& all_poses = {},
                       const DistanceMapUpdaterPtr<N>& distance_map_updater = nullptr,
                       const std::vector<SubGraphOfAgent<N> >& agent_sub_graphs = {},
                       const std::vector<std::vector<int> >& agents_heuristic_tables = {},
                       const std::vector<std::vector<int> >& agents_heuristic_tables_ignore_rotate = {},

                       double time_limit = 60
                       ) : instances_(instances), agents_(agents), dim_(dim), isoc_(isoc),
                           all_poses_(all_poses), distance_map_updater_(distance_map_updater),
                           agent_sub_graphs_(agent_sub_graphs),
                           agents_heuristic_tables_(agents_heuristic_tables),
                           agents_heuristic_tables_ignore_rotate_(agents_heuristic_tables_ignore_rotate),
                           time_limit_(time_limit),
                           remaining_time_(time_limit)
                       {

            start_time_ = clock();
            assert(instances.size() == agents.size());

            struct timezone tz;
            struct timeval  tv_pre;
            struct timeval  tv_after;
            gettimeofday(&tv_pre, &tz);

            // 0, init and final state overlap check
            for(int i=0; i<instances_.size(); i++) {
                for(int j=i+1; j<instances_.size(); j++) {
                    if(isCollide(agents_[i], instances_[i].first, agents_[j], instances_[j].first)) {
                        std::cout << " agent " << *agents_[i] << ", " << *agents_[j] << "'s start overlap" << std::endl;
                        solvable = false;
                        break;
                    }
                    if(isCollide(agents_[i], instances_[i].second, agents_[j], instances_[j].second)) {
                        std::cout << " agent " << *agents_[i] << ", " << *agents_[j] << "'s target overlap" << std::endl;
                        solvable = false;
                        break;
                    }
                }
            }

            // 1, construct all possible poses
            if(all_poses_.empty()) {
                std::cout << "  construct all possible poses" << std::endl;
                Id total_index = getTotalIndexOfSpace<N>(dim_);
                all_poses_.resize(total_index * 2 * N, nullptr); // a position with 2*N orientation
                Pointi <N> pt;
                for (Id id = 0; id < total_index; id++) {
                    pt = IdToPointi<N>(id, dim_);
                    if (!isoc_(pt)) {
                        for (int orient = 0; orient < 2 * N; orient++) {
                            PosePtr<int, N> pose_ptr = std::make_shared<Pose<int, N> >(pt, orient);
                            all_poses_[id * 2 * N + orient] = pose_ptr;
                        }
                    }
                }
            }
            // 2, construct each agents subgraph (vertex: where it can stay, edges: whether can it move from one vertex to another)
            if(distance_map_updater_ == nullptr) {
                std::cout << "  construct distance_map_update" << std::endl;
                distance_map_updater_ = std::make_shared<DistanceMapUpdater<N> >(this->isoc_, this->dim_);
            }


            if(agent_sub_graphs_.empty()) {
                std::cout << "  construct agent_sub_graphs" << std::endl;
                agent_sub_graphs_.reserve(instances_.size());
                for (int id = 0; id < agents.size(); id++) {
                    agent_sub_graphs_.push_back(constructSubGraphOfAgent(id));
                }
            }

            // debug
//            for(int i=0; i<agents.size(); i++) {
//                int count_of_nodes = 0;
//                for(const auto& ptr : agent_sub_graphs_[i].all_nodes_) {
//                    if(ptr != nullptr) {
//                        count_of_nodes ++;
//                    }
//                }
//                int count_of_edges = 0;
//                for(const auto& nodes : agent_sub_graphs_[i].all_edges_) {
//                    count_of_edges += nodes.size();
//                }
//                std::cout << "LA-MAPF Agent " << agents[i] << "'s subgraph have nodes/edges = "
//                          << count_of_edges << " / " << count_of_nodes << std::endl;
//            }

            if(instance_node_ids_.empty()) {
                instance_node_ids_.reserve(instances_.size());
                for (const auto &sub_graph : agent_sub_graphs_) {
                    instance_node_ids_.push_back({sub_graph.start_node_id_, sub_graph.target_node_id_});
                }
            }
            // 3, construct each agent's heuristic table, i.e., distance from each node to target
            if(agents_heuristic_tables_.empty()) {
                std::cout << "  construct agents_heuristic_tables" << std::endl;
                agents_heuristic_tables_.reserve(instances_.size());
                for (int agent = 0; agent < instances_.size(); agent++) {
                    agents_heuristic_tables_.push_back(
                            constructHeuristicTable(agent_sub_graphs_[agent], instance_node_ids_[agent].second));
                }
            }
           if(agents_heuristic_tables_ignore_rotate_.empty()) {
               std::cout << "  construct agents_heuristic_tables_ignore_rotate" << std::endl;
               agents_heuristic_tables_ignore_rotate_.reserve(instances_.size());
               for (int agent = 0; agent < instances_.size(); agent++) {
                   agents_heuristic_tables_ignore_rotate_.push_back(
                           constructHeuristicTableIgnoreRotate(agent_sub_graphs_[agent], instance_node_ids_[agent].second));
               }
           }
            gettimeofday(&tv_after, &tz);
            subgraph_and_heuristic_time_cost_ =
                    (tv_after.tv_sec - tv_pre.tv_sec) + (tv_after.tv_usec - tv_pre.tv_usec)/1e6;
            std::cout << "-- construct subgraph and heuristic table in " << subgraph_and_heuristic_time_cost_ << "s" << std::endl;
            remaining_time_ = remaining_time_ - subgraph_and_heuristic_time_cost_;
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


        SubGraphOfAgent<N> constructSubGraphOfAgent(int agent_id) {
            const AgentPtr<N> & agent = agents_[agent_id];
            Id total_index = getTotalIndexOfSpace<N>(dim_);

            assert(all_poses_.size() == total_index*2*N);

            // check start
            int start_node_id = PointiToId<N>(instances_[agent_id].first.pt_, dim_) * 2 * N +
                                instances_[agent_id].first.orient_;

            // check target
            int target_node_id = PointiToId<N>(instances_[agent_id].second.pt_, dim_) * 2 * N +
                                 instances_[agent_id].second.orient_;

            instance_node_ids_.push_back(std::make_pair(start_node_id, target_node_id));


            SubGraphOfAgent<N> sub_graph(agent);
            sub_graph.data_ptr_ = std::make_shared<SubGraphOfAgentData<N> >();

            sub_graph.data_ptr_->all_nodes_.resize(total_index * 2 * N, nullptr);
            sub_graph.start_node_id_ = start_node_id;
            sub_graph.target_node_id_ = target_node_id;

            // initial nodes in subgraph
            for(size_t id=0; id<all_poses_.size(); id++) {
                if(all_poses_[id] != nullptr) {
                    const auto& current_pose = all_poses_[id];
                    if(!agent->isCollide(*current_pose, dim_, isoc_, *distance_map_updater_)) {
                        sub_graph.data_ptr_->all_nodes_[id] = current_pose;
                    }
                }
            }

            if (sub_graph.data_ptr_->all_nodes_[start_node_id] == nullptr) {
                std::cout << "FATAL: agent " << agent << "'s start " << instances_[agent_id].first.pt_ << "^"
                          << instances_[agent_id].first.orient_ << " is unavailable " << std::endl;
                solvable = false;
            }

            if (sub_graph.data_ptr_->all_nodes_[target_node_id] == nullptr) {
                std::cout << "FATAL: agent " << agent << "'s target " << instances_[agent_id].second.pt_ << "^"
                          << instances_[agent_id].second.orient_ << " is unavailable " << std::endl;
                solvable = false;
            }

            // when add edges, assume agent can only change position or orientation, cannot change both of them
            // and orientation can only change 90 degree at one timestep, that means the two orient must be orthogonal
            sub_graph.data_ptr_->all_edges_.resize(total_index*2*N, {});
            sub_graph.data_ptr_->all_backward_edges_.resize(total_index*2*N, {});

            // debug
//            std::vector<std::set<size_t> > all_edges_set(total_index*2*N);
//            std::vector<std::set<size_t> > all_backward_edges_set(total_index*2*N);

            Pointis<N> neighbors = GetNearestOffsetGrids<N>();
            for(size_t pose_id=0; pose_id < sub_graph.data_ptr_->all_nodes_.size(); pose_id++) {
                const auto& node_ptr = sub_graph.data_ptr_->all_nodes_[pose_id];
                if(node_ptr != nullptr) {
                    // add edges about position changing
                    size_t origin_orient = pose_id%(2*N);
                    Pointi<N> origin_pt = node_ptr->pt_;
                    Pointi<N> new_pt;
                    for(const auto& offset : neighbors) {
                        new_pt = origin_pt + offset;
                        if(isOutOfBoundary(new_pt, dim_)) { continue; }
                        Id another_node_id = PointiToId<N>(new_pt, dim_)*2*N + origin_orient;
                        if(pose_id == another_node_id) { continue; }
                        PosePtr<int, N> another_node_ptr = sub_graph.data_ptr_->all_nodes_[another_node_id];
                        if(another_node_ptr == nullptr) { continue; }

                        // debug
                        // considering direction, some edge is not reversible
//                        bool direct_legal  = agent.isCollide(*another_node_ptr, *node_ptr, dim_, isoc_, *distance_map_updater_);
//                        bool reverse_legal = agent.isCollide(*node_ptr, *another_node_ptr, dim_, isoc_, *distance_map_updater_);
//                        assert(direct_legal == reverse_legal);

                        if(!agent->isCollide(*node_ptr, *another_node_ptr, dim_, isoc_, *distance_map_updater_)) {
                            sub_graph.data_ptr_->all_edges_[pose_id].push_back(another_node_id);
                            sub_graph.data_ptr_->all_backward_edges_[another_node_id].push_back(pose_id);

                            // debug
//                            all_edges_set[pose_id].insert(another_node_id);
//                            all_backward_edges_set[another_node_id].insert(pose_id);
                        }
                    }
                    // add edges about orientation changing
                    size_t base_id = pose_id / (2*N) * (2*N);
                    for(size_t orient=0; orient<2*N; orient++) {
                        // avoid repeat itself
                        if(node_ptr->orient_ == orient) { continue; }
                        // if another node in subgraph
                        size_t another_node_id = base_id + orient;
                        PosePtr<int, N> another_node_ptr = sub_graph.data_ptr_->all_nodes_[another_node_id];
                        if(another_node_ptr == nullptr) { continue; }
                        // check whether can transfer to another node
                        if(!agent->isCollide(*node_ptr, *another_node_ptr, dim_, isoc_, *distance_map_updater_)) {
                            sub_graph.data_ptr_->all_edges_[pose_id].push_back(another_node_id);
                            sub_graph.data_ptr_->all_backward_edges_[another_node_id].push_back(pose_id);

                            // debug
//                            all_edges_set[pose_id].insert(another_node_id);
//                            all_backward_edges_set[another_node_id].insert(pose_id);
                        }
                    }
                }
            }
            // debug
//            for(size_t i=0; i<all_edges_set.size(); i++) {
//                for(const size_t& j : all_edges_set[i]) {
//                    if(all_backward_edges_set[j].find(i) == all_backward_edges_set[j].end()) {
//                        std::cout << "FATAL: edges have more than backward_edges" << std::endl;
//                    }
//                }
//            }
//            for(size_t i=0; i<all_backward_edges_set.size(); i++) {
//                for(const size_t& j : all_backward_edges_set[i]) {
//                    if(all_edges_set[j].find(i) == all_edges_set[j].end()) {
//                        std::cout << "FATAL: backward_edges have more than edges" << std::endl;
//                    }
//                }
//            }
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
            std::vector<int> agent_heuristic(sub_graph.data_ptr_->all_nodes_.size(), MAX<int>);

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
                for (const size_t& next_location : sub_graph.data_ptr_->all_backward_edges_[curr.node_id]) {
                    int new_heuristic_value = curr.value + 1;
                    if (agent_heuristic[next_location] > new_heuristic_value) {
                        agent_heuristic[next_location] = new_heuristic_value;
                        Node next(next_location, new_heuristic_value);
                        heap.push(next);
                    }
                }
            }

            assert(agent_heuristic[instance_node_ids_[sub_graph.agent_->id_].first]  != MAX<int>);
            assert(agent_heuristic[instance_node_ids_[sub_graph.agent_->id_].second] != MAX<int>);

            // debug
//            for(int i=0; i<sub_graph.all_nodes_.size(); i++) {
//                if(sub_graph.all_nodes_[i] != nullptr) {
//                    assert(agent_heuristic[i] != MAX<int>);
//                }
//            }
            // shrink table
//            for(int k=0; k<sub_graph.all_nodes_.size()/(2*N); k++) {
//                //
//            }
            return agent_heuristic;
        }

        std::vector<int> constructHeuristicTableIgnoreRotate(const SubGraphOfAgent<N>& sub_graph,
                                                             const size_t& goal_id) const {
            struct Node {
                Pointi<N> node_pt;
                int value;

                Node() = default;

                Node(const Pointi<N>& node_pt, int value) : node_pt(node_pt), value(value) {}

                // the following is used to compare nodes in the OPEN list
                struct compare_node {
                    // returns true if n1 > n2 (note -- this gives us *min*-heap).
                    bool operator()(const Node &n1, const Node &n2) const {
                        return n1.value >= n2.value;
                    }
                };  // used by OPEN (heap) to compare nodes (top of the heap has min f-val, and then highest g-val)
            };

            std::vector<int> retv(sub_graph.data_ptr_->all_nodes_.size()/(2*N), MAX<int>);
            // 1, get grid connectivity graph
            std::vector<bool> c_graph(sub_graph.data_ptr_->all_nodes_.size()/(2*N), false);
            for(int i=0; i<sub_graph.data_ptr_->all_nodes_.size(); i++) {
                if(sub_graph.data_ptr_->all_nodes_[i] != nullptr) {
//                    std::cout << "flag 4" << std::endl;
                    c_graph[i/(2*N)] = true;
                }
            }
            // 2, wave front to get heuristic table
            // generate a heap that can save nodes (and an open_handle)
            boost::heap::pairing_heap<Node, boost::heap::compare<typename Node::compare_node> > heap;
            Pointi<N> target_pt = IdToPointi<N>(goal_id/(2*N), dim_);
            Node root(target_pt, 0);
            retv[goal_id/(2*N)] = 0;
            heap.push(root);  // add root to heap
            // yz: compute heuristic from goal to visible grid via Breadth First Search
            //     search the static shortest path
            Pointis<N> offsets = GetNearestOffsetGrids<N>();
            while (!heap.empty()) {
                Node curr = heap.top();
                heap.pop();
                for (const Pointi<N>& offset : offsets) {
                    Pointi<N> next_location = curr.node_pt + offset;
                    if(isoc_(next_location)) { continue; }
                    Id id = PointiToId<N>(next_location, dim_);
                    if(!c_graph[id]) { continue; }
                    int new_heuristic_value = curr.value + 1;
                    if (retv[id] > new_heuristic_value) {
                        retv[id] = new_heuristic_value;
                        Node next(next_location, new_heuristic_value);
                        heap.push(next);
                    }

                }
            }
            assert(retv[instance_node_ids_[sub_graph.agent_->id_].first/(2*N)]  != MAX<int>);
            assert(retv[instance_node_ids_[sub_graph.agent_->id_].second/(2*N)] != MAX<int>);
            return retv;
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
        InstanceOrients<N> instances_;
        std::vector<AgentPtr<N> > agents_;
        DimensionLength* dim_;
        const IS_OCCUPIED_FUNC<N>& isoc_;

        std::vector<std::pair<size_t, size_t> > instance_node_ids_;

        // intermediate variables
        std::vector<PosePtr<int, N> > all_poses_;
        DistanceMapUpdaterPtr<N> distance_map_updater_;

        std::vector<SubGraphOfAgent<N> > agent_sub_graphs_;
        std::vector<std::vector<int> > agents_heuristic_tables_;
        std::vector<std::vector<int> > agents_heuristic_tables_ignore_rotate_;

        // solutions
        std::vector<LAMAPF_Path> solutions_, initial_solutions_;

        double subgraph_and_heuristic_time_cost_ = 0; // time cost of get subgraph and heuristic table for each agent

        double time_limit_ = 60; // s
        double remaining_time_ = 0; // s

        clock_t start_time_;

    };


}

#endif //LAYEREDMAPF_LARGE_AGENT_MAPF_H
