//
// Created by yaozhuo on 6/26/25.
//

#ifndef LAYEREDMAPF_PRECOMPUTATION_FOR_MAPF_H
#define LAYEREDMAPF_PRECOMPUTATION_FOR_MAPF_H

#include "LA-MAPF/common.h"
#include "LA-MAPF/circle_shaped_agent.h"

namespace freeNav::LayeredMAPF {


    template<Dimension N>
    struct PrecomputationOfMAPFBase {
        PrecomputationOfMAPFBase(const InstanceOrients<N> & instances,
                                 const std::vector<LA_MAPF::AgentPtr<N> >& agents,
                                 DimensionLength* dim,
                                 const IS_OCCUPIED_FUNC<N> & isoc) :
                instances_(instances),
                agents_(agents),
                dim_(dim),
                isoc_(isoc) {}

        InstanceOrients<N> instances_;
        std::vector<LA_MAPF::AgentPtr<N> > agents_;
        DimensionLength* dim_;
        const IS_OCCUPIED_FUNC<N>& isoc_;

        std::vector<std::pair<size_t, size_t> > instance_node_ids_;

        // intermediate variables
        std::vector<PosePtr<int, N> > all_poses_;
        DistanceMapUpdaterPtr<N> distance_map_updater_;

        std::vector<LA_MAPF::SubGraphOfAgent<N> > agent_sub_graphs_;
        std::vector<std::vector<int> > agents_heuristic_tables_;
        std::vector<std::vector<int> > agents_heuristic_tables_ignore_rotate_;

        bool solvable_ = true;

        double subgraph_and_heuristic_time_cost_ = 0;

        double remaining_time_ = 0;

        MSTimer mst_;

    };


    template<Dimension N>
    using PrecomputationOfMAPFBasePtr = std::shared_ptr<PrecomputationOfMAPFBase<N> >;

    template<Dimension N, typename HyperNodeType>
    class PrecomputationOfLAMAPF : public PrecomputationOfMAPFBase<N> {
    public:

        PrecomputationOfLAMAPF(const InstanceOrients<N> & instances,
                               const std::vector<LA_MAPF::AgentPtr<N> >& agents,
                               DimensionLength* dim,
                               const IS_OCCUPIED_FUNC<N> & isoc,
                               bool with_sat_heu = true,
                               bool directed_graph = true,
                               int num_of_CPU = 6) :
                PrecomputationOfMAPFBase<N>(instances, agents, dim, isoc),
                directed_graph_(directed_graph),
                num_of_CPU_(num_of_CPU) {


            assert(instances.size() == agents.size());

            MSTimer mst;
            // 0, init and final state overlap check
            for(int i=0; i<this->instances_.size(); i++) {
                for(int j=i+1; j<this->instances_.size(); j++) {
                    if(isCollide(this->agents_[i], this->instances_[i].first, this->agents_[j], this->instances_[j].first)) {
                        std::cout << " agent " << *this->agents_[i] << ", " << *this->agents_[j] << "'s start overlap" << std::endl;
                        this->solvable_ = false;
                        break;
                    }
                    if(isCollide(this->agents_[i], this->instances_[i].second, this->agents_[j], this->instances_[j].second)) {
                        std::cout << " agent " << *this->agents_[i] << ", " << *this->agents_[j] << "'s target overlap" << std::endl;
                        this->solvable_ = false;
                        break;
                    }
                }
            }

            // 1, construct all possible poses
            std::cout << "  construct all possible poses" << std::endl;
            Id total_index = getTotalIndexOfSpace<N>(this->dim_);
            this->all_poses_.resize(total_index * 2 * N, nullptr); // a position with 2*N orientation
            Pointi <N> pt;
            for (Id id = 0; id < total_index; id++) {
                pt = IdToPointi<N>(id, this->dim_);
                if (!isoc_(pt)) {
                    for (int orient = 0; orient < 2 * N; orient++) {
                        PosePtr<int, N> pose_ptr = std::make_shared<Pose<int, N> >(pt, orient);
                        this->all_poses_[id * 2 * N + orient] = pose_ptr;
                    }
                }
            }

            // 2, construct each agents subgraph (vertex: where it can stay, edges: whether can it move from one vertex to another)
            std::cout << "  construct distance_map_update" << std::endl;
            this->distance_map_updater_ = std::make_shared<DistanceMapUpdater<N> >(this->isoc_, this->dim_);


            this->instance_node_ids_.reserve(this->instances_.size());
            for (int agent_id=0; agent_id<this->agents_.size(); agent_id++) {
                // check start
                int start_node_id = PointiToId<N>(this->instances_[agent_id].first.pt_, this->dim_) * 2 * N +
                                    this->instances_[agent_id].first.orient_;

                // check target
                int target_node_id = PointiToId<N>(this->instances_[agent_id].second.pt_, this->dim_) * 2 * N +
                                     this->instances_[agent_id].second.orient_;

                this->instance_node_ids_.push_back({start_node_id, target_node_id});
            }

            constructSubgraph();

            this->subgraph_and_heuristic_time_cost_ = this->mst_.elapsed();

            std::cout << "-- construct subgraph and heuristic table in " << this->subgraph_and_heuristic_time_cost_ << "ms" << std::endl;
            this->remaining_time_ = this->remaining_time_ - this->subgraph_and_heuristic_time_cost_;

        }


        LA_MAPF::SubGraphOfAgent<N> constructSubGraphOfAgent(int agent_id) {
            const LA_MAPF::AgentPtr<N> & agent = this->agents_[agent_id];
            Id total_index = getTotalIndexOfSpace<N>(this->dim_);

            assert(this->all_poses_.size() == total_index*2*N);

            // check start
            int start_node_id = PointiToId<N>(this->instances_[agent_id].first.pt_, this->dim_) * 2 * N +
                                this->instances_[agent_id].first.orient_;

            // check target
            int target_node_id = PointiToId<N>(this->instances_[agent_id].second.pt_, this->dim_) * 2 * N +
                                 this->instances_[agent_id].second.orient_;


            LA_MAPF::SubGraphOfAgent<N> sub_graph(agent);
            sub_graph.data_ptr_ = std::make_shared<LA_MAPF::SubGraphOfAgentData<N> >();

            sub_graph.data_ptr_->all_nodes_.resize(total_index * 2 * N, nullptr);
            sub_graph.start_node_id_ = start_node_id;
            sub_graph.target_node_id_ = target_node_id;

            // initial nodes in subgraph
            for(size_t id=0; id<this->all_poses_.size(); id++) {
                if(this->all_poses_[id] != nullptr) {
                    const auto& current_pose = this->all_poses_[id];
                    if(!agent->isCollide(*current_pose, this->dim_, this->isoc_, *this->distance_map_updater_)) {
                        sub_graph.data_ptr_->all_nodes_[id] = current_pose;
                    }
                }
            }

            if (sub_graph.data_ptr_->all_nodes_[start_node_id] == nullptr) {
                std::cout << "FATAL: agent " << agent << "'s start " << this->instances_[agent_id].first.pt_ << "^"
                          << this->instances_[agent_id].first.orient_ << " is unavailable " << std::endl;
                this->solvable_ = false;
            }

            if (sub_graph.data_ptr_->all_nodes_[target_node_id] == nullptr) {
                std::cout << "FATAL: agent " << agent << "'s target " << this->instances_[agent_id].second.pt_ << "^"
                          << this->instances_[agent_id].second.orient_ << " is unavailable " << std::endl;
                this->solvable_ = false;
            }

            // when add edges, assume agent can only change position or orientation, cannot change both of them
            // and orientation can only change 90 degree at one timestep, that means the two orient must be orthogonal
            sub_graph.data_ptr_->all_edges_.resize(total_index*2*N, {});
            sub_graph.data_ptr_->all_backward_edges_.resize(total_index*2*N, {});


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
                        if(isOutOfBoundary(new_pt, this->dim_)) { continue; }
                        Id another_node_id = PointiToId<N>(new_pt, this->dim_)*2*N + origin_orient;
                        if(pose_id == another_node_id) { continue; }
                        PosePtr<int, N> another_node_ptr = sub_graph.data_ptr_->all_nodes_[another_node_id];
                        if(another_node_ptr == nullptr) { continue; }
                        if(!agent->isCollide(*node_ptr, *another_node_ptr, this->dim_, this->isoc_, *this->distance_map_updater_)) {
                            sub_graph.data_ptr_->all_edges_[pose_id].push_back(another_node_id);
                            sub_graph.data_ptr_->all_backward_edges_[another_node_id].push_back(pose_id);
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
                        if(!agent->isCollide(*node_ptr, *another_node_ptr, this->dim_, this->isoc_, *this->distance_map_updater_)) {
                            sub_graph.data_ptr_->all_edges_[pose_id].push_back(another_node_id);
                            sub_graph.data_ptr_->all_backward_edges_[another_node_id].push_back(pose_id);
                        }
                    }
                }
            }
            return sub_graph;
        }

        void constructSubgraph() {

            // initialize in parallel thread to reduce time cost
            std::map<int, LA_MAPF::SubGraphOfAgent<N>> sub_graphs_map;
            std::map<int, std::vector<int> > heuristic_tables_;
            std::map<int, std::vector<int> > heuristic_tables_ig_rotate;
            int interval = this->agents.size()/num_of_CPU_;// set to larger value to reduce maximal memory usage and num of threads
            std::mutex lock_1, lock_2, lock_3, lock_4;
            std::vector<bool> finished(this->agents_.size(), false);
            int count_of_instance = 0;
            while(count_of_instance < this->instances_.size()) {
                auto lambda_func = [&]() {
                    auto start_t_3 = std::chrono::steady_clock::now();
                    //auto start_t_4 = std::chrono::steady_clock::now();
                    for (int k = 0; k < interval; k++) {
                        //std::cout << "thread_id = " << thread_id << std::endl;
                        lock_4.lock();
                        int map_id = count_of_instance;
                        count_of_instance ++;
                        lock_4.unlock();
                        //std::cout << "map_id = " << map_id << std::endl;

                        if (map_id >= this->agents_.size()) { break; }
                        if(this->agent_sub_graphs_.empty()) {
                            const auto &sub_graph = constructSubGraphOfAgent(map_id);
                            lock_1.lock();
                            sub_graphs_map.insert({map_id, sub_graph});
                            lock_1.unlock();
                        } else {
                            sub_graphs_map.insert({map_id, this->agent_sub_graphs_[map_id]});
                        }
                        if(this->agents_heuristic_tables_.empty()) {
                            const auto & table = constructHeuristicTable(sub_graphs_map.at(map_id),
                                                                         this->instance_node_ids_[map_id].second);
                            assert(table[this->instance_node_ids_[sub_graphs_map.at(map_id).agent_->id_].first]  != MAX<int>);
                            assert(table[this->instance_node_ids_[sub_graphs_map.at(map_id).agent_->id_].second] != MAX<int>);
                            lock_2.lock();
                            heuristic_tables_.insert({map_id, table});
                            lock_2.unlock();
                        }
                        if(this->agents_heuristic_tables_ignore_rotate_.empty()) {
                            const auto & table = constructHeuristicTableIgnoreRotate(sub_graphs_map.at(map_id),
                                                                                     this->instance_node_ids_[map_id].second);
                            assert(table[this->instance_node_ids_[sub_graphs_map.at(map_id).agent_->id_].first]  != MAX<int>);
                            assert(table[this->instance_node_ids_[sub_graphs_map.at(map_id).agent_->id_].second] != MAX<int>);
                            lock_3.lock();
                            heuristic_tables_ig_rotate.insert({map_id, table});
                            lock_3.unlock();
                        }
                        lock_4.lock();
                        finished[map_id] = true;
                        lock_4.unlock();
                    }
                    //auto end_t_4 = std::chrono::steady_clock::now();
                    //std::cout << "thread calculation take " << std::chrono::duration_cast<std::chrono::milliseconds>(end_t_4 - start_t_4).count() << "ms" << std::endl;
                    //auto end_t_3 = std::chrono::steady_clock::now();
                    //std::cout << "thread all take " << std::chrono::duration_cast<std::chrono::milliseconds>(end_t_3 - start_t_3).count() << "ms" << std::endl;
                    //std::cout << "thread " << thread_id_copy << " finished 1" << std::endl;
                };
                std::thread t(lambda_func);
                t.detach();
            }
            while(finished != std::vector<bool>(this->agents_.size(), true)) {
                usleep(5e4);
            }
            this->agent_sub_graphs_.resize(this->agents_.size(), LA_MAPF::SubGraphOfAgent<N>(nullptr));
            for (const auto &subgraph_pair : sub_graphs_map) {
                this->agent_sub_graphs_[subgraph_pair.first] = subgraph_pair.second;
            }

            this->agents_heuristic_tables_.resize(this->agents_.size());
            for (const auto &table_pair : heuristic_tables_) {
                this->agents_heuristic_tables_[table_pair.first] = table_pair.second;
            }

            this->agents_heuristic_tables_ignore_rotate_.resize(this->agents_.size());
            for (const auto &table_pair : heuristic_tables_ig_rotate) {
                this->agents_heuristic_tables_ignore_rotate_[table_pair.first] = table_pair.second;
            }
        }


        std::vector<int> constructHeuristicTable(const LA_MAPF::SubGraphOfAgent<N>& sub_graph, const size_t& goal_id) const {
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

        std::vector<int> constructHeuristicTableIgnoreRotate(const LA_MAPF::SubGraphOfAgent<N>& sub_graph,
                                                             const size_t& goal_id, DimensionLength* dim) const {
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
            Pointi<N> target_pt = IdToPointi<N>(goal_id/(2*N), dim);
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
                    Id id = PointiToId<N>(next_location, dim);
                    if(!c_graph[id]) { continue; }
                    int new_heuristic_value = curr.value + 1;
                    if (retv[id] > new_heuristic_value) {
                        retv[id] = new_heuristic_value;
                        Node next(next_location, new_heuristic_value);
                        heap.push(next);
                    }

                }
            }
            return retv;
        }

        bool directed_graph_ = true;

        float initialize_time_cost_ = 0;

        int num_of_CPU_ = 1;

    };

    template<Dimension N, typename HyperNodeType>
    class PrecomputationOfMAPF : public PrecomputationOfMAPFBase<N> {
    public:

        PrecomputationOfMAPF(const Instances<N>& instance,
                             DimensionLength* dim,
                             const IS_OCCUPIED_FUNC<N>& isoc) :
                             PrecomputationOfMAPFBase<N>(instance, {}, dim, isoc) {

            MSTimer mst;

            std::cout << "  construct all possible poses" << std::endl;
            Id total_index = getTotalIndexOfSpace<N>(this->dim_);
            this->all_poses_.resize(total_index, nullptr); // a position with 2*N orientation
            Pointi<N> pt;
            for (Id id = 0; id < total_index; id++) {
                pt = IdToPointi<N>(id, this->dim_);
                if (!isoc_(pt)) {
                    PosePtr<int, N> pose_ptr = std::make_shared<Pose<int, N> >(pt, 0);
                    this->all_poses_[id] = pose_ptr;
                }
            }

            LA_MAPF::SubGraphOfAgentDataPtr<N> subgraph_data_ptr = constructSubGraphOfAgent();

            for (int agent_id = 0; agent_id < this->instances_.size(); agent_id++) {
                // check start
                int start_node_id = PointiToId<N>(this->instances_[agent_id].first, this->dim_);

                // check target
                int target_node_id = PointiToId<N>(this->instances_[agent_id].second, this->dim_);

                this->instance_node_ids_.push_back(std::make_pair(start_node_id, target_node_id));

                LA_MAPF::AgentPtr<N> agent = std::make_shared<LA_MAPF::CircleAgent<N> >(0.1, agent_id, this->dim_);

                LA_MAPF::SubGraphOfAgent<N> sub_graph(agent);

                sub_graph.data_ptr_ = subgraph_data_ptr;

                if (sub_graph.data_ptr_->all_nodes_[start_node_id] == nullptr) {
                    std::cout << "FATAL: agent " << agent << "'s start " << this->instances_[agent_id].first << "^"
                              << this->instances_[agent_id].first << " is unavailable " << std::endl;
                    this->solvable_ = false;
                }

                if (sub_graph.data_ptr_->all_nodes_[target_node_id] == nullptr) {
                    std::cout << "FATAL: agent " << agent << "'s target " << this->instances_[agent_id].second << "^"
                              << this->instances_[agent_id].second << " is unavailable " << std::endl;
                    this->solvable_ = false;
                }

                sub_graph.start_node_id_ = start_node_id;
                sub_graph.target_node_id_ = target_node_id;

                this->agent_sub_graphs_.push_back(sub_graph);

            }

            this->distance_map_updater_ = std::make_shared<DistanceMapUpdater<N> >(this->isoc_, this->dim_);

            initialize_time_cost_ =  mst.elapsed();

            std::cout << "-- MAPF initialize_time_cost_ (ms) = " << initialize_time_cost_ << std::endl;
        }

        LA_MAPF::SubGraphOfAgentDataPtr<N> constructSubGraphOfAgent() {
            Id total_index = getTotalIndexOfSpace<N>(this->dim_);

            assert(this->all_poses_.size() == total_index);

            auto data_ptr = std::make_shared<LA_MAPF::SubGraphOfAgentData<N> >();

            data_ptr->all_nodes_.resize(total_index, nullptr);


            // initial nodes in subgraph
            for(size_t id=0; id<this->all_poses_.size(); id++) {
                if(this->all_poses_[id] != nullptr) {
                    const auto& current_pose = this->all_poses_[id];
                    data_ptr->all_nodes_[id] = current_pose;
                }
            }

            // when add edges, assume agent can only change position or orientation, cannot change both of them
            // and orientation can only change 90 degree at one timestep, that means the two orient must be orthogonal
            data_ptr->all_edges_.resize(total_index, {});
            data_ptr->all_backward_edges_.resize(total_index, {});

            Pointis<N> neighbors = GetNearestOffsetGrids<N>();
            for(size_t pose_id=0; pose_id < data_ptr->all_nodes_.size(); pose_id++) {
                const auto& node_ptr = data_ptr->all_nodes_[pose_id];
                if(node_ptr != nullptr) {
                    // add edges about position changing
                    Pointi<N> origin_pt = node_ptr->pt_;
                    Pointi<N> new_pt;
                    for(const auto& offset : neighbors) {
                        new_pt = origin_pt + offset;
                        if(isOutOfBoundary(new_pt, this->dim_)) { continue; }

                        Id another_node_id = PointiToId<N>(new_pt, this->dim_);
                        if(pose_id == another_node_id) { continue; }

                        PosePtr<int, N> another_node_ptr = data_ptr->all_nodes_[another_node_id];
                        if(another_node_ptr == nullptr) { continue; }

                        data_ptr->all_edges_[pose_id].push_back(another_node_id);
                        data_ptr->all_backward_edges_[another_node_id].push_back(pose_id);

                    }
                }
            }
            return data_ptr;
        }

        float initialize_time_cost_ = 0;

    };

}

#endif //LAYEREDMAPF_PRECOMPUTATION_FOR_MAPF_H
