//
// Created by yaozhuo on 6/16/25.
//

#ifndef LAYEREDMAPF_PRECOMPUTATION_FOR_DECOMPOSITION_H
#define LAYEREDMAPF_PRECOMPUTATION_FOR_DECOMPOSITION_H

#include "LA-MAPF/common.h"
//#include "LA-MAPF/large_agent_mapf.h"
//#include "LA-MAPF/circle_shaped_agent.h"
//#include "LA-MAPF/CBS/space_time_astar.h"
//#include "LA-MAPF/large_agent_dependency_path_search.h"
#include "precomputation_for_mapf.h"

namespace freeNav::LayeredMAPF {

    class PrecomputationOfMAPFDecompositionBase {
    public:

        PrecomputationOfMAPFDecompositionBase(bool with_sat_heu = true):
                with_ignore_sat_heu_(with_sat_heu)
        {}

        std::vector<LA_MAPF::ConnectivityGraph> connect_graphs_;

        std::vector<std::vector<int> > heuristic_tables_sat_; // distinguish_sat = true

        std::vector<std::vector<int> > heuristic_tables_; // distinguish_sat = false

        bool with_ignore_sat_heu_ = true; // whether calculate heuristic_tables_

        float initialize_time_cost_ = 0;

    };

    typedef std::shared_ptr<PrecomputationOfMAPFDecompositionBase> PrecomputationOfMAPFDecompositionBasePtr;

    template<Dimension N, typename HyperNodeType>
    class PrecomputationOfLAMAPFDecomposition :
            public PrecomputationOfMAPFDecompositionBase, public PrecomputationOfLAMAPF<N> {
    public:

        PrecomputationOfLAMAPFDecomposition(const InstanceOrients<N> & instances,
                               const std::vector<LA_MAPF::AgentPtr<N> >& agents,
                               DimensionLength* dim,
                               const IS_OCCUPIED_FUNC<N> & isoc,
                               bool with_sat_heu = true,
                               bool directed_graph = true,
                               int num_of_CPU = 6) :
                               PrecomputationOfLAMAPF<N>(instances, agents, dim, isoc),
                               PrecomputationOfMAPFDecompositionBase(with_sat_heu),
                               num_of_CPU_(num_of_CPU) {


            MSTimer mst;

            constructConnectivityGraphAndHeuristicTable();

            this->initialize_time_cost_ =  mst.elapsed();

            std::cout << "-- LA-MAPF decomposition precomputation time cost (ms) = " << this->initialize_time_cost_ << std::endl;

        }


        LA_MAPF::SubGraphOfAgent<N, Pose<int, N>> constructSubGraphOfAgent(int agent_id) {
            const LA_MAPF::AgentPtr<N> & agent = this->agents_[agent_id];
            Id total_index = getTotalIndexOfSpace<N>(this->dim_);

            assert(this->all_poses_.size() == total_index*2*N);

            // check start
            int start_node_id = PointiToId<N>(this->instances_[agent_id].first.pt_, this->dim_) * 2 * N +
                    this->instances_[agent_id].first.orient_;

            // check target
            int target_node_id = PointiToId<N>(this->instances_[agent_id].second.pt_, this->dim_) * 2 * N +
                    this->instances_[agent_id].second.orient_;


            LA_MAPF::SubGraphOfAgent<N, Pose<int, N>> sub_graph(agent);
            sub_graph.data_ptr_ = std::make_shared<LA_MAPF::SubGraphOfAgentData<N, Pose<int, N>> >();

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
            std::map<int, LA_MAPF::SubGraphOfAgent<N, Pose<int, N>>> sub_graphs_map;
            std::map<int, std::vector<int> > heuristic_tables_;
            std::map<int, std::vector<int> > heuristic_tables_ig_rotate;
            int interval = (int)std::ceil((double)this->agents.size()/num_of_CPU_);// set to larger value to reduce maximal memory usage and num of threads
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
                            lock_2.lock();
                            heuristic_tables_.insert({map_id, table});
                            lock_2.unlock();
                        }
                        if(this->agents_heuristic_tables_ignore_rotate_.empty()) {
                            const auto & table = constructHeuristicTableIgnoreRotate(sub_graphs_map.at(map_id),
                                                                                     this->instance_node_ids_[map_id].second);
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
            this->agent_sub_graphs_.resize(this->agents_.size(), LA_MAPF::SubGraphOfAgent<N, Pose<int, N>>(nullptr));
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

        void constructConnectivityGraphAndHeuristicTable() {

            double connect_time_cost      = 0;
            double heu_with_sat_time_cost = 0;
            double heu_ig_sat_time_cost   = 0;
#if 0
            // calculate one by one
            for(int i=0; i<agents.size(); i++) {
                connect_graphs_.push_back(getAgentConnectivityGraph(i));
                connect_time_cost += 1e3*((double)t_2 - t_1)/CLOCKS_PER_SEC;
                heuristic_tables_sat_.push_back(LA_MAPF::calculateLargeAgentHyperGraphStaticHeuristic<N, HyperGraphNodeDataRaw<N>>(i, this->dim_, connect_graphs_[i], true));
                heu_with_sat_time_cost += 1e3*((double)t_3 - t_2)/CLOCKS_PER_SEC;
                if(with_sat_heu_) {
                    heuristic_tables_.push_back(LA_MAPF::calculateLargeAgentHyperGraphStaticHeuristic<N, HyperGraphNodeDataRaw<N>>(i, this->dim_, connect_graphs_[i], false));
                }
                heu_ig_sat_time_cost += 1e3*((double)t_4 - t_3)/CLOCKS_PER_SEC;
            }
#else
            // initialize in parallel thread to reduce time cost
            std::map<int, LA_MAPF::ConnectivityGraph> connect_graphs_map;
            std::map<int, std::vector<int> > heuristic_tables_sat;
            std::map<int, std::vector<int> > heuristic_tables_ig_sat;

            // how many agents in a thread
            int interval = (int)std::ceil((double)this->agents_.size()/num_of_CPU_);// set to larger value to reduce maximal memory usage and num of threads
            // num of thread should close to the num of CPU
//            int num_threads = agents.size()/interval;
            //std::cout << " num of threads = " << num_threads << std::endl;
            std::mutex lock_1, lock_2, lock_3, lock_4;
            std::vector<bool> finished(this->agents_.size(), false);
            int count_of_instance = 0;
            while(count_of_instance < this->instances_.size()) {
                auto lambda_func = [&]() {
                    MSTimer mst_1;
                    for (int k = 0; k < interval; k++) {
                        //std::cout << "thread_id = " << thread_id << std::endl;
                        lock_4.lock();
                        int map_id = count_of_instance;
                        count_of_instance ++;
                        lock_4.unlock();
                        //std::cout << "map_id = " << map_id << std::endl;
                        if (map_id >= this->agents_.size()) { break; }
                        const auto& connect_graph = getAgentConnectivityGraph(map_id);
                        lock_1.lock();
                        connect_graphs_map.insert({map_id, connect_graph});
                        lock_1.unlock();
                        const auto& heu_sat_table =
                                LA_MAPF::calculateLargeAgentHyperGraphStaticHeuristic<N, HyperNodeType>
                                        (map_id, this->dim_, connect_graph, true);
                        lock_2.lock();
                        heuristic_tables_sat.insert({map_id, heu_sat_table});
                        lock_2.unlock();
                        if(this->with_ignore_sat_heu_) {
                            const auto& heu_ig_sat_table =
                                    LA_MAPF::calculateLargeAgentHyperGraphStaticHeuristic
                                            <N, HyperNodeType>
                                            (map_id, this->dim_, connect_graph, false);
                            lock_3.lock();
                            heuristic_tables_ig_sat.insert({map_id, heu_ig_sat_table});
                            lock_3.unlock();
                        }
                        lock_4.lock();
                        finished[map_id] = true;
                        lock_4.unlock();
                    }
                    //std::cout << "thread calculation take " << std::chrono::duration_cast<std::chrono::milliseconds>(end_t_4 - start_t_4).count() << "ms" << std::endl;
                    //std::cout << "thread all take " << mst_1.elapsed() << "ms" << std::endl;
                    //std::cout << "thread " << thread_id_copy << " finished 1" << std::endl;
                };
                std::thread t(lambda_func);
                t.detach();
                usleep(1e3); // sleep 1ms to wait current thread get correct thread_id, 1ms too less
            }
            while(finished != std::vector<bool>(this->agents_.size(), true)) {
                usleep(5e4);
            }
            connect_graphs_.resize(this->agents_.size());
            for(const auto& connec_pair : connect_graphs_map) {
                connect_graphs_[connec_pair.first] = connec_pair.second;
            }
            heuristic_tables_sat_.resize(this->agents_.size());
            for(const auto& connec_pair : heuristic_tables_sat) {
                heuristic_tables_sat_[connec_pair.first] = connec_pair.second;
            }
            heuristic_tables_.resize(this->agents_.size());
            for(const auto& heu_pair : heuristic_tables_ig_sat) {
                heuristic_tables_[heu_pair.first] = heu_pair.second;
            }
#endif
//            std::cout << "-- LA-MAPF connect_graphs_ (ms)       = " << connect_time_cost << std::endl;
//            std::cout << "-- LA-MAPF heuristic_tables_sat_ (ms) = " << heu_with_sat_time_cost << std::endl;
//            std::cout << "-- LA-MAPF heu_ig_sat_time_cost (ms)  = " << heu_ig_sat_time_cost << std::endl;

        }

        std::vector<std::set<int> > getRelatedAgentGraph(int agent_id, const std::vector<PosePtr<int, N> >& all_nodes) const {
            std::vector<std::set<int> > related_agents_map(all_nodes.size());
            for(int i=0; i<this->agents_.size(); i++) {
                //if(i == agent_id) { continue; }
                const auto& agent = this->agents_[agent_id];
                const auto& another_agent = this->agents_[i];
                const auto& another_subgraph = this->agent_sub_graphs_[i];

                assert(another_subgraph.data_ptr_->all_nodes_[this->instance_node_ids_[i].first] != nullptr);
                assert(another_subgraph.data_ptr_->all_nodes_[this->instance_node_ids_[i].second] != nullptr);

//                std::cout << "agent/another = " << agent_id << ", " << i << std::endl;
                const auto& another_agent_start_pose  = this->instances_[i].first;
                const auto& another_agent_target_pose = this->instances_[i].second;

                const auto& another_agent_start_pt  = another_agent_start_pose.pt_;
                const auto& another_agent_target_pt = another_agent_target_pose.pt_;

                // get the maximum range of conflict
                float max_range_radius = agent->excircle_radius_ + another_agent->excircle_radius_;
                int maximum_ralated_range = ceil(max_range_radius + sqrt(N) + 1); // considering the boundary of two shape in one grid

                int local_space_width = 2*maximum_ralated_range + 1;
                // construct a temp local space
                DimensionLength local_dim[N];
                Pointi<N> center_pt;
                for(int d=0; d<N; d++) {
                    local_dim[d] = local_space_width;
                    center_pt[d] = maximum_ralated_range;
                }
//                std::cout << "max_range_radius / ceil(max_range_radius) = " << max_range_radius << " / " << ceil(max_range_radius) << std::endl;
                int local_total_index = getTotalIndexOfSpace<N>(local_dim);
//                std::cout << " local_total_index = " << local_total_index << std::endl;
                Pointi<N> temp_pt, temp_start, temp_target;
                size_t temp_pose_id, other_temp_pose_id;
                Id temp_id;
                for(int gid=0; gid<local_total_index; gid++) {
                    temp_pt = IdToPointi<N>(gid, local_dim) - center_pt;
                    if(temp_pt.Norm() > maximum_ralated_range) { continue; }
                    // 1) for another agent's start
                    temp_start = another_agent_start_pt + temp_pt;
                    // isoc contain in range test implicitly
                    if(!this->isoc_(temp_start)) {
//                        if(agent_id == 4 && i == 3) {
//                            std::cout << " temp_start " << temp_start << std::endl;
//                        }
                        temp_id = PointiToId<N>(temp_start, this->dim_);
                        for(int orient=0; orient<2*N; orient++) {
                            // get a nearby pose id
                            temp_pose_id = temp_id*2*N + orient;
                            // check whether this node is available in current agent's subgraph
                            if(all_nodes[temp_pose_id] != nullptr) {
//                                std::cout << " check start conflict" << std::endl;
                                // if current node is collide with other agent
                                if(isCollide(agent, *all_nodes[temp_pose_id], another_agent, another_agent_start_pose))
                                {
                                    // if they have conflict
                                    related_agents_map[temp_pose_id].insert(2*i);
                                } else {
                                    // if current node's edge is collide with other agent
                                    for(int other_orient=0; other_orient<2*N; other_orient++) {
                                        if(other_orient == orient || orient/2 == other_orient/2) { continue; }
                                        other_temp_pose_id = temp_id*2*N + other_orient;
                                        if(all_nodes[other_temp_pose_id] == nullptr) { continue; }
                                        if(isCollide(agent, *all_nodes[temp_pose_id], *all_nodes[other_temp_pose_id],
                                                     another_agent, another_agent_start_pose))
                                        {
                                            // if they have conflict
                                            related_agents_map[temp_pose_id].insert(2*i);
                                            break;
                                        }
                                    }
                                }
                            }
                        }
                    }
                    // 2) for another agent's target
                    temp_target = another_agent_target_pt + temp_pt;
                    // isoc contain in range test implicitly
                    if(!this->isoc_(temp_target)) {
                        temp_id = PointiToId<N>(temp_target, this->dim_);
                        for(int orient=0; orient<2*N; orient++) {
                            // get a nearby pose id
                            temp_pose_id = temp_id*2*N + orient;
                            // check whether this node is available in current agent's subgraph
                            if(all_nodes[temp_pose_id] != nullptr) {
//                                std::cout << " check target conflict" << std::endl;
                                if(isCollide(agent, *all_nodes[temp_pose_id],
                                             another_agent, another_agent_target_pose))
                                {
                                    // if they have conflict
                                    related_agents_map[temp_pose_id].insert(2*i + 1);
                                } else {
                                    // if current node's edge is collide with other agent
                                    for(int other_orient=0; other_orient<2*N; other_orient++) {
                                        if(other_orient == orient || orient/2 == other_orient/2) { continue; }
                                        other_temp_pose_id = temp_id*2*N + other_orient;
                                        if(all_nodes[other_temp_pose_id] == nullptr) { continue; }
                                        if(isCollide(agent, *all_nodes[temp_pose_id], *all_nodes[other_temp_pose_id],
                                                     another_agent, another_agent_target_pose))
                                        {
                                            // if they have conflict
                                            related_agents_map[temp_pose_id].insert(2*i + 1);
                                            break;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
            assert(related_agents_map[this->instance_node_ids_[agent_id].first].find(2*agent_id) !=
                   related_agents_map[this->instance_node_ids_[agent_id].first].end());

            assert(related_agents_map[this->instance_node_ids_[agent_id].second].find(2*agent_id+1) !=
                   related_agents_map[this->instance_node_ids_[agent_id].second].end());

            return related_agents_map;
        }

        LA_MAPF::ConnectivityGraph getAgentConnectivityGraph(const int& agent_id) const {
            LA_MAPF::ConnectivityGraph graph;
            graph.data_ptr_ = std::make_shared<LA_MAPF::ConnectivityGraphData>(this->all_poses_.size());

            LA_MAPF::SubGraphOfAgent<N, Pose<int, N>> current_subgraph = this->agent_sub_graphs_[agent_id];
            assert(current_subgraph.data_ptr_->all_nodes_[this->instance_node_ids_[agent_id].first] != nullptr);
            assert(current_subgraph.data_ptr_->all_nodes_[this->instance_node_ids_[agent_id].second] != nullptr);

            // 1, get each pose's relation with other agent
            graph.data_ptr_->related_agents_map_ = getRelatedAgentGraph(agent_id, current_subgraph.data_ptr_->all_nodes_);
            // assert agent's start and target have no overlap with other agent's start or target
            assert(graph.data_ptr_->related_agents_map_[this->instance_node_ids_[agent_id].first].size() >= 1);
            assert(graph.data_ptr_->related_agents_map_[this->instance_node_ids_[agent_id].second].size() >= 1);

            // 2, construct connectivity graph and record boundary (where different hyper node converge)
            const auto& retv_pair = LA_MAPF::getStrongComponentFromSubGraph<Pose<int, N>>(current_subgraph.data_ptr_->all_nodes_,
                                                                                    current_subgraph.data_ptr_->all_edges_,
                                                                                    current_subgraph.data_ptr_->all_backward_edges_,
                                                                                    graph.data_ptr_->related_agents_map_,
                                                                                    directed_graph_);


            const auto& strong_components = retv_pair.first;
            const auto& component_id_map  = retv_pair.second;


            int max_hyper_node_id = 0;
            std::vector<std::pair<size_t, size_t> > boundary_nodes;

            std::vector<std::vector<size_t> > node_in_hyper_node;

            // considering the using boost to detect strong connected component of directed graph
            for(const std::set<size_t>& cur_component : strong_components) {
                //if(cur_component.size() == 1)
                {
                    if(current_subgraph.data_ptr_->all_nodes_[*cur_component.begin()] == nullptr) { continue; }
                }
                for (const size_t & node_id : cur_component) {
                    const auto &current_pose_ptr = current_subgraph.data_ptr_->all_nodes_[node_id];
                    if (current_pose_ptr == nullptr) { continue; }
                    if (graph.data_ptr_->hyper_node_id_map_[node_id] != MAX<size_t>) { continue; }
                    graph.data_ptr_->hyper_node_id_map_[node_id] = max_hyper_node_id;

                    node_in_hyper_node.push_back({});
                    node_in_hyper_node.back().push_back(node_id);

                    std::vector<size_t> nodes_buffer = {node_id}, next_buffer = {};
                    while (!nodes_buffer.empty()) {
                        next_buffer.clear();
                        for (const auto &cur_node : nodes_buffer) {
                            // traversal all neighboring nodes
                            for (const auto &neighbor_node_id : current_subgraph.data_ptr_->all_edges_[cur_node]) {

                                // if belong to different component
                                //if(cur_component.find(neighbor_node_id) == cur_component.end())
                                if(component_id_map[cur_node] != component_id_map[neighbor_node_id]) {
                                    // when two node belong to different component
                                    size_t node_from = MAX<size_t>, node_to = MAX<size_t>;
                                    node_from = cur_node;
                                    node_to = neighbor_node_id;

                                    //if(node_from != MAX<size_t> && node_to != MAX<size_t>)
                                    {
                                        boundary_nodes.push_back({node_from, node_to});
                                    }
                                } else {

                                    if (graph.data_ptr_->related_agents_map_[node_id] == graph.data_ptr_->related_agents_map_[neighbor_node_id]) {
                                        if (graph.data_ptr_->hyper_node_id_map_[neighbor_node_id] != MAX<size_t>) {
                                            if(graph.data_ptr_->hyper_node_id_map_[neighbor_node_id] != max_hyper_node_id) {
                                                // if two node have different related agent, they are on boundary
                                                size_t node_from = MAX<size_t>, node_to = MAX<size_t>;
                                                node_from = cur_node;
                                                node_to = neighbor_node_id;
                                                //if(node_from != MAX<size_t> && node_to != MAX<size_t>)
                                                {
                                                    boundary_nodes.push_back({node_from, node_to});
                                                }
                                            }
                                            continue;
                                        }
                                        graph.data_ptr_->hyper_node_id_map_[neighbor_node_id] = max_hyper_node_id;
                                        next_buffer.push_back(neighbor_node_id);

                                        node_in_hyper_node.back().push_back(neighbor_node_id);

                                    } else {
                                        // shouldn't reach here, because all node in the same component have the same related agents
                                        assert(0);
                                        // if two node have different related agent, they are on boundary
                                        size_t node_from = MAX<size_t>, node_to = MAX<size_t>;
                                        node_from = cur_node;
                                        node_to = neighbor_node_id;
                                        //if(node_from != MAX<size_t> && node_to != MAX<size_t>)
                                        {
                                            boundary_nodes.push_back({node_from, node_to});
                                        }
                                    }
                                }
                            }
                        }
                        std::swap(nodes_buffer, next_buffer);
                    }
                    max_hyper_node_id++;
                }
            }


            graph.data_ptr_->all_edges_set_.resize(max_hyper_node_id);
            graph.data_ptr_->all_edges_vec_.resize(max_hyper_node_id);
            graph.data_ptr_->all_edges_vec_backward_.resize(max_hyper_node_id);

            // 3, get start hyper node id and target hyper node id
            graph.start_hyper_node_  = graph.data_ptr_->hyper_node_id_map_[this->instance_node_ids_[agent_id].first];
            graph.target_hyper_node_ = graph.data_ptr_->hyper_node_id_map_[this->instance_node_ids_[agent_id].second];
            assert(graph.start_hyper_node_ != MAX<size_t>);
            assert(graph.target_hyper_node_ != MAX<size_t>);


            // 4, get connections between hyper graph nodes
            graph.data_ptr_->hyper_node_with_agents_.resize(max_hyper_node_id);
            for(int cur_hyper_node_id=0; cur_hyper_node_id<max_hyper_node_id; cur_hyper_node_id++) {
                graph.data_ptr_->hyper_node_with_agents_[cur_hyper_node_id] =
                        graph.data_ptr_->related_agents_map_[node_in_hyper_node[cur_hyper_node_id].front()];
            }

            for(const auto& node_id_pair : boundary_nodes) {
                const auto& cur_hyper_node_id  = graph.data_ptr_->hyper_node_id_map_[node_id_pair.first];
                const auto& next_hyper_node_id = graph.data_ptr_->hyper_node_id_map_[node_id_pair.second];

                assert(cur_hyper_node_id != next_hyper_node_id);
                assert(cur_hyper_node_id != MAX<size_t> && next_hyper_node_id != MAX<size_t>);

                if(graph.data_ptr_->all_edges_set_[cur_hyper_node_id].find(next_hyper_node_id) == graph.data_ptr_->all_edges_set_[cur_hyper_node_id].end()) {

                    graph.data_ptr_->all_edges_set_[cur_hyper_node_id].insert(next_hyper_node_id);
                    graph.data_ptr_->all_edges_vec_[cur_hyper_node_id].push_back(next_hyper_node_id);

                    graph.data_ptr_->all_edges_vec_backward_[next_hyper_node_id].push_back(cur_hyper_node_id);
                }
            }


            assert(graph.data_ptr_->hyper_node_with_agents_[graph.start_hyper_node_].find(2*agent_id) !=
                   graph.data_ptr_->hyper_node_with_agents_[graph.start_hyper_node_].end());

            assert(graph.data_ptr_->hyper_node_with_agents_[graph.target_hyper_node_].find(2*agent_id+1) !=
                   graph.data_ptr_->hyper_node_with_agents_[graph.target_hyper_node_].end());


            return graph;
        }


        std::vector<LA_MAPF::ConnectivityGraph> connect_graphs_;

        std::vector<std::vector<int> > heuristic_tables_sat_; // distinguish_sat = true

        std::vector<std::vector<int> > heuristic_tables_; // distinguish_sat = false

        bool directed_graph_ = true;

        int num_of_CPU_ = 1;

    };

    template<Dimension N, typename HyperNodeType>
    using PrecomputationOfLAMAPFDecompositionPtr = std::shared_ptr<PrecomputationOfLAMAPFDecomposition<N, HyperNodeType>>;

    // unordered_set
    template<Dimension N, typename HyperNodeType>
    std::set<int> getPassingAgents(const HyperNodeType& node_ptr,
                                   const LA_MAPF::ConnectivityGraph& graph,
                                   bool distinguish_sat = true) {
        std::set<int> retv;
        HyperNodeType buffer = node_ptr;
        while(buffer != nullptr) {

            std::set<int> raw_agent_ids = graph.data_ptr_->hyper_node_with_agents_[buffer->current_node_];
            for(const auto& raw_agent_id : raw_agent_ids) {
                if(distinguish_sat) {
                    retv.insert(raw_agent_id);
                } else {
                    retv.insert(raw_agent_id / 2);
                }
            }
            buffer = buffer->pa_;
        }
        return retv;
    }

    // calculate static heuristic table, using BFS, dist to target = 0
    // dist is defined like g_val and h_val, how many agent need to cross to reach target
    template <Dimension N, typename HyperNodeType>
    std::vector<int> calculateAgentHyperGraphStaticHeuristic(int agent_id, DimensionLength* dim, const LA_MAPF::ConnectivityGraph& graph, bool distinguish_sat = false) {
        // the two table update simultaneously

        //std::cout << __FUNCTION__ << "graph.nodes size = " << graph.data_ptr_->all_edges_set_.size() << std::endl;

        std::vector<int> heuristic_table(graph.data_ptr_->all_edges_vec_.size(), MAX<int>);

        std::vector<HyperNodeType*> current_set, all_ptr_set;

        HyperNodeType* init_node_ptr = new HyperNodeType(graph.target_hyper_node_, nullptr, graph, distinguish_sat);
//        init_node_ptr->visited_agent_ = { distinguish_sat ? 2*agent_id + 1 : agent_id };

        current_set.push_back(init_node_ptr);

        all_ptr_set.push_back(init_node_ptr);

        heuristic_table[init_node_ptr->current_node_] = init_node_ptr->g_val_; // in target, related agent is itself, so heuristic_table = 1

        int current_h, next_h;

        int count = 0;
        int count_set = 0;
        while(!current_set.empty()) {
            count_set += current_set.size();
//            std::cout << "current_set.size " << current_set.size() << std::endl;
            std::vector<HyperNodeType*> next_set;
            for(const auto& node_ptr : current_set) {

                current_h = heuristic_table[node_ptr->current_node_];

                for(const auto& neighbor_node_id : graph.data_ptr_->all_edges_vec_backward_[node_ptr->current_node_]) {

                    HyperNodeType* next_node_data_ptr = new HyperNodeType(neighbor_node_id, node_ptr, graph, distinguish_sat);
                    all_ptr_set.push_back(next_node_data_ptr);

                    next_h = next_node_data_ptr->g_val_;

                    if(heuristic_table[neighbor_node_id] > next_h) {
                        heuristic_table[neighbor_node_id] = next_h;
//                        std::cout << "set hyper node " << neighbor_node_id << " heuristic value to " << next_h << std::endl;
                        next_set.push_back(next_node_data_ptr);
                    }
                }
            }
            current_set.clear();
            std::swap(current_set, next_set);
            count ++;
        }
        // release data
        for(auto& a_ptr : all_ptr_set) {
            delete a_ptr;
            a_ptr = nullptr;
        }
        // use BFS to calculate heuristic value for all free grid, and obstacle heuristic
        assert(heuristic_table[graph.target_hyper_node_] != MAX<int>);
        assert(heuristic_table[graph.start_hyper_node_]  != MAX<int>);

        //std::cout << "count_set = " << count_set << std::endl;

        return heuristic_table;
    }



    template<Dimension N, typename HyperNodeType>
    class PrecomputationOfMAPFDecomposition : public PrecomputationOfMAPFDecompositionBase, public PrecomputationOfMAPF<N>  {
    public:

        PrecomputationOfMAPFDecomposition(const Instances<N>& instance,
                                          DimensionLength* dim,
                                          const IS_OCCUPIED_FUNC<N>& isoc,
                                          bool with_sat_heu = true) :
                                          PrecomputationOfMAPF<N>(instance, dim, isoc),
                                          PrecomputationOfMAPFDecompositionBase(with_sat_heu) {

            MSTimer mst;

            auto connect_data_ptr = getAgentConnectivityGraph();
            for(int agent_id=0; agent_id<this->instances_.size(); agent_id++) {
                LA_MAPF::ConnectivityGraph cg;
                cg.data_ptr_ = connect_data_ptr;
                cg.start_hyper_node_ = connect_data_ptr->hyper_node_id_map_[this->instance_node_ids_[agent_id].first];
                cg.target_hyper_node_  = connect_data_ptr->hyper_node_id_map_[this->instance_node_ids_[agent_id].second];
                this->connect_graphs_.push_back(cg);
                this->heuristic_tables_sat_.push_back(calculateAgentHyperGraphStaticHeuristic<N, HyperNodeType>(agent_id,
                                                                                                this->dim_,
                                                                                                this->connect_graphs_[agent_id],
                                                                                                true));

                if(this->with_ignore_sat_heu_) {
                    this->heuristic_tables_.push_back(calculateAgentHyperGraphStaticHeuristic<N, HyperNodeType>(agent_id,
                                                                                                this->dim_,
                                                                                                this->connect_graphs_[agent_id],
                                                                                                false));
                }
            }

            this->initialize_time_cost_ =  mst.elapsed();

            std::cout << "-- MAPF initialize_time_cost_ (ms) = " << this->initialize_time_cost_ << std::endl;
        }



        LA_MAPF::ConnectivityGraphDataPtr getAgentConnectivityGraph() const {
            assert(!this->agent_sub_graphs_.empty());
            LA_MAPF::ConnectivityGraphDataPtr data_ptr;
            data_ptr = std::make_shared<LA_MAPF::ConnectivityGraphData>(this->all_poses_.size());

            data_ptr->related_agents_map_.resize(this->all_poses_.size(), {});

            for(int i=0; i<this->agent_sub_graphs_.size(); i++) {
                data_ptr->related_agents_map_[this->instance_node_ids_[i].first] = { 2*i };
                data_ptr->related_agents_map_[this->instance_node_ids_[i].second] = { 2*i+1 };
            }

            LA_MAPF::SubGraphOfAgent<N, Pointi<N>> current_subgraph = this->agent_sub_graphs_.front();

            // 2, construct connectivity graph and record boundary (where different hyper node converge)
            const auto& retv_pair = LA_MAPF::getStrongComponentFromSubGraph<Pointi<N>>(current_subgraph.data_ptr_->all_nodes_,
                                                                                 current_subgraph.data_ptr_->all_edges_,
                                                                                 current_subgraph.data_ptr_->all_backward_edges_,
                                                                                 data_ptr->related_agents_map_,
                                                                                 true);


            const auto& strong_components = retv_pair.first;
            const auto& component_id_map  = retv_pair.second;

            int max_hyper_node_id = 0;
            std::vector<std::pair<size_t, size_t> > boundary_nodes;

            std::vector<std::vector<size_t> > node_in_hyper_node;

            // considering the using boost to detect strong connected component of directed graph
            for(const std::set<size_t>& cur_component : strong_components) {
                //if(cur_component.size() == 1)
                {
                    if(current_subgraph.data_ptr_->all_nodes_[*cur_component.begin()] == nullptr) { continue; }
                }
                for (const size_t & node_id : cur_component) {
                    const auto &current_pose_ptr = current_subgraph.data_ptr_->all_nodes_[node_id];
                    if (current_pose_ptr == nullptr) { continue; }
                    if (data_ptr->hyper_node_id_map_[node_id] != MAX<size_t>) { continue; }
                    data_ptr->hyper_node_id_map_[node_id] = max_hyper_node_id;

                    node_in_hyper_node.push_back({});
                    node_in_hyper_node.back().push_back(node_id);

                    std::vector<size_t> nodes_buffer = {node_id}, next_buffer = {};
                    while (!nodes_buffer.empty()) {
                        next_buffer.clear();
                        for (const auto &cur_node : nodes_buffer) {
                            // traversal all neighboring nodes
                            for (const auto &neighbor_node_id : current_subgraph.data_ptr_->all_edges_[cur_node]) {

                                // if belong to different component
                                //if(cur_component.find(neighbor_node_id) == cur_component.end())
                                if(component_id_map[cur_node] != component_id_map[neighbor_node_id]) {
                                    // when two node belong to different component
                                    size_t node_from = MAX<size_t>, node_to = MAX<size_t>;
                                    node_from = cur_node;
                                    node_to = neighbor_node_id;

                                    //if(node_from != MAX<size_t> && node_to != MAX<size_t>)
                                    {
                                        boundary_nodes.push_back({node_from, node_to});
                                    }
                                } else {

                                    if (data_ptr->related_agents_map_[node_id] == data_ptr->related_agents_map_[neighbor_node_id]) {
                                        if (data_ptr->hyper_node_id_map_[neighbor_node_id] != MAX<size_t>) {
                                            if(data_ptr->hyper_node_id_map_[neighbor_node_id] != max_hyper_node_id) {
                                                // if two node have different related agent, they are on boundary
                                                size_t node_from = MAX<size_t>, node_to = MAX<size_t>;
                                                node_from = cur_node;
                                                node_to = neighbor_node_id;
                                                //if(node_from != MAX<size_t> && node_to != MAX<size_t>)
                                                {
                                                    boundary_nodes.push_back({node_from, node_to});
                                                }
                                            }
                                            continue;
                                        }
                                        data_ptr->hyper_node_id_map_[neighbor_node_id] = max_hyper_node_id;
                                        next_buffer.push_back(neighbor_node_id);

                                        node_in_hyper_node.back().push_back(neighbor_node_id);

                                    } else {
                                        // shouldn't reach here, because all node in the same component have the same related agents
                                        assert(0);
                                        // if two node have different related agent, they are on boundary
                                        size_t node_from = MAX<size_t>, node_to = MAX<size_t>;
                                        node_from = cur_node;
                                        node_to = neighbor_node_id;
                                        //if(node_from != MAX<size_t> && node_to != MAX<size_t>)
                                        {
                                            boundary_nodes.push_back({node_from, node_to});
                                        }
                                    }
                                }
                            }
                        }
                        std::swap(nodes_buffer, next_buffer);
                    }
                    max_hyper_node_id++;
                }
            }


            data_ptr->all_edges_set_.resize(max_hyper_node_id);
            data_ptr->all_edges_vec_.resize(max_hyper_node_id);
            data_ptr->all_edges_vec_backward_.resize(max_hyper_node_id);


            // 4, get connections between hyper graph nodes
            data_ptr->hyper_node_with_agents_.resize(max_hyper_node_id);
            for(int cur_hyper_node_id=0; cur_hyper_node_id<max_hyper_node_id; cur_hyper_node_id++) {
                data_ptr->hyper_node_with_agents_[cur_hyper_node_id] =
                        data_ptr->related_agents_map_[node_in_hyper_node[cur_hyper_node_id].front()];
            }

            for(const auto& node_id_pair : boundary_nodes) {
                const auto& cur_hyper_node_id  = data_ptr->hyper_node_id_map_[node_id_pair.first];
                const auto& next_hyper_node_id = data_ptr->hyper_node_id_map_[node_id_pair.second];

                assert(cur_hyper_node_id != next_hyper_node_id);
                assert(cur_hyper_node_id != MAX<size_t> && next_hyper_node_id != MAX<size_t>);

                if(data_ptr->all_edges_set_[cur_hyper_node_id].find(next_hyper_node_id) == data_ptr->all_edges_set_[cur_hyper_node_id].end()) {

                    data_ptr->all_edges_set_[cur_hyper_node_id].insert(next_hyper_node_id);
                    data_ptr->all_edges_vec_[cur_hyper_node_id].push_back(next_hyper_node_id);

                    data_ptr->all_edges_vec_backward_[next_hyper_node_id].push_back(cur_hyper_node_id);
                }
            }


            return data_ptr;
        }

    };



    template<Dimension N, typename State>
    bool LA_MAPF_DecompositionValidCheckGridMap(const std::vector<std::set<int> >& all_levels,
                                                DimensionLength* dim,
                                                const IS_OCCUPIED_FUNC<N>& isoc,
                                                const LA_MAPF::AgentPtrs<N>& agents,
                                                const std::vector<std::pair<size_t, size_t> >& instance_node_ids,
                                                const std::vector<PosePtr<int, N> >& all_poses,
                                                const std::vector<LA_MAPF::SubGraphOfAgent<N, State> >& agent_sub_graphs,
                                                const std::vector<std::vector<int> >& agents_heuristic_tables,
                                                const std::vector<std::vector<int> >& agents_heuristic_tablesignore_rotate
    ) {

        float max_excircle_radius = getMaximumRadius(agents);
        std::vector<LA_MAPF::AgentPtr<N> > pre_agents;
        std::vector<size_t> pre_targets;


        for(int i=0; i<all_levels.size(); i++) {
            std::vector<LA_MAPF::AgentPtr<N> > cur_agents;
            std::vector<size_t> cur_targets;


            for(const auto& agent_id : all_levels[i]) {
                cur_agents.push_back(agents[agent_id]);
                cur_targets.push_back(instance_node_ids[agent_id].second);
            }


            LA_MAPF::LargeAgentStaticConstraintTablePtr<N, State>
                    new_constraint_table_ptr_ = std::make_shared<LA_MAPF::LargeAgentStaticConstraintTable<N, State> > (
                    max_excircle_radius, dim, isoc, agents, cur_agents, all_poses);

            new_constraint_table_ptr_->insertPoses(pre_agents, pre_targets);

            pre_agents.insert(pre_agents.end(), cur_agents.begin(), cur_agents.end());
            pre_targets.insert(pre_targets.end(), cur_targets.begin(), cur_targets.end());


            // insert future agents' start as static constraint
            for(int j = i+1; j<all_levels.size(); j++)
            {
                const auto& current_cluster = all_levels[j];
                for(const int& agent_id : current_cluster) {
                    new_constraint_table_ptr_->insertPose(agent_id, instance_node_ids[agent_id].first);
                }
            }


            new_constraint_table_ptr_->updateEarliestArriveTimeForAgents(cur_agents, cur_targets);
            for(const auto& agent_id : all_levels[i]) {
                //                    std::cout << "-- agent " << agent_id << " valid check ... " << std::endl;
                LA_MAPF::CBS::ConstraintTable<N, State> constraint_table(agent_id,
                                                                  agents,
                                                                  all_poses,
                                                                  dim,
                                                                  isoc);

                LA_MAPF::CBS::SpaceTimeAstar<N, State> solver(instance_node_ids[agent_id].first,
                                                                  instance_node_ids[agent_id].second,
                                                                  agents_heuristic_tables[agent_id],
                                                                  agents_heuristic_tablesignore_rotate[agent_id],
                                                                  agent_sub_graphs[agent_id],
                                                                  constraint_table,
                                                                  nullptr,
                                                                  new_constraint_table_ptr_,
                                                                  nullptr //&connect_graphs_[4] // only in debug !
                );
                LA_MAPF::LAMAPF_Path path = solver.solve();
                if(path.empty()) {
                    std::cout << "FATAL: cluster " << i << ", agent " << agent_id << " unsolvable after decomposition" << std::endl;
                    assert(0);
                    return false;
                }
            }
        }
        return true;
    }

}

#endif //LAYEREDMAPF_PRECOMPUTATION_FOR_DECOMPOSITION_H
