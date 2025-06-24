//
// Created by yaozhuo on 6/16/25.
//

#ifndef LAYEREDMAPF_CONNECTIVITY_GRAPH_AND_SUBPRGRAPH_H
#define LAYEREDMAPF_CONNECTIVITY_GRAPH_AND_SUBPRGRAPH_H

#include "LA-MAPF/common.h"
#include "LA-MAPF/large_agent_mapf.h"
#include "LA-MAPF/circle_shaped_agent.h"
#include "LA-MAPF/CBS/space_time_astar.h"
#include "LA-MAPF/large_agent_dependency_path_search.h"

namespace freeNav::LayeredMAPF {


    template<Dimension N>
    struct HyperGraphNodeDataRaw;

    template<Dimension N>
    using HyperGraphNodeDataRawPtr = HyperGraphNodeDataRaw<N>*;

    template<Dimension N>
    struct HyperGraphNodeDataRaw : public TreeNode<N, HyperGraphNodeDataRawPtr<N> > {

        explicit HyperGraphNodeDataRaw(const size_t & current_node,
                                       const HyperGraphNodeDataRawPtr<N>& parent,
                                       const LA_MAPF::ConnectivityGraph& graph,
                                       bool distinguish_sat = false, // whether visited grid distinguish start or target
                                       const std::vector<bool>& ignore_cost_agent_ids = {}) :
                current_node_(current_node), graph_(graph), TreeNode<N, HyperGraphNodeDataRawPtr<N>>(parent) {
            if(parent != nullptr) {
                g_val_ = parent->g_val_;
            }
            // if is a agent node, rather than a free group node
//            for(const int& agent_id : graph_.data_ptr_->hyper_node_with_agents_[current_node_]) {
//                if(!ignore_cost_agent_ids.empty() && ignore_cost_agent_ids[agent_id]) { continue; }
//                visited_agent_.insert(distinguish_sat ? agent_id : agent_id/2);
//            }
            g_val_ = g_val_ + graph_.data_ptr_->hyper_node_with_agents_[current_node_].size();
        }

        void copy(const HyperGraphNodeDataRaw<N>& other_node) {
            g_val_            = other_node.g_val_;
            h_val_            = other_node.h_val_;
            current_node_     = other_node.current_node_;
            this->pa_         = other_node.pa_;
            this->ch_         = other_node.ch_;
        }

        size_t current_node_;

        // only considering agent grid it path, ignore free grid group it pass
        // how many agent it need to cross till reach target, if both start and target of an agent occur in the path, dist plus only one
        // int g_val_ = MAX<int>; // dist from start to here
        // g_val = visited_agent_.size()

        //std::set<int> visited_agent_;

        int g_val_ = 0; // an agent id with not occur twice, so no need for std::set, which is every time consuming

        int h_val_ = 0; // estimation dist from here to target

        const LA_MAPF::ConnectivityGraph& graph_;

        int getFVal() {
            return g_val_ + h_val_;
        }

        struct compare_node {
            // returns true if n1 > n2 (note -- this gives us *min*-heap).
            bool operator()(const HyperGraphNodeDataRawPtr<N> &n1, const HyperGraphNodeDataRawPtr<N> &n2) const {
                return n1->g_val_ + n1->h_val_ >= n2->g_val_ + n2->h_val_;
            }
        };

        struct equal_node {
            // returns true if n1 > n2 (note -- this gives us *min*-heap).
            bool operator()(const HyperGraphNodeDataRawPtr<N> &n1, const HyperGraphNodeDataRawPtr<N> &n2) const {
                return n1->current_node_ == n2->current_node_;
            }
        };

        struct NodeHasher
        {
            size_t operator() (const HyperGraphNodeDataRawPtr<N>& n) const
            {
                return std::hash<int>()(n->current_node_); // yz: 按位异或
            }
        };

        bool in_openlist_ = false;

        typedef typename boost::heap::pairing_heap< HyperGraphNodeDataRawPtr<N>, boost::heap::compare<typename HyperGraphNodeDataRaw<N>::compare_node> >::handle_type open_handle_t;

        open_handle_t open_handle_;

    };

    // generate subgraph and connectivity graph for MAPF and LA-MAPF

    template<Dimension N, typename HyperNodeType>
    class PrecomputationOfLAMAPF : public LA_MAPF::LargeAgentMAPF<N> {
    public:

        PrecomputationOfLAMAPF(const InstanceOrients<N> & instances,
                               const std::vector<LA_MAPF::AgentPtr<N> >& agents,
                               DimensionLength* dim,
                               const IS_OCCUPIED_FUNC<N> & isoc,
                               bool with_sat_heu = true,
                               bool directed_graph = true,
                               int num_of_CPU = 6) :
                               LA_MAPF::LargeAgentMAPF<N>(instances, agents, dim, isoc),
                               with_sat_heu_(with_sat_heu),
                               directed_graph_(directed_graph) {
            if (!this->solvable) {
                std::cout << "detect un solvable instance" << std::endl;
                return;
            }
            MSTimer mst;
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
            int interval = (int)std::ceil(agents.size()/(double)num_of_CPU);// set to larger value to reduce maximal memory usage and num of threads
            // num of thread should close to the num of CPU
//            int num_threads = agents.size()/interval;
            //std::cout << " num of threads = " << num_threads << std::endl;
            //std::cout << "flag 2" << std::endl;
            std::mutex lock_1, lock_2, lock_3, lock_4;
            std::vector<bool> finished(agents.size(), false);
            int count_of_instance = 0;
            while(count_of_instance < instances.size()) {
                //std::cout << "count_of_instance = " << count_of_instance << std::endl;
                auto lambda_func = [&]() {
                    MSTimer mst_1;
                    for (int k = 0; k < interval; k++) {
                        //std::cout << "thread_id = " << thread_id << std::endl;
                        lock_4.lock();
                        int map_id = count_of_instance;
                        count_of_instance ++;
                        lock_4.unlock();
                        //std::cout << "PRE LAMAPF map_id = " << map_id << std::endl;
                        if (map_id >= agents.size()) { break; }
                        const auto& connect_graph = getAgentConnectivityGraph(map_id);
                        lock_1.lock();
                        connect_graphs_map.insert({map_id, connect_graph});
                        lock_1.unlock();
                        const auto& heu_sat_table = LA_MAPF::calculateLargeAgentHyperGraphStaticHeuristic<N, HyperGraphNodeDataRaw<N>>(map_id, this->dim_, connect_graph, true);
                        lock_2.lock();
                        heuristic_tables_sat.insert({map_id, heu_sat_table});
                        lock_2.unlock();
                        if(with_sat_heu_) {
                            const auto& heu_ig_sat_table = LA_MAPF::calculateLargeAgentHyperGraphStaticHeuristic<N, HyperGraphNodeDataRaw<N>>(map_id, this->dim_, connect_graph, false);
                            lock_3.lock();
                            heuristic_tables_ig_sat.insert({map_id, heu_ig_sat_table});
                            lock_3.unlock();
                        }
                        lock_4.lock();
                        finished[map_id] = true;
                        lock_4.unlock();
                        //std::cout << "map " << map_id << " take " << mst_1.elapsed() << "ms" << std::endl;
                    }

                };
                std::thread t(lambda_func);
                t.detach();
                usleep(1e3); // sleep 1ms to wait current thread get correct thread_id, 1ms too less
            }
            while(finished != std::vector<bool>(agents.size(), true)) {
                usleep(5e4);
            }
            connect_graphs_.resize(agents.size());
            for(const auto& connec_pair : connect_graphs_map) {
                connect_graphs_[connec_pair.first] = connec_pair.second;
            }
            heuristic_tables_sat_.resize(agents.size());
            for(const auto& connec_pair : heuristic_tables_sat) {
                heuristic_tables_sat_[connec_pair.first] = connec_pair.second;
            }
            heuristic_tables_.resize(agents.size());
            for(const auto& heu_pair : heuristic_tables_ig_sat) {
                heuristic_tables_[heu_pair.first] = heu_pair.second;
            }
#endif

            //auto duration_2 = end_t - start_t_2;
            //std::cout << "merge take " << std::chrono::duration_cast<std::chrono::milliseconds>(duration_2).count() << "ms" << std::endl;

            initialize_time_cost_ =  mst.elapsed(); // // this is not right

//            std::cout << "-- LA-MAPF connect_graphs_ (ms)       = " << connect_time_cost << std::endl;
//            std::cout << "-- LA-MAPF heuristic_tables_sat_ (ms) = " << heu_with_sat_time_cost << std::endl;
//            std::cout << "-- LA-MAPF heu_ig_sat_time_cost (ms)  = " << heu_ig_sat_time_cost << std::endl;

            std::cout << "-- LA-MAPF initialize_time_cost_ (ms) = " << initialize_time_cost_ << std::endl;

        }

        bool solve(int cost_lowerbound = 0, int cost_upperbound = MAX_COST) {
            std::cout << "shouldn't call this" << std::endl;
            assert(0);
            return false;
        }

        std::pair<std::vector<std::set<size_t> >, std::vector<int> > getStrongComponentFromSubGraph(
                const std::vector<PosePtr<int, N>>& all_poses,
                const std::vector<std::vector<size_t> >& all_edges,
                const std::vector<std::vector<size_t> >& all_backward_edges,
                const std::vector<std::set<int> >& related_agents_map,
                bool directed_graph = true) const {

            using namespace boost;
            using Vertex = size_t;

            if(directed_graph) {
                typedef adjacency_list<vecS, vecS, directedS, Vertex> Graph;
                Graph g(all_poses.size());
                for(size_t i=0; i<all_edges.size(); i++) {
                    if(all_poses[i] == nullptr) { continue; }
                    if(all_edges[i].empty() || all_backward_edges[i].empty()) {
                        continue;
                    }
                    for(const size_t& j : all_edges[i]) {
                        assert(i != MAX<size_t> && j != MAX<size_t>);
                        if(related_agents_map[i] != related_agents_map[j]) { continue; }
                        add_edge(Vertex(i), Vertex(j), g);
                    }
                }
                std::vector<int> component(num_vertices(g));
                int num = strong_components(g, &component[0]);
                std::vector<std::set<size_t> > retv(num);
                for (size_t i = 0; i < component.size(); i++) {
                    retv[component[i]].insert(i);
                }

                return {retv, component};
            } else {
                typedef adjacency_list<vecS, vecS, undirectedS, size_t> Graph;
                Graph g;
                for(size_t i=0; i<all_edges.size(); i++) {
                    if(all_edges[i].empty()) { continue; }
                    for(const size_t& j : all_edges[i]) {
                        add_edge(i, j, g);
                    }
                }
                std::vector<int> component(num_vertices(g));
                int num = connected_components(g, &component[0]);
                std::vector<std::set<size_t> > retv(num);
                std::cout << "Total number of strong components: " << num << std::endl;
                for (size_t i = 0; i < component.size(); ++i) {
                    std::cout << "Vertex " << i << " is in component " << component[i] << std::endl;
                    retv[component[i]].insert(i);
                }
                return {retv, component};
            }
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

            LA_MAPF::SubGraphOfAgent<N> current_subgraph = this->agent_sub_graphs_[agent_id];
            assert(current_subgraph.data_ptr_->all_nodes_[this->instance_node_ids_[agent_id].first] != nullptr);
            assert(current_subgraph.data_ptr_->all_nodes_[this->instance_node_ids_[agent_id].second] != nullptr);

            // 1, get each pose's relation with other agent
            graph.data_ptr_->related_agents_map_ = getRelatedAgentGraph(agent_id, current_subgraph.data_ptr_->all_nodes_);
            // assert agent's start and target have no overlap with other agent's start or target
            //assert(graph.data_ptr_->related_agents_map_[this->instance_node_ids_[agent_id].first].size() >= 1);
            //assert(graph.data_ptr_->related_agents_map_[this->instance_node_ids_[agent_id].second].size() >= 1);

            // 2, construct connectivity graph and record boundary (where different hyper node converge)
            const auto& retv_pair = getStrongComponentFromSubGraph(current_subgraph.data_ptr_->all_nodes_,
                                                                   current_subgraph.data_ptr_->all_edges_,
                                                                   current_subgraph.data_ptr_->all_backward_edges_,
                                                                   graph.data_ptr_->related_agents_map_,
                                                                   this->directed_graph_);


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

        bool with_sat_heu_ = true; // whether calculate heuristic_tables_

        bool directed_graph_ = true;

        float initialize_time_cost_ = 0;

    };



    // unordered_set
    template<Dimension N>
    std::set<int> getPassingAgents(const HyperGraphNodeDataRawPtr<N>& node_ptr,
                                   const LA_MAPF::ConnectivityGraph& graph,
                                   bool distinguish_sat = true) {
        std::set<int> retv;
        HyperGraphNodeDataRawPtr<N> buffer = node_ptr;
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
    class PrecomputationOfMAPF {
    public:

        PrecomputationOfMAPF(const Instances<N>& instance,
                             DimensionLength* dim,
                             const IS_OCCUPIED_FUNC<N>& isoc,
                             bool with_sat_heu = true) :
                             instances_(instance),
                             dim_(dim),
                             isoc_(isoc),
                             with_sat_heu_(with_sat_heu) {

            MSTimer mst;

            std::cout << "  construct all possible poses" << std::endl;
            Id total_index = getTotalIndexOfSpace<N>(dim_);
            all_poses_.resize(total_index, nullptr); // a position with 2*N orientation
            Pointi<N> pt;
            for (Id id = 0; id < total_index; id++) {
                pt = IdToPointi<N>(id, dim_);
                if (!isoc_(pt)) {
                    PosePtr<int, N> pose_ptr = std::make_shared<Pose<int, N> >(pt, 0);
                    all_poses_[id] = pose_ptr;
                }
            }

            LA_MAPF::SubGraphOfAgentDataPtr<N> subgraph_data_ptr = constructSubGraphOfAgent();

            for (int agent_id = 0; agent_id < instances_.size(); agent_id++) {
                // check start
                int start_node_id = PointiToId<N>(instances_[agent_id].first, dim_);

                // check target
                int target_node_id = PointiToId<N>(instances_[agent_id].second, dim_);

                instance_node_ids_.push_back(std::make_pair(start_node_id, target_node_id));

                LA_MAPF::AgentPtr<N> agent = std::make_shared<LA_MAPF::CircleAgent<N> >(0.1, agent_id, dim_);

                LA_MAPF::SubGraphOfAgent<N> sub_graph(agent);

                sub_graph.data_ptr_ = subgraph_data_ptr;

                if (sub_graph.data_ptr_->all_nodes_[start_node_id] == nullptr) {
                    std::cout << "FATAL: agent " << agent << "'s start " << instances_[agent_id].first << "^"
                              << instances_[agent_id].first << " is unavailable " << std::endl;
                    solvable_ = false;
                }

                if (sub_graph.data_ptr_->all_nodes_[target_node_id] == nullptr) {
                    std::cout << "FATAL: agent " << agent << "'s target " << instances_[agent_id].second << "^"
                              << instances_[agent_id].second << " is unavailable " << std::endl;
                    solvable_ = false;
                }

                sub_graph.start_node_id_ = start_node_id;
                sub_graph.target_node_id_ = target_node_id;

                agent_sub_graphs_.push_back(sub_graph);

            }

            distance_map_updater_ = std::make_shared<DistanceMapUpdater<N> >(this->isoc_, this->dim_);

            auto connect_data_ptr = getAgentConnectivityGraph();
            for(int agent_id=0; agent_id<instances_.size(); agent_id++) {
                LA_MAPF::ConnectivityGraph cg;
                cg.data_ptr_ = connect_data_ptr;
                cg.start_hyper_node_ = connect_data_ptr->hyper_node_id_map_[instance_node_ids_[agent_id].first];
                cg.target_hyper_node_  = connect_data_ptr->hyper_node_id_map_[instance_node_ids_[agent_id].second];
                connect_graphs_.push_back(cg);
                heuristic_tables_sat_.push_back(calculateAgentHyperGraphStaticHeuristic<N, HyperNodeType>(agent_id,
                                                                                                this->dim_,
                                                                                                connect_graphs_[agent_id],
                                                                                                true));

                if(with_sat_heu_) {
                    heuristic_tables_.push_back(calculateAgentHyperGraphStaticHeuristic<N, HyperNodeType>(agent_id,
                                                                                                this->dim_,
                                                                                                connect_graphs_[agent_id],
                                                                                                false));
                }
            }

            initialize_time_cost_ =  mst.elapsed();

            std::cout << "-- MAPF initialize_time_cost_ (ms) = " << initialize_time_cost_ << std::endl;
        }

        LA_MAPF::SubGraphOfAgentDataPtr<N> constructSubGraphOfAgent() {
            Id total_index = getTotalIndexOfSpace<N>(dim_);

            assert(all_poses_.size() == total_index);

            auto data_ptr = std::make_shared<LA_MAPF::SubGraphOfAgentData<N> >();

            data_ptr->all_nodes_.resize(total_index, nullptr);


            // initial nodes in subgraph
            for(size_t id=0; id<all_poses_.size(); id++) {
                if(all_poses_[id] != nullptr) {
                    const auto& current_pose = all_poses_[id];
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
                        if(isOutOfBoundary(new_pt, dim_)) { continue; }

                        Id another_node_id = PointiToId<N>(new_pt, dim_);
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

        std::pair<std::vector<std::set<size_t> >, std::vector<int> > getStrongComponentFromSubGraph(
                const std::vector<PosePtr<int, N>>& all_poses,
                const std::vector<std::vector<size_t> >& all_edges,
                const std::vector<std::vector<size_t> >& all_backward_edges,
                const std::vector<std::set<int> >& related_agents_map,
                bool directed_graph = true) const {

            using namespace boost;
            using Vertex = size_t;

            if(directed_graph) {
                typedef adjacency_list<vecS, vecS, directedS, Vertex> Graph;
                Graph g(all_poses.size());
                for(size_t i=0; i<all_edges.size(); i++) {
                    if(all_poses[i] == nullptr) { continue; }
                    if(all_edges[i].empty() || all_backward_edges[i].empty()) {
                        //add_vertex(i, g); // any non-nullptr should have a position
//                        add_edge(Vertex(i), Vertex(i), g);
                        continue;
                    }
                    for(const size_t& j : all_edges[i]) {
                        assert(i != MAX<size_t> && j != MAX<size_t>);
                        if(related_agents_map[i] != related_agents_map[j]) { continue; }
                        add_edge(Vertex(i), Vertex(j), g);
                    }
                }
                std::vector<int> component(num_vertices(g));
                int num = strong_components(g, &component[0]);
                std::vector<std::set<size_t> > retv(num);
//                std::cout << "Total number of strong components: " << num << std::endl;
                for (size_t i = 0; i < component.size(); i++) {
//                    std::cout << "Vertex " << i << " is in component " << component[i] << std::endl;
                    retv[component[i]].insert(i);
                }

                return {retv, component};
            } else {
                typedef adjacency_list<vecS, vecS, undirectedS, size_t> Graph;
                Graph g;
                for(size_t i=0; i<all_edges.size(); i++) {
                    if(all_edges[i].empty()) { continue; }
                    for(const size_t& j : all_edges[i]) {
                        add_edge(i, j, g);
                    }
                }
                std::vector<int> component(num_vertices(g));
                int num = connected_components(g, &component[0]);
                std::vector<std::set<size_t> > retv(num);
//                std::cout << "Total number of strong components: " << num << std::endl;
                for (size_t i = 0; i < component.size(); ++i) {
//                    std::cout << "Vertex " << i << " is in component " << component[i] << std::endl;
                    retv[component[i]].insert(i);
                }
                return {retv, component};
            }
        }

        LA_MAPF::ConnectivityGraphDataPtr getAgentConnectivityGraph() const {
            assert(!agent_sub_graphs_.empty());
            LA_MAPF::ConnectivityGraphDataPtr data_ptr;
            data_ptr = std::make_shared<LA_MAPF::ConnectivityGraphData>(this->all_poses_.size());

            data_ptr->related_agents_map_.resize(this->all_poses_.size(), {});

            for(int i=0; i<agent_sub_graphs_.size(); i++) {
                data_ptr->related_agents_map_[instance_node_ids_[i].first] = { 2*i };
                data_ptr->related_agents_map_[instance_node_ids_[i].second] = { 2*i+1 };
            }

            LA_MAPF::SubGraphOfAgent<N> current_subgraph = this->agent_sub_graphs_.front();

            // 2, construct connectivity graph and record boundary (where different hyper node converge)
            const auto& retv_pair = getStrongComponentFromSubGraph(current_subgraph.data_ptr_->all_nodes_,
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

        Instances<N> instances_;

        DimensionLength* dim_;

        IS_OCCUPIED_FUNC<N> isoc_;

        std::vector<PosePtr<int, N> > all_poses_;

        std::vector<std::pair<size_t, size_t> > instance_node_ids_;

        DistanceMapUpdaterPtr<N> distance_map_updater_;

        bool solvable_ = true;

        std::vector<LA_MAPF::SubGraphOfAgent<N> > agent_sub_graphs_;

        std::vector<LA_MAPF::ConnectivityGraph> connect_graphs_;

        std::vector<std::vector<int> > heuristic_tables_sat_; // distinguish_sat = true

        std::vector<std::vector<int> > heuristic_tables_; // distinguish_sat = false

        bool with_sat_heu_ = true; // whether calculate heuristic_tables_

        float initialize_time_cost_ = 0;

    };



    template<Dimension N>
    bool LA_MAPF_DecompositionValidCheckGridMap(const std::vector<std::set<int> >& all_levels,
                                                DimensionLength* dim,
                                                const IS_OCCUPIED_FUNC<N>& isoc,
                                                const LA_MAPF::AgentPtrs<N>& agents,
                                                const std::vector<std::pair<size_t, size_t> >& instance_node_ids,
                                                const std::vector<PosePtr<int, N> >& all_poses,
                                                const std::vector<LA_MAPF::SubGraphOfAgent<N> >& agent_sub_graphs,
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


            LA_MAPF::LargeAgentStaticConstraintTablePtr<N>
                    new_constraint_table_ptr_ = std::make_shared<LA_MAPF::LargeAgentStaticConstraintTable<N> > (
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
                LA_MAPF::CBS::ConstraintTable<N> constraint_table(agent_id,
                                                                  agents,
                                                                  all_poses,
                                                                  dim,
                                                                  isoc);

                LA_MAPF::CBS::SpaceTimeAstar<N> solver(instance_node_ids[agent_id].first,
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

    template<Dimension N>
    bool isMAPFInstanceSolvable(const Pointi<N>& start_pt, const Pointi<N>& target_pt, const IS_OCCUPIED_FUNC<N>& isoc, DimensionLength* dim) {
        Pointis<N> neighbors = GetNearestOffsetGrids<N>();

        Id total_index = getTotalIndexOfSpace<N>(dim);

        std::vector<bool> visited(total_index, false);
        std::vector<Pointi<N> > buffer = {start_pt}, next_buffer;
        visited[PointiToId<N>(start_pt, dim)] = true;

        while (!buffer.empty()) {
            next_buffer.clear();
            for(const Pointi<N>& pt : buffer) {
                for(const Pointi<N>& offset : neighbors) {
                    Pointi<N> new_pt = pt + offset;
                    if(isoc(new_pt)) { continue; }
                    if(new_pt == target_pt) {
                        return true;
                    }
                    Id new_id = PointiToId<N>(new_pt, dim);
                    if(visited[new_id]) { continue; }
                    visited[new_id] = true;
                    next_buffer.push_back(new_pt);
                }
            }
            std::swap(buffer, next_buffer);
        }
        return false;
    }

    template<Dimension N>
    bool MAPF_DecompositionValidCheckGridMap(const Instances<N>& instances,
                                             const std::vector<std::set<int> >& all_levels,
                                             DimensionLength* dim,
                                             const IS_OCCUPIED_FUNC<N>& isoc) {

        for(int i=0; i<all_levels.size(); i++) {
            Id total_index = getTotalIndexOfSpace<N>(dim);
            std::vector<bool> avoid_locs(total_index, false);

            for(int j = 0; j<all_levels.size(); j++)
            {
                if(j == i) continue;
                const auto& current_level = all_levels[j];
                for(const int& agent_id : current_level) {
                    Id id;
                    if(j < i) {
                        id = PointiToId(instances[agent_id].second, dim);
                    } else {
                        id = PointiToId(instances[agent_id].first, dim);
                    }
                    avoid_locs[id] = true;
                }
            }

            auto new_isoc = [&](const Pointi<N> & pt) -> bool {
                if(isOutOfBoundary(pt, dim)) { return true; }
                return isoc(pt) || avoid_locs[PointiToId(pt, dim)];
            };

            for(const int& agent_id : all_levels[i]) {
                if(!isMAPFInstanceSolvable<N>(instances[agent_id].first, instances[agent_id].second, new_isoc, dim)) {
                    return false;
                }
            }
        }
        return true;
    }

}

#endif //LAYEREDMAPF_CONNECTIVITY_GRAPH_AND_SUBPRGRAPH_H
