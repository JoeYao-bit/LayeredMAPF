//
// Created by yaozhuo on 6/16/25.
//

#ifndef LAYEREDMAPF_CONNECTIVITY_GRAPH_AND_SUBPRGRAPH_H
#define LAYEREDMAPF_CONNECTIVITY_GRAPH_AND_SUBPRGRAPH_H

#include "LA-MAPF/common.h"
#include "LA-MAPF/large_agent_mapf.h"

namespace freeNav::LayeredMAPF {


    // generate subgraph and connectivity graph for MAPF and LA-MAPF

    template<Dimension N>
    class PrecomputationOfLAMAPF : public LA_MAPF::LargeAgentMAPF<N> {
    public:

        PrecomputationOfLAMAPF(const InstanceOrients<N> & instances,
                               const std::vector<LA_MAPF::AgentPtr<N> >& agents,
                               DimensionLength* dim,
                               const IS_OCCUPIED_FUNC<N> & isoc,
                               bool with_sat_heu = true,
                               bool directed_graph = true) :
                               LA_MAPF::LargeAgentMAPF<N>(instances, agents, dim, isoc),
                               with_sat_heu_(with_sat_heu),
                               directed_graph_(directed_graph) {

            clock_t start_t = clock();
            for(int i=0; i<agents.size(); i++) {
                connect_graphs_.push_back(getAgentConnectivityGraph(i));
                heuristic_tables_sat_.push_back(calculateLargeAgentHyperGraphStaticHeuristic<N>(i, this->dim_, connect_graphs_[i], true));
                if(with_sat_heu_) {
                    heuristic_tables_.push_back(calculateLargeAgentHyperGraphStaticHeuristic<N>(i, this->dim_, connect_graphs_[i], false));
                }
            }
            clock_t now_t = clock();

            initialize_time_cost_ =  1e3*((double)now_t - start_t)/CLOCKS_PER_SEC;

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
            assert(graph.data_ptr_->related_agents_map_[this->instance_node_ids_[agent_id].first].size() >= 1);
            assert(graph.data_ptr_->related_agents_map_[this->instance_node_ids_[agent_id].second].size() >= 1);

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


    template<Dimension N>
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

            clock_t start_t = clock();

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

            SubGraphOfAgentDataPtr<N> subgraph_data_ptr = constructSubGraphOfAgent();

            for (int agent_id = 0; agent_id < instances_.size(); agent_id++) {
                // check start
                int start_node_id = PointiToId<N>(instances_[agent_id].first, dim_);

                // check target
                int target_node_id = PointiToId<N>(instances_[agent_id].second, dim_);

                instance_node_ids_.push_back(std::make_pair(start_node_id, target_node_id));

                AgentPtr<N> agent = std::make_shared<CircleAgent<N> >(0.1, agent_id, dim_);

                SubGraphOfAgent<N> sub_graph(agent);

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
                ConnectivityGraph cg;
                cg.data_ptr_ = connect_data_ptr;
                cg.start_hyper_node_ = connect_data_ptr->hyper_node_id_map_[instance_node_ids_[agent_id].first];
                cg.target_hyper_node_  = connect_data_ptr->hyper_node_id_map_[instance_node_ids_[agent_id].second];
                connect_graphs_.push_back(cg);
                heuristic_tables_sat_.push_back(calculateLargeAgentHyperGraphStaticHeuristic<N>(agent_id,
                                                                                                this->dim_,
                                                                                                connect_graphs_[agent_id],
                                                                                                true));

                if(with_sat_heu_) {
                    heuristic_tables_.push_back(calculateLargeAgentHyperGraphStaticHeuristic<N>(agent_id,
                                                                                                this->dim_,
                                                                                                connect_graphs_[agent_id],
                                                                                                false));
                }
            }

            clock_t now_t = clock();

            initialize_time_cost_ =  1e3*((double)now_t - start_t)/CLOCKS_PER_SEC;

            std::cout << "-- MAPF initialize_time_cost_ (ms) = " << initialize_time_cost_ << std::endl;
        }

        SubGraphOfAgentDataPtr<N> constructSubGraphOfAgent() {
            Id total_index = getTotalIndexOfSpace<N>(dim_);

            assert(all_poses_.size() == total_index);

            auto data_ptr = std::make_shared<SubGraphOfAgentData<N> >();

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

        std::vector<SubGraphOfAgent<N> > agent_sub_graphs_;

        std::vector<LA_MAPF::ConnectivityGraph> connect_graphs_;

        std::vector<std::vector<int> > heuristic_tables_sat_; // distinguish_sat = true

        std::vector<std::vector<int> > heuristic_tables_; // distinguish_sat = false

        bool with_sat_heu_ = true; // whether calculate heuristic_tables_

        float initialize_time_cost_ = 0;

    };
}

#endif //LAYEREDMAPF_CONNECTIVITY_GRAPH_AND_SUBPRGRAPH_H
