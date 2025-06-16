//
// Created by yaozhuo on 6/16/25.
//

#ifndef LAYEREDMAPF_CONNECTIVITY_GRAPH_AND_SUBPRGRAPH_H
#define LAYEREDMAPF_CONNECTIVITY_GRAPH_AND_SUBPRGRAPH_H

#include "LA-MAPF/common.h"
#include "LA-MAPF/large_agent_mapf.h"

namespace freeNav::LayeredMAPF {

    template<Dimension N>
    class PrecomputationOfLAMAPF : public LA_MAPF::LargeAgentMAPF<N> {
    public:

        PrecomputationOfLAMAPF(const InstanceOrients<N> & instances,
                               const std::vector<LA_MAPF::AgentPtr<N> >& agents,
                               DimensionLength* dim,
                               const IS_OCCUPIED_FUNC<N> & isoc,
                               bool directed_graph = true) :
                               LA_MAPF::LargeAgentMAPF<N>(instances, agents, dim, isoc),
                               directed_graph_(directed_graph) {

            for(int i=0; i<agents.size(); i++) {
                connect_graphs_.push_back(getAgentConnectivityGraph(i));
                heuristic_tables_sat_.push_back(calculateLargeAgentHyperGraphStaticHeuristic<N>(i, this->dim_, connect_graphs_[i], true));
            }

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

        bool directed_graph_ = true; // whether the subgraph is directed graph
    };

}

#endif //LAYEREDMAPF_CONNECTIVITY_GRAPH_AND_SUBPRGRAPH_H
