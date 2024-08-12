//
// Created by yaozhuo on 2024/8/10.
//

#ifndef LAYEREDMAPF_LARGE_AGENT_INSTANCE_DECOMPOSITION_H
#define LAYEREDMAPF_LARGE_AGENT_INSTANCE_DECOMPOSITION_H

#include "large_agent_mapf.h"
#include "large_agent_dependency_path_search.h"

namespace freeNav::LayeredMAPF::LA_MAPF {

    // inherit LargeAgentMAPF to avoid
    template<Dimension N, typename AgentType>
    class LargeAgentMAPFInstanceDecomposition : public LargeAgentMAPF<N, AgentType> {
    public:

        LargeAgentMAPFInstanceDecomposition(const InstanceOrients<N> & instances,
                                            const std::vector<AgentType>& agents,
                                            DimensionLength* dim,
                                            const IS_OCCUPIED_FUNC<N> & isoc)
                                            : LargeAgentMAPF<N, AgentType>(instances, agents, dim, isoc) {
            // 1, construct each agent's connectivity graph
            for(int i=0; i<agents.size(); i++) {
                connect_graphs_.push_back(getAgentConnectivityGraph(i));
            }
            // 2, calculate heuristic table for each connectivity graph
            for(int i=0; i<agents.size(); i++) {
                heuristic_tables_.push_back(calculateLargeAgentHyperGraphStaticHeuristic<N>(this->dim_, connect_graphs_[i]));
            }
        }

        ConnectivityGraph getAgentConnectivityGraph(const int& agent_id) const {
            ConnectivityGraph graph(this->all_poses_.size());
            SubGraphOfAgent<N> current_subgraph = this->agent_sub_graphs_[agent_id];
            // 1, get each pose's relation with other agent
            for(int i=0; i<this->agents_.size(); i++) {
                if(i == agent_id) { continue; }
                const auto& agent = this->agents_[agent_id];
                const auto& another_agent = this->agents_[i];
//                std::cout << "agent/another = " << agent_id << ", " << i << std::endl;
                const auto& another_agent_start_pose  = this->instances_[i].first;
                const auto& another_agent_target_pose = this->instances_[i].second;

                const auto& another_agent_start_pt  = another_agent_start_pose.pt_;
                const auto& another_agent_target_pt = another_agent_target_pose.pt_;

                // get the maximum range of conflict
                float max_range_radius = agent.excircle_radius_ + another_agent.excircle_radius_;
                int local_space_width = 2*ceil(max_range_radius) + 1;
                // construct a temp local space
                DimensionLength local_dim[N];
                Pointi<N> center_pt;
                for(int d=0; d<N; d++) {
                    local_dim[d] = local_space_width;
                    center_pt[d] = ceil(max_range_radius);
                }
//                std::cout << "max_range_radius / ceil(max_range_radius) = " << max_range_radius << " / " << ceil(max_range_radius) << std::endl;
                int local_total_index = getTotalIndexOfSpace<N>(local_dim);
//                std::cout << " local_total_index = " << local_total_index << std::endl;
                Pointi<N> temp_pt, temp_start, temp_target;
                size_t temp_pose_id;
                Id temp_id;
                for(int gid=0; gid<local_total_index; gid++) {
                    temp_pt = IdToPointi<N>(gid, local_dim) - center_pt;
//                    std::cout << "gid / temp_pt /center_pt " << gid << " / " << IdToPointi<N>(gid, dim) << ", " << center_pt << std::endl;
                    if(temp_pt.Norm() > max_range_radius) { continue; }
                    // 1) for another agent's start
                    temp_start = another_agent_start_pt + temp_pt;
                    // isoc contain in range test implicitly
                    if(!this->isoc_(temp_start)) {
//                        std::cout << " temp_start " << temp_start << std::endl;
                        temp_id = PointiToId<N>(temp_start, this->dim_);
                        for(int orient=0; orient<2*N; orient++) {
                            // get a nearby pose id
                            temp_pose_id = temp_id*2*N + orient;
                            // check whether this node is available in current agent's subgraph
                            if(current_subgraph.all_nodes_[temp_pose_id] != nullptr) {
//                                std::cout << " check start conflict" << std::endl;
                                if(isCollide(agent, *current_subgraph.all_nodes_[temp_pose_id],
                                             another_agent, another_agent_start_pose))
                                {
                                    // if they have conflict
                                    graph.related_agents_map_[temp_pose_id].push_back(2*i);
//                                    std::cout << " agent " << agent_id  << "," << i << "'s start have conflict at " << temp_start << std::endl;
                                }
                            }
                        }
                    }
                    // 2) for another agent's target
                    temp_target = another_agent_target_pt + temp_pt;
                    // isoc contain in range test implicitly
                    if(!this->isoc_(temp_target)) {
//                        std::cout << " temp_target " << temp_target << std::endl;
                        temp_id = PointiToId<N>(temp_target, this->dim_);
                        for(int orient=0; orient<2*N; orient++) {
                            // get a nearby pose id
                            temp_pose_id = temp_id*2*N + orient;
                            // check whether this node is available in current agent's subgraph
                            if(current_subgraph.all_nodes_[temp_pose_id] != nullptr) {
//                                std::cout << " check target conflict" << std::endl;
                                if(isCollide(agent, *current_subgraph.all_nodes_[temp_pose_id],
                                             another_agent, another_agent_target_pose))
                                {
                                    // if they have conflict
                                    graph.related_agents_map_[temp_pose_id].push_back(2*i + 1);
//                                    std::cout << " agent " << agent_id  << "," << i << "'s target have conflict at " << temp_target << std::endl;
                                }
                            }
                        }
                    }
                }
            }
            // 2, construct connectivity graph and record boundary (where different hyper node converge)
            int max_hyper_node_id = 0;
            std::vector<size_t> boundary_nodes;
            for(size_t i=0; i<current_subgraph.all_nodes_.size(); i++) {
                const auto& current_pose_ptr = current_subgraph.all_nodes_[i];
                if(current_pose_ptr == nullptr) { continue; }
                if(graph.hyper_node_id_map_[i] != MAX<size_t>) { continue; }
                std::vector<size_t> nodes_buffer = {i}, next_buffer = {};
                while(!nodes_buffer.empty()) {
                    next_buffer.clear();
                    for(const auto& cur_node : nodes_buffer) {
                        // traversal all neighboring nodes
                        for (const auto &neighbor_node_id : current_subgraph.all_edges_[cur_node]) {
                            if (graph.related_agents_map_[i] == graph.related_agents_map_[neighbor_node_id]) {
                                if (graph.hyper_node_id_map_[neighbor_node_id] != MAX<size_t>) {
                                    continue;
                                }
                                graph.hyper_node_id_map_[neighbor_node_id] = max_hyper_node_id;
                                next_buffer.push_back(neighbor_node_id);
                            } else {
                                if (graph.hyper_node_id_map_[neighbor_node_id] != MAX<size_t>) {
                                    boundary_nodes.push_back(cur_node);
                                    boundary_nodes.push_back(neighbor_node_id);
                                }
                            }
                        }
                    }
                    std::swap(nodes_buffer, next_buffer);
                }
                max_hyper_node_id ++;
            }
            graph.all_edges_set_.resize(max_hyper_node_id);
            graph.all_edges_vec_.resize(max_hyper_node_id);
            // 3, get start hyper node id and target hyper node id
            graph.start_hyper_node_  = graph.hyper_node_id_map_[this->instance_node_ids_[agent_id].first];
            graph.target_hyper_node_ = graph.hyper_node_id_map_[this->instance_node_ids_[agent_id].second];
            // 4, get connections between hyper graph nodes
            graph.hyper_node_with_agents_.resize(max_hyper_node_id);
            for(const auto& node_id : boundary_nodes) {
                const auto& cur_hyper_node_id = graph.hyper_node_id_map_[node_id];
                if(graph.hyper_node_with_agents_[cur_hyper_node_id].empty()) {
                    graph.hyper_node_with_agents_[cur_hyper_node_id] = graph.related_agents_map_[node_id];
                }
                for(const auto& nearby_node_id : current_subgraph.all_edges_[node_id]) {
                    const auto& next_hyper_node_id = graph.hyper_node_id_map_[nearby_node_id];
                    if(next_hyper_node_id != MAX<size_t> && cur_hyper_node_id != next_hyper_node_id) {
                        if(graph.all_edges_set_[cur_hyper_node_id].find(next_hyper_node_id) == graph.all_edges_set_[cur_hyper_node_id].end()) {
                            graph.all_edges_set_[cur_hyper_node_id].insert(next_hyper_node_id);
                            graph.all_edges_vec_[cur_hyper_node_id].push_back(next_hyper_node_id);
                        }
                        if(graph.all_edges_set_[next_hyper_node_id].find(cur_hyper_node_id) == graph.all_edges_set_[next_hyper_node_id].end()) {
                            graph.all_edges_set_[next_hyper_node_id].insert(cur_hyper_node_id);
                            graph.all_edges_vec_[next_hyper_node_id].push_back(cur_hyper_node_id);
                        }
                    }
                }
            }
            // print for debug
            std::cout << "agent_id " << agent_id << "'s hyper" << std::endl;
            for(int hyper_node_id=0; hyper_node_id < max_hyper_node_id; hyper_node_id++) {
                std::cout << "hyper_node " << hyper_node_id << " visible to: ";
                for(const int& another_hyper_id : graph.all_edges_vec_[hyper_node_id]) {
                    std::cout << another_hyper_id << " ";
                }
                //std::cout << std::endl;
                std::cout << ", relared to agent: ";
                for(const int& related_agent : graph.hyper_node_with_agents_[hyper_node_id]) {
                    std::cout << related_agent << " ";
                }
                std::cout << std::endl;
            }
            return graph;
        }

        bool solve(double time_limit, int cost_lowerbound = 0, int cost_upperbound = MAX_COST) {
            return false;
        }

        std::vector<ConnectivityGraph> connect_graphs_;

        std::vector<std::vector<int> > heuristic_tables_;

    };


}
#endif //LAYEREDMAPF_LARGE_AGENT_INSTANCE_DECOMPOSITION_H
