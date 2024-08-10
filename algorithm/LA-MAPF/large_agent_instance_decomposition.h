//
// Created by yaozhuo on 2024/8/10.
//

#ifndef LAYEREDMAPF_LARGE_AGENT_INSTANCE_DECOMPOSITION_H
#define LAYEREDMAPF_LARGE_AGENT_INSTANCE_DECOMPOSITION_H

#include "large_agent_mapf.h"

namespace freeNav::LayeredMAPF::LA_MAPF {



    template<Dimension N, typename AgentType>
    class LargeAgentMAPFInstanceDecomposition : public LargeAgentMAPF<N, AgentType> {
    public:

        // due to each agent may have different shape, each agent have their own connectivity graph
        struct ConnectivityGraph {

            explicit ConnectivityGraph(int total_nodes) {
                hyper_node_id_map_.resize(total_nodes, MAX<size_t>);
                related_agents_map_.resize(total_nodes, {});
            }

            std::vector<std::vector<int> > related_agents_map_; // store each pose collide with what agents' start(2*id) or target(2*id+1)

            // store each node's hyper node id, default to MAX<size_t>
            // may be disposed after we construct ConnectivityGraph
            std::vector<size_t> hyper_node_id_map_;

            // store what agent current hyper node associate with
            // a hyper node may associate with no agent may also associate with multiple agent
            std::vector<std::vector<int> > hyper_node_with_agents_;

            std::vector<std::vector<size_t> > all_edges_vec_; // each hyper node's connecting node, store in vector

            std::vector<std::set<size_t> > all_edges_set_; // each hyper node's connecting node, store in set

            size_t start_hyper_node_; // where is start in the hyper graph

            size_t target_hyper_node_; // where is target in the hyper graph

        };

        LargeAgentMAPFInstanceDecomposition(const InstanceOrients<N> & instances,
                                            const std::vector<AgentType>& agents,
                                            DimensionLength* dim,
                                            const IS_OCCUPIED_FUNC<N> & isoc)
                                            : LargeAgentMAPF<N, AgentType>(instances, agents, dim, isoc) {
#           // 1, construct each agent's connectivity graph
            for(int i=0; i<agents.size(); i++) {
                connect_graphs_.push_back(getAgentConnectivityGraph(i));
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

                const auto& another_agent_start_pose  = this->instances_[agent_id].first;
                const auto& another_agent_target_pose = this->instances_[agent_id].second;

                const auto& another_agent_start_pt  = another_agent_start_pose.pt_;
                const auto& another_agent_target_pt = another_agent_target_pose.pt_;

                // get the maximum range of conflict
                float max_range_radius = agent.excircle_radius_ + another_agent.excircle_radius_;
                int local_space_width = 2*ceil(max_range_radius) + 1;
                // construct a temp local space
                DimensionLength dim[N];
                Pointi<N> center_pt;
                for(int d=0; d<N; d++) {
                    dim[d] = local_space_width;
                    center_pt[d] = ceil(max_range_radius);
                }
                int local_total_index = getTotalIndexOfSpace<N>(dim);
                Pointi<N> temp_pt, temp_start, temp_target;
                size_t temp_pose_id;
                Id temp_id;
                for(int gid=0; gid<local_total_index; gid++) {
                    temp_pt = IdToPointi<N>(gid, dim) - center_pt;
                    if(temp_pt.Norm() > max_range_radius || temp_pt.Norm() <= another_agent.incircle_radius_) { continue; }
                    // 1) for another agent's start
                    temp_start = another_agent_start_pt + temp_pt;
                    // isoc contain in range test implicitly
                    if(!this->isoc_(temp_start)) {
                        temp_id = PointiToId<N>(temp_start, dim);
                        for(int orient=0; orient<2*N; orient++) {
                            // get a nearby pose id
                            temp_pose_id = temp_id*2*N + orient;
                            // check whether this node is available in current agent's subgraph
                            if(current_subgraph.all_nodes_[temp_pose_id] != nullptr) {
                                if(isCollide(agent, *current_subgraph.all_nodes_[temp_pose_id],
                                             another_agent, another_agent_start_pose)) {
                                    // if they have conflict
                                    graph.related_agents_map_[temp_pose_id].push_back(2*agent_id);
                                }
                            }
                        }
                    }
                    // 2) for another agent's target
                    temp_target = another_agent_target_pt + temp_pt;
                    // isoc contain in range test implicitly
                    if(!this->isoc_(temp_target)) {
                        temp_id = PointiToId<N>(temp_target, dim);
                        for(int orient=0; orient<2*N; orient++) {
                            // get a nearby pose id
                            temp_pose_id = temp_id*2*N + orient;
                            // check whether this node is available in current agent's subgraph
                            if(current_subgraph.all_nodes_[temp_pose_id] != nullptr) {
                                if(isCollide(agent, *current_subgraph.all_nodes_[temp_pose_id],
                                             another_agent, another_agent_target_pose)) {
                                    // if they have conflict
                                    graph.related_agents_map_[temp_pose_id].push_back(2*agent_id + 1);
                                }
                            }
                        }
                    }
                }
            }
            // 2, construct connectivity graph and record boundary (where different hyper node converge)
            int current_hyper_node_id = 0;
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
                                    if (graph.hyper_node_id_map_[neighbor_node_id] != current_hyper_node_id) {
                                        boundary_nodes.push_back(neighbor_node_id);
                                    }
                                    continue;
                                }
                                graph.hyper_node_id_map_[neighbor_node_id] = current_hyper_node_id;
                                next_buffer.push_back(neighbor_node_id);
                            }
                        }
                    }
                    std::swap(nodes_buffer, next_buffer);
                }
                current_hyper_node_id ++;
            }
            graph.all_edges_set_.resize(current_hyper_node_id);
            graph.all_edges_vec_.resize(current_hyper_node_id);
            // 3, get start hyper node id and target hyper node id
            graph.start_hyper_node_  = graph.hyper_node_id_map_[this->instance_node_ids_[agent_id].first];
            graph.target_hyper_node_ = graph.hyper_node_id_map_[this->instance_node_ids_[agent_id].second];
            // 4, get connections between hyper graph nodes
            for(const auto& node_id : boundary_nodes) {
                const auto& cur_hyper_node_id = graph.hyper_node_id_map_[node_id];
                if(graph.hyper_node_with_agents_[current_hyper_node_id].empty()) {
                    graph.hyper_node_with_agents_[current_hyper_node_id] = graph.related_agents_map_[node_id];
                }
                for(const auto& nearby_node_id : current_subgraph.all_edges_[node_id]) {
                    const auto& next_hyper_node_id = graph.hyper_node_id_map_[nearby_node_id];
                    if(cur_hyper_node_id != next_hyper_node_id) {
                        if(graph.all_edges_set_[node_id].find(next_hyper_node_id) == graph.all_edges_set_[node_id].end()) {
                            graph.all_edges_set_[cur_hyper_node_id].insert(next_hyper_node_id);
                            graph.all_edges_vec_[cur_hyper_node_id].push_back(next_hyper_node_id);
                        }
                        if(graph.all_edges_set_[nearby_node_id].find(cur_hyper_node_id) == graph.all_edges_set_[nearby_node_id].end()) {
                            graph.all_edges_set_[next_hyper_node_id].insert(cur_hyper_node_id);
                            graph.all_edges_vec_[next_hyper_node_id].push_back(cur_hyper_node_id);
                        }
                    }
                }
            }
            return graph;
        }

        bool solve(double time_limit, int cost_lowerbound = 0, int cost_upperbound = MAX_COST) {
            return false;
        }

        std::vector<ConnectivityGraph> connect_graphs_;

    };


}
#endif //LAYEREDMAPF_LARGE_AGENT_INSTANCE_DECOMPOSITION_H
