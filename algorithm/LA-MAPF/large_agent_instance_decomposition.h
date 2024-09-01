//
// Created by yaozhuo on 2024/8/10.
//

#ifndef LAYEREDMAPF_LARGE_AGENT_INSTANCE_DECOMPOSITION_H
#define LAYEREDMAPF_LARGE_AGENT_INSTANCE_DECOMPOSITION_H

#include "large_agent_mapf.h"
#include "large_agent_dependency_path_search.h"
#include "../../freeNav-base/basic_elements/point.h"
#include "CBS/space_time_astar.h"


#include <sys/time.h>
#include <boost/graph/subgraph.hpp>
#include <boost/graph/adjacency_list.hpp>

#include <boost/config.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/strong_components.hpp>

namespace freeNav::LayeredMAPF::LA_MAPF {

    // inherit LargeAgentMAPF to avoid
    template<Dimension N, typename AgentType>
    class LargeAgentMAPFInstanceDecomposition : public LargeAgentMAPF<N, AgentType> {
    public:

        LargeAgentMAPFInstanceDecomposition(const InstanceOrients<N> & instances,
                                            const std::vector<AgentType>& agents,
                                            DimensionLength* dim,
                                            const IS_OCCUPIED_FUNC<N> & isoc,
                                            bool directed_graph = true, // whether edge between poses is always reversible
                                            int decompose_level=3)
                                            : LargeAgentMAPF<N, AgentType>(instances, agents, dim, isoc),
                                              directed_graph_(directed_graph) {

            assert(decompose_level >= 0 && decompose_level <= 3);

            struct timezone tz;
            struct timeval  tv_pre;
            struct timeval  tv_after;
            gettimeofday(&tv_pre, &tz);
            for(int i=0; i<instances.size(); i++) {
                instance_id_set_.insert(i);
            }
            std::cout << "-- start getAgentConnectivityGraph" << std::endl;
            // 1, construct each agent's connectivity graph
            for(int i=0; i<agents.size(); i++) {
                connect_graphs_.push_back(getAgentConnectivityGraph(i));
            }
            std::cout << "-- finish getAgentConnectivityGraph" << std::endl;
            std::cout << "-- start calculateLargeAgentHyperGraphStaticHeuristic" << std::endl;
            // 2, calculate heuristic table for each connectivity graph
            for(int i=0; i<agents.size(); i++) {
                heuristic_tables_.push_back(calculateLargeAgentHyperGraphStaticHeuristic<N>(i, this->dim_, connect_graphs_[i], false));
                heuristic_tables_sat_.push_back(calculateLargeAgentHyperGraphStaticHeuristic<N>(i, this->dim_, connect_graphs_[i], true));
            }
            std::cout << "-- finish calculateLargeAgentHyperGraphStaticHeuristic" << std::endl;

            gettimeofday(&tv_after, &tz);
            initialize_time_cost_ =
                    (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3;

            // 3, decompose all instances into multiple cluster
            if(decompose_level >= 1) {
                gettimeofday(&tv_pre, &tz);
                instanceDecomposition();
                gettimeofday(&tv_after, &tz);
                instance_decomposition_time_cost_ =
                        (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3;
                //printAllSubProblem(std::string("instance decomposition"));
            }

            // 4，bi-partition clusters till cannot bi-partition
            if(decompose_level >= 2) {
                gettimeofday(&tv_pre, &tz);
                clusterDecomposition();
                gettimeofday(&tv_after, &tz);
                cluster_bipartition_time_cost_ =
                        (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3;
                //printAllSubProblem(std::string("cluster bi-partition"));
            }

            // 5, level sorting
            if(decompose_level >= 3) {
                gettimeofday(&tv_pre, &tz);
                levelSorting();
                gettimeofday(&tv_after, &tz);
                level_sorting_time_cost_ =
                        (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3;
                //printAllSubProblem(std::string("level sorting"));
            }

//            printAllSubProblem(std::string("decomposition final"));

            // initialize_time_cost_
            std::cout << "-- initialize_time_cost_             (ms) = " << initialize_time_cost_ << std::endl;
            std::cout << "-- instance_decomposition_time_cost_ (ms) = " << instance_decomposition_time_cost_ << std::endl;
            std::cout << "-- cluster_bipartition_time_cost_    (ms) = " << cluster_bipartition_time_cost_ << std::endl;
            std::cout << "-- level_sorting_time_cost_          (ms) = " << level_sorting_time_cost_ << std::endl;

            /* print details of decomposition */
            int total_count = 0;
            int max_cluster_size = 0;
            for(int i=0; i<all_clusters_.size(); i++) {
                total_count += all_clusters_[i].size();
                if(all_clusters_[i].size() > max_cluster_size) { max_cluster_size = all_clusters_[i].size(); }
            }
            assert(total_count == this->instances_.size());
            std::cout << "-- max/total size " << max_cluster_size << " / " << this->instances_.size() << std::endl;
            std::cout << "-- Decomposition completeness ? " << decompositionValidCheck(all_clusters_) << std::endl;

            grid_paths_.resize(agents.size(), {});
            agent_visited_grids_.resize(agents.size(), std::vector<int>(this->all_poses_.size(), MAX<int>));
            std::cout << "-- Decomposition completeness (grid map) ? " << decompositionValidCheckGridMap(all_clusters_) << std::endl;

        }

        bool decompositionValidCheck(const std::vector<std::set<int> >& all_levels) const {
            for(int i=0; i<all_levels.size(); i++) {
//                std::cout << " level " << i << " valid check ... " << std::endl;
                std::vector<bool> avoid_sats(2*this->instances_.size(), false);
                for(int j=0; j<all_levels.size(); j++) {
                    if(i == j) { continue; }
                    if(i > j) {
                        for(const int& pre_agent : all_levels[j]) {
                            avoid_sats[2*pre_agent + 1] = true;
//                            std::cout << "set " << pre_agent << " target to occupied" << std::endl;
                        }
                    } else {
                        for(const int& after_agent : all_levels[j]) {
                            avoid_sats[2*after_agent] = true;
//                            std::cout << "set " << after_agent << " start to occupied" << std::endl;
                        }
                    }
                }
                for(const int& agent : all_levels[i]) {
                    auto passing_agents = searchAgent(agent, avoid_sats, {}, true);
                    if(passing_agents.empty()) {
                        std::cout << "ERROR: cluster " << i << ", agent " << agent << " is im-complete" << std::endl;
//                        assert(0);
                        return false;
                    }
                }
            }
            return true;
        }

        bool decompositionValidCheckGridMap(const std::vector<std::set<int> >& all_levels) {

            float max_excircle_radius = getMaximumRadius(this->agents_);
            std::vector<AgentType> pre_agents;
            std::vector<size_t> pre_targets;

            for(int i=0; i<all_levels.size(); i++) {
//                std::cout << "-- level " << i << " valid check ... " << std::endl;

                std::vector<AgentType> cur_agents;
                std::vector<size_t> cur_targets;
                for(const auto& agent_id : all_levels[i]) {
                    cur_agents.push_back(this->agents_[agent_id]);
                    cur_targets.push_back(this->instance_node_ids_[agent_id].second);
                }

                LargeAgentStaticConstraintTablePtr<N, AgentType>
                        new_constraint_table_ptr_ = std::make_shared<LargeAgentStaticConstraintTable<N, AgentType> > (
                        max_excircle_radius, this->dim_, this->isoc_, this->agents_, cur_agents, this->all_poses_);

                new_constraint_table_ptr_->insertPoses(pre_agents, pre_targets);

                pre_agents.insert(pre_agents.end(), cur_agents.begin(), cur_agents.end());
                pre_targets.insert(pre_targets.end(), cur_targets.begin(), cur_targets.end());

                // insert future agents' start as static constraint
                for(int j = i+1; j<all_levels.size(); j++)
                {
                    const auto& current_cluster = all_levels[j];
                    for(const int& agent_id : current_cluster) {
                        new_constraint_table_ptr_->insertPose(agent_id, this->instance_node_ids_[agent_id].first);
                    }
                }
                new_constraint_table_ptr_->updateEarliestArriveTimeForAgents(cur_agents, cur_targets);
                for(const auto& agent_id : all_levels[i]) {
//                    std::cout << "-- agent " << agent_id << " valid check ... " << std::endl;
                    CBS::ConstraintTable<N, AgentType> constraint_table(agent_id,
                                                                        this->agents_,
                                                                        this->all_poses_,
                                                                        this->dim_,
                                                                        this->isoc_);

                    CBS::SpaceTimeAstar<N, AgentType> solver(this->instance_node_ids_[agent_id].first,
                                                             this->instance_node_ids_[agent_id].second,
                                                             this->agents_heuristic_tables_[agent_id],
                                                             this->agents_heuristic_tables_ignore_rotate_[agent_id],
                                                             this->agent_sub_graphs_[agent_id],
                                                             constraint_table,
                                                             nullptr,
                                                             new_constraint_table_ptr_
                                                             );
                    LAMAPF_Path path = solver.solve();
                    grid_paths_[agent_id] = path;
                    agent_visited_grids_[agent_id] = solver.grid_visit_count_table_;

//                    if(agent_id == 2) {
//                        std::cout << "agent path : ";
//                        for(const auto& node_id : path) {
//                            std::cout << "(" <<node_id << ", h=" << connect_graphs_[agent_id].hyper_node_id_map_[node_id] << ")->";
//                        }
//                        std::cout << std::endl;
//                    }

                    if(path.empty()) {
                        std::cout << "FATAL: cluster " << i << ", agent " << agent_id << " unsolvable after decomposition" << std::endl;
//                        std::vector<size_t> visited_nodes = {1550, 1742, 1548, 1549};
//                        for(const auto& visited_node : visited_nodes) {
//                            std::cout << *this->all_poses_[visited_node] << " related to agent: ";
//                            for(const auto& temp_agent_id : this->connect_graphs_[agent_id].related_agents_map_[visited_node]) {
//                                std::cout << temp_agent_id << " ";
//                            }
//                            std::cout << std::endl;
//                        }

                        assert(0);
//                        return false;
                    }
                }
            }
            return true;
        }

        std::vector<std::set<int> > getRelatedAgentGraph(int agent_id, const std::vector<PosePtr<int, N> >& all_nodes) const {
            std::vector<std::set<int> > related_agents_map(all_nodes.size());
            for(int i=0; i<this->agents_.size(); i++) {
                //if(i == agent_id) { continue; }
                const auto& agent = this->agents_[agent_id];
                const auto& another_agent = this->agents_[i];
                const auto& another_subgraph = this->agent_sub_graphs_[i];

                assert(another_subgraph.all_nodes_[this->instance_node_ids_[i].first] != nullptr);
                assert(another_subgraph.all_nodes_[this->instance_node_ids_[i].second] != nullptr);

//                std::cout << "agent/another = " << agent_id << ", " << i << std::endl;
                const auto& another_agent_start_pose  = this->instances_[i].first;
                const auto& another_agent_target_pose = this->instances_[i].second;

                const auto& another_agent_start_pt  = another_agent_start_pose.pt_;
                const auto& another_agent_target_pt = another_agent_target_pose.pt_;

                // get the maximum range of conflict
                float max_range_radius = agent.excircle_radius_ + another_agent.excircle_radius_;
                int maximum_ralated_range = ceil(max_range_radius + sqrt(N)); // considering the boundary of two shape in one grid
//                std::cout << " maximum_related_range = " << maximum_ralated_range << std::endl;
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
//                    if(agent_id == 9 && i == 10) {
//                        std::cout << "gid / temp_pt /center_pt " << gid << " / " << temp_pt
//                                  << ", " << center_pt << std::endl;
//                    }
                    // 1) for another agent's start
                    temp_start = another_agent_start_pt + temp_pt;
                    // isoc contain in range test implicitly
                    if(!this->isoc_(temp_start)) {
//                        if(agent_id == 9 && i == 10) {
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
//                                    if(agent_id == 9 && i == 10) {
//                                        std::cout << "ReAG: " << agent << " at " << *all_nodes[temp_pose_id]
//                                                  << " and " << another_agent << " at " << another_agent_start_pose
//                                                  << " have conflict" << std::endl;
//                                    }
                                } else {
                                    // if current node's edge is collide with other agent
                                    for(int other_orient=0; other_orient<2*N; other_orient++) {
                                        if(other_orient == orient || orient/2 == other_orient/2) { continue; }
                                        other_temp_pose_id = temp_id*2*N + other_orient;
                                        if(all_nodes[other_temp_pose_id] == nullptr) { continue; }
                                        if(isCollide(agent, *all_nodes[temp_pose_id], *all_nodes[other_temp_pose_id],
                                                     another_agent, another_agent_start_pose))
                                        {
//                                            if(agent_id == 9 && i == 10) {
//                                                std::cout << "ReAG: " << agent << " at " << *all_nodes[temp_pose_id] << "->" << *all_nodes[other_temp_pose_id]
//                                                          << " and " << another_agent << " at " << another_agent_start_pose
//                                                          << " have conflict" << std::endl;
//                                            }
                                            // if they have conflict
                                            related_agents_map[temp_pose_id].insert(2*i);
                                            break;
                                        } else {
//                                            if(agent_id == 9 && i == 10) {
//                                                std::cout << agent << " at " << *all_nodes[temp_pose_id] << "->" << *all_nodes[other_temp_pose_id]
//                                                          << " and " << another_agent << " at " << another_agent_start_pose
//                                                          << " have no conflict" << std::endl;
//                                            }
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
//                        std::cout << " temp_target " << temp_target << std::endl;
//                        if(agent_id == 9 && i == 10) {
//                            std::cout << " temp_target " << temp_target << std::endl;
//                        }
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
//                                    std::cout << " agent " << agent_id  << "," << i << "'s target have conflict at " << temp_target << std::endl;
//                                    if(agent_id == 9 && i == 10) {
//                                        std::cout << "ReAG: " << agent << " at " << *all_nodes[temp_pose_id]
//                                                  << " and " << another_agent << " at " << another_agent_target_pose
//                                                  << " have conflict" << std::endl;
//                                    }
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
//                                            if(agent_id == 9 && i == 10) {
//                                                std::cout << "ReAG: " << agent << " at " << *all_nodes[temp_pose_id] << "->" << *all_nodes[other_temp_pose_id]
//                                                          << " and " << another_agent << " at " << another_agent_target_pose
//                                                          << " have conflict" << std::endl;
//                                            }
                                            break;
                                        } else {
//                                            if(agent_id == 9 && i == 10) {
//                                                std::cout << agent << " at " << *all_nodes[temp_pose_id] << "->" << *all_nodes[other_temp_pose_id]
//                                                          << " and " << another_agent << " at " << another_agent_target_pose
//                                                          << " have no conflict" << std::endl;
//                                            }
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

//            std::cout << " agent " << agent_id << " start related agent = ";
//            for(const size_t & id : related_agents_map[this->instance_node_ids_[agent_id].first]) {
//                std::cout << id << " ";
//            }
//            std::cout << std::endl;
//
//            std::cout << " agent " << agent_id << " target related agent = ";
//            for(const size_t & id : related_agents_map[this->instance_node_ids_[agent_id].second]) {
//                std::cout << id << " ";
//            }
//            std::cout << std::endl;


            return related_agents_map;
        }

        ConnectivityGraph getAgentConnectivityGraph(const int& agent_id) const {
            ConnectivityGraph graph(this->all_poses_.size());
            SubGraphOfAgent<N, AgentType> current_subgraph = this->agent_sub_graphs_[agent_id];
            assert(current_subgraph.all_nodes_[this->instance_node_ids_[agent_id].first] != nullptr);
            assert(current_subgraph.all_nodes_[this->instance_node_ids_[agent_id].second] != nullptr);

            // 1, get each pose's relation with other agent
            graph.related_agents_map_ = getRelatedAgentGraph(agent_id, current_subgraph.all_nodes_);

            // assert agent's start and target have no overlap with other agent's start or target
//            assert(graph.related_agents_map_[this->instance_node_ids_[agent_id].first].size() == 1);
//            assert(graph.related_agents_map_[this->instance_node_ids_[agent_id].second].size() == 1);

            // 2, construct connectivity graph and record boundary (where different hyper node converge)
            std::vector<std::set<size_t> > strong_components = getStrongComponentFromSubGraph(current_subgraph.all_nodes_,
                                                                                              current_subgraph.all_edges_,
                                                                                              directed_graph_);

            int max_hyper_node_id = 0;
            std::vector<std::pair<size_t, size_t> > boundary_nodes;

            std::vector<std::vector<size_t> > node_in_hyper_node;

            // considering the using boost to detect strong connected component of directed graph
            for(const std::set<size_t>& cur_component : strong_components) {
                if(cur_component.size() == 1) {
                    if(current_subgraph.all_nodes_[*cur_component.begin()] == nullptr) { continue; }
                }
                for (const size_t & node_id : cur_component) {
                    const auto &current_pose_ptr = current_subgraph.all_nodes_[node_id];
                    if (current_pose_ptr == nullptr) { continue; }
                    if (graph.hyper_node_id_map_[node_id] != MAX<size_t>) { continue; }
                    graph.hyper_node_id_map_[node_id] = max_hyper_node_id;

                    node_in_hyper_node.push_back({});
                    node_in_hyper_node.back().push_back(node_id);

                    std::vector<size_t> nodes_buffer = {node_id}, next_buffer = {};
                    while (!nodes_buffer.empty()) {
                        next_buffer.clear();
                        for (const auto &cur_node : nodes_buffer) {
                            // traversal all neighboring nodes
                            for (const auto &neighbor_node_id : current_subgraph.all_edges_[cur_node]) {

                                // if belong to different component
                                if(cur_component.find(neighbor_node_id) == cur_component.end()) {
                                    // when two node belong to different component
                                    size_t node_from = MAX<size_t>, node_to = MAX<size_t>;
                                    node_from = cur_node;
                                    node_to = neighbor_node_id;

                                    //if(node_from != MAX<size_t> && node_to != MAX<size_t>)
                                    {
                                        boundary_nodes.push_back({node_from, node_to});
                                    }
                                } else {

                                    if (graph.related_agents_map_[node_id] == graph.related_agents_map_[neighbor_node_id]) {
                                        if (graph.hyper_node_id_map_[neighbor_node_id] != MAX<size_t>) {
                                            if(graph.hyper_node_id_map_[neighbor_node_id] != max_hyper_node_id) {
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
                                        graph.hyper_node_id_map_[neighbor_node_id] = max_hyper_node_id;
                                        next_buffer.push_back(neighbor_node_id);

                                        node_in_hyper_node.back().push_back(neighbor_node_id);

                                    } else {
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

            // debug
            // all node belong to the same each hyper node should have the same related_agents_map_
            assert(node_in_hyper_node.size() == max_hyper_node_id);
            for(int cid = 0; cid<max_hyper_node_id; cid++) {
                assert(!node_in_hyper_node[cid].empty());
                for(const auto& another_id : node_in_hyper_node[cid]) {
                    assert(graph.related_agents_map_[node_in_hyper_node[cid].front()] == graph.related_agents_map_[another_id]);
                }
            }

//            for(int node_id=0; node_id<current_subgraph.all_nodes_.size(); node_id++) {
//                const auto &current_pose_ptr = current_subgraph.all_nodes_[node_id];
//                if (current_pose_ptr == nullptr) { continue; }
//            }

            // debug // not pass
            int non_null_count = 0;
            for(int i=0; i<this->agent_sub_graphs_[agent_id].all_nodes_.size(); i++) {
                if(current_subgraph.all_nodes_[i] != nullptr) {
                    non_null_count ++;
                }
            }
            int component_count = 0;
            for(int i=0; i<strong_components.size(); i++) {
                // ignore nullptr
                if(strong_components[i].size() == 1) {
                    if(this->agent_sub_graphs_[agent_id].all_nodes_[*strong_components[i].begin()] == nullptr) { continue; }
                }
                component_count = component_count + strong_components[i].size();
            }
//            std::cout << " non_null_count = " << non_null_count << " / component_count = " << component_count
//                      << " / total_node_count = " << getTotalIndexOfSpace<N>(this->dim_)*2*N << std::endl;

            assert(non_null_count == component_count);

            // debug // pass
            for(int i=0; i<this->all_poses_.size(); i++) {
                if(current_subgraph.all_nodes_[i] != nullptr) {
                    assert(graph.hyper_node_id_map_[i] != MAX<size_t>);
                } else {
                    assert(graph.hyper_node_id_map_[i] == MAX<size_t>);
                }
            }

            graph.all_edges_set_.resize(max_hyper_node_id);
            graph.all_edges_vec_.resize(max_hyper_node_id);
            graph.all_edges_vec_backward_.resize(max_hyper_node_id);

            // 3, get start hyper node id and target hyper node id
            graph.start_hyper_node_  = graph.hyper_node_id_map_[this->instance_node_ids_[agent_id].first];
            graph.target_hyper_node_ = graph.hyper_node_id_map_[this->instance_node_ids_[agent_id].second];
            assert(graph.start_hyper_node_ != MAX<size_t>);
            assert(graph.target_hyper_node_ != MAX<size_t>);



            // 4, get connections between hyper graph nodes
            graph.hyper_node_with_agents_.resize(max_hyper_node_id);
            for(int cur_hyper_node_id=0; cur_hyper_node_id<max_hyper_node_id; cur_hyper_node_id++) {
                graph.hyper_node_with_agents_[cur_hyper_node_id] =
                        graph.related_agents_map_[node_in_hyper_node[cur_hyper_node_id].front()];
            }

            for(const auto& node_id_pair : boundary_nodes) {
//                if(agent_id == 2) {
//                    std::cout << "boundary_node = " << *this->all_poses_[node_id_pair.first]
//                              << "(" << node_id_pair.first << ")"
//                              << "->" << *this->all_poses_[node_id_pair.second]
//                              << "(" << node_id_pair.second << ")"  << std::endl;
//                }
                const auto& cur_hyper_node_id  = graph.hyper_node_id_map_[node_id_pair.first];
                const auto& next_hyper_node_id = graph.hyper_node_id_map_[node_id_pair.second];
                assert(cur_hyper_node_id != next_hyper_node_id);
                assert(cur_hyper_node_id != MAX<size_t> && next_hyper_node_id != MAX<size_t>);

                if(graph.all_edges_set_[cur_hyper_node_id].find(next_hyper_node_id) == graph.all_edges_set_[cur_hyper_node_id].end()) {

                    graph.all_edges_set_[cur_hyper_node_id].insert(next_hyper_node_id);
                    graph.all_edges_vec_[cur_hyper_node_id].push_back(next_hyper_node_id);

                    graph.all_edges_vec_backward_[next_hyper_node_id].push_back(cur_hyper_node_id);
                }
            }

//            assert(hyper_node_set_flag == std::vector<bool>(max_hyper_node_id, true));

            assert(graph.hyper_node_with_agents_[graph.start_hyper_node_].find(2*agent_id) !=
                           graph.hyper_node_with_agents_[graph.start_hyper_node_].end());

            assert(graph.hyper_node_with_agents_[graph.target_hyper_node_].find(2*agent_id+1) !=
                           graph.hyper_node_with_agents_[graph.target_hyper_node_].end());

//            if(agent_id == 9) {
//                std::vector<size_t> node_list = {//186009, 183961, 182937, 184985, 186011
//                        186013,
//                        183965,
//                        182941,
//                        184989,
//                        187035};
//                for(const auto& node_id : node_list) {
//                    std::cout << "node " << node_id << "(hyper id = " << graph.hyper_node_id_map_[node_id] << ")"
//                              << " related agent: ";
//                    for(const size_t & id : graph.related_agents_map_[node_id]) {
//                        std::cout << id << " ";
//                    }
//                    std::cout << std::endl;
//                }
//            }

            // print for debug
//            if(agent_id == 2) {
//                std::cout << "agent_id " << agent_id << "'s hyper" << std::endl;
//                for (int hyper_node_id = 0; hyper_node_id < max_hyper_node_id; hyper_node_id++) {
//                    std::cout << "hyper_node " << hyper_node_id << " visible to: ";
//                    for (const int &another_hyper_id : graph.all_edges_vec_[hyper_node_id]) {
//                        std::cout << another_hyper_id << " ";
//                    }
//                    //std::cout << std::endl;
//                    std::cout << ", relared to agent: ";
//                    for (const int &related_agent : graph.hyper_node_with_agents_[hyper_node_id]) {
//                        std::cout << related_agent << " ";
//                    }
//                    std::cout << std::endl;
//                }
//            }

            return graph;
        }


        std::vector<std::set<size_t> > getStrongComponentFromSubGraph(const std::vector<PosePtr<int, N>>& all_poses,
                                                                      const std::vector<std::vector<size_t> >& all_edges,
                                                                      bool directed_graph = true) const {

            using namespace boost;

            if(directed_graph) {
                typedef adjacency_list<vecS, vecS, directedS, size_t> Graph;
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
                return retv;
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
                return retv;
            }
        }

        // return path that consists of agents
        // distinguish_sat whether consider start and target as one in calculate cost
        std::set<int> searchAgent(int agent_id,
                                  const std::vector<bool>& avoid_agents,
                                  const std::vector<bool>& passing_agents,
                                  bool distinguish_sat = false,
                                  const std::vector<bool>& ignore_cost_set = {}) const {
            assert(!heuristic_tables_.empty() && !heuristic_tables_sat_.empty());
            DependencyPathSearch<N, AgentType> search_machine;
            /*
             * DependencyPathSearch::search(int agent_id,
                                            int start_hyper_node_id,
                                            const SubGraphOfAgent<N>& sub_graph,
                                            const ConnectivityGraph& con_graph,
                                            const std::vector<bool> &avoid_agents,
                                            const std::vector<bool> &passing_agents,
                                            const std::vector<int> heuristic_table,
                                            bool distinguish_sat = false,
                                            const std::vector<int> & ignore_cost_set = {}
                                             )
             * */
            assert(connect_graphs_[agent_id].start_hyper_node_ != MAX<size_t>);
            assert(connect_graphs_[agent_id].target_hyper_node_ != MAX<size_t>);

            return search_machine.search(agent_id, connect_graphs_[agent_id].start_hyper_node_,
                                         this->agent_sub_graphs_[agent_id],
                                         this->connect_graphs_[agent_id],
                                         avoid_agents, passing_agents,
                                         distinguish_sat ? heuristic_tables_sat_[agent_id] : heuristic_tables_[agent_id],
                                         distinguish_sat, ignore_cost_set);
        }

        // transform agents' id to 2*id and 2*id + 1
        std::vector<bool> AgentIdsToSATID(const std::set<int>& agent_ids) const {
            std::vector<bool> retv(2*this->instances_.size(), false);
            for(const int& agent_id : agent_ids) {
                retv[2*agent_id] = true;
                retv[2*agent_id + 1] = true;
            }
            return retv;
        }

        // an instance is split into multiple un-realted cluster
        void instanceDecomposition() {

            // decompose the whole instance to multiple unrelated cluster

            std::set<int> buffer_agents = instance_id_set_; // all agents
            std::vector<std::set<int> > all_clusters;

            // get top level isolated clusters
            std::map<int, std::set<int> > all_agents_path;
            std::vector<bool> buffer_sat = AgentIdsToSATID(buffer_agents);
            for(const int& agent_id : buffer_agents) {
                auto passing_agents = searchAgent(agent_id, {}, buffer_sat); // pass test
//                assert(!passing_agents.empty());
//                if(agent_id == 9) {
//                    std::cout << "--agent " << agent_id << "'s passing_agents = ";
//                    for(const auto& passing_agent : passing_agents) {
//                        std::cout << passing_agent << " ";
//                    }
//                    std::cout << std::endl;
//                }
                all_agents_path.insert({agent_id, passing_agents});
            }
            all_passing_agent_ = all_agents_path;
            // get each agent's dependence agents
            std::map<int, std::set<int> > all_related_agent = updateRelatedGraphFromPassingGraph(all_agents_path);
            // for debug
//            std::cout << "print instanceDecomposition related_graph: " << std::endl;
//            for(const auto& related_pair : all_related_agent) {
//                std::cout << related_pair.first << ": ";
//                for(const auto& other_agent : related_pair.second) {
//                    std::cout << other_agent << " ";
//                }
//                std::cout << std::endl;
//            }
            std::map<int, std::set<int> > cluster_of_agents = clusterAgents(all_related_agent);
            all_clusters_.clear();
            for(const auto& iter : cluster_of_agents) {
                all_clusters_.push_back(iter.second);
            }
            std::sort(all_clusters_.begin(), all_clusters_.end(),[](std::set<int> x,std::set<int> y){return x.size()>y.size();});
        }

        // select the largest cluster and merge remaining agent into one cluster and return
        std::pair<std::set<int>, std::set<int> > splitPartitionCluster(const std::map<int, std::set<int> >& clusters) const {
            std::set<int> max_cluster;
            std::set<int> remaining_clusters;
            for(const auto& cluster : clusters) {
                if(max_cluster.size() < cluster.second.size()) {
                    remaining_clusters.insert(max_cluster.begin(), max_cluster.end());
                    max_cluster = cluster.second;
                } else {
                    remaining_clusters.insert(cluster.second.begin(), cluster.second.end());
                }
            }
            return {max_cluster, remaining_clusters};
        }

        // search what agents is unavoidable for each agent
        // any legal pass agents graph contain all unavoidable agent
        // input: instance_id_set: all agent in current cluster
        //        all_passing_agent: all path of current cluster
        std::map<int, std::set<int> > searchUnAvoidAgentForEachAgent(
                const std::map<int, std::set<int> >& all_passing_agent, const std::set<int>& instance_id_set, bool distinguish_sat = false) const {
            std::map<int, std::set<int> > all_unavoidable_agent;
            std::vector<bool> within_set = AgentIdsToSATID(instance_id_set);
            for(const auto& id_agent_pair : all_passing_agent) {
                const int& agent_id = id_agent_pair.first;
//                std::cout << __FUNCTION__ << " agent_id = " << agent_id << std::endl;
                const std::set<int>& agent_path = id_agent_pair.second;

                for(const int& other_agent : agent_path) {

                    if(all_unavoidable_agent.find(agent_id) == all_unavoidable_agent.end()) {
                        all_unavoidable_agent.insert(std::pair<int, std::set<int> >(agent_id, {}));
                    }

                    std::vector<bool> avoid_list(2*this->instances_.size(), false);
                    avoid_list[other_agent*2] = true, avoid_list[other_agent*2 + 1] = true;
//                    std::cout << __FUNCTION__ << " try avoid  " << other_agent << std::endl;

                    if((searchAgent(agent_id, avoid_list, within_set, distinguish_sat)).empty()) {
                        all_unavoidable_agent.at(agent_id).insert(other_agent);
                        //std::cout << " failed " << std::endl;
                    } else {
                        //std::cout << " success " << std::endl;
                    }
                }
            }
            return all_unavoidable_agent;
        }

        // check whether current set is self-relevant, the first is self-relevant, the second is which agent depend external agent
        std::pair<std::set<int>, std::set<int>> clusterIndependentCheck(const std::set<int>& instance_id_set, const std::set<int>& specific_agents = {}) {
            std::set<int> success_in_set, failed_in_set;
            auto set_need_check = specific_agents.empty() ? instance_id_set : specific_agents;
            std::vector<bool> buffer_sat = AgentIdsToSATID(instance_id_set);
            for (const auto &unavoid_agent : set_need_check) {
                auto passing_agents = searchAgent(unavoid_agent, {}, buffer_sat); // pass test
                if (passing_agents.empty()) {
                    // failed always caused by cluster in remaining set that block the whole way
                    failed_in_set.insert(unavoid_agent);
                } else {
                    success_in_set.insert(unavoid_agent);
                }
            }
            return {success_in_set, failed_in_set};
        }

        bool isClusterIndependent(const std::set<int>& instance_id_set, const std::set<int>& specific_agents = {}) {
            auto set_need_check = specific_agents.empty() ? instance_id_set : specific_agents;
            std::vector<bool> buffer_sat = AgentIdsToSATID(instance_id_set);
            for (const auto &unavoid_agent : set_need_check) {
                auto passing_agents = searchAgent(unavoid_agent, {}, buffer_sat); // pass test
                if (passing_agents.empty()) {
                    // failed always caused by cluster in remaining set that block the whole way
                    return false;
                }
            }
            return true;
        }

        // 1, the input agent set must be solvable
        // 2, split current instance set into to a set (based on largest unavoidable agent's cluster), and remaining agents as the other set

        // 1, divide current instance into unavoidable cluster, in other words, what agents must be MAPF together
        // 2, pick the largest unavoidable cluster as base, check it and remaining cluster whether belong to two separating
        // 3, if can, return to step 1 and input remaining agents
        // 4, if can't, move agents from remaining to the largest unavoidable cluster, until it can
        std::pair<std::set<int>, std::set<int> > biPartitionCluster(const std::set<int>& agents) {
            // upper bound of cluster size
            std::map<int, std::set<int> > all_agents_path;
            std::vector<bool> buffer_sat = AgentIdsToSATID(agents);
            for(const int& agent_id : agents) {
                auto passing_agents = searchAgent(agent_id, {}, buffer_sat); // pass test
                all_agents_path.insert({agent_id, passing_agents});
            }
//            std::cout << "all_agents_path 0: " << std::endl;
//            for(const auto& pair : all_agents_path) {
//                std::cout << "agent " << pair.first << ": ";
//                for(const auto& id : pair.second) {
//                    std::cout << id << " ";
//                }
//                std::cout << std::endl;
//            }

            // determine agent's unavoidable agent
            std::map<int, std::set<int> > all_unavoidable_agent = searchUnAvoidAgentForEachAgent(all_agents_path, agents);
//            std::cout << "all_unavoidable_agent: " << std::endl;
//            for(const auto& pair : all_unavoidable_agent) {
//                std::cout << "agent " << pair.first << ": ";
//                for(const auto& id : pair.second) {
//                    std::cout << id << " ";
//                }
//                std::cout << std::endl;
//            }

            // lower bound of cluster size
            // determine each agent's related agent (unavoidable)
            std::map<int, std::set<int> > all_unavoidable_related_agent = updateRelatedGraphFromPassingGraph(all_unavoidable_agent);
            //
            std::map<int, std::set<int> > cluster_of_unavoidable_agents = clusterAgents(all_unavoidable_related_agent);

            // get the largest cluster and merge the remaining into a secondary set
            auto cluster_pair = splitPartitionCluster(cluster_of_unavoidable_agents);

            int count_of_phase = 0;
            while(1) {
                while (1) {
                    auto retv = clusterIndependentCheck(cluster_pair.second);
                    std::set<int> keep_in_remaining = retv.first, move_to_unavoid = retv.second;
                    // move agent that cannot stay in remaining set to unavoidable set
                    for (const int &moved_agent_id : move_to_unavoid) {
                        cluster_pair.first.insert(moved_agent_id);
                        cluster_pair.second.erase(moved_agent_id);
                    }
                    // move agent to remaining set to unavoidable cluster
                    if (move_to_unavoid.empty()) { break; }
                }
                // till here, the remaining set is independent, it related no external agent to keep completeness
                std::set<int> specific_set_unavoidable = {};
                while (1) {
                    // when add new agent to unavoidable set, only check new added agent, to save time cost
                    auto retv = clusterIndependentCheck(cluster_pair.first, specific_set_unavoidable);
                    specific_set_unavoidable.clear();
                    std::set<int> success_in_unavoid = retv.first, failed_in_unavoid = retv.second;
                    if (failed_in_unavoid.empty()) {
                        break;
                    }
                    // pick the shortest failed path's agent
                    int failed_shortest_agent_id;
                    int shortest_path_size = MAX<int>;
                    for (const int &failed_agent : failed_in_unavoid) {
                        if (shortest_path_size > all_agents_path.at(failed_agent).size()) {
                            shortest_path_size = all_agents_path.at(failed_agent).size();
                            failed_shortest_agent_id = failed_agent;
                        }
                    }
                    // add all related agent of the shortest (containing fewer agents) path into unavoidable set
                    // there are multiple solution path, pick one with least modification to unavoid set, i.e., involve less new agent
                    //const auto& alternative_path = searchAgent(failed_shortest_agent_id, {}, AgentIdsToSATID(agents), false, AgentIdsToSATID(cluster_pair.first));
                    const auto& alternative_path = all_agents_path.at(failed_shortest_agent_id);
                    for (const int &new_agent_to_unavoid : alternative_path) {
                        if(cluster_pair.first.find(new_agent_to_unavoid) == cluster_pair.first.end()) {
                            cluster_pair.first.insert(new_agent_to_unavoid);
                            cluster_pair.second.erase(new_agent_to_unavoid);
                            specific_set_unavoidable.insert(new_agent_to_unavoid);
                        }
                    }
                }

                bool unavoid_independent = isClusterIndependent(cluster_pair.first, specific_set_unavoidable),
                        remaining_independent = isClusterIndependent(cluster_pair.second);
                count_of_phase ++;
                // if both remaining set and unavoid set is independent, we find a legal bi-partition
                if(unavoid_independent && remaining_independent) {
                    break;
                }

            }
            return cluster_pair;
        }

        void clusterDecomposition() {
            std::vector<std::set<int> > all_clusters;
            auto cluster_of_agents = all_clusters_;
            int count_top_cluster = 0;
            std::set<int> buffer_agents;
            for(const auto& top_cluster : cluster_of_agents) {
                if(top_cluster.size() < 2) {
                    // add small clusters at this stage to all_clusters, no need to join further bi-partition
                    all_clusters.push_back(top_cluster);
                } else {
                    count_top_cluster ++;
                    int count = 0;
                    buffer_agents = top_cluster;
                    // bi-partition until can not bi-partition
                    //if(count_top_cluster == 1)
                    {
                        while (buffer_agents.size() > 1) {
                            auto agents_pair = biPartitionCluster(buffer_agents);
                            std::swap(buffer_agents, agents_pair.second);
                            all_clusters.push_back(agents_pair.first);
                            count++;
                        }
                    }
                    if (!buffer_agents.empty()) {
                        all_clusters.push_back(buffer_agents);
                    }
                }
            }
            // sorting in increase size, to enable large cluster have fewer external path constraint
            std::sort(all_clusters.begin(), all_clusters.end(), [](std::set<int> x,std::set<int> y){return x.size()>y.size();});
            all_clusters_ = all_clusters;

        }

        // a cluster is split into multiple time indexed level
        // each level may have one or multiple agent
        // by update sat path of agents
        std::vector<std::set<int> >  clusterDecomposeToLevel(const std::set<int>& cluster, bool active_loop_avoidance = false) const {
            // 1, get each agent's sat path
            std::map<int, std::set<int> > all_agents_path;
            // search path which length is in an increasing order
            // provide more room to avoid large loops
            std::vector<std::pair<int, int>> sat_path_length_and_agent;

            std::vector<bool> cluster_sat = AgentIdsToSATID(cluster);
            for(const int& agent_id : cluster) {
                auto passing_sats = searchAgent(agent_id, {}, cluster_sat, true); // pass test
                assert(!passing_sats.empty());
                //std::cout << agent_id << " agent passing_sats " << passing_sats << std::endl;
                sat_path_length_and_agent.push_back({agent_id, passing_sats.size()});
            }

            // if >, path is in decrease order, if <, path length in increase order
            // TODO: it's still un clear that which way is better, massive test is need ...
            std::sort(sat_path_length_and_agent.begin(), sat_path_length_and_agent.end(),
                      [=](std::pair<int, int>& a, std::pair<int, int>& b) { return a.second > b.second; });

#define SORTED 1
#if SORTED
            for(const auto& temp_pair : sat_path_length_and_agent) {
                const int& agent_id = temp_pair.first;
#else
                for(const auto& agent_id : cluster) {
#endif
                auto passing_sats = searchAgent(agent_id, {}, cluster_sat, true); // pass test
                assert(!passing_sats.empty());

                // add agent's sat path in an incremental way
                all_agents_path.insert({agent_id, passing_sats});
            }

//            std::cout << "all_agents_path: " << std::endl;
//            for(const auto& pair : all_agents_path) {
//                std::cout << "agent " << pair.first << ": ";
//                for(const auto& id : pair.second) {
//                    std::cout << id << " ";
//                }
//                std::cout << std::endl;
//            }

            // 2, get all strong components
            auto ahead_and_later_sequence = getAheadAndLaterSequence(all_agents_path);

            const auto& ahead_sequence  = ahead_and_later_sequence.first;
            const auto& later_sequence = ahead_and_later_sequence.second;
            const auto& retv = getStrongComponentFromAheadSequence(ahead_sequence);
            std::vector<std::set<int> > all_strong_components = retv.first;
            std::map<int, int> agent_and_sub_graph = retv.second;

            // 3, get sorted level from all strong components, after all strong components are determined
            const auto& sorted_level = getSortedLevelFromStrongComponent(all_strong_components, ahead_sequence, later_sequence);

            return sorted_level;
        }

        std::vector<std::set<int> > getSortedLevelFromStrongComponent(const std::vector<std::set<int> >& all_strong_components,
                                                                      const std::map<int, std::set<int> >& ahead_sequence,
                                                                      const std::map<int, std::set<int> >& later_sequence) const {

            // the order get by by depth first traversal of all sub-graphs
            // store the sub-graph id of each node
            std::map<int, int> node_sub_graph_id;
            for(int i=0; i<all_strong_components.size(); i++) {
                for(const int& agent : all_strong_components[i]) {
                    node_sub_graph_id.insert({agent, i});
                }
            }
            // pick the root sub-graph from all sub-graph, which is not later than any external sub-graph

            std::vector<bool> sub_graph_root_check(all_strong_components.size(), true);
            for(int i=0; i<all_strong_components.size(); i++) {
                for(const int& agent : all_strong_components[i]) {
                    // if a level have a agent that not later than any other agent, it is a root level
                    for(const int& later_agent : later_sequence.at(agent)) {
                        if(all_strong_components[i].find(later_agent) == all_strong_components[i].end()) {
                            sub_graph_root_check[i] = false;
                            break;
                        }
                    }
                    // if have proof current level is a root level, no need to check further
                    if(sub_graph_root_check[i] == false) { break; }
                }
            }
            // set all root level as seeds, to traversal all level in a BFS mode
            std::set<int> root_sub_graphs;
            for(int i=0; i<sub_graph_root_check.size(); i++) {
                if(sub_graph_root_check[i]) { root_sub_graphs.insert(i); }
            }
            // storage each level's index
            std::vector<int> sub_graphs_level(all_strong_components.size(), 0);
            // start from root sub graph, get the order to traversal all sub-graph
            std::set<int> buffer_sub_graphs = root_sub_graphs;
            int max_level = 0;
            while(!buffer_sub_graphs.empty()) {
                std::set<int> next_sub_graphs;
                for(const int& current_sub_graph : buffer_sub_graphs) {
                    //std::cout << " sub_graph " << current_sub_graph << std::endl;
                    const std::set<int> current_sub_graph_agents = all_strong_components[current_sub_graph];
                    // traversal all agent current sub-graph
                    for(const int& agent : current_sub_graph_agents) {
                        // traversal all agent that later than this agent
                        for(const int& next_agent : ahead_sequence.at(agent)) {
                            // if belong to the same sub-graph, no need to update

                            if(node_sub_graph_id[agent] == node_sub_graph_id[next_agent]) { continue; }
                            //std::cout << "new sub_graph " << node_sub_graph_id[next_agent] << std::endl;

                            if(sub_graphs_level[current_sub_graph] >= sub_graphs_level[node_sub_graph_id[next_agent]]) {
                                // if current sub-graph find un-visited sub-graph
                                next_sub_graphs.insert(node_sub_graph_id[next_agent]);

                                sub_graphs_level[node_sub_graph_id[next_agent]] = sub_graphs_level[current_sub_graph] + 1;

                                max_level = std::max(max_level, sub_graphs_level[current_sub_graph] + 1);
                            }
                        }
                    }
                }
                if(!next_sub_graphs.empty()) {
                    //sorted_sub_graphs.push_back(next_sub_graphs);
                    std::swap(next_sub_graphs, buffer_sub_graphs);
                } else { break; }
            }

            std::vector<std::vector<std::set<int> > > sorted_sub_graphs(max_level + 1);
            for(int i=0; i<sub_graphs_level.size(); i++) {
                sorted_sub_graphs[ sub_graphs_level[i] ].push_back(all_strong_components[i]);
            }
            std::vector<std::set<int> > sorted_levels;
            sorted_levels.reserve(all_strong_components.size());
            for(const auto& levels : sorted_sub_graphs) {
                for(const auto& level : levels) {
                    sorted_levels.push_back(level);
                }
            }
            assert(all_strong_components.size() == sorted_levels.size());

            return sorted_levels; // sorted levels
        }



        std::pair<std::vector<std::set<int> >, std::map<int, int>>
        getStrongComponentFromAheadSequence(const std::map<int, std::set<int> >& ahead_sequence) const {

            typedef boost::subgraph< boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS,
                    boost::property< boost::vertex_color_t, int>, boost::property< boost::edge_index_t, int> > > Graph;

            typedef boost::graph_traits<Graph>::vertex_iterator vertex_iter;

            // 1, transform all nodes in ahead_sequence to continuous int sequence
            std::vector<int> id_to_node_table;
            std::map<int, int> node_to_id_table;
            for(const auto& temp_pair : ahead_sequence) {
                node_to_id_table.insert({temp_pair.first, id_to_node_table.size()});
                id_to_node_table.push_back(temp_pair.first);
            }
            // 2, get all strong component in the graph
            Graph g;
            int count_of_node = 0;
            for(const auto& temp_pair : ahead_sequence) {
                const int& node = temp_pair.first;
                count_of_node ++;
                if(temp_pair.second.empty()) {
                    //std::cerr << " empty node " << node_to_id_table[node] << std::endl;
                } else {
                    for (const int &next_node : temp_pair.second) {
                        boost::add_edge(node_to_id_table[node], node_to_id_table[next_node], g);
                    }
                }
            }
            //std::cout << " count_of_node : " << count_of_node << std::endl;
            std::vector<int> comp(num_vertices(g));

            int num = boost::strong_components(g, comp.data());

            std::vector<Graph *> comps(num);
            for (size_t i = 0; i < num; ++i) {
                comps[i] = &g.create_subgraph();
            }

            std::map<int, int> agent_and_sub_graph; // agent and it's sub-graph id
            for (size_t i = 0; i < num_vertices(g); ++i) {
                //cout << "add vertex " << i << " to sub graph " << comp[i] << endl;
                add_vertex(i, *comps[comp[i]]);
                agent_and_sub_graph.insert({id_to_node_table[i], comp[i]});
            }

            // 3, transform to multiple level, but unsorted
            std::vector<std::set<int> > retv;
            for (size_t i = 0; i < num; i++) {
                std::set<int> sub_graph;
                std::pair<vertex_iter, vertex_iter> lvip;
                lvip = vertices(*comps[i]);
                for (vertex_iter vi = lvip.first; vi != lvip.second; ++vi) {
                    sub_graph.insert(id_to_node_table[ (*comps[i]).local_to_global(*vi) ]);
                }
                retv.push_back(sub_graph);
            }
            return {retv, agent_and_sub_graph};
        }


        // ahead_sequence store agent > another agent
        // later_sequence agent < another agent
        std::pair<std::map<int, std::set<int> >, std::map<int, std::set<int> > >
            getAheadAndLaterSequence(const std::map<int, std::set<int> >& all_agents_path) const {
            // 1, construct the graph about the early and later relationship between agents
            // ahead_sequence: first: agent / second: which agent is later than this
            // later_sequence: first: agent / second: which agent is earlier than this
            std::map<int, std::set<int> > ahead_sequence, later_sequence;

            for(const auto& temp_pair : all_agents_path) {
                const int& agent_id = temp_pair.first;
                const std::set<int>& related_sat = temp_pair.second;
                for(const int& sat : related_sat) {
                    // filter edge that connect itself
                    //if(agent_id == sat/2) { continue; }
                    if(ahead_sequence.find(agent_id) == ahead_sequence.end()) { ahead_sequence.insert({agent_id, {}}); }
                    if(ahead_sequence.find(sat/2) == ahead_sequence.end()) { ahead_sequence.insert({sat/2, {}}); }
                    if(later_sequence.find(agent_id) == later_sequence.end()) { later_sequence.insert({agent_id, {}}); }
                    if(later_sequence.find(sat/2) == later_sequence.end()) { later_sequence.insert({sat/2, {}}); }

                    if(sat%2 == 0) { // sat is a start
                        ahead_sequence[sat/2].insert(agent_id);
                        later_sequence[agent_id].insert(sat/2);
                    } else { // sat is a target
                        ahead_sequence[agent_id].insert(sat/2);
                        later_sequence[sat/2].insert(agent_id);
                    }
                }
            }
            return {ahead_sequence, later_sequence};
        }

        void levelSorting() {
            // decompose each cluster into multiple time indexed sequence
            // cluster decomposition into level
            std::vector<std::set<int> > all_levels_;
            for(const auto& cluster : all_clusters_) {
                if(cluster.size() > 1) {
                    auto current_levels = clusterDecomposeToLevel(cluster);
                    all_levels_.insert(all_levels_.end(), current_levels.begin(), current_levels.end());
                } else {
                    all_levels_.push_back(cluster);
                }
            }
            all_clusters_ = all_levels_;
        }

        // when try to split one cluster into multiple sub-clusters,
        // considering all other cluster's agent as avoidance, avoid increases of queue size involve other agent
        // pick out the largest cluster to decompose (decrease agent in the cluster), if success, repeat
        // otherwise pick the second largest cluster to decompose, until there is no cluster can be decompose
        // # the size of clusters is in a power law (幂律分布), the biggest one have lots of agents, while the end have a few agents
        // # select which edge (relation of two agents) to break (candidate), is generated by graph partitioning (edge partitioning)
        // # graph partitioning try to split a graph into multiple same size sub-graph, meets the requirement of efficient MAPF
        // find multiple sub-graph that connect only by one edge and try to break it

        // get each agent's related agents
        std::map<int, std::set<int> > updateRelatedGraphFromPassingGraph(const std::map<int, std::set<int> >& ref) const {
            std::map<int, std::set<int> > red_related;
            red_related = ref;
            for(const auto& agent : ref) {
                for(const auto& other_agent : ref) {
                    if(agent.first == other_agent.first) {
                        continue;
                    }
                    // if agent j cross agent i
                    if(ref.at(other_agent.first).find(agent.first) != ref.at(other_agent.first).end()) {
                        red_related.at(agent.first).insert(other_agent.first);
                    }
                }
            }
            return red_related;
        }

        std::map<int, std::set<int> > clusterAgents(const std::map<int, std::set<int> >& ref_related) const {
            std::map<int, std::set<int> > cluster_of_agents;// multiple cluster
            std::map<int, int> cluster_id_of_agents; // cluster id of each agent
            for(const auto& pair: ref_related) {
                cluster_id_of_agents.insert({pair.first, MAX<int>});
            }
            for(const auto& pair : ref_related) {
                if(cluster_id_of_agents[pair.first] != MAX<int>) {
                    continue;
                }
                // while loop exit when current cluster have no new member
                std::set<int> current_set = {pair.first};
                std::set<int> buffer_set  = {pair.first}, next_buffer_set;
                cluster_id_of_agents.at(pair.first)   = cluster_of_agents.size();
                while(!buffer_set.empty()) {
                    next_buffer_set.clear();
                    for(const int& new_agent_id : buffer_set) {
                        std::set<int> relative_agents = ref_related.at(new_agent_id);
                        for(const int& depended_agent_id : relative_agents) {
                            if(cluster_id_of_agents.at(depended_agent_id) != MAX<int>) {
                                if(cluster_id_of_agents.at(depended_agent_id) != cluster_of_agents.size()) {
                                    // what belong is an error condition
                                    std::cerr << " different cluster intersect " <<
                                              cluster_id_of_agents.at(depended_agent_id) << " and " << cluster_of_agents.size()
                                              << std::endl;
                                }
                            } else{
                                cluster_id_of_agents.at(depended_agent_id) = cluster_of_agents.size();
                                current_set.insert(depended_agent_id);
                                next_buffer_set.insert(depended_agent_id);
                            }
                        }
                    }
                    std::swap(buffer_set, next_buffer_set);
                }
                cluster_of_agents.insert({cluster_of_agents.size(), current_set});
            }
            return cluster_of_agents;
        }

        bool solve(double time_limit, int cost_lowerbound = 0, int cost_upperbound = MAX_COST) {
            return false;
        }

        void printAllSubProblem(const std::string& str) const {
            std::cout << str.c_str() << " decomposition result: " << std::endl;
            for(int i=0; i<all_clusters_.size(); i++) {
                std::cout << i << ": ";
                for(const auto& id : all_clusters_[i]) {
                    std::cout << id << " ";
                }
                std::cout << std::endl;
            }
        }

        std::vector<ConnectivityGraph> connect_graphs_;

        std::vector<std::vector<int> > heuristic_tables_sat_; // distinguish_sat = true

        std::vector<std::vector<int> > heuristic_tables_; // distinguish_sat = false

        bool directed_graph_ = true;

        std::set<int> instance_id_set_; // set of all agent's id

        std::vector<std::set<int> > all_clusters_;

        // store which agents current agent passing, may change after method
        // NOTICE: the goal of the method is to partition this matrix into lots of small block, thus MAPF is more efficient
        std::map<int, std::set<int> > all_passing_agent_;

        float initialize_time_cost_             = 0;
        float instance_decomposition_time_cost_ = 0;
        float cluster_bipartition_time_cost_    = 0;
        float level_sorting_time_cost_          = 0;

        // variables for debug
        LAMAPF_Paths grid_paths_;
        std::vector<std::vector<int> > agent_visited_grids_;

    };

    template<Dimension N, typename AgentType>
    using LargeAgentMAPFInstanceDecompositionPtr = std::shared_ptr<LargeAgentMAPFInstanceDecomposition<N, AgentType> >;

}
#endif //LAYEREDMAPF_LARGE_AGENT_INSTANCE_DECOMPOSITION_H
