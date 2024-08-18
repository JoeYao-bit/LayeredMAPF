//
// Created by yaozhuo on 2024/8/14.
//

#ifndef LAYEREDMAPF_LARYERED_LARGE_AGENT_MAPF_H
#define LAYEREDMAPF_LARYERED_LARGE_AGENT_MAPF_H

#include "common.h"

#include "large_agent_instance_decomposition.h"
#include "../../algorithm/LA-MAPF/CBS/constraint.h"

namespace freeNav::LayeredMAPF::LA_MAPF {
    // current only considering methods that take external path as constraint, like LA-CBS
    template<Dimension N, typename AgentType>
    std::vector<LAMAPF_Path> layeredLargeAgentMAPF(const InstanceOrients<N> & instances,
                                                   const std::vector<AgentType>& agents,
                                                   DimensionLength* dim,
                                                   const IS_OCCUPIED_FUNC<N> & isoc,
                                                   const LA_MAPF_FUNC<N, AgentType> & mapf_func,
                                                   std::vector<std::vector<int> >& grid_visit_count_table,
                                                   int cutoff_time = 60,
                                                   LargeAgentMAPFInstanceDecompositionPtr<2, AgentType>& decomposer_copy = nullptr,
                                                   bool use_path_constraint = false
                                                   ) {

        struct timezone tz;
        struct timeval  tv_pre;
        struct timeval  tv_after;
        gettimeofday(&tv_pre, &tz);

        LargeAgentMAPFInstanceDecompositionPtr<2, AgentType> decomposer =
                std::make_shared<LargeAgentMAPFInstanceDecomposition<2, AgentType> >(instances,
                                                                     agents, dim, isoc);

        decomposer_copy = decomposer;

        gettimeofday(&tv_after, &tz);
        double decomposition_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
        std::cout << "-- decomposition take " << decomposition_cost << " ms to get "
                  << decomposer->all_clusters_.size() << " clusters " << std::endl;
        std::cout << std::endl;

        assert(decomposer->all_clusters_.size() >= 1);

        LargeAgentPathConstraintTablePtr<N, AgentType> layered_cts =
                std::make_shared<LargeAgentPathConstraintTable<N, AgentType> >(decomposer->all_poses_, agents, dim);

        std::vector<std::vector<std::pair<int, LAMAPF_Path> > > pathss;
        std::vector<LAMAPF_Path> retv(instances.size());
        grid_visit_count_table.resize(instances.size());

        for(int i=0; i<decomposer->all_clusters_.size(); i++) {
            gettimeofday(&tv_pre, &tz);
            // instance_decompose->all_clusters_[i] to instances
            std::set<int> current_id_set = decomposer->all_clusters_[i];

            if (use_path_constraint) {
                // insert previous path as static constraint
                if (!pathss.empty()) {
                    layered_cts->insertPaths(pathss.back());
                }
            } else {
                // insert previous agents' target as static constraint
                for (int j = 0; j < i; j++) {
                    if (j == i) { continue; }
                    const auto &current_cluster = decomposer->all_clusters_[j];
                    for (const int &agent_id : current_cluster) {
                        layered_cts->insertPath({agent_id, {decomposer->instance_node_ids_[agent_id].second}});
                    }
                }
            }

            // insert future agents' start as static constraint
            for(int j = i+1; j<decomposer->all_clusters_.size(); j++)
            {
                if(j == i) { continue; }
                const auto& current_cluster = decomposer->all_clusters_[j];
                for(const int& agent_id : current_cluster) {
                    layered_cts->insertPath({agent_id, {decomposer->instance_node_ids_[agent_id].first}});
                }
            }


            double remaining_time = cutoff_time - (tv_after.tv_sec - tv_pre.tv_sec) + (tv_after.tv_usec - tv_pre.tv_usec)/1e6;
            if(remaining_time < 0) {
                return {};//retv;
            }

            InstanceOrients<N> cluster_instances;
            std::vector<AgentType> cluster_agents;

            std::vector<int> current_id_vec;
            std::vector<size_t> target_node_ids;

            /*    pre calculated variables
             *    const std::vector<PosePtr<int, N> >& all_poses = {},
                  const DistanceMapUpdaterPtr<N>& distance_map_updater = nullptr,
                  const std::vector<SubGraphOfAgent<N> >& agent_sub_graphs = {},
                  const std::vector<std::vector<int> >& agents_heuristic_tables = {},
                  const std::vector<std::vector<int> >& agents_heuristic_tables_ignore_rotate = {}
             * */

            const std::vector<PosePtr<int, N> >& local_all_poses = decomposer->all_poses_;
            const DistanceMapUpdaterPtr<N>&      local_distance_map_updater = decomposer->distance_map_updater_;
            std::vector<SubGraphOfAgent<N> >     local_agent_sub_graphs;
            std::vector<std::vector<int> >       local_agents_heuristic_tables;
            std::vector<std::vector<int> >       local_agents_heuristic_tables_ignore_rotate;

            for(const auto& current_id : current_id_set) {
                current_id_vec.push_back(current_id);
                cluster_instances.push_back(instances[current_id]);
                cluster_agents.push_back(agents[current_id]);
                target_node_ids.push_back(decomposer->instance_node_ids_[current_id].second);

                local_agent_sub_graphs.push_back(decomposer->agent_sub_graphs_[current_id]);
                local_agent_sub_graphs.back().agent_id_ = target_node_ids.size() - 1;

                local_agents_heuristic_tables.push_back(
                        decomposer->agents_heuristic_tables_[current_id]);

                local_agents_heuristic_tables_ignore_rotate.push_back(
                        decomposer->agents_heuristic_tables_ignore_rotate_[current_id]);

            }

            layered_cts->updateEarliestArriveTimeForAgents(cluster_agents, target_node_ids);
            gettimeofday(&tv_after, &tz);
            double update_path_constraint_time_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
            std::cout << "-- " << i << " th cluster update path constraint take " << update_path_constraint_time_cost << "ms" << std::endl;


            gettimeofday(&tv_pre, &tz);
            std::vector<std::vector<int> > grid_visit_count_table_local;
            std::vector<LAMAPF_Path> next_paths = mapf_func(cluster_instances, cluster_agents, dim, isoc, layered_cts,
                                                            grid_visit_count_table_local, remaining_time,
                                                            local_all_poses,
                                                            local_distance_map_updater,
                                                            local_agent_sub_graphs,
                                                            local_agents_heuristic_tables,
                                                            local_agents_heuristic_tables_ignore_rotate
                                                            );
            gettimeofday(&tv_after, &tz);

            double mapf_func_time_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
            std::cout << "-- " << i << " th cluster mapf func take " << mapf_func_time_cost << "ms" << std::endl << std::endl;

            if(next_paths.empty()) {
                std::cout << " layered MAPF failed " << i << " th cluster: ";
                for(const auto& id : current_id_set) {
                    std::cout << id << " ";
                }
                std::cout << std::endl;
                return {};//retv;
            }
            // debug: path constraint check
//            if(layered_cts != nullptr) {
//                for (int k = 0; k < current_id_vec.size(); k++) {
//                    for (int t = 0; t < next_paths[k].size() - 1; t++) {
//                        if (layered_cts->hasCollide(cluster_agents[k], t, next_paths[k][t], next_paths[k][t + 1])) {
//                            std::cout << "FATAL: " << current_id_vec[k] << " collide with previous path when t = " << t << std::endl;
//                            return {};//retv;
//                        }
//                    }
//                }
//            }
            assert(next_paths.size() == current_id_set.size());
            std::vector<std::pair<int, LAMAPF_Path> > next_paths_with_id;
            for(int k=0; k<current_id_vec.size(); k++) {
                next_paths_with_id.push_back({current_id_vec[k], next_paths[k]});
                retv[current_id_vec[k]] = next_paths[k];
                grid_visit_count_table[current_id_vec[k]] = grid_visit_count_table_local[k];
            }
            pathss.push_back(next_paths_with_id);
        }
        // debug
        assert(instances.size() == retv.size());
//        for(int i=0; i<instances.size(); i++) {
//            for(int j=i+1; j<instances.size(); j++) {
//                auto conflicts = detectAllConflictBetweenPaths(retv[i], retv[j], agents[i], agents[j], decomposer->all_poses_);
//                if(!conflicts.empty()) {
//                    std::cout << "FATAL: detect conflicts between agent "
//                    << agents[i] << "path_size(" << retv[i].size() << ")" << " and "
//                    << agents[j] << "path_size(" << retv[j].size() << ")"
//                    << ", at t = " << conflicts.front()->t1 << std::endl;
//                }
//            }
//        }
//        if(!use_path_constraint) { retv = multiLayerCompress(dim, pathss); }
        std::cout << "-- layered large agent mapf success " << !retv.empty() << std::endl;
        std::cout << "-- SOC = " << getSOC(retv) << ", makespan = " << getMakeSpan(retv) << std::endl;
        return retv;
    }

}

#endif //LAYEREDMAPF_LARYERED_LARGE_AGENT_MAPF_H
