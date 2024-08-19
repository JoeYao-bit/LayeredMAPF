//
// Created by yaozhuo on 2024/8/14.
//

#ifndef LAYEREDMAPF_LARYERED_LARGE_AGENT_MAPF_H
#define LAYEREDMAPF_LARYERED_LARGE_AGENT_MAPF_H

#include "common.h"

#include "large_agent_instance_decomposition.h"
#include "../../algorithm/LA-MAPF/CBS/constraint.h"

namespace freeNav::LayeredMAPF::LA_MAPF {

    // delay a set of paths one step simultaneously at time step t
    template <Dimension N, typename AgentType>
    void delayPathLargeAgent(LAMAPF_Paths& paths, int t, int count = 1) {
//        std::cout << "delay t = " << t << ", delay count = " << count << std::endl;
        // traversal all path, wait at previous frame
        for(auto& path : paths) {
            // do not delay which have arrived target
            if(t > path.size()-1) { continue; }
            for(int i=0; i<count; i++) {
                path.insert(path.begin() + t, path[t]);
            }
        }
    }

    // return the path have been modified to avoid conflict with lasted_occupied_time_table
    // synchronous is easy but too long, current way
    // asynchronous is not easy to implement, but too
    template <Dimension N, typename AgentType>
    LAMAPF_Paths SingleLayerCompressLargeAgent(const std::vector<PosePtr<int, N> >& all_poses,
                                               const LAMAPF_Paths& pre_paths,
                                               const std::vector<AgentType>& pre_agents,
                                               const LAMAPF_Paths& paths,
                                               const std::vector<AgentType>& agents) {
        int t=1;
        std::vector<bool> reach_states(paths.size(), false);
        LAMAPF_Paths modified_paths(paths);
        while(1) {
//            std::cout << " t = " << t << std::endl;
            // check whether current proceed
            bool proceed = true, all_finished = true;
            int delay_start_time = MAX<int>, delay_count = 0;
            for(int i=0; i<pre_paths.size(); i++) {
                const auto& pre_path  = pre_paths[i];
                const auto& pre_agent = pre_agents[i];

                if(t >= pre_path.size() - 1) {
                    // if pre path have reach target
                    continue;
                } else {
                    all_finished = false;
                }

                for(int j=0; j<modified_paths.size(); j++) {
                    const auto& cur_path  = modified_paths[j];
                    const auto& cur_agent = agents[j];

//                    detectFirstConflictBetweenPaths(cur_path, cur_agent, pre_path, pre_agent, );

                    // check whether finished
                    if(t <= cur_path.size() - 1) { all_finished = false; }
                    // if not finish, check whether proceed
                    // if collide cannot proceed

                    if(t < cur_path.size() - 1) {
                        if (isCollide(pre_agent, *all_poses[pre_path[t]], *all_poses[pre_path[t+1]],
                                      cur_agent, *all_poses[cur_path[t]], *all_poses[cur_path[t+1]])) {

//                            std::cout << "collide 1 t = " << t << ", " << t+1 << std::endl;
//                            std::cout << "pre / cur agent " << pre_agent << " / " << cur_agent << std::endl;
//                            std::cout << *all_poses[pre_path[t]] << ", " << *all_poses[pre_path[t+1]] << " / "
//                                      << *all_poses[cur_path[t]] << ", " << *all_poses[cur_path[t+1]] << std::endl;
//                            std::cout << cur_agent << " cur pre = " << *all_poses[cur_path[t-1]]
//                                      << ", cur pre pre " << *all_poses[cur_path[t-2]] << std::endl;

                            proceed = false;

                            // move forward to find first collide free start
                            int local_delay_start = t, local_delay_count = 0;
                            while(1) {
                                if(local_delay_start > 0) {
                                    if(!isCollide(pre_agent, *all_poses[pre_path[t]], *all_poses[pre_path[t+1]],
                                                  cur_agent, *all_poses[cur_path[local_delay_start]], *all_poses[cur_path[local_delay_start]])) {
//                                        std::cout << " cur at " << local_delay_start << ", " << local_delay_start << " is free" << std::endl;
                                        local_delay_count = t + 1 - local_delay_start;
                                        break;
                                    }
                                } else if (local_delay_start == 0) {
                                    if(!isCollide(pre_agent, *all_poses[pre_path[t]], *all_poses[pre_path[t+1]],
                                                  cur_agent, *all_poses[cur_path[0]], *all_poses[cur_path[0]])) {
//                                        std::cout << " cur at " << 0 << ", " << 0 << " is free" << std::endl;
                                        local_delay_count = t + 1;
                                        break;
                                    } else {
                                        std::cout << "FATAL: delay at start cant avoid conflict" << std::endl;
                                    }
                                } else  {
                                    std::cout << "FATAL: delay start shouldn't < 0" << std::endl;
                                }
                                local_delay_start--;
                            }
//                            std::cout << "local_delay_start / local_delay_count = "
//                                      << local_delay_start << " / " << local_delay_count << std::endl;
                            if(delay_start_time > local_delay_start) {
                                delay_start_time = local_delay_start;
                                delay_count = local_delay_count;
                            }
//                            break;
                        }
                    } else {
                        if (isCollide(pre_agent, *all_poses[pre_path[t]], *all_poses[pre_path[t+1]],
                                      cur_agent, *all_poses[cur_path.back()])) {

//                            std::cout << "collide 2 t = " << t << ", " << t+1 << std::endl;
//                            std::cout << *all_poses[pre_path[t]] << ", " << *all_poses[pre_path[t+1]] << " / "
//                                      << *all_poses[cur_path.back()] << std::endl;

                            proceed = false;

                            int local_delay_start = cur_path.size()-2, local_delay_count = 0;
                            while(1) {
                                if(local_delay_start > 0) {
                                    if(!isCollide(pre_agent, *all_poses[pre_path[t]], *all_poses[pre_path[t+1]],
                                                  cur_agent, *all_poses[cur_path[local_delay_start]], *all_poses[cur_path[local_delay_start]])) {
                                        local_delay_count = t + 1 - local_delay_start;
//                                        std::cout << " cur at " << local_delay_start << ", " << local_delay_start << " is free" << std::endl;
                                        break;
                                    }
                                } else if (local_delay_start == 0) {
                                    if(!isCollide(pre_agent, *all_poses[pre_path[t]], *all_poses[pre_path[t+1]],
                                                  cur_agent, *all_poses[cur_path[0]], *all_poses[cur_path[0]])) {
//                                        std::cout << " cur at " << 0 << ", " << 0 << " is free" << std::endl;
                                        local_delay_count = t + 1;
                                        break;
                                    } else {
                                        std::cout << "FATAL: delay at start cant avoid conflict" << std::endl;
                                    }
                                } else  {
                                    std::cout << "FATAL: delay start shouldn't < 0" << std::endl;
                                }
                                local_delay_start--;
                            }

//                            std::cout << "local_delay_start / local_delay_count = "
//                                      << local_delay_start << " / " << local_delay_count << std::endl;

                            if(delay_start_time > local_delay_start) {
                                delay_start_time = local_delay_start;
                                delay_count = local_delay_count;
                            }

//                            break;
                        }
                    }
                }
//                if(!proceed) {
//                    break;
//                }
            }
            if(proceed && all_finished) { break; }
            if(!proceed) {
//                std::cout << " delay_start_time / delay_count = " << delay_start_time << " / " << delay_count << std::endl;
                // traversal all path, wait at previous frame
//                for(auto& path : modified_paths) {
//                    if(t-1 >= path.size()-1) { continue; }
//                    path.insert(path.begin() + t-1, path[t-1]);
//                }
                delayPathLargeAgent<N, AgentType>(modified_paths, delay_start_time, delay_count);
                t = delay_start_time;
            } else {
                t++;
            }
        }
        auto pre_paths_copy = pre_paths;
        pre_paths_copy.insert(pre_paths_copy.end(), modified_paths.begin(), modified_paths.end());
        return pre_paths_copy;
    }

    template <Dimension N, typename AgentType>
    LAMAPF_Paths multiLayerCompressLargeAgent(const std::vector<PosePtr<int, N> >& all_poses,
                                          const std::vector<LAMAPF_Paths>& pathss,
                                          const std::vector<std::vector<AgentType> >& agents) {
        if(pathss.size() == 1) { return pathss[0]; }
        else if(pathss.empty()) { return {}; }

        // delay other path to avoid conflict with previous
        LAMAPF_Paths mergedPaths = pathss[0];
        std::vector<AgentType> pre_agents = agents[0];

        for(int i=1; i<pathss.size(); i++) {
            std::cout << "merge " << i << " th cluster: ";
            for(const auto& agent : agents[i]) {
                std::cout << agent.id_ << " ";
            }
            std::cout << std::endl;
            mergedPaths = SingleLayerCompressLargeAgent<N>(all_poses, mergedPaths, pre_agents, pathss[i], agents[i]);
            pre_agents.insert(pre_agents.end(), agents[i].begin(), agents[i].end());


            if(!isSolutionValid(mergedPaths, pre_agents, all_poses)) { break; }
        }
        mergedPaths.resize(agents.size());
        return mergedPaths;
    }

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
        std::vector<std::vector<AgentType> > all_cluster_agents;

        std::vector<LAMAPF_Paths> all_paths;
        std::vector<std::vector<int> > all_cluster_vec;
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

            all_cluster_agents.push_back(cluster_agents);
            all_cluster_vec.push_back(current_id_vec);
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
            all_paths.push_back(next_paths);
            std::vector<std::pair<int, LAMAPF_Path> > next_paths_with_id;
            for(int k=0; k<current_id_vec.size(); k++) {
                next_paths_with_id.push_back({current_id_vec[k], next_paths[k]});
                retv[current_id_vec[k]] = next_paths[k];
                grid_visit_count_table[current_id_vec[k]] = grid_visit_count_table_local[k];
            }
            pathss.push_back(next_paths_with_id);
        }
        LAMAPF_Paths merged_paths;
        if(!use_path_constraint) {
            gettimeofday(&tv_pre, &tz);

            merged_paths = multiLayerCompressLargeAgent(decomposer->all_poses_, all_paths, all_cluster_agents);
            assert(instances.size() == merged_paths.size());

            int global_count = 0;
            for(int i=0; i<all_cluster_vec.size(); i++) {
                for(int j=0; j<all_cluster_vec[i].size(); j++) {
                    retv[all_cluster_vec[i][j]] = merged_paths[global_count];
                    assert(all_cluster_agents[i][j].id_ == all_cluster_vec[i][j]);
                    global_count ++;
                }
            }
            assert(global_count == agents.size());
            gettimeofday(&tv_after, &tz);
            double merge_path_time_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
            std::cout << "-- take " << merge_path_time_cost << "ms to merge paths" << std::endl << std::endl;
        }
        std::cout << "final check" << std::endl;
        isSolutionValid(retv, agents, decomposer->all_poses_);

        std::cout << "-- layered large agent mapf success " << !retv.empty() << std::endl;
        std::cout << "-- SOC = " << getSOC(retv) << ", makespan = " << getMakeSpan(retv) << std::endl;

        return retv;
    }

}

#endif //LAYEREDMAPF_LARYERED_LARGE_AGENT_MAPF_H
