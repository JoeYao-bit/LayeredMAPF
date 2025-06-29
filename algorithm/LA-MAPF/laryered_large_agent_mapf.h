//
// Created by yaozhuo on 2024/8/14.
//

#ifndef LAYEREDMAPF_LARYERED_LARGE_AGENT_MAPF_H
#define LAYEREDMAPF_LARYERED_LARGE_AGENT_MAPF_H

#include "common.h"

#include "../../algorithm/LA-MAPF/CBS/constraint.h"
#include "solvability_safe_guard.h"
#include "../basic.h"

namespace freeNav::LayeredMAPF::LA_MAPF {

    // delay a set of paths one step simultaneously at time step t
    template <Dimension N>
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
    template <Dimension N, typename State>
    LAMAPF_Paths SingleLayerCompressLargeAgent(const std::vector<std::shared_ptr<State> >& all_poses,
                                               const LAMAPF_Paths& pre_paths,
                                               const std::vector<AgentPtr<N> >& pre_agents,
                                               const LAMAPF_Paths& paths,
                                               const std::vector<AgentPtr<N> >& agents) {
        int t=0;
//        std::vector<bool> reach_states(paths.size(), false);
        LAMAPF_Paths modified_paths(paths);
        int total_delay_count = 0;
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
//                                std::cout << " local_delay_start = " << local_delay_start << std::endl;
                                if(local_delay_start > 0) {
                                    if(!isCollide(pre_agent, *all_poses[pre_path[t]], *all_poses[pre_path[t+1]],
                                                  cur_agent, *all_poses[cur_path[local_delay_start]], *all_poses[cur_path[local_delay_start]])) {
//                                        std::cout << " cur at " << local_delay_start << ", " << local_delay_start << " is free" << std::endl;
                                        local_delay_count = t + 1 - local_delay_start;
                                        break;
                                    }
                                } else if (local_delay_start == 0) {
                                    if(!isCollide(pre_agent, *all_poses[pre_path[t]], *all_poses[pre_path[t+1]],
                                                  cur_agent, *all_poses[cur_path[0]])) {
//                                        std::cout << " cur at " << 0 << ", " << 0 << " is free" << std::endl;
                                        local_delay_count = t + 1;
                                        break;
                                    } else {
                                        std::cout << "FATAL: delay at start cant avoid conflict 1 between agent "
                                                  << pre_agent->id_ << " and " << cur_agent->id_ << " at t = " << t << std::endl;
                                        std::cout << "pre agent = " << pre_agent << " : "
                                                  << *all_poses[pre_path[t]] << "->" << *all_poses[pre_path[t+1]] << std::endl;
                                        std::cout << "cur agent = " << cur_agent << " : "
                                                  << *all_poses[cur_path[0]] << std::endl;

                                        std::cout << "cur path = " << printPath<N, State>(cur_path, all_poses) << std::endl;

                                        exit(0);
                                    }
                                } else  {
                                    std::cout << "FATAL: delay start shouldn't < 0 1 "  << std::endl;
                                    exit(0);
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
                                        std::cout << "FATAL: delay at start cant avoid conflict 2" << std::endl;
                                        exit(0);
                                    }
                                } else {
                                    std::cout << "FATAL: delay start shouldn't < 0 2" << std::endl;
                                    exit(0);
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
            }
            if(proceed && all_finished) { break; }
            if(!proceed) {
//                std::cout << " delay_start_time / delay_count = " << delay_start_time << " / " << delay_count << std::endl;
                // traversal all path, wait at previous frame
//                for(auto& path : modified_paths) {
//                    if(t-1 >= path.size()-1) { continue; }
//                    path.insert(path.begin() + t-1, path[t-1]);
//                }
//                delayPathLargeAgent<N, AgentType>(modified_paths, delay_start_time, delay_count);
                delayPathLargeAgent<N>(modified_paths, 0, delay_count);

                //t = delay_start_time;
                total_delay_count += delay_count;
                t = total_delay_count;
            } else {
                t++;
            }
        }
        auto pre_paths_copy = pre_paths;
        pre_paths_copy.insert(pre_paths_copy.end(), modified_paths.begin(), modified_paths.end());
        return pre_paths_copy;
    }

    template <Dimension N, typename State>
    LAMAPF_Paths multiLayerCompressLargeAgent(const std::vector<std::shared_ptr<State> >& all_poses,
                                              const std::vector<LAMAPF_Paths>& pathss,
                                              const std::vector<std::vector<AgentPtr<N>> >& agents) {
        assert(pathss.size() == agents.size());
        if(pathss.size() == 1) { return pathss[0]; }
        else if(pathss.empty()) { return {}; }

        int total_size = 0;
        for(int i=0; i<agents.size(); i++) {
            assert(pathss[i].size() == agents[i].size());
            total_size += agents[i].size();
        }

        // delay other path to avoid conflict with previous
        LAMAPF_Paths mergedPaths = pathss[0];
        std::vector<AgentPtr<N> > pre_agents = agents[0];

        for(int i=1; i<pathss.size(); i++) {
            // debug
//            std::cout << "merge " << i << " th paths" << std::endl;

            mergedPaths = SingleLayerCompressLargeAgent<N>(all_poses, mergedPaths, pre_agents, pathss[i], agents[i]);
            pre_agents.insert(pre_agents.end(), agents[i].begin(), agents[i].end());

            // debug, check whether merged path have conflicts
            if(!isSolutionValid(mergedPaths, pre_agents, all_poses)) { break; }
        }
        mergedPaths.resize(total_size);
        return mergedPaths;
    }

    template<Dimension N, typename State>
    std::vector<LAMAPF_Path> solveSubproblem(int subproblem_id, // level_global_id
                                             const std::vector<std::set<int> > levels,
                                             const std::vector<AgentPtr<N> >& pre_agents,
                                             const std::vector<size_t>& pre_targets,
                                             const std::map<int, LAMAPF_Path>& pre_pathss,

                                             std::vector<std::vector<int> >& grid_visit_count_table,
                                             const PrecomputationOfMAPFBasePtr<N, State>& pre,
                                             double cutoff_time,
                                             const float& max_excircle_radius,
                                             const LA_MAPF_FUNC<N, State> & mapf_func,
                                             bool use_path_constraint = false) {

        std::vector<std::pair<Pointi<N>, Pointi<N>>> cluster_instances;
        std::vector<AgentPtr<N> > cluster_agents;

        std::set<int> current_id_set = levels[subproblem_id];

        std::vector<int> current_id_vec;
        std::vector<size_t> target_node_ids;

        const std::vector<std::shared_ptr<State> >& local_all_poses     = pre->all_poses_;
        const DistanceMapUpdaterPtr<N>&      local_distance_map_updater = pre->distance_map_updater_;
        std::vector<SubGraphOfAgent<N, State> >     local_agent_sub_graphs;
        std::vector<std::vector<int> >       local_agents_heuristic_tables;
        std::vector<std::vector<int> >       local_agents_heuristic_tables_ignore_rotate;
        std::vector<std::pair<size_t, size_t> > local_instance_node_ids;

        for(const auto& current_id : current_id_set) {
            current_id_vec.push_back(current_id);
            cluster_instances.push_back(pre->instances_[current_id]);
            cluster_agents.push_back(pre->agents_[current_id]);


            target_node_ids.push_back(pre->instance_node_ids_[current_id].second);

            local_agent_sub_graphs.push_back(pre->agent_sub_graphs_[current_id]);

            local_agents_heuristic_tables.push_back(
                    pre->agents_heuristic_tables_[current_id]);

            local_agents_heuristic_tables_ignore_rotate.push_back(
                    pre->agents_heuristic_tables_ignore_rotate_[current_id]);

            local_instance_node_ids.push_back(pre->instance_node_ids_[current_id]);
        }
        MSTimer mst;
        LargeAgentStaticConstraintTablePtr<N, State>
                new_constraint_table_ptr_ = std::make_shared<LargeAgentStaticConstraintTable<N, State> > (
                max_excircle_radius, pre->dim_, pre->isoc_, pre->agents_, cluster_agents, pre->all_poses_);

        new_constraint_table_ptr_->insertPoses(pre_agents, pre_targets);

        if(use_path_constraint) {
            for(auto iter=pre_pathss.begin(); iter!=pre_pathss.end(); iter++) {
                new_constraint_table_ptr_->insertPath(iter->first, iter->second);
            }
        }
        // insert future agents' start as static constraint
        for(int j = subproblem_id+1; j<levels.size(); j++)
        {
            const auto& current_cluster = levels[j];
            for(const int& agent_id : current_cluster) {
                new_constraint_table_ptr_->insertPose(agent_id, pre->instance_node_ids_[agent_id].first);
            }
        }
        // insert previous agents' target as static constraint
        new_constraint_table_ptr_->updateEarliestArriveTimeForAgents(cluster_agents, target_node_ids);

        std::vector<AgentPtr<N> > local_cluster_agents; // copy of agent, with local id
        for(int k=0; k<current_id_vec.size(); k++) {

            const auto& agent_id = current_id_vec[k];
            AgentPtr<N> local_copy = pre->agents_[agent_id]->copy();
            local_copy->id_ = k;
            local_cluster_agents.push_back(local_copy);

        }

        std::cout << "-- agent in " << subproblem_id << " th cluster (layered MAPF): ";
        for(const auto& id : current_id_set) {
            assert(pre->agents_[id]->id_ == id);
            std::cout << id << " ";
        }
        std::cout << std::endl;

        double update_path_constraint_time_cost = mst.elapsed()/1e3;
        std::cout << "-- " << subproblem_id << " th cluster update path constraint take " << update_path_constraint_time_cost << "ms" << std::endl;


        double remaining_time = (double)cutoff_time - mst.elapsed()/1e3;
        std::cout << "remaining_time = " << remaining_time << "s" << std::endl;
        if(remaining_time <= 0) {
            return {};//retv;
        }
        mst.reset();
        std::vector<std::vector<int> > grid_visit_count_table_local;
        std::vector<LAMAPF_Path> next_paths = mapf_func(cluster_instances,
                                                        local_cluster_agents,
                                                        pre->dim_, pre->isoc_,
                                                        new_constraint_table_ptr_,
                                                        grid_visit_count_table_local, remaining_time,
                                                        local_instance_node_ids,
                                                        local_all_poses,
                                                        local_distance_map_updater,
                                                        local_agent_sub_graphs,
                                                        local_agents_heuristic_tables,
                                                        local_agents_heuristic_tables_ignore_rotate,
                                                        nullptr
        );

        double mapf_func_time_cost = mst.elapsed()/1e3;
        std::cout << "-- " << subproblem_id << " th cluster mapf func take " << mapf_func_time_cost << "s" << std::endl << std::endl;

        if(grid_visit_count_table_local.size() == current_id_vec.size()) {
            for (int k = 0; k < current_id_vec.size(); k++) {
                grid_visit_count_table[current_id_vec[k]] = grid_visit_count_table_local[k];
            }
        }
        return next_paths;
    }

    // current only considering methods that take external path as constraint, like LA-CBS
    template<Dimension N, typename State>
    std::vector<LAMAPF_Path> layeredLargeAgentMAPF(const std::vector<std::set<int> >& raw_levels,
                                                   const LA_MAPF_FUNC<N, State> & mapf_func,
                                                   std::vector<std::vector<int> >& grid_visit_count_table,
                                                   bool& detect_loss_of_solvability,
                                                   const PrecomputationOfMAPFBasePtr<N, State>& pre,
                                                   double cutoff_time = 60,
                                                   bool use_path_constraint = false
                                                   ) {

        MSTimer mst;

        assert(raw_levels.size() >= 1);

        float max_excircle_radius = getMaximumRadius<N>(pre->agents_);

        std::map<int, LAMAPF_Path> pre_pathss;
        std::vector<LAMAPF_Path> retv(pre->instances_.size());
        grid_visit_count_table.resize(pre->instances_.size());
        std::vector<std::vector<AgentPtr<N> > > all_cluster_agents;

        std::vector<LAMAPF_Paths> all_paths;
        std::vector<std::vector<int> > all_cluster_vec;

        std::vector<AgentPtr<N> > pre_agents;
        std::vector<size_t> pre_targets;

        std::vector<std::set<int> > levels = raw_levels;

        int level_id = 0; // id in current level (may merge if detect of solvability)

        while(level_id < levels.size()) {
            std::set<int> current_id_set = levels[level_id];

            double remaining_time = (double)cutoff_time - mst.elapsed()/1e3;

            LAMAPF_Paths next_paths = solveSubproblem<N, State>(level_id, levels, pre_agents, pre_targets, pre_pathss,
                                                      grid_visit_count_table, pre,
                                                      remaining_time,
                                                      max_excircle_radius,
                                                      mapf_func,
                                                      use_path_constraint);

            if(next_paths.empty()) {
                std::cout << " layered MAPF failed " << level_id << " th cluster: ";
                for(const auto& id : current_id_set) {
                    std::cout << id << " ";
                }
                std::cout << std::endl;
                //return {};//retv;
                // add solvability guard to merge unsolvable subproblem until it is solvable or run out of time
                remaining_time = (double)cutoff_time - mst.elapsed()/1e3;
                if(remaining_time < 2) {
                    std::cout << " failed run out of time " << std::endl;
                    return {};
                } else {
                    std::cout << "unsolvable subproblem detected, solvability safeguard start...  " << std::endl;
                    detect_loss_of_solvability = true;
                }
                SolvabilitySafeguard<N, State> safeguard(pre);

                bool success = safeguard.mergeSubproblemTillSolvable(levels,
                                                                     level_id,
                                                                     mapf_func,
                                                                     pre->isoc_, remaining_time);
                if(!success) {
                    std::cout << "solvability safe guard failed " << std::endl;
                    remaining_time = (double)cutoff_time - mst.elapsed()/1e3;
                    if(remaining_time > 1) {
                        std::cout << "but still have time, shouldn't reach here " << std::endl;
                    }
                    return {};
                } else {
                    std::cout << "solvability safe success " << std::endl;
                    // update progress of subproblems, as safeguard merge subproblem and solve them
//                    safeguard.new_level_paths_;
//                    safeguard.merged_subproblem_id_;
                    assert(safeguard.new_levels_.size() <= levels.size());
                    int backup_i = level_id;
                    std::cout << "level before merge: ";
                    for(const auto& level : levels) {
                        std::cout << level << ", ";
                    }
                    std::cout << std::endl;

                    //std::cout << "flag 0" << std::endl;

                    // update paths of solved subproblems
                    const auto& new_level = safeguard.new_levels_[safeguard.merged_subproblem_id_];
                    int k_count = 0;
                    for(const auto& agent_id : new_level) {
                        LAMAPF_Path agent_path = safeguard.new_level_paths_[k_count];
                        retv[agent_id] = agent_path;
                        k_count ++;
                    }
                    //std::cout << "flag 1" << std::endl;
                    // update all previous paths
                    for(int k=0; k<=safeguard.merged_subproblem_id_; k++) {
                        std::vector<std::pair<int, LAMAPF_Path> > path_id_s;
                        for(int agent_id : safeguard.new_levels_[k]) {
                            pre_pathss[agent_id] = retv[agent_id];
                        }
                    }
                    //std::cout << "flag 2" << std::endl;
                    // update pre_agents and pre_targets
                    for(int k=backup_i; k<=safeguard.end_of_merge_; k++) {
                        for(const auto& agent_id : levels[k]) {
                            pre_agents.push_back(pre->agents_[agent_id]);
                            pre_targets.push_back(pre->instance_node_ids_[agent_id].second);
                        }
                    }
                    //std::cout << "flag 3" << std::endl;
                    // update progress of solve subproblems
                    levels = safeguard.new_levels_;
                    level_id = safeguard.merged_subproblem_id_ + 1;

                    std::cout << "level after merge: ";
                    for(const auto& level : levels) {
                        std::cout << level << ", ";
                    }
                    std::cout << std::endl;
                    std::cout << "merged_subproblem_id_ = " << safeguard.merged_subproblem_id_ << std::endl;

                }
                continue;
            } else {
                assert(next_paths.size() == current_id_set.size());
                std::vector<std::pair<int, LAMAPF_Path> > next_paths_with_id;
                std::vector<int> current_id_vec;
                for(const auto& current_id : current_id_set) {
                    current_id_vec.push_back(current_id);
                }
                for(int k=0; k<current_id_vec.size(); k++) {
                    int agent_id = current_id_vec[k];
                    pre_pathss[agent_id] = next_paths[k];
                    retv[agent_id] = next_paths[k];

                    pre_agents.push_back(pre->agents_[agent_id]);
                    pre_targets.push_back(pre->instance_node_ids_[agent_id].second);
                }
                level_id ++;
            }
            // debug: path constraint check
//            if(new_constraint_table_ptr_ != nullptr) {
//                for (int k = 0; k < current_id_vec.size(); k++) {
//                    for (int t = 0; t < next_paths[k].size() - 1; t++) {
//                        if (new_constraint_table_ptr_->hasCollide(current_id_vec[k], t, next_paths[k][t], next_paths[k][t + 1])) {
//                            std::cout << "FATAL: agent " << current_id_vec[k] << " collide with previous path when t = " << t << std::endl;
//                            return {};//retv;
//                        }
//                    }
//                }
//            }
//            std::cout << "flag 2.6 " << std::endl;

        }
        LAMAPF_Paths merged_paths;
        if(!use_path_constraint) {
            mst.reset();
            std::vector<AgentPtrs<N> > agents_merge;
            std::vector<LAMAPF_Paths> paths_merge;
            for(const auto& level : levels) {
                agents_merge.push_back({});
                paths_merge.push_back({});
                for(const auto& agent_id : level) {
                    agents_merge.back().push_back(pre->agents_[agent_id]);
                    paths_merge.back().push_back(retv[agent_id]);
                }
            }
            merged_paths = multiLayerCompressLargeAgent(pre->all_poses_, paths_merge, agents_merge);
            assert(pre->instances_.size() == merged_paths.size());

            int global_count = 0;
            for(int i=0; i<agents_merge.size(); i++) {
                for(int j=0; j<agents_merge[i].size(); j++) {
                    retv[agents_merge[i][j]->id_] = merged_paths[global_count];
                    //assert(all_cluster_agents[i][j]->id_ == all_cluster_vec[i][j]);
                    global_count ++;
                }
            }
            assert(global_count == pre->agents_.size());
            assert(pre->agents_.size() == retv.size());
            for(int i=0; i<pre->agents_.size(); i++) {
                assert(retv[i].front() == pre->instance_node_ids_[i].first);
                assert(retv[i].back()  == pre->instance_node_ids_[i].second);
            }
            double merge_path_time_cost = (double)cutoff_time - mst.elapsed()/1e3;
            std::cout << "-- take " << merge_path_time_cost << "ms to merge paths" << std::endl << std::endl;
        }
        std::cout << "final check" << std::endl;
        assert(isSolutionValid(retv, pre->agents_, pre->all_poses_));

        std::cout << "-- layered large agent mapf success " << !retv.empty() << std::endl;
        std::cout << "-- SOC = " << getSOC(retv) << ", makespan = " << getMakeSpan(retv) << std::endl;

        return retv;
    }

}

#endif //LAYEREDMAPF_LARYERED_LARGE_AGENT_MAPF_H
