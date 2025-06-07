//
// Created by yaozhuo on 6/7/25.
//

#ifndef LAYEREDMAPF_SOLVABILITY_SAFE_GUARD_H
#define LAYEREDMAPF_SOLVABILITY_SAFE_GUARD_H

#include "large_agent_instance_decomposition.h"

namespace freeNav::LayeredMAPF::LA_MAPF {

    // when MAPF failed to solve a subproblem within time limit,
    // solve it with ignore of all other subproblem's agent
    // if the raw problem is solvable, part of it should still be solvable
    // get solution pass what target of previous subproblem's agent and start of subsequent subproblem
    // use heuristic during planning to reduce size of final subproblem
    template<Dimension N>
    class SolvabilitySafeguard : public LargeAgentMAPF<N> {

        SolvabilitySafeguard(const InstanceOrients<N> & instances,
                             const std::vector<AgentPtr<N> >& agents,
                             DimensionLength* dim,
                             const IS_OCCUPIED_FUNC<N> & isoc,

                             // reusable variable, try do not construct them more than once
                             const std::vector<PosePtr<int, N> >& all_poses = {},
                             const DistanceMapUpdaterPtr<N>& distance_map_updater = nullptr,
                             const std::vector<SubGraphOfAgent<N> >& agent_sub_graphs = {},
                             const std::vector<std::vector<int> >& agents_heuristic_tables = {},
                             const std::vector<std::vector<int> >& agents_heuristic_tables_ignore_rotate = {},

                             double time_limit = 60
        ) : LargeAgentMAPF<N>(instances, agents, dim, isoc,
                              all_poses,
                              distance_map_updater,
                              agent_sub_graphs,
                              agents_heuristic_tables,
                              agents_heuristic_tables_ignore_rotate,
                              time_limit) {
            //
        }

        bool isStateCollideWithSolution(const AgentPtr<N>& a1, const Pose<int, N>& st,
                                        const AgentPtr<N>& a2, const LAMAPF_Path& path) {
            for(int t=0; t<path.size()-1; t++) {
                if(isCollide(a1, st, a2, *this->all_poses_[path[t]], *this->all_poses_[path[t+1]])) {
                    return true;
                }
            }
            return false;
        }

        bool isStateCollideWithSolutions(const AgentPtr<N>& a1, const Pose<int, N>& st,
                                         const AgentPtrs<N>& agents, const std::vector<LAMAPF_Path>& paths) {
            assert(agents.size() == paths.size());
            for(int i=0; i<agents.size(); i++) {
                if(isStateCollideWithSolution(a1, st, agents[i], paths[i])) {
                    return true;
                }
            }
            return false;
        }

        std::vector<std::set<int> > mergeSubproblemTillSolvable(const std::vector<std::set<int> >& levels,
                                         const int & failed_subproblem_id,
                                         const LA_MAPF_FUNC<N>& mapf_func,
                                         double time_limit = 30) {
            struct timezone tz;
            struct timeval tv_pre, tv_cur;
            struct timeval tv_after;
            gettimeofday(&tv_pre, &tz);
            // the loop will end when run out of time, or merge all subproblem and get the raw problem
            std::vector<std::set<int> > local_levels = levels;
            int local_failed_subproblem_id = failed_subproblem_id;

            InstanceOrients<N> local_instance;
            AgentPtrs<N> local_agents;
            std::vector<SubGraphOfAgent<N> >     local_agent_sub_graphs;
            std::vector<std::vector<int> >       local_agents_heuristic_tables;
            std::vector<std::vector<int> >       local_agents_heuristic_tables_ignore_rotate;

           LAMAPF_Paths final_solutions;

            while(true) {
                // 1, get local instance
                local_instance.clear();
                local_agents.clear();
                local_agent_sub_graphs.clear();
                local_agents_heuristic_tables.clear();
                local_agents_heuristic_tables_ignore_rotate.clear();

                for(const auto& failed_agent_id : levels[local_failed_subproblem_id]) {
                    local_agents.push_back(this->agents_[failed_agent_id]);
                    local_instance.push_back(this->instances_[failed_agent_id]);
                    local_agent_sub_graphs.push_back(this->agent_sub_graphs_[failed_agent_id]);
                    local_agents_heuristic_tables.push_back(this->agents_heuristic_tables_[failed_agent_id]);
                    local_agents_heuristic_tables_ignore_rotate.push_back(
                            this->agents_heuristic_tables_ignore_rotate_[failed_agent_id]);
                }
                gettimeofday(&tv_after, &tz);
                double time_cost_yet = (tv_after.tv_sec - tv_pre.tv_sec) + (tv_after.tv_usec - tv_pre.tv_usec) / 1e6;
                double remaining_time = time_limit - time_cost_yet;
                if(remaining_time < 0) { return {}; }
                // 2, solve it with ignoring other agent
                std::vector<std::vector<int> > grid_visit_count_table_local;
                std::vector<LAMAPF_Path> local_paths = mapf_func(local_instance,
                                                                local_agents,
                                                                this->dim_m, this->isoc_,
                                                                nullptr,
                                                                grid_visit_count_table_local, remaining_time,
                                                                this->all_poses_,
                                                                this->distance_map_updater_,
                                                                local_agent_sub_graphs,
                                                                local_agents_heuristic_tables,
                                                                local_agents_heuristic_tables_ignore_rotate,
                                                                nullptr
                );
                if(local_paths.empty()) { return {}; } // par of the raw MAPF problems shouldn't failed, except run out of time
                // 3, should be success, exit merge into a new level and do this again
                // default value means no need to merge previous subproblem
                int start_of_merge = local_failed_subproblem_id;
                for(int i=0; i<local_failed_subproblem_id; i++) {
                    for(const auto& agent_id : local_levels[i]) {
                        const auto& another_agent = this->agents_[agent_id];
                        // pass previous agent's target need to merge
                        const auto& another_state = this->instances_[agent_id].second;
                        if(isStateCollideWithSolutions(another_agent, another_state, local_agents, local_paths)) {
                            start_of_merge = i;
                        }
                    }
                }
                // default value means no need to merge subsequent subproblem
                int end_of_merge = local_failed_subproblem_id;
                for(int i=local_levels.size()-1; i>local_failed_subproblem_id; i--) {
                    for(const auto& agent_id : local_levels[i]) {
                        const auto& another_agent = this->agents_[agent_id];
                        // pass subsequent agent's start need to merge
                        const auto& another_state = this->instances_[agent_id].first;
                        if(isStateCollideWithSolutions(another_agent, another_state, local_agents, local_paths)) {
                            end_of_merge = i;
                        }
                    }
                }
                // construct merged subproblem
                std::vector<std::set<int> > new_levels;
                for(int i=0; i<local_levels.size(); i++) {
                    if(i <= start_of_merge || i > end_of_merge) {
                        new_levels.push_back(local_levels[i]);
                    } else {
                        new_levels.back().insert(local_levels[i].begin(), local_levels[i].end());
                    }
                }
                // the merged subproblem's index is start_of_merge
                // there should be local_levels.size - (end_of_merge - start_of_merge) in the new levels
                assert(new_levels.size() == local_levels.size() - (end_of_merge - start_of_merge));
                local_levels = new_levels;
                local_failed_subproblem_id = start_of_merge;
                // 4, check whether merged subproblem is solvable
                // 1, get local instance
                local_instance.clear();
                local_agents.clear();
                local_agent_sub_graphs.clear();
                local_agents_heuristic_tables.clear();
                local_agents_heuristic_tables_ignore_rotate.clear();

                for(const auto& failed_agent_id : levels[local_failed_subproblem_id]) {
                    local_agents.push_back(this->agents_[failed_agent_id]);
                    local_instance.push_back(this->instances_[failed_agent_id]);
                    local_agent_sub_graphs.push_back(this->agent_sub_graphs_[failed_agent_id]);
                    local_agents_heuristic_tables.push_back(this->agents_heuristic_tables_[failed_agent_id]);
                    local_agents_heuristic_tables_ignore_rotate.push_back(
                            this->agents_heuristic_tables_ignore_rotate_[failed_agent_id]);
                }
                gettimeofday(&tv_after, &tz);
                time_cost_yet = (tv_after.tv_sec - tv_pre.tv_sec) + (tv_after.tv_usec - tv_pre.tv_usec) / 1e6;
                remaining_time = time_limit - time_cost_yet;
                if(remaining_time < 0) { return {}; }
                // 4, try solve it with avoid previous subproblem's target and subsequent subproblem's start
                float max_excircle_radius = getMaximumRadius<N>(this->agents_);
                LargeAgentStaticConstraintTablePtr<N>
                        new_constraint_table_ptr_ = std::make_shared<LargeAgentStaticConstraintTable<N> > (
                        max_excircle_radius, this->dim_, this->isoc_, this->agents_, local_agents, this->all_poses_);

                local_paths = mapf_func(local_instance,
                                                                 local_agents,
                                                                 this->dim_m, this->isoc_,
                                                                 nullptr,
                                                                 grid_visit_count_table_local, remaining_time,
                                                                 this->all_poses_,
                                                                 this->distance_map_updater_,
                                                                 local_agent_sub_graphs,
                                                                 local_agents_heuristic_tables,
                                                                 local_agents_heuristic_tables_ignore_rotate,
                                                                 nullptr
                );
                if(!local_paths.empty()) {
                    merged_subproblem_id_ = local_failed_subproblem_id;
                    new_levels_ = local_levels;
                    new_level_paths_ = local_paths;
                    break;
                } // par of the raw MAPF problems shouldn't failed, except run out of time
                // 5, repeat above process until new subproblem is solved
            }
        }

        const std::vector<std::set<int> > raw_levels_;
        const int failed_subproblem_id_;

        std::vector<std::set<int> > new_levels_;
        int merged_subproblem_id_;
        LAMAPF_Paths new_level_paths_;
    };

}

#endif //LAYEREDMAPF_SOLVABILITY_SAFE_GUARD_H
