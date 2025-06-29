//
// Created by yaozhuo on 6/7/25.
//

#ifndef LAYEREDMAPF_SOLVABILITY_SAFE_GUARD_H
#define LAYEREDMAPF_SOLVABILITY_SAFE_GUARD_H

#include "large_agent_instance_decomposition.h"
#include "../basic.h"
#include "../precomputation_for_mapf.h"

namespace freeNav::LayeredMAPF::LA_MAPF {

    // when MAPF failed to solve a subproblem within time limit,
    // solve it with ignore of all other subproblem's agent
    // if the raw problem is solvable, part of it should still be solvable
    // get solution pass what target of previous subproblem's agent and start of subsequent subproblem
    // use heuristic during planning to reduce size of final subproblem
    template<Dimension N, typename State>
    class SolvabilitySafeguard {
    public:
        SolvabilitySafeguard(PrecomputationOfMAPFBasePtr<N, State> pre) : pre_(pre) {
            //std::cout << "SolvabilitySafeguard start " << std::endl;
        }

        // do not call this function, we inherit LargeAgentMAPF just want to use precomputation data from outside
        bool solve(int cost_lowerbound = 0, int cost_upperbound = MAX_COST) {
            return false;
        }

        bool isStateCollideWithSolution(const AgentPtr<N>& a1, const State& st,
                                        const AgentPtr<N>& a2, const LAMAPF_Path& path) {
            for(int t=0; t<path.size()-1; t++) {
                if(isCollide(a1, st, a2, *pre_->all_poses_[path[t]], *pre_->all_poses_[path[t+1]])) {
                    return true;
                }
            }
            return false;
        }

        bool isStateCollideWithSolutions(const AgentPtr<N>& a1, const State& st,
                                         const AgentPtrs<N>& agents, const std::vector<LAMAPF_Path>& paths) {
            assert(agents.size() == paths.size());
            for(int i=0; i<agents.size(); i++) {
                if(isStateCollideWithSolution(a1, st, agents[i], paths[i])) {
                    return true;
                }
            }
            return false;
        }

        bool mergeSubproblemTillSolvable(const std::vector<std::set<int> >& levels,
                                                  const int & failed_subproblem_id,
                                                  const LA_MAPF_FUNC<N, State>& mapf_func,
                                                  const IS_OCCUPIED_FUNC<N>& ex_isoc,
                                                  double time_limit = 30) {
            MSTimer mst;
            // the loop will end when run out of time, or merge all subproblem and get the raw problem
            std::vector<std::set<int> > local_levels = levels;
            int local_failed_subproblem_id = failed_subproblem_id;

            InstanceOrients<N> local_instance;
            AgentPtrs<N> local_agents;
            std::vector<SubGraphOfAgent<N, State> > local_agent_sub_graphs;
            std::vector<std::vector<int> >          local_agents_heuristic_tables;
            std::vector<std::vector<int> >          local_agents_heuristic_tables_ignore_rotate;
            std::vector<std::pair<size_t, size_t> > local_instance_node_ids;

            LAMAPF_Paths final_solutions;
            //std::cout << "flag 1 " << std::endl;

            while(true) {
                // 1, get local instance
                local_instance.clear();
                local_agents.clear();
                local_agent_sub_graphs.clear();
                local_agents_heuristic_tables.clear();
                local_agents_heuristic_tables_ignore_rotate.clear();
                local_instance_node_ids.clear();
                int count_of_ag = 0;
                //std::cout << "this->isoc_ 1.11 " << this->isoc_(Pointi<N>()) << std::endl;
                for(const auto& failed_agent_id : levels[local_failed_subproblem_id]) {

                    AgentPtr<N> local_copy = pre_->agents_[failed_agent_id]->copy();
                    local_copy->id_ = count_of_ag;
                    count_of_ag ++;
                    local_agents.push_back(local_copy);


                    local_instance.push_back(pre_->instances_[failed_agent_id]);
                    local_agent_sub_graphs.push_back(pre_->agent_sub_graphs_[failed_agent_id]);
                    local_agents_heuristic_tables.push_back(pre_->agents_heuristic_tables_[failed_agent_id]);
                    local_agents_heuristic_tables_ignore_rotate.push_back(
                            pre_->agents_heuristic_tables_ignore_rotate_[failed_agent_id]);
                    local_instance_node_ids.push_back(pre_->instance_node_ids_[failed_agent_id]);

                }
                double time_cost_yet = mst.elapsed()/1e3;
                double remaining_time = time_limit - time_cost_yet;
                if(remaining_time < 0) { return false; }
                // 2, solve it with ignoring other agent
                //std::cout << "flag 2 " << std::endl;
                std::vector<std::vector<int> > grid_visit_count_table_local;
                std::vector<LAMAPF_Path> local_paths = mapf_func(local_instance,
                                                                 local_agents,
                                                                 pre_->dim_, ex_isoc,
                                                                 nullptr,
                                                                 grid_visit_count_table_local, remaining_time,
                                                                 local_instance_node_ids,
                                                                 pre_->all_poses_,
                                                                 pre_->distance_map_updater_,
                                                                 local_agent_sub_graphs,
                                                                 local_agents_heuristic_tables,
                                                                 local_agents_heuristic_tables_ignore_rotate,
                                                                 nullptr
                );
                //std::cout << "flag 3 " << std::endl;

                if(local_paths.empty()) { return false; } // part of the raw MAPF problems shouldn't failed, except run out of time
                //std::cout << " find solution if ignore other agent " << std::endl;
                // 3, should be success, exit merge into a new level and do this again
                // default value means no need to merge previous subproblem
                int start_of_merge = local_failed_subproblem_id;
                for(int i=0; i<local_failed_subproblem_id; i++) {
                    for(const auto& agent_id : local_levels[i]) {
                        const auto& another_agent = pre_->agents_[agent_id];
                        // pass previous agent's target need to merge
                        const auto& another_state = pre_->instances_[agent_id].second;
                        if(isStateCollideWithSolutions(another_agent, another_state, local_agents, local_paths)) {
                            start_of_merge = i;
                        }
                    }
                }
                // default value means no need to merge subsequent subproblem
                int end_of_merge = local_failed_subproblem_id;
                for(int i=local_levels.size()-1; i>local_failed_subproblem_id; i--) {
                    for(const auto& agent_id : local_levels[i]) {
                        const auto& another_agent = pre_->agents_[agent_id];
                        // pass subsequent agent's start need to merge
                        const auto& another_state = pre_->instances_[agent_id].first;
                        if(isStateCollideWithSolutions(another_agent, another_state, local_agents, local_paths)) {
                            end_of_merge = i;
                        }
                    }
                }

                //std::cout << "before merge" << std::endl;
                // construct merged subproblem
                std::vector<std::set<int> > new_levels;
                for(int i=0; i<local_levels.size(); i++) {
                    if(i <= start_of_merge || i > end_of_merge) {
                        new_levels.push_back(local_levels[i]);
                    } else {
                        new_levels.back().insert(local_levels[i].begin(), local_levels[i].end());
                    }
                }
                start_of_merge_ = start_of_merge;
                end_of_merge_ = end_of_merge;
                //std::cout << "after merge" << std::endl;
                // the merged subproblem's index is start_of_merge
                // there should be local_levels.size - (end_of_merge - start_of_merge) in the new levels
                assert(new_levels.size() == local_levels.size() - (end_of_merge - start_of_merge));
                local_levels = new_levels;
                local_failed_subproblem_id = start_of_merge;
                // 4, check whether merged subproblem is solvable
                // get local instance
                local_instance.clear();
                local_agents.clear();
                local_agent_sub_graphs.clear();
                local_agents_heuristic_tables.clear();
                local_agents_heuristic_tables_ignore_rotate.clear();
                local_instance_node_ids.clear();

                AgentPtrs<N> local_agents_with_global_id;
                std::vector<size_t> target_node_ids;
                count_of_ag = 0;
                for(const auto& failed_agent_id : local_levels[local_failed_subproblem_id]) {

                    target_node_ids.push_back(pre_->instance_node_ids_[failed_agent_id].second);

                    AgentPtr<N> local_copy = pre_->agents_[failed_agent_id]->copy();
                    local_copy->id_ = count_of_ag;
                    count_of_ag ++;
                    local_agents.push_back(local_copy);

                    local_agents_with_global_id.push_back(pre_->agents_[failed_agent_id]);

                    local_instance.push_back(pre_->instances_[failed_agent_id]);
                    local_agent_sub_graphs.push_back(pre_->agent_sub_graphs_[failed_agent_id]);
                    local_agents_heuristic_tables.push_back(pre_->agents_heuristic_tables_[failed_agent_id]);
                    local_agents_heuristic_tables_ignore_rotate.push_back(
                            pre_->agents_heuristic_tables_ignore_rotate_[failed_agent_id]);
                    local_instance_node_ids.push_back(pre_->instance_node_ids_[failed_agent_id]);

                }
                time_cost_yet = mst.elapsed()/1e3;
                remaining_time = time_limit - time_cost_yet;

                if(remaining_time < 0) { return {}; }
                // 4, try solve it with avoid previous subproblem's target and subsequent subproblem's start
                float max_excircle_radius = getMaximumRadius<N>(pre_->agents_);
                //std::cout << "this->isoc_ 2 " << this->isoc_(Pointi<N>()) << std::endl;
                LargeAgentStaticConstraintTablePtr<N, State>
                        new_constraint_table_ptr = std::make_shared<LargeAgentStaticConstraintTable<N, State> > (
                        max_excircle_radius, pre_->dim_, ex_isoc, pre_->agents_, local_agents_with_global_id,
                        pre_->all_poses_); // this->isoc_ may fail

                // insert previous agents' target as static constraint
                for(int i=0; i<local_failed_subproblem_id; i++)
                {
                    const auto& current_level = local_levels[i];
                    for(const int& agent_id : current_level) {
                        new_constraint_table_ptr->insertPose(agent_id, pre_->instance_node_ids_[agent_id].second);
                    }
                }

                // insert future agents' start as static constraint
                for(int j = local_failed_subproblem_id + 1 ; j<local_levels.size(); j++)
                {
                    const auto& current_level = local_levels[j];
                    for(const int& agent_id : current_level) {
                        new_constraint_table_ptr->insertPose(agent_id, pre_->instance_node_ids_[agent_id].first);
                    }
                }

                // insert previous agents' target as static constraint
                new_constraint_table_ptr->updateEarliestArriveTimeForAgents(local_agents, target_node_ids);

                local_paths = mapf_func(local_instance,
                                        local_agents,
                                        pre_->dim_, pre_->isoc_,
                                        new_constraint_table_ptr,
                                        grid_visit_count_table_local, remaining_time,
                                        local_instance_node_ids,
                                        pre_->all_poses_,
                                        pre_->distance_map_updater_,
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
            return true;
        }

        //const std::vector<std::set<int> > raw_levels_;
        //const int failed_subproblem_id_;

        std::vector<std::set<int> > new_levels_;
        int merged_subproblem_id_;
        LAMAPF_Paths new_level_paths_;

        int start_of_merge_, end_of_merge_;

        PrecomputationOfMAPFBasePtr<N, State> pre_;

    };

}

#endif //LAYEREDMAPF_SOLVABILITY_SAFE_GUARD_H
