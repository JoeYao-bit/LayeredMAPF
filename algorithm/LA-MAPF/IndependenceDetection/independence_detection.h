//
// Created by yaozhuo on 2025/2/25.
//

#ifndef LAYERED_LAMAPF_INDEPENDENCE_DETECTION_H
#define LAYERED_LAMAPF_INDEPENDENCE_DETECTION_H

#include "../common.h"
#include <algorithm>
#include "../CBS/space_time_astar.h"
#include "../../precomputation_for_mapf.h"

namespace freeNav::LayeredMAPF::LA_MAPF::ID {


/*
 * Bibtex:
 *
  @inproceedings{surynek2018variants,
  title={Variants of independence detection in sat-based optimal multi-agent path finding},
  author={Surynek, Pavel and {\v{S}}vancara, Ji{\v{r}}{\'\i} and Felner, Ariel and Boyarski, Eli},
  booktitle={Agents and Artificial Intelligence: 9th International Conference, ICAART 2017, Porto, Portugal, February 24--26, 2017, Revised Selected Papers 9},
  pages={116--136},
  year={2018},
  organization={Springer}
}

 * */

    template <Dimension N, typename State>
    class ID {
    public:
        ID(const LA_MAPF_FUNC<N, State> & mapf_func,
           const PrecomputationOfMAPFBasePtr<N, State>& pre,
           int runtime = 3e4,
           int cf = 1, int fID = 1): pre_(pre), la_mapf_func_(mapf_func) {
            cost_function = cf;
            full_ID = fID;
            max_excircle_radius = getMaximumRadius<N>(pre->agents_);
            time_limit = runtime;
        }

        ~ID() {
            //
        }

        bool solve(int cost_lowerbound = 0, int cost_upperbound = MAX_COST) {
            if(SolveProblem() == 0) {
                solutions_ = current_plan;
                return true;
            } else {
                return false;
            }
        }

        virtual std::vector<LAMAPF_Path> getSolution() const {
            return solutions_;
        }

        int SolveProblem() {
//            std::cout << "flag 1" << std::endl;
            //current_plan = std::vector<std::vector<int> >(inst->agents, std::vector<int>());
            current_plan = LAMAPF_Paths(pre_->agents_.size());
            freeNav::LayeredMAPF::max_size_of_stack_layered = 0;

            // fill groups with single agents
            for (size_t i = 0; i < pre_->agents_.size(); i++) {
                groups.push_back(std::vector<int>(1, i));
                agent_to_group.push_back(i);
            }
//            std::cout << "flag 2" << std::endl;

            // solve single agent paths
            int max_time = 0;
            for (size_t i = 0; i < groups.size(); i++) {
//                std::cout << "flag 3, i = " << i << std::endl;
                //single_path->ShortestPath(inst->start[i], inst->goal[i], current_plan[i]); // yz: get single path
                const auto& start_node_id  = pre_->instance_node_ids_[i].first;
                const auto& target_node_id = pre_->instance_node_ids_[i].second;
                const auto& agent = i;
                CBS::ConstraintTable<N, State> constraint_table(agent, pre_->agents_, pre_->all_poses_, pre_->dim_, pre_->isoc_);
                CBS::SpaceTimeAstar<N, State> astar(start_node_id, target_node_id,
                                                    pre_->agents_heuristic_tables_[agent],
                                                    pre_->agents_heuristic_tables_ignore_rotate_[agent],
                                                    pre_->agent_sub_graphs_[agent],
                                        constraint_table, nullptr, nullptr, nullptr);
                LAMAPF_Path solution = astar.solve();
                if(solution.empty()) {
                    std::cout << "FATAL: unsolvable instance agent " << i << " in Independence Detection " << std::endl;
                }
                assert(solution.front() == start_node_id);
                assert(solution.back() == target_node_id);
                current_plan[i] = solution;
                if (current_plan[i].size() > max_time)
                    max_time = current_plan[i].size();
            }
//            std::cout << "flag 4" << std::endl;

            for (size_t i = 0; i < current_plan.size(); i++) {
                const auto& target_node_id = pre_->instance_node_ids_[i].second;
                current_plan[i].resize(max_time, target_node_id); // yz: initial paths, make all path have the same length
            }
//            std::cout << "flag 5" << std::endl;
            int ret_val;
            int g1, g2;
            int count = 0;
            while (CheckForConflicts(g1, g2)) {
            std::cout << "step = " << count << std::endl;
                count ++;
//            if(count >=1000) { break; }
                if (full_ID == 1) // if we use simple ID, just merge the groups
                {
//                    std::cout << "flag 6" << std::endl;
                    if (CheckPastConflicts(g1, g2)) {
                        // if g1 and g2 conflicted before, merge them and replan without constraints
                        MergeGroups(g1, g2);
                        ret_val = PlanForGroups(g1, -1, -1);
                        if (ret_val == -100)    // timeout
                        {
                            std::cout << "PlanForGroups 1 timeout"  << std::endl;
                            return -1;
                        } else if (ret_val == 0)    // ok
                            continue;
                    }

                    conflicted_groups.push_back(std::make_pair(g1, g2));
//                    std::cout << "flag 7" << std::endl;
                    // plan for g1 while avoiding g2
                    ret_val = PlanForGroups(g1, g2, ComputeGroupCost(g1));
                    if (ret_val == -100)    // timeout
                    {
                        std::cout << "PlanForGroups 2 timeout"  << std::endl;
                        return -1;
                    } else if (ret_val == 0)    // ok
                        continue;
//                    std::cout << "flag 8" << std::endl;

                    // plan for g2 while avoiding g1
                    ret_val = PlanForGroups(g2, g1, ComputeGroupCost(g2));
                    if (ret_val == -100)    // timeout
                    {
                        std::cout << "PlanForGroups 3 timeout"  << std::endl;
                        return -1;
                    } else if (ret_val == 0)    // ok
                        continue;
                }
//                std::cout << "flag 9" << std::endl;
                // if nothing is possible, merge g1 and g2 and replan without constraints
                MergeGroups(g1, g2);
                if (PlanForGroups(g1, -1, -1) == -100) {
                    std::cout << "PlanForGroups 4 timeout"  << std::endl;
                    return -1;
                }
//                std::cout << "flag 10" << std::endl;
            }

            solved = isSolutionValid<N>(current_plan, pre_->agents_, pre_->all_poses_);//inst->CheckPlan(current_plan);
            final_makespan = getMakeSpan(current_plan);//inst->GetPlanMakespan(current_plan);
            final_soc = getSOC(current_plan);//inst->GetPlanSoC(current_plan);

            if(solved) {
                paths_fr.clear();
                paths_fr = current_plan;
            }

            return 0;
        }

        size_t getMaximalSubProblem() const {
            size_t max_subproblem_size = 0;
            for(const auto& g : groups) {
                max_subproblem_size = std::max(g.size(), max_subproblem_size);
            }
            return max_subproblem_size;
        }

        size_t getNumberOfSubProblem() const {
            int count_of_group = 0;
            for(int i=0; i<groups.size(); i++) {
                if(!groups[i].empty()) { count_of_group++; }
            }
            return count_of_group;
        }

        LA_MAPF_FUNC<N, State> la_mapf_func_;

        // statistic variables
        int final_makespan;
        int final_soc;

        //private:
        //Instance *inst; // yz: grid map and graph about mapf
        //Dijkstra *single_path; // yz: single agent path planner
        //std::vector<Solver *> solvers; // yz: multiple mapf path planner

        int cost_function; // 1 - Makespan, 2 - Sum of Costs
        int full_ID; // 0 - simple ID, 1 - full ID

        std::vector<LAMAPF_Path> solutions_ ;

        float max_excircle_radius;

        std::vector<std::vector<int> > groups; // yz: which agents in groups
        std::vector<int> agent_to_group; // yz: mapping from agent to group id
        std::vector<std::pair<int, int> > conflicted_groups;
        LAMAPF_Paths current_plan; // yz: all agent's paths, store node id

        bool solved = false; // yz: whether current instance is solved

        LAMAPF_Paths paths_fr; // yz: freeNav style paths

        long long time_limit; // yz: in ms

        PrecomputationOfMAPFBasePtr<N, State> pre_;

        // yz: check conflict between two groups, start from t=0
        bool CheckForConflicts(int & g1, int & g2) {
            size_t plan_length = 0;
            for (size_t i = 0; i < current_plan.size(); i++)
                plan_length = std::max(plan_length, current_plan[i].size());

            if (plan_length == 0)
                return false;

            for (size_t i = 0; i < plan_length; i++) {
                for (size_t j = 0; j < current_plan.size(); j++) {
                    for (size_t k = j + 1; k < current_plan.size(); k++) {
                        if (j == k)
                            continue;
                        // yz: vertex conflict
//                        if (current_plan[j][i] == current_plan[k][i]) {
//                            g1 = agent_to_group[j];
//                            g2 = agent_to_group[k];
//                            if (g1 > g2)
//                                std::swap(g1, g2);
//                            return true;
//                        }
                        // yz: edge conflict
//                        if (i > 0 && current_plan[j][i - 1] == current_plan[k][i] &&
//                            current_plan[k][i - 1] == current_plan[j][i]) {
//                            g1 = agent_to_group[j];
//                            g2 = agent_to_group[k];
//                            if (g1 > g2)
//                                std::swap(g1, g2);
//                            return true;
//                        }
                        auto conflict_ptr = detectFirstConflictBetweenPaths(current_plan[j], current_plan[k],
                                                                            pre_->agents_[j], pre_->agents_[k], pre_->all_poses_);
                        if(conflict_ptr != nullptr) {
                            g1 = agent_to_group[j];
                            g2 = agent_to_group[k];
                            if (g1 > g2)
                                std::swap(g1, g2);
                            return true;
                        }
                    }
                }
            }
            return false;
        }

        bool CheckPastConflicts(int g1, int g2) {
            for (size_t i = 0; i < conflicted_groups.size(); i++)
                if (conflicted_groups[i].first == g1 && conflicted_groups[i].second == g2)
                    return true;
            return false;
        }

        void MergeGroups(int g1, int g2) {
            if (g1 > g2)
                std::swap(g1, g2);

            groups[g1].insert(groups[g1].end(), groups[g2].begin(), groups[g2].end());

            for (size_t i = 0; i < groups[g2].size(); i++)
                agent_to_group[groups[g2][i]] = g1;

            groups[g2].erase(groups[g2].begin(), groups[g2].end());
        }

        int ComputeGroupCost(int g1) {
            int cost = 0;

            // Makespan
            if (cost_function == 1) {
                for (size_t i = 0; i < groups[g1].size(); i++) {
                    const auto& goal = pre_->instance_node_ids_[groups[g1][i]].second;
                    for (size_t j = current_plan[groups[g1][i]].size() - 1; j > cost; j--) {
                        if (current_plan[groups[g1][i]][j] == goal &&
                            current_plan[groups[g1][i]][j - 1] != goal) {
                            cost = j;
                            break;
                        }
                    }
                }
                cost++;
            }
                // Sum of Costs
            else if (cost_function == 2) {
                for (size_t i = 0; i < groups[g1].size(); i++) {
                    const auto& goal = pre_->instance_node_ids_[groups[g1][i]].second;
                    for (int j = current_plan[groups[g1][i]].size() - 1; j >= 0; j--) {
                        if (current_plan[groups[g1][i]][j] != goal) {
                            cost += j + 1;
                            break;
                        }
                    }
                }
            }

            return cost;
        }

        void FixPlan(const std::vector<int> & agents_to_plan, LAMAPF_Paths & found_plan) {
            int old_makespan = current_plan[0].size();
            int new_makespan = old_makespan;

            // makespan of new solution
            for (size_t i = 0; i < found_plan.size(); i++) {
                for (size_t j = found_plan[i].size() - 1; j > 0; j--) {
                    if (found_plan[i][j] != found_plan[i][j - 1]) {
                        new_makespan = std::max(new_makespan, (int) j + 1);
                        break;
                    }
                }
            }

            // fix current plan
            if (new_makespan != old_makespan) {
                for (size_t i = 0; i < current_plan.size(); i++) {
                    const auto& goal = pre_->instance_node_ids_[i].second;
                    current_plan[i].resize(new_makespan, goal);
                }
            }

            // fix added plan
            if (new_makespan != found_plan[0].size()) {
                for (size_t i = 0; i < found_plan.size(); i++) {
                    const auto& goal = pre_->instance_node_ids_[agents_to_plan[i]].second;
                    found_plan[i].resize(new_makespan, goal);
                }
            }

            // add new plan
            for (size_t i = 0; i < agents_to_plan.size(); i++) {
                current_plan[agents_to_plan[i]] = found_plan[i];
            }
        }

        int PlanForGroups(int g1, int g2, int Cost) {
            std::vector<int> agents_to_plan;
            std::vector<std::vector<int> > agents_to_avoid;
            double time_cost = pre_->mst_.elapsed()/1e3;

            auto remain_s = time_limit/1e3 - time_cost;
            std::cout << "remain_s = " << remain_s << std::endl;
            if(remain_s < 0 ) { return -100; }
            // is in g1 -> to plan
            agents_to_plan = groups[g1];

            std::vector<int> current_id_set = groups[g1];

            std::vector<std::pair<State, State>> cluster_instances;
            std::vector<AgentPtr<N> > cluster_agents;

            std::vector<int> current_id_vec;
            std::vector<size_t> target_node_ids;

            const std::vector<std::shared_ptr<State> >& local_all_poses = pre_->all_poses_;
            const DistanceMapUpdaterPtr<N>&      local_distance_map_updater = pre_->distance_map_updater_;
            std::vector<SubGraphOfAgent<N, State> >     local_agent_sub_graphs;
            std::vector<std::vector<int> >       local_agents_heuristic_tables;
            std::vector<std::vector<int> >       local_agents_heuristic_tables_ignore_rotate;
            std::vector<std::pair<size_t, size_t> > local_instance_node_ids;


            for(const auto& current_id : current_id_set) {
                current_id_vec.push_back(current_id);
                cluster_instances.push_back(pre_->instances_[current_id]);
                cluster_agents.push_back(pre_->agents_[current_id]);


                target_node_ids.push_back(pre_->instance_node_ids_[current_id].second);

                local_agent_sub_graphs.push_back(pre_->agent_sub_graphs_[current_id]);

                local_agents_heuristic_tables.push_back(
                        pre_->agents_heuristic_tables_[current_id]);

                local_agents_heuristic_tables_ignore_rotate.push_back(
                        pre_->agents_heuristic_tables_ignore_rotate_[current_id]);

                local_instance_node_ids.push_back(pre_->instance_node_ids_[current_id]);


            }
//            std::cout << "flag 9.2" << std::endl;

            std::vector<AgentPtr<N> > local_cluster_agents; // copy of agent, with local id
            for(int k=0; k<current_id_vec.size(); k++) {

                const auto &agent_id = current_id_vec[k];
                AgentPtr<N> local_copy = pre_->agents_[agent_id]->copy();
                local_copy->id_ = k;
                local_cluster_agents.push_back(local_copy);
            }
//            std::cout << "flag 9.3" << std::endl;

            LargeAgentStaticConstraintTablePtr<N, State>
                    new_constraint_table_ptr_ = std::make_shared<LargeAgentStaticConstraintTable<N, State> > (
                    max_excircle_radius, pre_->dim_, pre_->isoc_, pre_->agents_, cluster_agents, pre_->all_poses_);

            // insert previous agents' target as static constraint
            new_constraint_table_ptr_->updateEarliestArriveTimeForAgents(cluster_agents, target_node_ids);

//            std::cout << "flag 9.4" << std::endl;

            if(g2 != -1) {
                // avoid previous separated group's path as hard constraint
                for(size_t id = 0; id < groups[g2].size(); id++) {
                    new_constraint_table_ptr_->insertPath(groups[g2][id], current_plan[groups[g2][id]]);
                }
            }
//            std::cout << "flag 9.5" << std::endl;

            if(Cost != -1) {
                // set path length limitation
                for(size_t id = 0; id < current_plan.size(); id++) {
                    new_constraint_table_ptr_->maximum_length_of_paths_.push_back(current_plan[id].size());
                }
                assert(new_constraint_table_ptr_->maximum_length_of_paths_.size() == current_plan.size());
            }

//            std::cout << "flag 9.6" << std::endl;


            int min_index = 0;

            std::vector<std::vector<int> > grid_visit_count_table_local;
            std::vector<LAMAPF_Path> retv_path = la_mapf_func_(cluster_instances,
                                                               local_cluster_agents,
                                                               pre_->dim_, pre_->isoc_,
                                                               new_constraint_table_ptr_,
                                                               grid_visit_count_table_local, remain_s,
                                                               local_instance_node_ids,
                                                               local_all_poses,
                                                               local_distance_map_updater,
                                                               local_agent_sub_graphs,
                                                               local_agents_heuristic_tables,
                                                               local_agents_heuristic_tables_ignore_rotate,
                                                            nullptr
            );
//            std::cout << "flag 9.7" << std::endl;

            if(!retv_path.empty()) {
                // yz: find solution
//            std::cout << "find solution of group " << g1 << "'s " << retv_path.size() << " agents' path" << std::endl;
                std::vector<LAMAPF_Path> found_plan;
                for(int i=0; i<retv_path.size(); i++) {
                    assert(retv_path[i].front() == pre_->instance_node_ids_[groups[g1][i]].first);
                    assert(retv_path[i].back()  == pre_->instance_node_ids_[groups[g1][i]].second);
                    found_plan.push_back(retv_path[i]);
                }
                FixPlan(agents_to_plan, found_plan);
                return 0;
            } else {
//            std::cout << "find solution failed" << std::endl;
                return 1;
            }

            // solver timeouted, or there was another error -> use another solver
            std::cout << " solver timeouted, or there was another error -> use another solver " << std::endl;
            return -100;
        }

    };

}


#endif //LAYEREDMAPF_INDEPENDENCE_DETECTION_H
