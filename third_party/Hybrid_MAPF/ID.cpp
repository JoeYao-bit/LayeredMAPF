#include "ID.h"


using namespace std;
namespace Hybird_MAPF {

    ID::ID(Instance *in, int cf, int fID) {
        inst = in;
        single_path = new Dijkstra(inst);
        cost_function = cf;
        full_ID = fID;
        runtime = 6e5;
        gettimeofday(&tv_pre, &tz);
    }

    int ID::SolveProblem(const vector<bool> &solvers_to_use) {

        current_plan = vector<vector<int> >(inst->agents, vector<int>());

        // select all possible solvers
//        if (solvers_to_use[0])
//            solvers.push_back(new PicatSolver(inst, cost_function));
//        if (solvers_to_use[1])
//            solvers.push_back(new CBSSolver(inst, cost_function));
//        if (solvers_to_use[2])
//            solvers.push_back(new ICTSSolver(inst, cost_function));
        freeNav::LayeredMAPF::max_size_of_stack_layered = 0;
        solver_computed = vector<int>(solvers.size());
        solver_used = vector<int>(solvers.size());
        solver_time = vector<vector<long long> >(solvers.size(), vector<long long>());


        // fill groups with single agents
        for (size_t i = 0; i < inst->agents; i++) {
            groups.push_back(vector<int>(1, i));
            agent_to_group.push_back(i);
        }

        // solve single agent paths
        int max_time = 0;
        for (size_t i = 0; i < groups.size(); i++) {
            single_path->ShortestPath(inst->start[i], inst->goal[i], current_plan[i]); // yz: get single path
            if (current_plan[i].size() > max_time)
                max_time = current_plan[i].size();
            inst->distance[inst->start[i]][inst->goal[i]] = current_plan[i].size() - 1; // update distance from start to goal
        }

        for (size_t i = 0; i < current_plan.size(); i++)
            current_plan[i].resize(max_time, inst->goal[i]); // yz: initial paths

        int ret_val;
        int g1, g2;
        int count = 0;
        while (CheckForConflicts(g1, g2)) {
//            std::cout << "step = " << count << std::endl;
            count ++;
//            if(count >=1000) { break; }
            if (full_ID == 1) // if we use simple ID, just merge the groups
            {
                if (CheckPastConflicts(g1, g2)) {
                    // if g1 and g2 conflicted before, merge them and replan without constraints
                    MergeGroups(g1, g2);
                    ret_val = PlanForGroups(g1, -1, -1);
                    if (ret_val == -100)    // timeout
                    {
                        std::cout << "PlanForGroups 1 timeout"  << std::endl;
                        return CleanUp(-1);
                    } else if (ret_val == 0)    // ok
                        continue;
                }

                conflicted_groups.push_back(make_pair(g1, g2));

                // plan for g1 while avoiding g2
                ret_val = PlanForGroups(g1, g2, ComputeGroupCost(g1));
                if (ret_val == -100)    // timeout
                {
                    std::cout << "PlanForGroups 2 timeout"  << std::endl;
                    return CleanUp(-1);
                } else if (ret_val == 0)    // ok
                    continue;

                // plan for g2 while avoiding g1
                ret_val = PlanForGroups(g2, g1, ComputeGroupCost(g2));
                if (ret_val == -100)    // timeout
                {
                    std::cout << "PlanForGroups 3 timeout"  << std::endl;
                    return CleanUp(-1);
                } else if (ret_val == 0)    // ok
                    continue;
            }

            // if nothing is possible, merge g1 and g2 and replan without constraints
            MergeGroups(g1, g2);
            if (PlanForGroups(g1, -1, -1) == -100) {
                std::cout << "PlanForGroups 4 timeout"  << std::endl;
                return CleanUp(-1);
            }
        }

        solved = inst->CheckPlan(current_plan);
//        inst->PrintPlan(current_plan);
        final_makespan = inst->GetPlanMakespan(current_plan);
        final_soc = inst->GetPlanSoC(current_plan);

        if(solved) {
            paths_fr.clear();
            for(int i=0; i<inst->instance_sat_.size(); i++) {
                freeNav::Path<2> path;
                auto raw_path = current_plan[i];
                for(int j=0; j<raw_path.size(); j++) {
                    path.push_back(inst->node_to_pt_map[raw_path[j]]);
                }
                paths_fr.push_back(path);
            }
        }

        return CleanUp(0);
    }


    bool ID::CheckForConflicts(int &g1, int &g2) {
        size_t plan_length = 0;
        for (size_t i = 0; i < current_plan.size(); i++)
            plan_length = max(plan_length, current_plan[i].size());

        if (plan_length == 0)
            return false;

        for (size_t i = 0; i < plan_length; i++) {
            for (size_t j = 0; j < current_plan.size(); j++) {
                for (size_t k = j + 1; k < current_plan.size(); k++) {
                    if (j == k)
                        continue;
                    if (current_plan[j][i] == current_plan[k][i]) {
                        g1 = agent_to_group[j];
                        g2 = agent_to_group[k];
                        if (g1 > g2)
                            swap(g1, g2);
                        return true;
                    }
                    if (i > 0 && current_plan[j][i - 1] == current_plan[k][i] &&
                        current_plan[k][i - 1] == current_plan[j][i]) {
                        g1 = agent_to_group[j];
                        g2 = agent_to_group[k];
                        if (g1 > g2)
                            swap(g1, g2);
                        return true;
                    }
                }
            }
        }
        return false;
    }


    bool ID::CheckPastConflicts(int g1, int g2) {
        for (size_t i = 0; i < conflicted_groups.size(); i++)
            if (conflicted_groups[i].first == g1 && conflicted_groups[i].second == g2)
                return true;
        return false;
    }


    void ID::MergeGroups(int g1, int g2) {
        if (g1 > g2)
            swap(g1, g2);

        groups[g1].insert(groups[g1].end(), groups[g2].begin(), groups[g2].end());

        for (size_t i = 0; i < groups[g2].size(); i++)
            agent_to_group[groups[g2][i]] = g1;

        groups[g2].erase(groups[g2].begin(), groups[g2].end());
    }


    int ID::ComputeGroupCost(int g1) {
        int cost = 0;

        // Makespan
        if (cost_function == 1) {
            for (size_t i = 0; i < groups[g1].size(); i++) {
                for (size_t j = current_plan[groups[g1][i]].size() - 1; j > cost; j--) {
                    if (current_plan[groups[g1][i]][j] == inst->goal[groups[g1][i]] &&
                        current_plan[groups[g1][i]][j - 1] != inst->goal[groups[g1][i]]) {
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
                for (int j = current_plan[groups[g1][i]].size() - 1; j >= 0; j--) {
                    if (current_plan[groups[g1][i]][j] != inst->goal[groups[g1][i]]) {
                        cost += j + 1;
                        break;
                    }
                }
            }
        }

        return cost;
    }


    void ID::FixPlan(vector<int> &agents_to_plan, vector<vector<int> > &found_plan) {
        int old_makespan = current_plan[0].size();
        int new_makespan = old_makespan;

        // makespan of new solution
        for (size_t i = 0; i < found_plan.size(); i++) {
            for (size_t j = found_plan[i].size() - 1; j > 0; j--) {
                if (found_plan[i][j] != found_plan[i][j - 1]) {
                    new_makespan = max(new_makespan, (int) j + 1); // yz: update makespan to larger value
                    break;
                }
            }
        }

        // fix current plan
        if (new_makespan != old_makespan) {
            for (size_t i = 0; i < current_plan.size(); i++) {
                current_plan[i].resize(new_makespan, inst->goal[i]);
            }
        }

        // fix added plan
        if (new_makespan != found_plan[0].size()) {
            for (size_t i = 0; i < found_plan.size(); i++) {
                found_plan[i].resize(new_makespan, inst->goal[agents_to_plan[i]]);
            }
        }

        // add new plan
        for (size_t i = 0; i < agents_to_plan.size(); i++) {
            current_plan[agents_to_plan[i]] = found_plan[i];
        }
    }

    int ID::CleanUp(int ret_val) {
        for (size_t i = 0; i < solvers.size(); i++) {
            if(solvers[i] != nullptr) {
                delete solvers[i];
            }
        }
        solvers.clear();
//        groups.clear();
        agent_to_group.clear();
        conflicted_groups.clear();

        //delete single_path;

        return ret_val;
    }

    ID::~ID() {
        delete single_path;
    }

/***************************************/
/* Different solving techniques follow */
/***************************************/
/* interface */
// plan for groups g1 and g2 with Cost or less
// if g2 == -1 -> no constraints, plan only for g1
// if SoC == -1 -> no constraint on SoC
// return -100 = timeout or error, 0 = ok, 1 = Can not be solved (unsolveable or too large cost)



// solver both solver after each other, take the faster (simulation of parallel)
//    int ID::PlanForGroups(int g1, int g2, int Cost) {
//        vector<int> agents_to_plan;
//        vector<vector<int> > agents_to_avoid;
//
//        // is in g1 -> to plan
//        agents_to_plan = groups[g1];
//
//        // set avoidance table - [agent][time] = node
//        if (g2 != -1) {
//            agents_to_avoid = vector<vector<int> >(groups[g2].size());
//            for (size_t i = 0; i < groups[g2].size(); i++)
//                for (size_t j = 0; j < current_plan[groups[g2][i]].size(); j++)
//                    agents_to_avoid[i].push_back(current_plan[groups[g2][i]][j]);
//
//            //for (size_t i = 0; i < agents_to_avoid.size(); i++)
//            //	agents_to_avoid[i].resize(Cost, inst->goal[groups[g2][i]]);
//        }
//
//        int timelimit = timelimit = inst->timeout; // in milli-seconds
//        if (timelimit > inst->timeout - runtime)
//            timelimit = inst->timeout - runtime;
//
//        vector<long long> vc_time_spent_now;
//        vector<int> vc_ret_val;
//        vector<vector<vector<int> > > found_plan(solvers.size());
//
//        for (size_t i = 0; i < solvers.size(); i++) {
//            if (runtime >= inst->timeout)
//                return -100;
//
//            if (i > 0) {
//                timelimit = min((long long) timelimit, vc_time_spent_now[i - 1]);
//                timelimit += 5000;
//            }
//
//            chrono::steady_clock::time_point begin = chrono::steady_clock::now();
//            solvers[i]->Solve(agents_to_plan, agents_to_avoid, Cost, timelimit);
//            chrono::steady_clock::time_point end = chrono::steady_clock::now();
//
//            long long time_spent_now = chrono::duration_cast<chrono::milliseconds>(end - begin).count();
//            vc_time_spent_now.push_back(time_spent_now);
//
//            int ret_val = solvers[i]->ReadResults(found_plan[i], Cost);
//            vc_ret_val.push_back(ret_val);
//        }
//
//        int min_index = 0;
//        for (size_t i = 0; i < solvers.size(); i++) {
//            if (vc_time_spent_now[i] < vc_time_spent_now[min_index])
//                min_index = i;
//        }
//
//        runtime += vc_time_spent_now[min_index];
//        solver_computed[min_index]++;
//        solver_time[min_index].push_back(vc_time_spent_now[min_index]);
//
//        if (vc_ret_val[min_index] == 0) {
//            // ok, we have good solution
//            FixPlan(agents_to_plan, found_plan[min_index]);
//
//            solver_used[min_index]++;
//            return 0;
//        } else if (vc_ret_val[min_index] == 1) {
//            // can not be solved
//            return 1;
//        }
//
//        // solver timeouted, or there was another error -> use another solver
//        std::cout << " solver timeouted, or there was another error -> use another solver " << std::endl;
//        return -100;
//    }

    int ID::PlanForGroups(int g1, int g2, int Cost) {
        vector<int> agents_to_plan;
        vector<vector<int> > agents_to_avoid;
        auto t = clock();
        gettimeofday(&tv_after, &tz);
        double time_cost = (tv_after.tv_sec - tv_pre.tv_sec) + (tv_after.tv_usec - tv_pre.tv_usec)/1e6;
        auto remain_s = runtime/1e3 - time_cost;
//        std::cout << "remain_s = " << remain_s << std::endl;
        if(remain_s < 0 ) { return -100; }
        // is in g1 -> to plan
        agents_to_plan = groups[g1];

        // set avoidance table - [agent][time] = node
//        if (g2 != -1) {
//            agents_to_avoid = vector<vector<int> >(groups[g2].size());
//            for (size_t i = 0; i < groups[g2].size(); i++)
//                for (size_t j = 0; j < current_plan[groups[g2][i]].size(); j++)
//                    agents_to_avoid[i].push_back(current_plan[groups[g2][i]][j]);
//
//            //for (size_t i = 0; i < agents_to_avoid.size(); i++)
//            //	agents_to_avoid[i].resize(Cost, inst->goal[groups[g2][i]]);
//        }
        CBS_Li::ConstraintTable *layered_ct = new CBS_Li::ConstraintTable(inst->dim_[0], inst->dim_[0]*inst->dim_[1]);

        if(g2 != -1) {
            // avoid previous separated group's path as hard constraint
            for(size_t id = 0; id < groups[g2].size(); id++) {
                CBS_Li::MAPFPath path_eecbs;
                for (int t = 0; t < current_plan[id].size(); t++) {
                    path_eecbs.push_back(CBS_Li::PathEntry(PointiToId(inst->node_to_pt_map[current_plan[id][t]], inst->dim_)));
                }
//                other_group_paths.push_back(paths_[i]);
                layered_ct->insert2CT(path_eecbs);
            }
        }

        if(Cost != -1) {
            // set path length limitation
            layered_ct->maximum_length_of_paths_;
            for(size_t id = 0; id < groups[g1].size(); id++) {
                layered_ct->maximum_length_of_paths_.push_back(current_plan[groups[g1][id]].size());
            }
            assert(layered_ct->maximum_length_of_paths_.size() == groups[g1].size());
        }

//        int timelimit = timelimit = inst->timeout; // in milli-seconds
//        if (timelimit > inst->timeout - runtime)
//            timelimit = inst->timeout - runtime;

        vector<long long> vc_time_spent_now;
        vector<int> vc_ret_val;
//        vector<vector<vector<int> > > found_plan(solvers.size());

        freeNav::Instances<2> sat;
        for(size_t id = 0; id < groups[g1].size(); id++) {
            sat.push_back(inst->instance_sat_[groups[g1][id]]);
        }

//        for (size_t i = 0; i < solvers.size(); i++) {
//            if (runtime >= inst->timeout)
//                return -100;
//
//            if (i > 0) {
//                timelimit = min((long long) timelimit, vc_time_spent_now[i - 1]);
//                timelimit += 5000;
//            }

//        chrono::steady_clock::time_point begin = chrono::steady_clock::now();
//        std::cout << "sat.size() = " << sat.size() << std::endl;
        freeNav::Paths<2> retv_path = mapf_func_(inst->dim_, inst->isoc_, sat, layered_ct, remain_s);
        freeNav::LayeredMAPF::max_size_of_stack_layered = std::max(freeNav::LayeredMAPF::max_size_of_stack_layered,
                                                                   freeNav::LayeredMAPF::max_size_of_stack);

//            solvers[i]->Solve(agents_to_plan, agents_to_avoid, Cost, timelimit);

//        chrono::steady_clock::time_point end = chrono::steady_clock::now();
//
//        long long time_spent_now = chrono::duration_cast<chrono::milliseconds>(end - begin).count();
//        vc_time_spent_now.push_back(time_spent_now);

//            int ret_val = solvers[i]->ReadResults(found_plan[i], Cost);
//            vc_ret_val.push_back(ret_val);
//        }

        int min_index = 0;
//        for (size_t i = 0; i < solvers.size(); i++) {
//            if (vc_time_spent_now[i] < vc_time_spent_now[min_index])
//                min_index = i;
//        }

//        runtime += vc_time_spent_now[min_index];
//        solver_computed[min_index]++;
//        solver_time[min_index].push_back(vc_time_spent_now[min_index]);

        if(!retv_path.empty()) {
            // yz: find solution
//            std::cout << "find solution of group " << g1 << "'s " << retv_path.size() << " agents' path" << std::endl;
            vector<vector<int> > found_plan;
            for(int i=0; i<retv_path.size(); i++) {
                vector<int> path;
//                std::cout << "path " << i << std::endl;
                for(int j=0; j<retv_path[i].size(); j++) {
                    const auto& pt = retv_path[i][j];
                    path.push_back(inst->int_graph[pt[1]][pt[0]]);
//                    std::cout << pt << " ";
                }
                assert(path.front() == inst->start[groups[g1][i]]);
                assert(path.back() == inst->goal[groups[g1][i]]);
//                std::cout << std::endl;
                found_plan.push_back(path);
            }
            FixPlan(agents_to_plan, found_plan);
            delete layered_ct;
            return 0;
        } else {
//            std::cout << "find solution failed" << std::endl;
            delete layered_ct;
            return 1;
        }

//        if (vc_ret_val[min_index] == 0) {
//            // ok, we have good solution
//            FixPlan(agents_to_plan, found_plan[min_index]);
//
//            solver_used[min_index]++;
//            return 0;
//        } else if (vc_ret_val[min_index] == 1) {
//            // can not be solved
//            return 1;
//        }

        // solver timeouted, or there was another error -> use another solver
        std::cout << " solver timeouted, or there was another error -> use another solver " << std::endl;
        return -100;
    }



// solve incrementaly for fixed order - outdated!!!
/*int ID::PlanForGroups(int g1, int g2, int Cost)
{
	vector<int> agents_to_plan;
	vector<vector<int> > agents_to_avoid;

	// is in g1 -> to plan
	agents_to_plan = groups[g1];

	// set avoidance table - [agent][time] = node
	if (g2 > 0)
	{
		agents_to_avoid = vector<vector<int> >(groups[g2].size());
		for (size_t i = 0; i < groups[g2].size(); i++)
			for (size_t j = 0; j < current_plan[groups[g2][i]].size(); j++)
				agents_to_avoid[i].push_back(current_plan[groups[g2][i]][j]);

		for (size_t i = 0; i < agents_to_avoid.size(); i++)
			agents_to_avoid[i].resize(Cost, inst->goal[groups[g2][i]]);
	}

	int timelimit = 15000; // in milli-seconds

	// if there is only one solver, no need to increment timelimit
	if (solvers.size() == 1)
		timelimit = inst->timeout;
	else
		timelimit = 15000;

	while (true)
	{
		for (size_t i = 0; i < solvers.size(); i++)
		{
			if (runtime >= inst->timeout)
				return -100;

			if (timelimit > inst->timeout - runtime)
				timelimit = inst->timeout - runtime;

			chrono::steady_clock::time_point begin = chrono::steady_clock::now();
			solvers[i]->Solve(agents_to_plan, agents_to_avoid, Cost, timelimit);
			chrono::steady_clock::time_point end = chrono::steady_clock::now();

			long long time_spent_now = chrono::duration_cast<chrono::milliseconds>(end - begin).count();

			runtime += time_spent_now;
			solver_computed[i]++;
			solver_time[i].push_back(time_spent_now);

			vector<vector<int> > found_plan;
			int ret_val = solvers[i]->ReadResults(found_plan, Cost);

			if (ret_val == 0)
			{
				// ok, we have good solution
				FixPlan(agents_to_plan, found_plan);

				solver_used[i]++;
				return 0;
			}
			else if (ret_val == 1)
			{
				// can not be solved
				return 1;
			}

			// solver timeouted, or there was another error -> use another solver
		}
		timelimit *= 2;
	}
}*/

}