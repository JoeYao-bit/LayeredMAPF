#include "../include/lacam2.hpp"
namespace LaCAM2 {

    Solution solve(const Instance &ins, std::string &additional_info,
                   const int verbose, const Deadline *deadline, std::mt19937 *MT,
                   const Objective objective, const float restart_rate) {
        auto planner = Planner(&ins, deadline, MT, verbose, objective, restart_rate);
        return planner.solve(additional_info);
    }


    freeNav::Paths<2> lacam2_MAPF(freeNav::DimensionLength *dim,
                                  const freeNav::IS_OCCUPIED_FUNC<2> &isoc,
                                  const freeNav::Instances<2> &instance_sat,
                                  CBS_Li::ConstraintTable *ct,
                                  int cutoff_time) {

        // setup instance
        const auto verbose = 1;//std::stoi(program.get<std::string>("verbose"));
        const auto time_limit_sec = cutoff_time;
        //std::stoi(program.get<std::string>("time_limit_sec"));
        const auto seed = 0;
        auto MT = std::mt19937(seed);

        Instance ins(dim, isoc, instance_sat);
        if (!ins.is_valid(1)) {
            std::cout << " result is invalid " << std::endl;
            return {};
        }

        ins.ct = ct;
        // solve
        const Deadline deadline = Deadline(time_limit_sec * 1000);
        //std::cout << " ins->ct " << ins.ct << std::endl;
        auto additional_info = std::string("");
        const Solution solution = solve(ins, additional_info, verbose - 1, &deadline, &MT);
        //const double comp_time_ms = deadline.elapsed_ms();

        // failure
        if (solution.empty()) {
            //info(1, verbose, "failed to solve");
            std::cout << "failed to solve" << std::endl;
            return {};
        } else {
            //std::cout << " solution size " << solution[0].size() << std::endl;
        }

        // check feasibility
        if (!is_feasible_solution(ins, solution, verbose)) {
            //info(0, verbose, "invalid solution");
            std::cout << " invalid solution " << std::endl;
            return {};
        }
        // yz: transform to freeNav style path
        freeNav::Paths<2> retv(solution.front().size());
        for(int t=0; t<solution.size(); t++) {
            assert(solution[t].size() == instance_sat.size());
            for(int agent=0; agent<solution[t].size(); agent++) {
                retv[agent].push_back(freeNav::IdToPointi<2>(solution[t][agent]->index, dim));
                assert(retv[agent].size()-1 == t);
            }
        }
        // yz: remove way point when agent is stop
        for(int agent=0; agent<instance_sat.size(); agent++) {
            const freeNav::Pointi<2>& target = retv[agent].back();
            auto& path = retv[agent];
//            std::cout << "before prune" << path << std::endl;
            for(auto iter = path.end(); iter != path.begin(); ) {
                if(*(iter-2) == target) {
                    iter = path.erase(iter-1);
                } else {
                    break;
                }
            }
//            std::cout << "target " << target << " instance_sat[agent].second " << instance_sat[agent].second << std::endl;
//            std::cout << "start " << path.front() << " instance_sat[agent].first " << instance_sat[agent].first << std::endl;
            assert(path.front() == instance_sat[agent].first);
            assert(path.back() == instance_sat[agent].second);
//            std::cout << "after prune" << path << std::endl;
        }
        // debug: check whether resulted path meet ct
//        std::cout << "** lacam internal new path meet ct check, previous path size " << previous_paths.size()
//                  << " current new path size " <<  retv.size() << std::endl;
//        freeNav::Paths<2> new_paths = previous_paths;
//        new_paths.insert(new_paths.end(), retv.begin(), retv.end());
//        std::cout << " is previous path with new paths valid (validateSolution) ? " << freeNav::validateSolution<2>(new_paths) << std::endl;
//        for(int agent=0; agent<instance_sat.size(); agent++) {
//            const auto& current_path = retv[agent];
//            for(int t=1; t<current_path.size(); t++) {
//                int next_location = dim[0] * current_path[t][1] + current_path[t][0],
//                    curr_location = dim[0] * current_path[t-1][1] + current_path[t-1][0];
//                int next_timestep = t;
//                if (ct->constrained(next_location, next_timestep)) {
//                    cout << "CT check Agent " << agent << " have vertex conflict at " << freeNav::IdToPointi<2>(next_location, dim) << " at timestep " << next_timestep << endl;
//                    return {};
//                }
//                if(ct->constrained(curr_location, next_location, next_timestep)) {
//                    cout << "CT check  Agent " << agent << " have edge conflict from " << freeNav::IdToPointi<2>(curr_location, dim)  << " to " << freeNav::IdToPointi<2>(next_location, dim)  << " at timestep " << next_timestep << endl;
//                    return {};
//                }
//            }
//        }

//        if (!solution.empty()) {
//            for (const auto &previous_path : retv) {
//                MAPFPath path_eecbs;
//                for (int i = 0; i < previous_path.size(); i++) {
//                    path_eecbs.push_back(
//                            PathEntry(dim[0] * previous_path[i][1] + previous_path[i][0]));
//                }
//                ct->insert2CT(path_eecbs);
//            }
//        }
        return retv;
    }

}