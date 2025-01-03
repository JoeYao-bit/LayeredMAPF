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

        Instance* ins = nullptr;
        if(external_graph_ptr != nullptr) {
//            std::cout << "LaCAM2 use external graph ptr" <<  std::endl;
            ins = new Instance(external_graph_ptr, instance_sat);
        } else {
            ins = new Instance(dim, isoc, instance_sat);
        }
        if (!ins->is_valid(1)) {
            std::cout << " result is invalid " << std::endl;
            delete ins;
            return {};
        }

        ins->ct = ct;
        // solve
        const Deadline deadline = Deadline(time_limit_sec * 1000);
        //std::cout << " ins->ct " << ins.ct << std::endl;
        auto additional_info = std::string("");
        const Solution solution = solve(*ins, additional_info, verbose - 1, &deadline, &MT);
        //const double comp_time_ms = deadline.elapsed_ms();

        // failure
        if (solution.empty()) {
            //info(1, verbose, "failed to solve");
            std::cout << "failed to solve" << std::endl;
            delete ins;
            return {};
        } else {
            //std::cout << " solution size " << solution[0].size() << std::endl;
        }

        // check feasibility
        if (!is_feasible_solution(*ins, solution, verbose)) {
            //info(0, verbose, "invalid solution");
            std::cout << " invalid solution " << std::endl;
            delete ins;
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
            for(auto iter = path.end(); iter != path.begin(); ) {
                if(*(iter-2) == target) {
                    iter = path.erase(iter-1);
                } else {
                    break;
                }
            }
            assert(path.front() == instance_sat[agent].first);
            assert(path.back() == instance_sat[agent].second);
        }
        delete ins;
        return retv;
    }

}