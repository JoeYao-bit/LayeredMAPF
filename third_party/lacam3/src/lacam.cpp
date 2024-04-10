#include "../include/lacam.hpp"
namespace LaCAM3 {

    Solution solve(const Instance &ins, int verbose, const Deadline *deadline,
                   int seed) {
        info(1, verbose, deadline, "pre-processing");
        auto planner = Planner(&ins, verbose, deadline, seed);
        return planner.solve();
    }

    freeNav::Paths<2> lacam3_MAPF(freeNav::DimensionLength *dim,
                                  const freeNav::IS_OCCUPIED_FUNC<2> &isoc,
                                  const freeNav::Instances<2> &instance_sat,
                                  CBS_Li::ConstraintTable *ct,
                                  int cutoff_time) {
        // arguments parser
        argparse::ArgumentParser program("lacam3", "0.1.0");
        program.add_argument("-m", "--map").help("map file");
        program.add_argument("-i", "--scen")
                .help("scenario file")
                .default_value(std::string(""));
        program.add_argument("-N", "--num").help("number of agents");
        program.add_argument("-s", "--seed")
                .help("seed")
                .default_value(std::string("0"));
        program.add_argument("-v", "--verbose")
                .help("verbose")
                .default_value(std::string("0"));
        program.add_argument("-t", "--time_limit_sec")
                .help("time limit sec")
                .default_value(std::string("3"));
        program.add_argument("-o", "--output")
                .help("output file")
                .default_value(std::string("./build/result.txt"));
        program.add_argument("-l", "--log_short")
                .default_value(false)
                .implicit_value(true);

        // solver parameters
        program.add_argument("--no-all")
                .help("turn off all options, i.e., vanilla LaCAM")
                .default_value(false)
                .implicit_value(true);
        program.add_argument("--no-star")
                .help("turn off the anytime part, i.e., usual LaCAM")
                .default_value(false)
                .implicit_value(true);
        program.add_argument("--random-insert-prob1")
                .help("probability of inserting the start node")
                .default_value(std::string("0.001"));
        program.add_argument("--random-insert-prob2")
                .help("probability of inserting a node after finding the goal")
                .default_value(std::string("0.01"));
        program.add_argument("--random-insert-init-node")
                .help("insert start node instead of random ones")
                .default_value(false)
                .implicit_value(true);
        program.add_argument("--no-swap")
                .help("turn off swap operation in PIBT")
                .default_value(false)
                .implicit_value(true);
        program.add_argument("--no-multi-thread")
                .help("turn off multi-threading")
                .default_value(false)
                .implicit_value(true);
        program.add_argument("--pibt-num")
                .help("used in Monte-Carlo configuration generation")
                .default_value(std::string("10"));
        program.add_argument("--no-scatter")
                .help("turn off SUO")
                .default_value(false)
                .implicit_value(true);
        program.add_argument("--scatter-margin")
                .help("allowing non-shortest paths in SUO")
                .default_value(std::string("10"));
        program.add_argument("--no-refiner")
                .help("turn off iterative refinement")
                .default_value(false)
                .implicit_value(true);
        program.add_argument("--refiner-num")
                .help("specify the number of refiners")
                .default_value(std::string("4"));
        program.add_argument("--recursive-rate")
                .help("specify the rate of the recursive call of LaCAM")
                .default_value(std::string("0.2"));
        program.add_argument("--recursive-time-limit")
                .help("time limit (sec) of the recursive call")
                .default_value(std::string("1"));
        program.add_argument("--checkpoints-duration")
                .help("for recording")
                .default_value(std::string("5"));
        // construct fake argument
        int fake_argc = 1;
        char arg0[] = "";
        char **fake_argv = new char *[fake_argc + 1]{arg0};

        try {
            program.parse_known_args(fake_argc, fake_argv);
        } catch (const std::runtime_error &err) {
            std::cerr << err.what() << std::endl;
            std::cerr << program;
            std::exit(1);
        }

        delete [] fake_argv;

        // setup instance
        const auto verbose = std::stoi(program.get<std::string>("verbose"));
        const auto time_limit_sec =
                std::stoi(program.get<std::string>("time_limit_sec"));
        const auto scen_name = program.get<std::string>("scen");
        const auto seed = std::stoi(program.get<std::string>("seed"));
//        const auto map_name = program.get<std::string>("map");
//        const auto output_name = program.get<std::string>("output");
//        const auto log_short = program.get<bool>("log_short");
//        const auto N = std::stoi(program.get<std::string>("num"));

//        const auto ins = scen_name.size() > 0 ? Instance(scen_name, map_name, N)
//                                              : Instance(map_name, N, seed);

        const auto ins = Instance(dim, isoc, instance_sat);

        if (!ins.is_valid(1)) { return {}; }

        // solver parameters
        const auto flg_no_all = program.get<bool>("no-all");
        Planner::FLG_SWAP = !program.get<bool>("no-swap") && !flg_no_all;
        Planner::FLG_STAR = !program.get<bool>("no-star") && !flg_no_all;
        Planner::FLG_MULTI_THREAD =
                !program.get<bool>("no-multi-thread") && !flg_no_all;
        Planner::PIBT_NUM =
                flg_no_all ? 1 : std::stoi(program.get<std::string>("pibt-num"));
        Planner::FLG_REFINER = !program.get<bool>("no-refiner") && !flg_no_all;
        Planner::REFINER_NUM = std::stoi(program.get<std::string>("refiner-num"));
        Planner::FLG_SCATTER = !program.get<bool>("no-scatter") && !flg_no_all;
        Planner::SCATTER_MARGIN =
                std::stoi(program.get<std::string>("scatter-margin"));
        Planner::RANDOM_INSERT_PROB1 =
                flg_no_all ? 0
                           : std::stof(program.get<std::string>("random-insert-prob1"));
        Planner::RANDOM_INSERT_PROB2 =
                flg_no_all ? 0
                           : std::stof(program.get<std::string>("random-insert-prob2"));
        Planner::FLG_RANDOM_INSERT_INIT_NODE =
                program.get<bool>("random-insert-init-node") && !flg_no_all;
        Planner::RECURSIVE_RATE =
                flg_no_all ? 0 : std::stof(program.get<std::string>("recursive-rate"));
        Planner::RECURSIVE_TIME_LIMIT =
                flg_no_all
                ? 0
                : std::stof(program.get<std::string>("recursive-time-limit")) * 1000;
        Planner::CHECKPOINTS_DURATION =
                std::stof(program.get<std::string>("checkpoints-duration")) * 1000;

        // solve
        const auto deadline = Deadline(time_limit_sec * 1000);
        const auto solution = solve(ins, verbose - 1, &deadline, seed);
        const auto comp_time_ms = deadline.elapsed_ms();

        // failure
        if (solution.empty()) {
            info(1, verbose, &deadline, "failed to solve");
            return {};
        }

        // check feasibility
        if (!is_feasible_solution(ins, solution, verbose)) {
            info(0, verbose, &deadline, "invalid solution");
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
        return retv;
    }

}