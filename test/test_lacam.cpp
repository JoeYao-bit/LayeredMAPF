//
// Created by yaozhuo on 2023/12/27.
//
#include "lacam/include/lacam.hpp"
#include "../test/test_data.h"
#include "../freeNav-base/dependencies/memory_analysis.h"
#include "gtest/gtest.h"


// MAPFTestConfig_random_32_32_20
// MAPFTestConfig_maze_32_32_2
// MAPFTestConfig_maze_32_32_4
// MAPFTestConfig_den312d
// MAPFTestConfig_Berlin_1_256
// MAPFTestConfig_Paris_1_256
// MAPFTestConfig_warehouse_10_20_10_2_1 // LaCAM is run out of memory (6G) when agent is 400
// MAPFTestConfig_den520d
// MAPFTestConfig_empty_32_32

auto map_test_config = freeNav::LayeredMAPF::MAPFTestConfig_empty_16_16;


using namespace LaCAM;

const auto scen_filename = "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/Multi-Agent Path-Finding (MAPF) Benchmarks/mapf-scen-random/scen-random/random-32-32-10-random-1.scen";
const auto map_filename = "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/Multi-Agent Path-Finding (MAPF) Benchmarks/mapf-map/random-32-32-10.map";

TEST(planner, solve) {

const auto ins = Instance(scen_filename, map_filename, 3);

    auto solution = solve(ins);
    assert(is_feasible_solution(ins, solution));
}



TEST(Memory, Analysis)
{
    int current_pid = GetCurrentPid(); // or you can set a outside program pid
    float cpu_usage_ratio = GetCpuUsageRatio(current_pid);
    float memory_usage = GetMemoryUsage(current_pid);

    while (true)
    {
        std::cout << "current pid: " << current_pid << std::endl;
        std::cout << "cpu usage ratio: " << cpu_usage_ratio * 100 << "%" << std::endl;
        std::cout << "memory usage: " << memory_usage << "MB" << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

TEST(LaCAM, Instance)
{
    // setup instance
    const auto verbose = 1;//std::stoi(program.get<std::string>("verbose"));
    const auto time_limit_sec = 60;
            //std::stoi(program.get<std::string>("time_limit_sec"));
    const std::string scen_name = map_test_config.at("scene_path");
    const auto seed = 0;
    auto MT = std::mt19937(seed);
    const auto map_name = map_test_config.at("map_path");
    const auto output_name = "./build/result.txt";
    const auto log_short = true;
    const int N = atoi((map_test_config.at("agent_num")).c_str());
    const auto ins = scen_name.size() > 0 ? Instance(scen_name, map_name, N)
                                          : Instance(map_name, &MT, N);

    if (!ins.is_valid(1)) {
        std::cout << " result is invalid " << std::endl;
        return;
    }

    // solve
    const Deadline deadline = Deadline(time_limit_sec * 1000);
    const Solution solution = solve(ins, verbose - 1, &deadline, &MT);
    const double comp_time_ms = deadline.elapsed_ms();

    // failure
    if (solution.empty()) {
        info(1, verbose, "failed to solve");
        std::cout << "failed to solve" << std::endl;
    } else {
        std::cout << " solution size " << solution[0].size() << std::endl;
    }
    // check feasibility
    if (!is_feasible_solution(ins, solution, verbose)) {
        info(0, verbose, "invalid solution");
        return;
    }


    // post processing
    print_stats(verbose, ins, solution, comp_time_ms);
    make_log(ins, solution, output_name, comp_time_ms, map_name, seed, log_short);
    return;
}