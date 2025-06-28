//
// Created by yaozhuo on 6/7/25.
//

#include "../algorithm/LA-MAPF/solvability_safe_guard.h"
#include "gtest/gtest.h"
#include "common_interfaces.h"

#include "../../algorithm/LA-MAPF/CBS/layered_large_agent_CBS.h"
#include "../../algorithm/LA-MAPF/CBS/large_agent_CBS.h"

#include "../../algorithm/LA-MAPF/LaCAM/layered_large_agent_LaCAM.h"
#include "../../algorithm/LA-MAPF/LaCAM/large_agent_lacam.h"
#include "../../algorithm/precomputation_for_decomposition.h"

using namespace freeNav::LayeredMAPF::LA_MAPF;

//TEST(simple_case, SAFE_GUARD) {
//    //
//}

SingleMapTestConfig<2> map_test_config_local = MAPFTestConfig_empty_48_48;

auto loader_local = TextMapLoader(map_test_config_local.at("map_path"), is_char_occupied1);


auto is_occupied_local = [](const Pointi<2> & pt) -> bool { return loader_local.isOccupied(pt); };


void testSolvabilitySafeguard(const LA_MAPF_FUNC<2, Pose<int, 2>> & mapf_func, double time_cost_limit = 30) {
    auto file_path_local = map_test_config_local.at("la_ins_path");

    auto dim_local = loader_local.getDimensionInfo();

    InstanceDeserializer<2> deserializer;
    if (deserializer.loadInstanceFromFile(file_path_local, dim_local)) {
        std::cout << "load from path " << file_path_local << " success" << std::endl;
    } else {
        std::cout << "load from path " << file_path_local << " failed" << std::endl;
        return;
    }


    std::cout << "map scale = " << dim_local[0] << "*" << dim_local[1] << std::endl;


    auto instances = deserializer.getTestInstance({15}, 1);

    std::cout << "instance.size() = " << instances.size() << std::endl;

    AgentPtrs<2> agent_ptrs;
    InstanceOrients<2> poses;

    agent_ptrs = instances.front().first;
    poses      = instances.front().second;

    std::vector<std::vector<int> > grid_visit_count_table;
    MSTimer mst;
    std::vector<LAMAPF_Path> solution = mapf_func(poses, agent_ptrs, dim_local, is_occupied_local,
                                                  nullptr,
                                                  grid_visit_count_table,
                                                  time_cost_limit, {}, nullptr, {}, {}, {}, nullptr
                                                  );
//    std::vector<LAMAPF_Path> solution = LaCAM::LargeAgentLaCAM_func<2>(poses, agent_ptrs, dim_local, is_occupied_local,
//                                                                       nullptr,
//                                                                       grid_visit_count_table);
    double time_cost = mst.elapsed() / 1e3;

    std::cout << "find raw problem solution " << !solution.empty() << " in " << time_cost << "ms" << std::endl;

    std::vector<std::set<int> > levels;// = {{0}, {1}, {2}};

    for(int i=0; i<agent_ptrs.size(); i++) {
        levels.push_back({i});
    }

    int failed_subproblem_id = rand() % levels.size();

    auto pre = std::make_shared<PrecomputationOfLAMAPF<2>>(poses, agent_ptrs, dim_local, is_occupied_local);

    SolvabilitySafeguard<2, Pose<int, 2>> safeguard(pre);
    std::cout << "this->isoc_ ex " << is_occupied_local(Pointi<2>()) << std::endl;

    bool success = safeguard.mergeSubproblemTillSolvable(levels, failed_subproblem_id,
            //LaCAM::LargeAgentLaCAM_func<2>,
                                                         mapf_func,
                                                         is_occupied_local, time_cost_limit);
    std::cout << "is merge success " << success << std::endl;

}

int main() {

    //testSolvabilitySafeguard<2>(LaCAM::LargeAgentLaCAM_func<2>);
    testSolvabilitySafeguard(CBS::LargeAgentCBS_func<2, Pose<int, 2>>, 30);

    return 0;

}

