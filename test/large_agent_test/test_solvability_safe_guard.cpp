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


template<Dimension N, typename HyperNodeType>
void testSolvabilitySafeguard(const LA_MAPF_FUNC<2> & mapf_func) {
    auto file_path_local = map_test_config_local.at("la_ins_path");

    auto dim_local = loader_local.getDimensionInfo();

    InstanceDeserializer<N> deserializer;
    if (deserializer.loadInstanceFromFile(file_path_local, dim_local)) {
        std::cout << "load from path " << file_path_local << " success" << std::endl;
    } else {
        std::cout << "load from path " << file_path_local << " failed" << std::endl;
        return;
    }


    std::cout << "map scale = " << dim_local[0] << "*" << dim_local[1] << std::endl;


    LargeAgentMAPFInstanceDecompositionPtr<N, HyperNodeType> decomposer_ptr = nullptr;
    std::vector<std::vector<int> > grid_visit_count_table;

    auto instances = deserializer.getTestInstance({15}, 1);

    std::cout << "instance.size() = " << instances.size() << std::endl;

    AgentPtrs<2> agent_ptrs;
    InstanceOrients<2> poses;

    agent_ptrs = instances.front().first;
    poses      = instances.front().second;
    gettimeofday(&tv_pre, &tz);
    std::vector<LAMAPF_Path> solution = mapf_func(poses, agent_ptrs, dim_local, is_occupied_local,
                                                  nullptr,
                                                  grid_visit_count_table,
                                                  30, {}, nullptr, {}, {}, {}, nullptr
                                                  );
//    std::vector<LAMAPF_Path> solution = LaCAM::LargeAgentLaCAM_func<2>(poses, agent_ptrs, dim_local, is_occupied_local,
//                                                                       nullptr,
//                                                                       grid_visit_count_table);
    gettimeofday(&tv_after, &tz);
    double time_cost = (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3;

    std::cout << "find raw problem solution " << !solution.empty() << " in " << time_cost << "ms" << std::endl;

    std::vector<std::set<int> > levels;// = {{0}, {1}, {2}};

    for(int i=0; i<agent_ptrs.size(); i++) {
        levels.push_back({i});
    }

    int failed_subproblem_id = rand() % levels.size();

    SolvabilitySafeguard<N> safeguard(poses, agent_ptrs, dim_local, is_occupied_local);
    std::cout << "this->isoc_ ex " << is_occupied_local(Pointi<N>()) << std::endl;

    bool success = safeguard.mergeSubproblemTillSolvable(levels, failed_subproblem_id,
            //LaCAM::LargeAgentLaCAM_func<2>,
                                                         mapf_func,
                                                         is_occupied_local, 30);
    std::cout << "is merge success " << success << std::endl;

}

int main() {

    //testSolvabilitySafeguard<2>(LaCAM::LargeAgentLaCAM_func<2>);
    testSolvabilitySafeguard<2, HyperGraphNodeDataRaw<2>>(CBS::LargeAgentCBS_func<2>);

    return 0;

}

