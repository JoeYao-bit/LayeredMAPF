//
// Created by yaozhuo on 6/17/25.
//

//
// Created by yaozhuo on 6/13/25.
//

#include "../algorithm/break_loop_decomposition/break_loop_decomposition.h"
#include "../algorithm/break_loop_decomposition/biparition_decomposition.h"

#include "common_interfaces.h"
#include "../algorithm/connectivity_graph_and_subprgraph.h"

#include <gtest/gtest.h>
#include <iostream>

using namespace freeNav::LayeredMAPF;

void compareLNSAndBiDecompose_MAPF(const SingleMapTestConfig<2>& map_file, int number_of_agents) {

    map_test_config = map_file;
    auto loader_local = TextMapLoader(map_test_config.at("map_path"), is_char_occupied1);

    auto dim_local = loader_local.getDimensionInfo();
    auto is_occupied_local = [&](const Pointi<2> & pt) -> bool { return loader_local.isOccupied(pt); };

    ScenarioLoader2D sl(map_test_config.at("scene_path").c_str());
    int count_of_experiments = sl.GetNumExperiments();

    std::vector<int> agent_in_instances = {number_of_agents};
    const auto& insts_ids = pickCasesFromScene(count_of_experiments, agent_in_instances, 1);


    InstancesS<2> istss;
    for(const auto& ids : insts_ids) {
        freeNav::Instances<2> ists;
        for(const int& id : ids) {
            const auto &experiment = sl.GetNthExperiment(id);
            Pointi<2> pt1({experiment.GetStartX(), experiment.GetStartY()});
            Pointi<2> pt2({experiment.GetGoalX(), experiment.GetGoalY()});
            freeNav::Instance<2> ist = {pt1, pt2};
            ists.push_back(ist);
        }
        istss.push_back(ists);
    }

    PrecomputationOfMAPF<2> pre(istss.front(), dim_local, is_occupied_local);

    auto start_t = clock();

//    auto bi_decompose = std::make_shared<LargeAgentMAPFInstanceDecomposition<2> >(agent_and_instances.front().second,
//                                                                                  agent_and_instances.front().first,
//                                                                                  dim_local,
//                                                                                  is_occupied_local,
//                                                                                  true,
//                                                                                  4,
//                                                                                  true);

    auto bi_decompose = std::make_shared<MAPFInstanceDecompositionBipartition<2> >(dim_local,
                                                                                   pre.connect_graphs_,
                                                                                   pre.agent_sub_graphs_,
                                                                                   pre.heuristic_tables_sat_,
                                                                                   pre.heuristic_tables_);

    auto now_t = clock();
    double total_time_cost = ((double)now_t - start_t)/CLOCKS_PER_SEC;

    bool is_bi_valid = MAPF_DecompositionValidCheckGridMap<2>(istss.front(), bi_decompose->all_clusters_,
                                                              dim_local, is_occupied_local);

//    bool is_bi_valid = bi_decompose->decompositionValidCheckGridMap(bi_decompose->all_clusters_);

    std::cout << "biparition finish in " << total_time_cost <<  "s, valid = " << is_bi_valid << std::endl;
    std::cout << "biparition max subproblem = " << getMaxLevelSize(bi_decompose->all_clusters_) << std::endl;


    start_t = clock();

    auto ns_decompose = std::make_shared<MAPFInstanceDecompositionBreakLoop<2> >(dim_local,
                                                                                 pre.connect_graphs_,
                                                                                 pre.agent_sub_graphs_,
                                                                                 pre.heuristic_tables_sat_,
                                                                                 100,
                                                                                 1000,
                                                                                 50,
                                                                                 1);

    ns_decompose->breakMaxLoopIteratively();

    now_t = clock();

    bool is_ns_valid = MAPF_DecompositionValidCheckGridMap<2>(istss.front(), ns_decompose->all_levels_, dim_local, is_occupied_local);


    total_time_cost = ((double)now_t - start_t)/CLOCKS_PER_SEC;
    std::cout << "ns finish in " << total_time_cost <<  "s, valid = " << is_ns_valid << std::endl;

    std::cout << "ns / bi max subproblem size(decomposition rate) = "
              << getMaxLevelSize(ns_decompose->all_levels_)   << "(" << (double)getMaxLevelSize(ns_decompose->all_levels_)/istss.front().size() << ") / "
              << getMaxLevelSize(bi_decompose->all_clusters_) << "(" << (double)getMaxLevelSize(bi_decompose->all_clusters_) / istss.front().size() << ")"
              << std::endl;

    std::cout << "ns / bi number of subproblem = " << ns_decompose->all_levels_.size() << " / "
              << bi_decompose->all_clusters_.size() << std::endl;

}



int main() {
//TEST(simple_test, LNS_decomposition) {

    compareLNSAndBiDecompose_MAPF(MAPFTestConfig_Paris_1_256,     1000);

//    compareLNSAndBiDecompose_MAPF(MAPFTestConfig_empty_48_48,     1000);

    //compareLNSAndBiDecompose_MAPF(MAPFTestConfig_Berlin_1_256,    1000);

    //compareLNSAndBiDecompose_MAPF(MAPFTestConfig_maze_128_128_10, 1000);

    //compareLNSAndBiDecompose_MAPF(MAPFTestConfig_den520d,          1000);

    //compareLNSAndBiDecompose_MAPF(MAPFTestConfig_ost003d,          100);

    //compareLNSAndBiDecompose_MAPF(MAPFTestConfig_Boston_2_256,     150);

    //compareLNSAndBiDecompose_MAPF(MAPFTestConfig_Sydney_2_256,     150);

    //compareLNSAndBiDecompose_MAPF(MAPFTestConfig_AR0044SR,         50);

    //compareLNSAndBiDecompose_MAPF(MAPFTestConfig_AR0203SR,         50);

    //compareLNSAndBiDecompose_MAPF(MAPFTestConfig_AR0072SR,         80);

    //compareLNSAndBiDecompose_MAPF(MAPFTestConfig_Denver_2_256,     150);

}

