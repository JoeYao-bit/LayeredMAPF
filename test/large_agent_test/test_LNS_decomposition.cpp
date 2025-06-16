//
// Created by yaozhuo on 6/13/25.
//

#include "../algorithm/neighbourhood_search_decomposition/neighborhood_search_decomposition.h"
#include "common_interfaces.h"
#include "../algorithm/connectivity_graph_and_subprgraph.h"

#include <gtest/gtest.h>
#include <iostream>

using namespace freeNav::LayeredMAPF::LA_MAPF;

void compareLNSAndBiparitionIteratively(const SingleMapTestConfig<2>& map_file, int number_of_agents) {

    map_test_config = map_file;
    auto loader_local = TextMapLoader(map_test_config.at("map_path"), is_char_occupied1);

    auto dim_local = loader_local.getDimensionInfo();
    auto is_occupied_local = [&](const Pointi<2> & pt) -> bool { return loader_local.isOccupied(pt); };

    //clearFile(map_test_config.at("la_comp_path"));

    InstanceDeserializer<2> deserializer;
    if(deserializer.loadInstanceFromFile(map_test_config.at("la_ins_path"), dim)) {
        std::cout << "load from path " << map_test_config.at("la_ins_path") << " success" << std::endl;
    } else {
        std::cout << "load from path " << map_test_config.at("la_ins_path") << " failed" << std::endl;
        return;
    }
    assert(!deserializer.getAgents().empty());
    std::vector<int> required_counts = {number_of_agents};

    auto agent_and_instances = deserializer.getTestInstance(required_counts, 1);

    auto start_t = clock();

    auto bi_decompose = std::make_shared<LargeAgentMAPFInstanceDecomposition<2> >(agent_and_instances.front().second,
                                                                                  agent_and_instances.front().first,
                                                                                  dim_local,
                                                                                  is_occupied_local,
                                                                                  true,
                                                                                  4,
                                                                                  true);

    auto now_t = clock();
    double total_time_cost = ((double)now_t - start_t)/CLOCKS_PER_SEC;

    bool is_bi_valid = bi_decompose->decompositionValidCheckGridMap(bi_decompose->all_clusters_);

    std::cout << "biparition finish in " << total_time_cost <<  "s, valid = " << is_bi_valid << std::endl;
    std::cout << "biparition max subproblem = " << getMaxLevelSize(bi_decompose->all_clusters_) << std::endl;

    PrecomputationOfLAMAPF<2> pre(agent_and_instances.front().second,
                                  agent_and_instances.front().first,
                                  dim_local,
                                  is_occupied_local);

    start_t = clock();

    auto ns_decompose = std::make_shared<MAPFInstanceDecompositionLNS<2> >(dim_local,
                                                                           pre.connect_graphs_,
                                                                           pre.agent_sub_graphs_,
                                                                           pre.heuristic_tables_sat_,
                                                                           2,
                                                                           1000,
                                                                           50,
                                                                           1);

    ns_decompose->breakMaxLoopIteratively();

    now_t = clock();

    bool is_ns_valid = bi_decompose->decompositionValidCheckGridMap(ns_decompose->all_levels_);

    total_time_cost = ((double)now_t - start_t)/CLOCKS_PER_SEC;
    std::cout << "ns finish in " << total_time_cost <<  "s, valid = " << is_ns_valid << std::endl;

    std::cout << "ns / bi max subproblem size(decomposition rate) = "
              << getMaxLevelSize(ns_decompose->all_levels_)   << "(" << (double)getMaxLevelSize(ns_decompose->all_levels_)/agent_and_instances.front().second.size() << ") / "
              << getMaxLevelSize(bi_decompose->all_clusters_) << "(" << (double)getMaxLevelSize(bi_decompose->all_clusters_) / agent_and_instances.front().second.size() << ")"
              << std::endl;

    std::cout << "ns / bi number of subproblem = " << ns_decompose->all_levels_.size() << " / "
              << bi_decompose->all_clusters_.size() << std::endl;

}
// pass
TEST(getMaxLevel, test) {
//int main() {
    std::vector<std::set<int> > all_levels = {{7,8,9,10}, {0,1}, {2}, {3}, {4,5,6}};

    for(int i=0; i<all_levels.size(); i++) {
        auto retv_pair = getMaxLevel(all_levels, i);
        std::cout << i << " th largest level: " << retv_pair.first << ", " << retv_pair.second << std::endl;
    }
}


// {MAPFTestConfig_Paris_1_256,     1, 80, 10, 10}, // 80, 10, 10 / 20, 2, 2s
// {MAPFTestConfig_empty_48_48,     1, 50, 10, 10}, // 50, 10, 10
// {MAPFTestConfig_Berlin_1_256,    1, 80, 10, 10}, // 80, 10, 10
// {MAPFTestConfig_maze_128_128_10, 1, 60, 10, 10}, // 60, 10, 10

// {MAPFTestConfig_den520d,         1, 100, 10, 10},// 100, 10, 10
// {MAPFTestConfig_ost003d,         1, 100, 10, 10},// 100, 10, 10
// {MAPFTestConfig_Boston_2_256, 1, 70, 10, 10}, //  70, 10, 10
// {MAPFTestConfig_Sydney_2_256, 1, 70, 10, 10}, // 70, 10, 10

// {MAPFTestConfig_AR0044SR, 1, 20, 5, 5}, // 50, 5, 5
// {MAPFTestConfig_AR0203SR, 1, 40, 5, 5}, // 40, 5, 5
// {MAPFTestConfig_AR0072SR, 1, 30, 5, 5}, // 30, 5, 5
// {MAPFTestConfig_Denver_2_256, 1, 80, 10, 10}, // 80, 10, 10

int main() {
//TEST(simple_test, LNS_decomposition) {

//    // ns / bi max subproblem size(decomposition rate) = 1(0.00666667) / 1(0.00666667)
//    compareLNSAndBiparitionIteratively(MAPFTestConfig_Paris_1_256,     150);

    // ns / bi max subproblem size(decomposition rate) = 1(0.0166667) / 15(0.25)
    //compareLNSAndBiparitionIteratively(MAPFTestConfig_empty_48_48,     60);

    // ns / bi max subproblem size(decomposition rate) = 1(0.00666667) / 1(0.00666667)
    //compareLNSAndBiparitionIteratively(MAPFTestConfig_Berlin_1_256,    150);

    // ns / bi max subproblem size(decomposition rate) = 15(0.15) / 28(0.28)
    compareLNSAndBiparitionIteratively(MAPFTestConfig_maze_128_128_10, 1000);

//    // ns / bi max subproblem size(decomposition rate) = 1(0.00666667) / 1(0.00666667)
//    compareLNSAndBiparitionIteratively(MAPFTestConfig_den520d,          150);
//
//    // ns / bi max subproblem size(decomposition rate) = 1(0.01) / 2(0.02)
//    compareLNSAndBiparitionIteratively(MAPFTestConfig_ost003d,          100);
//
//    // ns / bi max subproblem size(decomposition rate) = 1(0.00666667) / 1(0.00666667)
//    compareLNSAndBiparitionIteratively(MAPFTestConfig_Boston_2_256,     150);
//
//    // ns / bi max subproblem size(decomposition rate) = 1(0.00666667) / 1(0.00666667)
//    compareLNSAndBiparitionIteratively(MAPFTestConfig_Sydney_2_256,     150);

    // ns / bi max subproblem size(decomposition rate) = 1(0.02) / 1(0.02)
    //compareLNSAndBiparitionIteratively(MAPFTestConfig_AR0044SR,         50);

    // ns / bi max subproblem size(decomposition rate) = 5(0.1) / 5(0.1)
    //compareLNSAndBiparitionIteratively(MAPFTestConfig_AR0203SR,         50);

    // ns / bi max subproblem size(decomposition rate) = 1(0.0125) / 1(0.0125)
    //compareLNSAndBiparitionIteratively(MAPFTestConfig_AR0072SR,         80);

    // ns / bi max subproblem size(decomposition rate) = 1(0.00666667) / 1(0.00666667)
    //compareLNSAndBiparitionIteratively(MAPFTestConfig_Denver_2_256,     150);

}

//int main(int argc, char **argv) {
//    ::testing::InitGoogleTest(&argc, argv);
//    return RUN_ALL_TESTS();
//}