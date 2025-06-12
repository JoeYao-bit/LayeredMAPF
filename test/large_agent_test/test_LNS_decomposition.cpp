//
// Created by yaozhuo on 6/13/25.
//

#include "../algorithm/neighbourhood_search_decomposition/neighborhood_search_decomposition.h"
#include "common_interfaces.h"

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

    auto bipartition_decompose = std::make_shared<LargeAgentMAPFInstanceDecomposition<2> >(agent_and_instances.front().second,
                                                                                           agent_and_instances.front().first,
                                                                                           dim_local,
                                                                                           is_occupied_local,
                                                                                        true,
                                                                                        4,
                                                                                        true);

    auto now_t = clock();
    double total_time_cost = ((double)now_t - start_t)/CLOCKS_PER_SEC;

    bool is_bi_valid = bipartition_decompose->decompositionValidCheckGridMap(bipartition_decompose->all_clusters_);

    std::cout << "biparition finish in " << total_time_cost <<  "s, valid = " << is_bi_valid << std::endl;
    std::cout << "biparition max subproblem = " << getMaxLevelSize(bipartition_decompose->all_clusters_) << std::endl;

    start_t = clock();

    auto ns_decompose = std::make_shared<MAPFInstanceDecompositionLNS<2> >(dim_local,
                                                                           bipartition_decompose->connect_graphs_,
                                                                           bipartition_decompose->agent_sub_graphs_
                                                                           );

    now_t = clock();

    bool is_ns_valid = bipartition_decompose->decompositionValidCheckGridMap(ns_decompose->all_levels_);

    total_time_cost = ((double)now_t - start_t)/CLOCKS_PER_SEC;

    std::cout << "ns finish in " << total_time_cost <<  "s, valid = " << is_ns_valid << std::endl;

    std::cout << "ns max subproblem = " << getMaxLevelSize(ns_decompose->all_levels_) << std::endl;

}

TEST(simple_test, LNS_decomposition) {

    compareLNSAndBiparitionIteratively(MAPFTestConfig_AR0011SR, 20);

}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}