//
// Created by yaozhuo on 6/25/25.
//

#include <gtest/gtest.h>
#include <random>
#include "../../algorithm/LA-MAPF/large_agent_instance_generator.h"
#include "../../freeNav-base/dependencies/2d_grid/text_map_loader.h"
#include "../../algorithm/LA-MAPF/CBS/large_agent_CBS.h"
#include "../../algorithm/LA-MAPF/LaCAM/large_agent_lacam.h"
#include "../../algorithm/LA-MAPF/CBS/constraint_avoidance_table.h"

#include "../../algorithm/LA-MAPF/LaCAM/layered_large_agent_LaCAM.h"
#include "../../algorithm/LA-MAPF/CBS/layered_large_agent_CBS.h"
//
#include "common_interfaces.h"
//
#include "../test_data.h"
#include "../../algorithm/break_loop_decomposition/biparition_decomposition.h"



void multiLoadAgentAndCompare(const SingleMapTestConfig<2>& map_file,
                              int count_of_test,
                              int maximum_agents,
                              int minimum_agents,
                              int agent_interval,
                              double time_limit = 60) {
    map_test_config = map_file;
    loader = TextMapLoader(map_test_config.at("map_path"), is_char_occupied1);

    //clearFile(map_test_config.at("la_comp_path"));

    InstanceDeserializer<2> deserializer;
    if(deserializer.loadInstanceFromFile(map_test_config.at("la_ins_path"), dim)) {
        std::cout << "load from path " << map_test_config.at("la_ins_path") << " success" << std::endl;
    } else {
        std::cout << "load from path " << map_test_config.at("la_ins_path") << " failed" << std::endl;
        return;
    }
    assert(!deserializer.getAgents().empty());
    std::vector<int> required_counts;
    for(int c=minimum_agents; c<=std::min(maximum_agents, (int)deserializer.getAgents().size()); c+=agent_interval) {
        required_counts.push_back(c);
    }
    auto agent_and_instances = deserializer.getTestInstance(required_counts, count_of_test);

    for (int i = 0; i < agent_and_instances.size(); i++) {

        // auto strs1 = LayeredLAMAPFCompare<2>(agent_and_instances[i].second,
        //                                     agent_and_instances[i].first,
        //                                     CBS::LargeAgentCBS_func<2>, //LaCAM::LargeAgentLaCAM_func<2>,
        //                                     std::string("CBS"),
        //                                     time_limit,
        //                                     false,
        //                                     4,
        //                                     true);

        // for (const auto &str : strs1) {
        //     std::cout << str << std::endl;
        // }
        // writeStrsToEndOfFile(strs1, map_test_config.at("la_comp_path"));

        auto strs2 = LayeredLAMAPFCompare<2>(agent_and_instances[i].second,
                                             agent_and_instances[i].first,
                                             LaCAM::LargeAgentLaCAM_func<2>,
                                             std::string("LaCAM"),
                                             time_limit,
                                             false,
                                             4,
                                             true);

        for (const auto &str : strs2) {
            std::cout << str << std::endl;
        }

        writeStrsToEndOfFile(strs2, map_test_config.at("la_comp_path"));
    }

}


int main() {
    // file_path, count_of_test, max_agent_count, min_agent_count, interval, max_sample
    std::vector<std::tuple<SingleMapTestConfig<2>, int, int, int, int> >
                                                                  map_configs = {
            //      {MAPFTestConfig_Paris_1_256,     1, 80, 10, 10}, // 80, 10, 10 / 20, 2, 2s
            //     {MAPFTestConfig_empty_48_48,     1, 50, 10, 10}, // 50, 10, 10
            //     {MAPFTestConfig_Berlin_1_256,    1, 80, 10, 10}, // 80, 10, 10
            //    {MAPFTestConfig_maze_128_128_10, 1, 60, 10, 10}, // 60, 10, 10

            // {MAPFTestConfig_den520d,         1, 100, 10, 10},// 100, 10, 10
            // {MAPFTestConfig_ost003d,         1, 100, 10, 10},// 100, 10, 10
            //  {MAPFTestConfig_Boston_2_256, 1, 70, 10, 10}, //  70, 10, 10
            //   {MAPFTestConfig_Sydney_2_256, 1, 70, 10, 10}, // 70, 10, 10

            {MAPFTestConfig_AR0044SR, 1, 20, 5, 5}, // 50, 5, 5
            {MAPFTestConfig_AR0203SR, 1, 40, 5, 5}, // 40, 5, 5
            {MAPFTestConfig_AR0072SR, 1, 30, 5, 5}, // 30, 5, 5
            {MAPFTestConfig_Denver_2_256, 1, 80, 10, 10}, // 80, 10, 10

            // not in test
            //        {MAPFTestConfig_Boston_2_256, 1, 20, 2, 2}, // ok
            //        {MAPFTestConfig_Sydney_2_256, 1, 20, 2, 2}, // ok
            //        {MAPFTestConfig_AR0044SR, 1, 20, 2, 2}, // ok
            //        {MAPFTestConfig_AR0203SR, 1, 20, 2, 2}, // ok
            //    {MAPFTestConfig_AR0072SR, 1, 20, 2, 2}, // ok
            //     {MAPFTestConfig_Denver_2_256, 1, 20, 2, 2} // ok

    };
    for(int i=0; i<100;i++)
    {
        std::cout << "global layered" << i << std::endl;
        for(const auto& file_config : map_configs) {
            multiLoadAgentAndCompare(std::get<0>(file_config),
                                     std::get<1>(file_config),
                                     std::get<2>(file_config),
                                     std::get<3>(file_config),
                                     std::get<4>(file_config),
                                     60);
        }
    }
    return 0;
}

