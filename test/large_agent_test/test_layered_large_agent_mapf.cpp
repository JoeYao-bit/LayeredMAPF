//
// Created by yaozhuo on 2024/8/15.
//

#ifndef LAYEREDMAPF_TEST_LAYERED_LARGE_AGENT_MAPF_H
#define LAYEREDMAPF_TEST_LAYERED_LARGE_AGENT_MAPF_H

#include <gtest/gtest.h>
#include "common_interfaces.h"
#include "../../algorithm/LA-MAPF/laryered_large_agent_mapf.h"
#include "../../algorithm/LA-MAPF/CBS/laryered_large_agent_CBS.h"
#include "../../algorithm/LA-MAPF/CBS/large_agent_CBS.h"

//#include "../../algorithm/LA-MAPF/LaCAM/large_agent_lacam.h"

using namespace freeNav::LayeredMAPF::LA_MAPF;

template<typename AgentType>
void layeredLargeAgentMAPFTest(const std::string& file_path) {
    InstanceDeserializer<2, AgentType> deserializer;
    if (deserializer.loadInstanceFromFile(file_path, dim)) {
        std::cout << "load from path " << file_path << " success" << std::endl;
    } else {
        std::cout << "load from path " << file_path << " failed" << std::endl;
        return;
    }
    std::cout << "map scale = " << dim[0] << "*" << dim[1] << std::endl;
    gettimeofday(&tv_pre, &tz);


    double time_cost = (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3;

    auto paths = layeredLargeAgentMAPF<2, AgentType>(deserializer.getInstances(), deserializer.getAgents(),
                                     dim, is_occupied, CBS::LargeAgentCBS_func<2, CircleAgent<2> >, 60);

    gettimeofday(&tv_after, &tz);
    std::cout << (paths.empty() ? "success" : "failed") << " large agent mapf in " << time_cost << "ms " << std::endl;
}

TEST(test, layered_large_agent_CBS) {

    layeredLargeAgentMAPFTest<CircleAgent<2> >(map_test_config.at("crc_ins_path"));

}

#endif //LAYEREDMAPF_TEST_LAYERED_LARGE_AGENT_MAPF_H
