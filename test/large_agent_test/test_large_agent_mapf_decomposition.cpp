//
// Created by yaozhuo on 2024/8/10.
//

#include "../../algorithm/LA-MAPF/large_agent_instance_decomposition.h"

#include <gtest/gtest.h>

#include "common_interfaces.h"


using namespace freeNav::LayeredMAPF::LA_MAPF;


TEST(basic_test, LA_MAPF_decomposition) {
    const std::string file_path = map_test_config.at("crc_ins_path");
    loadInstanceAndDecomposition<CircleAgent<2> >(file_path);
}
