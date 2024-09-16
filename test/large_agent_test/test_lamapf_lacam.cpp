//
// Created by yaozhuo on 2024/6/21.
//


#include <gtest/gtest.h>
#include <sstream>
#include <string>
#include "../../algorithm/LA-MAPF/circle_shaped_agent.h"
#include "../../algorithm/LA-MAPF/block_shaped_agent.h"

#include "../../algorithm/LA-MAPF/LaCAM/large_agent_lacam.h"
#include "../../algorithm/LA-MAPF/LaCAM/large_agent_lacam.h"

#include "../../freeNav-base/visualization/canvas/canvas.h"
#include "../../freeNav-base/dependencies/2d_grid/text_map_loader.h"
#include "../test_data.h"
#include "common_interfaces.h"

using namespace freeNav;
using namespace freeNav::LayeredMAPF;
using namespace freeNav::LayeredMAPF::LA_MAPF;


TEST(AgentSubGraph, lacam_test) {

//    // fake instances
//    InstanceOrients<2> instances = {
//            {{{5, 3}, 0},  {{23, 22},0} },
//            {{{9, 2}, 0},  {{5, 22}, 0}},
//            {{{2, 5}, 0},  {{17, 22}, 3}},
//            {{{5, 7}, 0},  {{20, 23}, 2}},
//            {{{30, 7}, 0}, {{2, 23}, 2}}
//    };
//
//    const Pointf<2> min_pt_0{-.4, -.4},  max_pt_0{.4, .4},
//                    min_pt_1{-.6, -.4},  max_pt_1{1., .4},
//                    min_pt_2{-.3, -1.2}, max_pt_2{1., 1.2},
//                    min_pt_3{-.3, -1.2}, max_pt_3{1., 1.2},
//                    min_pt_4{-.3, -.6},  max_pt_4{1.2, .6};
//
//    // NOTICE: initialize pt in constructor cause constant changed, unknown reason
//    const BlockAgents_2D agents({
//                                        BlockAgent_2D({-.4, -.4},  {.4, .4}, 0, dim),
//                                        BlockAgent_2D({-.6, -.4},  {1., .4}, 1, dim),
//                                        BlockAgent_2D({-.3, -.6}, {.8, .6}, 2, dim),
//                                        BlockAgent_2D({-.3, -.6}, {.8, .6}, 3, dim),
//                                        BlockAgent_2D({-.3, -.6},  {1.2, .6}, 4, dim)
//
//                                });

    const std::string file_path = map_test_config.at("la_ins_path");

    InstanceDeserializer<2> deserializer;
    if (deserializer.loadInstanceFromFile(file_path, dim)) {
        std::cout << "load from path " << file_path << " success" << std::endl;
    } else {
        std::cout << "load from path " << file_path << " failed" << std::endl;
        return;
    }
    std::cout << "map scale = " << dim[0] << "*" << dim[1] << std::endl;

    startLargeAgentMAPFTest<2, LaCAM::LargeAgentLaCAM<2, LaCAM::LargeAgentConstraints<2> > >(deserializer.getAgents(),
                                                                               deserializer.getInstances());

}

TEST(sort, test) {
    std::vector<std::pair<size_t, int> > ids = {{0, 6}, {1, 3}, {2, 4}, {3, 5}, {4, 1}, {5, 0}};
    std::sort(ids.begin(), ids.end(), [&](const std::pair<size_t, int>& v, const std::pair<size_t, int>& u) {
        return v.second < u.second;
    });

    for(const auto& temp_pair : ids) {
        std::cout << "{" << temp_pair.first << ", " << temp_pair.second << "} ";
    }
    std::cout << std::endl;
    // {5, 0} {4, 1} {1, 3} {2, 4} {3, 5} {0, 6}
}

TEST(division, test) {
    std::cout << 0/2 << " " << 1/2 << " " << 2/2 << " " << 3/2 << std::endl;
    std::cout << 0/2 << " " << 1/2 << " " << 2/2 << " " << 3/2 << " " << 4/2 << " " << 5/2 << std::endl;

}







