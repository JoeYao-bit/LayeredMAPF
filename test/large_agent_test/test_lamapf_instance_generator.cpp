//
// Created by yaozhuo on 2024/6/26.
//
#include <gtest/gtest.h>
#include <random>
#include "../../algorithm/LA-MAPF/large_agent_instance_generator.h"
#include "../../freeNav-base/visualization/canvas/canvas.h"
#include "../../freeNav-base/dependencies/2d_grid/text_map_loader.h"
#include "../../algorithm/LA-MAPF/CBS/large_agent_CBS.h"
#include "../../algorithm/LA-MAPF/LaCAM/large_agent_lacam.h"
#include "../../algorithm/LA-MAPF/CBS/constraint_avoidance_table.h"
#include "common_interfaces.h"

#include "../test_data.h"

using namespace freeNav;
using namespace freeNav::LayeredMAPF;
using namespace freeNav::LayeredMAPF::LA_MAPF;

TEST(CircleAgentTest, generator_test) {
//    std::random_device rd;
//    std::mt19937 *MT = new std::mt19937(rd());
//    std::cout << get_random_float(MT, .1, 2.4) << std::endl;
//    delete MT;

    CircleAgents<2> agents = RandomCircleAgentsGenerator<2>(5,
                                                            .4, 2.3,
                                                            .1, nullptr);
    for(int i=0; i<agents.size(); i++) {
        const auto& agent = agents[i];
        std::cout << agent << std::endl;
    }
}

TEST(Block2DAgentTest, generator_test) {

    const BlockAgents_2D& agents = RandomBlock2DAgentsGenerator(5,
                                                          -.4, -.2,
                                                          .2, 2.,
                                                          .2, 1.4,
                                                          .1, nullptr);


    for(int i=0; i<agents.size(); i++) {
        const auto& agent = agents[i];
        std::cout << agent << std::endl;
    }
}

int canvas_size_x = 1000, canvas_size_y = 700;


TEST(GenerateCircleInstance, test)
{
    const CircleAgents<2>& agents = RandomCircleAgentsGenerator<2>(20,
                                                                   .4, 2.3,
                                                                   .1,
                                                                   dim);

    generateInstance<CircleAgent<2>, CBS::LargeAgentCBS<2, CircleAgent<2> > > (agents, map_test_config.at("crc_ins_path"));

//    generateInstance<CircleAgent<2>,
//        LaCAM::LargeAgentLaCAM<2, CircleAgent<2>, LaCAM::LargeAgentConstraintTableForLarge<2, CircleAgent<2> > > >
//        (agents, map_test_config.at("crc_ins_path"));

};

TEST(GenerateBlock_2DInstance, test)
{
    const BlockAgents_2D& agents = RandomBlock2DAgentsGenerator(20,
                                                                -2.4, -.2,
                                                                .2, 2.4,
                                                                .2, 2.4,
                                                                .1, dim);
    generateInstance<BlockAgent_2D, CBS::LargeAgentCBS<2, BlockAgent_2D > >(agents, map_test_config.at("blk_ins_path"));

};

bool use_circle_agent = true;

TEST(LoadCircleInstance, test)
{
    const std::string file_path = map_test_config.at("crc_ins_path");

//    loadInstanceAndPlanning<CircleAgent<2>,
//                            LaCAM::LargeAgentLaCAM<2,
//                            CircleAgent<2>, LaCAM::LargeAgentConstraintTableForLarge<2, CircleAgent<2> > > >(file_path);

    // LargeAgentConstraintTableForLarge
    // LargeAgentConstraintTable

//    loadInstanceAndPlanning<CircleAgent<2>, CBS::LargeAgentCBS<2, CircleAgent<2> > >(file_path, 30);

    loadInstanceAndPlanningLayeredCBS<CircleAgent<2> >(file_path, 60, false);

};

TEST(LoadBlock_2DInstance, test)
{
    const std::string file_path = map_test_config.at("blk_ins_path");

//    ok
//    loadInstanceAndPlanning<BlockAgent_2D,
//                            LaCAM::LargeAgentLaCAM<2,
//                            BlockAgent_2D, LaCAM::LargeAgentConstraintTable<2, BlockAgent_2D > > >(file_path);

//    loadInstanceAndPlanning<BlockAgent_2D,
//            CBS::LargeAgentCBS<2, BlockAgent_2D > >(file_path, 100);

    loadInstanceAndPlanningLayeredCBS<BlockAgent_2D >(file_path, 60, false);

};

TEST(resize, test)
{
    std::vector<int> values = {1, 2, 3};
    values.resize(5, 10);
    for(const auto& value : values) {
        std::cout << value << " ";
    }
    std::cout << std::endl;
    // 1 2 3 10 10
}

TEST(max_size_t, test) {
    std::cout << "MAX<size_t> = " << MAX<size_t> << std::endl;
}