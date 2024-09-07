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

#include "../../algorithm/LA-MAPF/LaCAM/layered_large_agent_LaCAM.h"
#include "../../algorithm/LA-MAPF/CBS/layered_large_agent_CBS.h"

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
    const CircleAgents<2>& agents = RandomCircleAgentsGenerator<2>(18,
                                                                   .4, 2.3,
                                                                   .1,
                                                                   dim);

    generateInstance<CircleAgent<2> > (agents,
                                       map_test_config.at("crc_ins_path"),
                                       CBS::LargeAgentCBS_func<2, CircleAgent<2> >,
                                               1e3);

//    generateInstance<CircleAgent<2>,
//        LaCAM::LargeAgentLaCAM<2, CircleAgent<2>, LaCAM::LargeAgentConstraintTableForLarge<2, CircleAgent<2> > > >
//        (agents, map_test_config.at("crc_ins_path"));

};

TEST(GenerateBlock_2DInstance, test)
{
    const BlockAgents_2D& agents = RandomBlock2DAgentsGenerator(3,
                                                                -1.4, -.2,
                                                                .2, 1.4,
                                                                .2, 1.4,
                                                                .1, dim);
    generateInstance<BlockAgent_2D>(agents,
                                    map_test_config.at("blk_ins_path"),
                                    CBS::LargeAgentCBS_func<2, BlockAgent_2D >,
                                    1e3);

};

bool use_circle_agent = true;

TEST(LoadCircleInstance_CBS, test)
{
    const std::string file_path = map_test_config.at("crc_ins_path");

//    loadInstanceAndPlanning<CircleAgent<2>,
//                            LaCAM::LargeAgentLaCAM<2,
//                            CircleAgent<2>, LaCAM::LargeAgentConstraintTableForLarge<2, CircleAgent<2> > > >(file_path);

    // LargeAgentConstraintTableForLarge
    // LargeAgentConstraintTable

//    loadInstanceAndPlanning<CircleAgent<2>, CBS::LargeAgentCBS<2, CircleAgent<2> > >(file_path, 30);

    loadInstanceAndPlanningLayeredLAMAPF<CircleAgent<2>>(CBS::LargeAgentCBS_func<2, CircleAgent<2> >,
                                                        file_path, 60, false);

};

TEST(LoadCircleInstance_LaCAM, test)
{
    const std::string file_path = map_test_config.at("crc_ins_path");

//    loadInstanceAndPlanning<CircleAgent<2>,
//                            LaCAM::LargeAgentLaCAM<2,
//                            CircleAgent<2>, LaCAM::LargeAgentConstraintTableForLarge<2, CircleAgent<2> > > >(file_path);

    // LargeAgentConstraintTableForLarge
    // LargeAgentConstraintTable

//    loadInstanceAndPlanning<CircleAgent<2>, CBS::LargeAgentCBS<2, CircleAgent<2> > >(file_path, 30);

    loadInstanceAndPlanningLayeredLAMAPF<CircleAgent<2>>(LaCAM::LargeAgentLaCAM_func<2, CircleAgent<2> >,
                                                         file_path, 60, false);

};

TEST(LoadBlock_2DInstance_CBS, test)
{
    const std::string file_path = map_test_config.at("blk_ins_path");

//    ok
//    loadInstanceAndPlanning<BlockAgent_2D,
//                            LaCAM::LargeAgentLaCAM<2,
//                            BlockAgent_2D, LaCAM::LargeAgentConstraintTable<2, BlockAgent_2D > > >(file_path);

//    loadInstanceAndPlanning<BlockAgent_2D,
//            CBS::LargeAgentCBS<2, BlockAgent_2D > >(file_path, 100);

    loadInstanceAndPlanningLayeredLAMAPF<BlockAgent_2D>(CBS::LargeAgentCBS_func<2, BlockAgent_2D>,
                                                        file_path,
                                                        60,
                                                        false);

};

TEST(LoadBlock_2DInstance_LaCAM, test)
{
    const std::string file_path = map_test_config.at("blk_ins_path");

//    ok
//    loadInstanceAndPlanning<BlockAgent_2D,
//                            LaCAM::LargeAgentLaCAM<2,
//                            BlockAgent_2D, LaCAM::LargeAgentConstraintTable<2, BlockAgent_2D > > >(file_path);

//    loadInstanceAndPlanning<BlockAgent_2D,
//            CBS::LargeAgentCBS<2, BlockAgent_2D > >(file_path, 100);

    loadInstanceAndPlanningLayeredLAMAPF<BlockAgent_2D>(LaCAM::LargeAgentLaCAM_func<2, BlockAgent_2D>,
                                                        file_path,
                                                        60,
                                                        false);

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

TEST(Multi_GenerateCircleInstance, test) {
    for(int i=0; i<100; i++) {
        const CircleAgents<2>& agents = RandomCircleAgentsGenerator<2>(16,
                                                                       .4, 1.4,
                                                                       .1,
                                                                       dim);

        generateInstance<CircleAgent<2> > (agents,
                                           map_test_config.at("crc_ins_path"),
                                           //CBS::LargeAgentCBS_func<2, CircleAgent<2> >,
                                           LaCAM::LargeAgentLaCAM_func<2, CircleAgent<2> >,
                                           1e4);
    }
}

TEST(Multi_GenerateBlock_2DInstance, test) {
    int count_of_test = 100;
    for (int i = 0; i < 2; i++) {
        for (int i = 0; i < count_of_test; i++) {
            const BlockAgents_2D &agents = RandomBlock2DAgentsGenerator(15,
                                                                        -1.4, -.2,
                                                                        .2, 1.4,
                                                                        .2, 1.4,
                                                                        .1, dim);

//            const BlockAgents_2D &agents = RandomBlock2DAgentsGenerator(5,
//                                                                        -1.4, -.2,
//                                                                        .2, 1.4,
//                                                                        .2, 1.4,
//                                                                        .1, dim);

            generateInstance<BlockAgent_2D>(agents,
                                            map_test_config.at("blk_ins_path"),
                                            //CBS::LargeAgentCBS_func<2, BlockAgent_2D>,
                                            LaCAM::LargeAgentLaCAM_func<2, BlockAgent_2D >,
                                            1e4);
        }
        for (int i = 0; i < count_of_test; i++) {
            const CircleAgents<2> &agents = RandomCircleAgentsGenerator<2>(15,
                                                                           .4, 2.3,
                                                                           .1,
                                                                           dim);

//            const CircleAgents<2> &agents = RandomCircleAgentsGenerator<2>(5,
//                                                                           .4, 1.4,
//                                                                           .1,
//                                                                           dim);

            generateInstance<CircleAgent<2> >(agents,
                                              map_test_config.at("crc_ins_path"),
                                              //CBS::LargeAgentCBS_func<2, CircleAgent<2> >,
                                              LaCAM::LargeAgentLaCAM_func<2, CircleAgent<2> >,
                                              1e4);
        }
    }
}

template<typename AgentType>
void Generator_test() {
    InstanceDeserializer<2, AgentType> deserializer;

    const std::string file_path = map_test_config.at("blk_ins_path");

    if(deserializer.loadInstanceFromFile(file_path, dim)) {
        std::cout << "load from path " << file_path << " success" << std::endl;
    } else {
        std::cout << "load from path " << file_path << " failed" << std::endl;
        return;
    }
    std::cout << " map scale " << dim[0] << "*" << dim[1] << std::endl;

    LargeAgentMAPF_InstanceGenerator<2, BlockAgent_2D> generator(deserializer.getAgents(), is_occupied, dim, 1e7);


    for(int i=0; i<deserializer.getAgents().size(); i++) {
        size_t start_id = generator.agent_sub_graphs_[i].start_node_id_;
        size_t target_id = generator.agent_sub_graphs_[i].target_node_id_;

//        assert(generator.agent_sub_graphs_[i].all_nodes_[start_id] != nullptr);
//        assert(generator.agent_sub_graphs_[i].all_nodes_[target_id] != nullptr);
    }

//    LargeAgentMAPFInstanceDecompositionPtr<2, AgentType> decomposer =
//            std::make_shared<LargeAgentMAPFInstanceDecomposition<2, AgentType> >(deserializer.getInstances(),
//                                                                                 deserializer.getAgents(), dim, is_occupied);

    CBS::LargeAgentCBS<2, AgentType> solver(deserializer.getInstances(), deserializer.getAgents(), dim, is_occupied);

    for(int i=0; i<deserializer.getAgents().size(); i++) {
        size_t start_id  = solver.agent_sub_graphs_[i].start_node_id_;
        size_t target_id = solver.agent_sub_graphs_[i].target_node_id_;

//        assert(solver.agent_sub_graphs_[i].all_nodes_[start_id] != nullptr);
//        assert(solver.agent_sub_graphs_[i].all_nodes_[target_id] != nullptr);
    }

}

TEST(Generator_BlockAgent_2D, test) {
    Generator_test<BlockAgent_2D>();
}

template<typename AgentType>
void Decomposition_test() {
    InstanceDeserializer<2, AgentType> deserializer;

    const std::string file_path = map_test_config.at("blk_ins_path");

    if(deserializer.loadInstanceFromFile(file_path, dim)) {
        std::cout << "load from path " << file_path << " success" << std::endl;
    } else {
        std::cout << "load from path " << file_path << " failed" << std::endl;
        return;
    }
    std::cout << " map scale " << dim[0] << "*" << dim[1] << std::endl;

    std::cout << "Instance: " << std::endl;
    std::vector<std::string> strs = deserializer.getTextString();
    for(const auto& str : strs) {
        std::cout << str << std::endl;
    }
    std::cout << std::endl;

    LargeAgentMAPFInstanceDecomposition<2, AgentType> decomposer =
            LargeAgentMAPFInstanceDecomposition<2, AgentType>(deserializer.getInstances(),
                                                              deserializer.getAgents(),
                                                              dim,
                                                              is_occupied);

    InstanceVisualization<AgentType>(deserializer.getAgents(),
                                     decomposer.getAllPoses(),
                                     deserializer.getInstances(),
                                     decomposer.grid_paths_,
                                     decomposer.agent_visited_grids_);

}

TEST(Decomposition_BlockAgent_2D, test) {
    Decomposition_test<BlockAgent_2D>();
}

TEST(Decomposition_Circle, test) {
    Decomposition_test<CircleAgent<2> >();
}