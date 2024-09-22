////
//// Created by yaozhuo on 2024/6/26.
////
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
//
#include "common_interfaces.h"
//
#include "../test_data.h"
//
//using namespace freeNav;
//using namespace freeNav::LayeredMAPF;
//using namespace freeNav::LayeredMAPF::LA_MAPF;
//
TEST(CircleAgentTest, generator_test) {
//    std::random_device rd;
//    std::mt19937 *MT = new std::mt19937(rd());
//    std::cout << get_random_float(MT, .1, 2.4) << std::endl;
//    delete MT;

    AgentPtrs<2> agents = RandomCircleAgentsGenerator<2>(5,
                                                            .4, 2.3,
                                                            .1, nullptr);
    for(int i=0; i<agents.size(); i++) {
        const auto& agent = agents[i];
        std::cout << *agent << std::endl;
    }
}

TEST(Block2DAgentTest, generator_test) {

    const AgentPtrs<2>& agents = RandomBlock2DAgentsGenerator(5,
                                                          -.4, -.2,
                                                          .2, 2.,
                                                          .2, 1.4,
                                                          .1, nullptr);


    for(int i=0; i<agents.size(); i++) {
        const auto& agent = agents[i];
        std::cout << *agent << std::endl;
    }
}


int canvas_size_x = 1000, canvas_size_y = 700;

// generateInstanceAndDecomposition

TEST(GenerateCircleInstance_decomposition, test)
{
    const AgentPtrs<2>& agents = RandomCircleAgentsGenerator<2>(30,
                                                                .4, 2.3,
                                                                .1,
                                                                dim);

    generateInstanceAndDecomposition<2> (agents,
                                        map_test_config.at("la_ins_path"),
                                        CBS::LargeAgentCBS_func<2>,
                                        1e4, true, true);

//    generateInstance<CircleAgent<2>,
//        LaCAM::LargeAgentLaCAM<2, CircleAgent<2>, LaCAM::LargeAgentConstraintTableForLarge<2, CircleAgent<2> > > >
//        (agents, map_test_config.at("la_ins_path"));

};

TEST(GenerateBlockInstance_decomposition, test)
{
    const AgentPtrs<2>& agents = RandomBlock2DAgentsGenerator(40,
                                                              -2.4, -.2,
                                                              .2, 2.4,
                                                              .2, 2.4,
                                                              .1, dim);

    generateInstanceAndDecomposition<2> (agents,
                                         map_test_config.at("la_ins_path"),
                                         CBS::LargeAgentCBS_func<2>,
                                         1e4, true, true);

//    generateInstance<CircleAgent<2>,
//        LaCAM::LargeAgentLaCAM<2, CircleAgent<2>, LaCAM::LargeAgentConstraintTableForLarge<2, CircleAgent<2> > > >
//        (agents, map_test_config.at("la_ins_path"));

};

TEST(GenerateMixedInstance_decomposition, test)
{
    AgentPtrs<2> agents = RandomMixedAgentsGenerator(10,
                                                     .4, 2.3,
                                                     10,
                                                     -2.4, -.2,
                                                     .2, 2.4,
                                                     .2, 2.4,
                                                     .1, dim);
    generateInstanceAndDecomposition<2> (agents,
                                         map_test_config.at("la_ins_path"),
                                         CBS::LargeAgentCBS_func<2>,
                                         5e4, true, true);

//    generateInstance<CircleAgent<2>,
//        LaCAM::LargeAgentLaCAM<2, CircleAgent<2>, LaCAM::LargeAgentConstraintTableForLarge<2, CircleAgent<2> > > >
//        (agents, map_test_config.at("la_ins_path"));

};

TEST(GenerateCircleInstance, test)
{
    const AgentPtrs<2>& agents = RandomCircleAgentsGenerator<2>(40,
                                                                   .4, 2.3,
                                                                   .1,
                                                                   dim);

    generateInstanceAndPlanning<2> (agents,
                                       map_test_config.at("la_ins_path"),
                                       CBS::LargeAgentCBS_func<2>,
                                               1e4, true, true);

//    generateInstance<CircleAgent<2>,
//        LaCAM::LargeAgentLaCAM<2, CircleAgent<2>, LaCAM::LargeAgentConstraintTableForLarge<2, CircleAgent<2> > > >
//        (agents, map_test_config.at("la_ins_path"));

};


TEST(GenerateBlock_2DInstance, test)
{
    const AgentPtrs<2>& agents = RandomBlock2DAgentsGenerator(10,
                                                                -1.4, -.2,
                                                                .2, 1.4,
                                                                .2, 1.4,
                                                                .1, dim);
    generateInstanceAndPlanning<2>(agents,
                                    map_test_config.at("la_ins_path"),
                                    CBS::LargeAgentCBS_func<2>,
                                    1e4, true, true);

};

TEST(GenerateMixedInstanceAndPlanning, test)
{

    AgentPtrs<2> agents = RandomMixedAgentsGenerator(10,
                                                     .4, 2.3,
                                                     10,
                                                     -1.4, -.2,
                                                     .2, 1.4,
                                                     .2, 1.4,
                                                     .1, dim);

    generateInstanceAndPlanning<2> (agents,
                                    map_test_config.at("la_ins_path"),
                                    CBS::LargeAgentCBS_func<2>,
                                    1e4, true, true);

//    generateInstanceAndPlanning<2>(agents,
//                                   map_test_config.at("la_ins_path"),
//                                   LaCAM::LargeAgentLaCAM_func<2>,
//                                   1e4, true, true);

};

TEST(LoadInstanceAndVisualize, test) {
    const std::string file_path = map_test_config.at("la_ins_path");

    loadInstanceAndVisualize<2>(file_path);
}

TEST(LoadInstance_CBS, test)
{
    const std::string file_path = map_test_config.at("la_ins_path");

//    loadInstanceAndPlanning<CircleAgent<2>,
//                            LaCAM::LargeAgentLaCAM<2,
//                            CircleAgent<2>, LaCAM::LargeAgentConstraintTableForLarge<2, CircleAgent<2> > > >(file_path);

    // LargeAgentConstraintTableForLarge
    // LargeAgentConstraintTable

    loadInstanceAndPlanningLayeredLAMAPF<2>(CBS::LargeAgentCBS_func<2>,
                                                        file_path, 60, false, true, true);

};

TEST(LoadInstance_LaCAM, test)
{
    const std::string file_path = map_test_config.at("la_ins_path");

//    loadInstanceAndPlanning<CircleAgent<2>,
//                            LaCAM::LargeAgentLaCAM<2,
//                            CircleAgent<2>, LaCAM::LargeAgentConstraintTableForLarge<2, CircleAgent<2> > > >(file_path);

    // LargeAgentConstraintTableForLarge
    // LargeAgentConstraintTable

//    loadInstanceAndPlanning<CircleAgent<2>, CBS::LargeAgentCBS<2, CircleAgent<2> > >(file_path, 30);

    loadInstanceAndPlanningLayeredLAMAPF<2>(LaCAM::LargeAgentLaCAM_func<2>,
                                                         file_path, 60, false, true);

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
    for(int i=0; i<1; i++) {
        const AgentPtrs<2>& agents = RandomCircleAgentsGenerator<2>(15,
                                                                       .4, 1.4,
                                                                       .1,
                                                                       dim);

        generateInstanceAndPlanning<2> (agents,
                                           map_test_config.at("la_ins_path"),
                                           //CBS::LargeAgentCBS_func<2, CircleAgent<2> >,
                                           LaCAM::LargeAgentLaCAM_func<2>,
                                           1e4, true, false);
    }
}

TEST(Multi_GenerateBlock_2DInstance, test) {
    int count_of_test = 1;
    int reapte = 1;
    for (int i = 0; i < reapte; i++) {
        for (int i = 0; i < count_of_test; i++) {
//            const BlockAgents_2D &agents = RandomBlock2DAgentsGenerator(15,
//                                                                        -1.4, -.2,
//                                                                        .2, 1.4,
//                                                                        .2, 1.4,
//                                                                        .1, dim);

            const AgentPtrs<2> &agents = RandomBlock2DAgentsGenerator(10,
                                                                        -1.4, -.2,
                                                                        .2, 1.4,
                                                                        .2, 1.4,
                                                                        .1, dim);

            generateInstanceAndPlanning<2>(agents,
                                            map_test_config.at("la_ins_path"),
                                            CBS::LargeAgentCBS_func<2>,
                                            //LaCAM::LargeAgentLaCAM_func<2, BlockAgent_2D >,
                                            1e4,
                                            true, false);
        }

    }
}


void Generator_test() {
    InstanceDeserializer<2> deserializer;

    const std::string file_path = map_test_config.at("la_ins_path");

    if(deserializer.loadInstanceFromFile(file_path, dim)) {
        std::cout << "load from path " << file_path << " success" << std::endl;
    } else {
        std::cout << "load from path " << file_path << " failed" << std::endl;
        return;
    }
    std::cout << " map scale " << dim[0] << "*" << dim[1] << std::endl;

    LargeAgentMAPF_InstanceGenerator<2> generator(deserializer.getAgents(), is_occupied, dim, 1e7);


    for(int i=0; i<deserializer.getAgents().size(); i++) {
        size_t start_id = generator.agent_sub_graphs_[i].start_node_id_;
        size_t target_id = generator.agent_sub_graphs_[i].target_node_id_;

//        assert(generator.agent_sub_graphs_[i].all_nodes_[start_id] != nullptr);
//        assert(generator.agent_sub_graphs_[i].all_nodes_[target_id] != nullptr);
    }

//    LargeAgentMAPFInstanceDecompositionPtr<2, AgentType> decomposer =
//            std::make_shared<LargeAgentMAPFInstanceDecomposition<2, AgentType> >(deserializer.getInstances(),
//                                                                                 deserializer.getAgents(), dim, is_occupied);

    CBS::LargeAgentCBS<2> solver(deserializer.getInstances(), deserializer.getAgents(), dim, is_occupied);

    for(int i=0; i<deserializer.getAgents().size(); i++) {
        size_t start_id  = solver.agent_sub_graphs_[i].start_node_id_;
        size_t target_id = solver.agent_sub_graphs_[i].target_node_id_;

//        assert(solver.agent_sub_graphs_[i].all_nodes_[start_id] != nullptr);
//        assert(solver.agent_sub_graphs_[i].all_nodes_[target_id] != nullptr);
    }

}

TEST(Generator, test) {
    Generator_test();
}

void Decomposition_test() {
    InstanceDeserializer<2> deserializer;

    const std::string file_path = map_test_config.at("la_ins_path");

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

    LargeAgentMAPFInstanceDecomposition<2> decomposer =
            LargeAgentMAPFInstanceDecomposition<2>(deserializer.getInstances(),
                                                              deserializer.getAgents(),
                                                              dim,
                                                              is_occupied);

    InstanceVisualization(deserializer.getAgents(),
                                     decomposer.getAllPoses(),
                                     deserializer.getInstances(),
                                     decomposer.grid_paths_,
                                     decomposer.agent_visited_grids_);

}

TEST(Decomposition, test) {
    Decomposition_test();
}


void multiGenerateAgentAndDecomposition(const SingleMapTestConfig<2>& map_file,
                                        int count_of_test,
                                        int maximum_agents,
                                        int minimum_agents,
                                        int agent_interval,
                                        int maximum_sample_count = 1e4) {
    map_test_config = map_file;
    loader = TextMapLoader(map_test_config.at("map_path"), is_char_occupied1);

    //clearFile(map_test_config.at("la_dec_path"));
    for (int i = 0; i < count_of_test; i++) {
        for(int count_of_agent = minimum_agents; count_of_agent<= maximum_agents; count_of_agent += agent_interval) {


//            const AgentPtrs<2> &agents = RandomCircleAgentsGenerator<2>(count_of_agent,
//                                                                           .4, 1.4,
//                                                                           .1,
//                                                                           dim);

            AgentPtrs<2> agents = RandomMixedAgentsGenerator(count_of_agent/2,
                                                             .4, 2.3,
                                                             count_of_agent/2,
                                                             -2.4, -.2,
                                                             .2, 2.4,
                                                             .2, 2.4,
                                                             .1, dim);

            if(!generateInstance<2>(agents, map_test_config.at("la_ins_path"), maximum_sample_count)) {
                continue;
            }

            InstanceDeserializer<2> deserializer;
            if(deserializer.loadInstanceFromFile(map_test_config.at("la_ins_path"), dim)) {
                std::cout << "load from path " << map_test_config.at("la_ins_path") << " success" << std::endl;
            } else {
                std::cout << "load from path " << map_test_config.at("la_ins_path") << " failed" << std::endl;
                return;
            }

            std::vector<OutputStream> strs;
            OutputStream str;
//            decompositionOfSingleInstance<2>(deserializer.getInstances(), deserializer.getAgents(),
//                                             dim, is_occupied, str, 1);
//            strs.push_back(str);
//
//            decompositionOfSingleInstance<2>(deserializer.getInstances(), deserializer.getAgents(),
//                                             dim, is_occupied, str, 2);
//            strs.push_back(str);
////
//            decompositionOfSingleInstance<2>(deserializer.getInstances(), deserializer.getAgents(),
//                                             dim, is_occupied, str, 3);
//            strs.push_back(str);

//            decompositionOfSingleInstance<2>(deserializer.getInstances(), deserializer.getAgents(),
//                                             dim, is_occupied, str, 4);
//            strs.push_back(str);

            auto instance_decompose = std::make_shared<LargeAgentMAPFInstanceDecomposition<2> >(deserializer.getInstances(),
                                                                                                deserializer.getAgents(),
                                                                                                dim,
                                                                                                is_occupied,
                                                                                                true,
                                                                                                4,
                                                                                                true);
            /*

               ss << time_cost << " "
               << max_cluster_size << " "
               << instances.size() << " "
               << is_legal << " " << level << " "
               << memory_usage << " "
               << instance_decompose->all_clusters_.size() << " "
               << instance_decompose->initialize_time_cost_ << " "
               << instance_decompose->instance_decomposition_time_cost_ << " "
               << instance_decompose->cluster_bipartition_time_cost_ << " "
               << instance_decompose->level_sorting_time_cost_ << " "
               << instance_decompose->level_bipartition_time_cost_;

             * */
            for(const auto& a_data : instance_decompose->debug_data_) {
                assert(a_data.size() == 12);
                std::stringstream ss;
                ss << a_data[0] << " "
                   << (int)a_data[1] << " "
                   << (int)a_data[2] << " "
                   << (int)a_data[3] << " " << (int)a_data[4] << " "
                   << a_data[5] << " "
                   << a_data[6] << " "
                   << a_data[7] << " "
                   << a_data[8] << " "
                   << a_data[9] << " "
                   << a_data[10] << " "
                   << a_data[11];
                strs.push_back(ss.str());
            }

            for (const auto &str : strs) {
                std::cout << str << std::endl;
            }
            writeStrsToEndOfFile(strs, map_test_config.at("la_dec_path"));
        }
    }
}

TEST(Multi_Generate_Agent_And_Decomposition, test) {
    // file_path, count_of_test, max_agent_count, min_agent_count, interval, max_sample
    std::vector<std::tuple<SingleMapTestConfig<2>, int, int, int, int, int> >
            map_configs = {{MAPFTestConfig_Paris_1_256, 10000, 80, 10, 10, 2e5},
                           //{MAPFTestConfig_empty_48_48, 10000, 50, 10, 1e4},
    };

    for(const auto& file_config : map_configs) {
        multiGenerateAgentAndDecomposition(std::get<0>(file_config),
                                           std::get<1>(file_config),
                                           std::get<2>(file_config),
                                           std::get<3>(file_config),
                                           std::get<4>(file_config),
                                           std::get<5>(file_config));
    }
}


void multiGenerateAgentAndCompare(const SingleMapTestConfig<2>& map_file,
                                  int count_of_test,
                                  int maximum_agents,
                                  int minimum_agents,
                                  int agent_interval,
                                  int maximum_sample_count = 1e4) {
    map_test_config = map_file;
    loader = TextMapLoader(map_test_config.at("map_path"), is_char_occupied1);

    clearFile(map_test_config.at("la_comp_path"));
    for (int i = 0; i < count_of_test; i++) {
        for(int count_of_agent = agent_interval; count_of_agent<= maximum_agents; count_of_agent += agent_interval) {


//            const AgentPtrs<2> &agents = RandomCircleAgentsGenerator<2>(count_of_agent,
//                                                                           .4, 1.4,
//                                                                           .1,
//                                                                           dim);

            AgentPtrs<2> agents = RandomMixedAgentsGenerator(count_of_agent/2,
                                                             .4, 2.3,
                                                             count_of_agent/2,
                                                             -2.4, -.2,
                                                             .2, 2.4,
                                                             .2, 2.4,
                                                             .1, dim);

            auto strs = generateInstanceAndCompare<2>(agents,
                                                      map_test_config.at("la_ins_path"),
                                                      CBS::LargeAgentCBS_func<2>, //LaCAM::LargeAgentLaCAM_func<2>,
                                                      std::string("CBS"),
                                                      30,
                                                      false,
                                                      maximum_sample_count, true);

            for (const auto &str : strs) {
                std::cout << str << std::endl;
            }
            writeStrsToEndOfFile(strs, map_test_config.at("la_comp_path"));
        }
    }

}

TEST(Multi_Generate_Agent_And_Compare, test) {

    // file_path, count_of_test, max_agent_count, min_agent_count, interval, max_sample
    std::vector<std::tuple<SingleMapTestConfig<2>, int, int, int, int, int> >
            map_configs = {{MAPFTestConfig_Paris_1_256, 1, 12, 4, 4, 1e5},
                           {MAPFTestConfig_empty_48_48, 1, 12, 4, 4, 1e4},
    };

    for(const auto& file_config : map_configs) {
        multiGenerateAgentAndCompare(std::get<0>(file_config),
                                     std::get<1>(file_config),
                                     std::get<2>(file_config),
                                     std::get<3>(file_config),
                                     std::get<4>(file_config),
                                     std::get<5>(file_config));
    }
}