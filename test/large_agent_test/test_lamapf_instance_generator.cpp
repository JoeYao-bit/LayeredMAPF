////
//// Created by yaozhuo on 2024/6/26.
////
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

void Generator_test() {
    auto file_path = MAPFTestConfig_empty_48_48;

    TextMapLoader tl(file_path.at("map_path"), is_char_occupied1);
    auto dim = tl.getDimensionInfo();
    auto is_occupied = [&tl](const freeNav::Pointi<2> &pt) -> bool { return tl.isOccupied(pt); };
    IS_OCCUPIED_FUNC<2> is_occupied_func = is_occupied;

    InstanceDeserializer<2> deserializer;

    const std::string map_path = map_test_config.at("la_ins_path");

    if(deserializer.loadInstanceFromFile(map_path, dim)) {
        std::cout << "load from path " << map_path << " success" << std::endl;
    } else {
        std::cout << "load from path " << map_path << " failed" << std::endl;
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

//    CBS::LargeAgentCBS<2, Pose<int, 2>> solver(deserializer.getInstances(), deserializer.getAgents(), dim, is_occupied);

//    for(int i=0; i<deserializer.getAgents().size(); i++) {
//        size_t start_id  = solver.agent_sub_graphs_[i].start_node_id_;
//        size_t target_id = solver.agent_sub_graphs_[i].target_node_id_;

//        assert(solver.agent_sub_graphs_[i].all_nodes_[start_id] != nullptr);
//        assert(solver.agent_sub_graphs_[i].all_nodes_[target_id] != nullptr);

}

TEST(Generator, test) {
    Generator_test();
}

void Decomposition_test(const SingleMapTestConfig<2>& map_file_local) {

    TextMapLoader tl_local(map_file_local.at("map_path"), is_char_occupied1);
    auto dim_local = tl_local.getDimensionInfo();
    auto is_occupied_local = [&tl_local](const freeNav::Pointi<2> &pt) -> bool { return tl_local.isOccupied(pt); };
    IS_OCCUPIED_FUNC<2> is_occupied_func_local = is_occupied_local;


    InstanceDeserializer<2> deserializer_local;
    const std::string file_path_local = map_file_local.at("la_ins_path");
    if(deserializer_local.loadInstanceFromFile(file_path_local, dim_local)) {
        std::cout << "load from path " << file_path_local << " success" << std::endl;
    } else {
        std::cout << "load from path " << file_path_local << " failed" << std::endl;
        return;
    }

    std::cout << " map scale " << dim_local[0] << "*" << dim_local[1] << std::endl;

    std::cout << "Instance: " << std::endl;
    std::vector<std::string> strs = deserializer_local.getTextString();
    for(const auto& str : strs) {
        std::cout << str << std::endl;
    }
    std::cout << std::endl;

    PrecomputationOfLAMAPFDecomposition<2, HyperGraphNodeDataRaw<2>> pre(deserializer_local.getInstances(),
                                                            deserializer_local.getAgents(),
                                                            dim_local, is_occupied_local, true);

    MAPFInstanceDecompositionBipartition<2, HyperGraphNodeDataRaw<2>, Pose<int, 2> > bi_decompose(dim_local,
                                                                                 pre.connect_graphs_,
                                                                                 pre.agent_sub_graphs_,
                                                                                 pre.heuristic_tables_sat_,
                                                                                 pre.heuristic_tables_,
                                                                                 60,// - pre.initialize_time_cost_/1e3,
                                                                                 3);

    std::cout << "bi/raw = " << LA_MAPF::getMaxLevelSize(bi_decompose.all_levels_) << "/"
              << deserializer_local.getAgents().size() << std::endl;

//    InstanceVisualization(deserializer.getAgents(),
//                                     decomposer.getAllPoses(),
//                                     deserializer.getInstances(),
//                                     decomposer.grid_paths_,
//                                     decomposer.agent_visited_grids_);

}
//
//void multiLoadLargeAgentAndDecomposition(const SingleMapTestConfig<2>& map_file,
//                                    int count_of_test,
//                                    int maximum_agents,
//                                    int minimum_agents,
//                                    int agent_interval) {
//
//
//    map_test_config = map_file;
//    TextMapLoader loader = TextMapLoader(map_test_config.at("map_path"), is_char_occupied1);
//
//    auto dim = loader.getDimensionInfo();
//    auto is_occupied = [&loader](const freeNav::Pointi<2> &pt) -> bool { return loader.isOccupied(pt); };
//    IS_OCCUPIED_FUNC<2> is_occupied_func = is_occupied;
//    //clearFile(map_test_config.at("la_comp_path"));
//
//    InstanceDeserializer<2> deserializer;
//    if(deserializer.loadInstanceFromFile(map_test_config.at("la_ins_path"), dim)) {
//        std::cout << "load from path " << map_test_config.at("la_ins_path") << " success" << std::endl;
//    } else {
//        std::cout << "load from path " << map_test_config.at("la_ins_path") << " failed" << std::endl;
//        return;
//    }
//    assert(!deserializer.getAgents().empty());
//    std::vector<int> required_counts;
//    for(int c=minimum_agents; c<=std::min(maximum_agents, (int)deserializer.getAgents().size()); c+=agent_interval) {
//        required_counts.push_back(c);
//    }
//    std::cout << " required_counts size = " << required_counts.size() << std::endl;
//
//    auto agent_and_instances = deserializer.getTestInstance(required_counts, count_of_test);
//
//    std::cout << " get " << agent_and_instances.size() << " instance" << std::endl;
//
//    //clearFile(map_test_config.at("la_dec_path"));
//    for (int i = 0; i < agent_and_instances.size(); i++) {
//
//        std::vector<OutputStream> strs;
//        OutputStream str;
//
//        auto instance_decompose =
//                std::make_shared<LargeAgentMAPFInstanceDecomposition<2, HyperGraphNodeDataRaw<2>, Pose<int, 2>> >(
//                agent_and_instances[i].second,
//                agent_and_instances[i].first,
//                dim,
//                is_occupied,
//                true,
//                4,
//                true);
//        /*
//
//           ss << time_cost << " "
//           << max_cluster_size << " "
//           << instances.size() << " "
//           << is_legal << " " << level << " "
//           << memory_usage << " "
//           << instance_decompose->all_clusters_.size() << " "
//           << instance_decompose->initialize_time_cost_ << " "
//           << instance_decompose->instance_decomposition_time_cost_ << " "
//           << instance_decompose->cluster_bipartition_time_cost_ << " "
//           << instance_decompose->level_sorting_time_cost_ << " "
//           << instance_decompose->level_bipartition_time_cost_;
//
//         * */
//        for(const auto& a_data : instance_decompose->debug_data_) {
//            assert(a_data.size() == 12);
//            std::stringstream ss;
//            ss << a_data[0] << " "
//               << (int)a_data[1] << " "
//               << (int)a_data[2] << " "
//               << (int)a_data[3] << " " << (int)a_data[4] << " "
//               << a_data[5] << " "
//               << a_data[6] << " "
//               << a_data[7] << " "
//               << a_data[8] << " "
//               << a_data[9] << " "
//               << a_data[10] << " "
//               << a_data[11];
//            strs.push_back(ss.str());
//        }
//
//        for (const auto &str : strs) {
//            std::cout << str << std::endl;
//        }
//        writeStrsToEndOfFile(strs, map_test_config.at("la_dec_path"));
//    }
//}

void generateLargeAgentInstanceForMap(const SingleMapTestConfig<2>& map_file,
                                      int required_agents,
                                      int times_of_try,
                                      int maximum_sample_count = 1e7) {
    map_test_config = map_file;
    TextMapLoader loader = TextMapLoader(map_test_config.at("map_path"), is_char_occupied1);

    auto dim = loader.getDimensionInfo();
    auto is_occupied = [&loader](const freeNav::Pointi<2> &pt) -> bool { return loader.isOccupied(pt); };
    IS_OCCUPIED_FUNC<2> is_occupied_func = is_occupied;

    for(int i=0; i<times_of_try; i++) {
        AgentPtrs<2> agents = RandomMixedAgentsGenerator(required_agents/2,
                                                         .4, 2.,
                                                         required_agents/2,
                                                         -2., -.2,
                                                         .2, 2.,
                                                         .2, 2.,
                                                         .1, dim);

        LargeAgentMAPF_InstanceGenerator<2> generator(agents, is_occupied, dim, maximum_sample_count);
        auto instances_and_path = generator.getNewInstance();
        InstanceOrients<2> instances;
        for(int j=0; j<instances_and_path.size(); j++) {
            instances.push_back(instances_and_path[j].first);
        }
        if(!instances.empty()) {
            std::cout << "-- find " << required_agents << " agents and poses after " << i << " sample" <<  std::endl;
            InstanceSerializer<2> serializer(agents, instances);
            if(serializer.saveToFile(map_test_config.at("la_ins_path"))) {
                std::cout << "save to path " << map_test_config.at("la_ins_path") << " success" << std::endl;
                return;
            } else {
                std::cout << "save to path " << map_test_config.at("la_ins_path") << " failed" << std::endl;
                return;
            }
        } else {
            std::cout << "-- find no " << required_agents << " agents and poses after " << i << " sample" <<  std::endl;
        }
    }
}

// MAPFTestConfig_Berlin_1_256
//TEST(generateLargeAgentInstanceForMap, test) {
int main() {
    // file_path, times_of_try, required_agents, maximum_sample_count
    std::vector<std::tuple<SingleMapTestConfig<2>, int, int, int> >
            map_configs = {
//                           {MAPFTestConfig_Paris_1_256, 140, 100, 1e7}, // ok
//                           {MAPFTestConfig_empty_48_48, 60,  100, 1e7}, // ok
//                           {MAPFTestConfig_Berlin_1_256, 140, 100, 5e7}, // ok
//                           {MAPFTestConfig_maze_128_128_10, 100, 100, 5e7}, // ok
//                           {MAPFTestConfig_den520d, 140, 100, 5e7}, // load failed
//                           {MAPFTestConfig_ost003d, 100, 100, 5e7} // target overlap

                            {MAPFTestConfig_Boston_2_256, 40, 100, 1e7}, // ok
                            //{MAPFTestConfig_Sydney_2_256, 140,  100, 1e7}, // ok
                            //{MAPFTestConfig_AR0044SR, 50, 100, 5e7}, // ok
                            //{MAPFTestConfig_AR0203SR, 40, 100, 5e7}, // ok
                            //{MAPFTestConfig_AR0072SR, 70, 100, 5e7}, // ok
//                            {MAPFTestConfig_Denver_2_256, 140, 100, 5e7} // ok

    };

//    for(const auto & map_config : map_configs) {
//        generateLargeAgentInstanceForMap(std::get<0>(map_config),
//                                         std::get<1>(map_config),
//                                         std::get<2>(map_config),
//                                         std::get<3>(map_config));
//    }
    for(const auto& file_config : map_configs) {
        Decomposition_test(std::get<0>(file_config));
    }
    return 0;
}


//TEST(Multi_Generate_Agent_And_Decomposition, test) { // 2360 count
int main1() {
// file_path, count_of_test, max_agent_count, min_agent_count, interval, max_sample
    std::vector<std::tuple<SingleMapTestConfig<2>, int, int, int, int> >
            map_configs = {//{MAPFTestConfig_Paris_1_256,     100, 200, 10, 10},
                           //{MAPFTestConfig_empty_48_48,     100, 200, 10, 10},
//                           {MAPFTestConfig_Berlin_1_256,    100, 200, 10, 10},
//                           {MAPFTestConfig_maze_128_128_10, 100, 200, 10, 10},
//                           {MAPFTestConfig_den520d,         100, 200, 10, 10},
//                           {MAPFTestConfig_ost003d,         100, 200, 10, 10},
                            // {MAPFTestConfig_Boston_2_256, 1, 200, 10, 10}, // ok
                            // {MAPFTestConfig_Sydney_2_256, 1, 200, 10, 10}, // ok
                            // {MAPFTestConfig_AR0044SR, 1, 200, 10, 5}, // ok
                            // {MAPFTestConfig_AR0203SR, 1, 200, 10, 5}, // ok
                            {MAPFTestConfig_AR0072SR, 1, 200, 10, 5}, // ok
                            {MAPFTestConfig_Denver_2_256, 1, 200, 10, 10} // ok
    };
    for(int i=0; i<200; i++) {
        std::cout << "global decom " << i << std::endl;
        for (const auto &file_config : map_configs) {
//            multiLoadLargeAgentAndDecomposition(std::get<0>(file_config),
//                                               1, //std::get<1>(file_config),
//                                               std::get<2>(file_config),
//                                               std::get<3>(file_config),
//                                               std::get<4>(file_config));
        }
    }
    return 0;
}



