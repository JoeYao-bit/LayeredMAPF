//
// Created by yaozhuo on 6/20/25.
//

#include "../algorithm/break_loop_decomposition/break_loop_decomposition.h"
#include "../algorithm/break_loop_decomposition/biparition_decomposition.h"
#include "../algorithm/connectivity_graph_and_subprgraph.h"

#include "../freeNav-base/dependencies/2d_grid/text_map_loader.h"
#include "../freeNav-base/dependencies/memory_analysis.h"

#include <sys/resource.h>

#include "../test/test_data.h"

#include "common_interfaces.h"


using namespace freeNav;
using namespace freeNav::LayeredMAPF;
using namespace freeNav::LayeredMAPF::LA_MAPF;

// level 5 is a flag to different from bipartition decomposition
template<Dimension N>
bool decompositionOfSingleInstanceBipartitionLAMAPF(const InstanceOrients<N> & instances,
                                                    const std::vector<AgentPtr<N> >& agents,
                                                    DimensionLength* dim,
                                                    const IS_OCCUPIED_FUNC<N> & isoc,
                                                    OutputStream& outputStream,
                                                    double time_limit_s, int level=3) {


    memory_recorder.clear();
    sleep(1);
    float basic_usage = memory_recorder.getMaximalMemoryUsage();

    PrecomputationOfLAMAPF<2, HyperGraphNodeDataRaw<2>> pre(instances, agents, dim, isoc, true);

    auto start_t = clock();
    auto bi_decompose = std::make_shared<MAPFInstanceDecompositionBipartition<2, HyperGraphNodeDataRaw<2> > >(dim,
            pre.connect_graphs_,
            pre.agent_sub_graphs_,
            pre.heuristic_tables_sat_,
            pre.heuristic_tables_,
            time_limit_s,// - pre.initialize_time_cost_/1e3,
            level);

    auto now_t = clock();
    double time_cost =  1e3*((double)now_t - start_t)/CLOCKS_PER_SEC + pre.initialize_time_cost_;

    sleep(1);
    float peak_usage = memory_recorder.getMaximalMemoryUsage();
    float memory_usage = peak_usage - basic_usage;
    bool is_bi_valid = LA_MAPF_DecompositionValidCheckGridMap<2>(bi_decompose->all_clusters_,
                                                                 dim,
                                                                 isoc,
                                                                 agents,
                                                                 pre.instance_node_ids_,
                                                                 pre.all_poses_,
                                                                 pre.agent_sub_graphs_,
                                                                 pre.agents_heuristic_tables_,
                                                                 pre.agents_heuristic_tables_ignore_rotate_
    );

    outputStream.clear();
    std::stringstream ss;
    ss << " " << time_cost << " "
       << LA_MAPF::getMaxLevelSize(bi_decompose->all_clusters_) << " "
       << agents.size() << " "
       << is_bi_valid << " "
       << level
       << " "
       << memory_usage << " "
       << bi_decompose->all_clusters_.size() << " "
       << bi_decompose->instance_decomposition_time_cost_ << " "
       << bi_decompose->cluster_bipartition_time_cost_ << " "
       << bi_decompose->level_sorting_time_cost_ << " "
       << bi_decompose->level_bipartition_time_cost_ << " ";

    outputStream = ss.str();
    std::cout << " memory_usage = " << memory_usage << std::endl;
    return is_bi_valid;
}


// level 5 is a flag to different from bipartition decomposition
template<Dimension N>
bool decompositionOfSingleInstanceBreakLoopLAMAPF(const InstanceOrients<N> & instances,
                                                  const std::vector<LA_MAPF::AgentPtr<N> >& agents,
                                                  DimensionLength* dim,
                                                  const IS_OCCUPIED_FUNC<N> & isoc,
                                                  OutputStream& outputStream,
                                                  double time_limit_s, int level=5) {


    memory_recorder.clear();
    sleep(1);
    float basic_usage = memory_recorder.getMaximalMemoryUsage();

    PrecomputationOfLAMAPF<2, HyperGraphNodeDataRaw<2>> pre(instances, agents, dim, isoc, true);

    auto start_t = clock();
    auto ns_decompose = std::make_shared<MAPFInstanceDecompositionBreakLoop<2, HyperGraphNodeDataRaw<2>> >(dim,
            pre.connect_graphs_,
            pre.agent_sub_graphs_,
            pre.heuristic_tables_sat_,
            time_limit_s,// - pre.initialize_time_cost_/1e3,
            1e4,
            100,
            1);

    auto now_t = clock();
    double time_cost =  1e3*((double)now_t - start_t)/CLOCKS_PER_SEC + pre.initialize_time_cost_;

    sleep(1);
    float peak_usage = memory_recorder.getMaximalMemoryUsage();
    float memory_usage = peak_usage - basic_usage;
    bool is_bi_valid = LA_MAPF_DecompositionValidCheckGridMap<2>(ns_decompose->all_levels_,
                                                                 dim,
                                                                 isoc,
                                                                 agents,
                                                                 pre.instance_node_ids_,
                                                                 pre.all_poses_,
                                                                 pre.agent_sub_graphs_,
                                                                 pre.agents_heuristic_tables_,
                                                                 pre.agents_heuristic_tables_ignore_rotate_
    );

    outputStream.clear();
    std::stringstream ss;
    ss << " " << time_cost << " "
       << LA_MAPF::getMaxLevelSize(ns_decompose->all_levels_) << " "
       << agents.size() << " "
       << is_bi_valid << " "
       << level
       << " "
       << memory_usage << " "
       << ns_decompose->all_levels_.size() << " "
       << 0 << " "
       << 0 << " "
       << 0 << " "
       << 0 << " ";

    outputStream = ss.str();
    std::cout << " memory_usage = " << memory_usage << std::endl;
    return is_bi_valid;
}

int count_of_instance_total = 0;

//auto is_char_occupied1 = [](const char& value) -> bool {
//    if (value == '.') return false;
//    return true;
//};

bool SingleMapDecompositionTestLAMAPF(const SingleMapTestConfig <2> &map_test_config,
                                      const std::vector<int>& agent_in_instances,
                                      const int& count_of_instance,
                                      double timecost_limit_s = 10) {

    count_of_instance_total += agent_in_instances.size();
    // 0, load scenerio
    TextMapLoader tl(map_test_config.at("map_path"), is_char_occupied1);
    std::cout << "start SingleMapTest from map " << map_test_config.at("map_path") << std::endl;
    IS_OCCUPIED_FUNC<2> is_occupied_func;
    SET_OCCUPIED_FUNC<2> set_occupied_func;
    auto is_occupied = [&tl](const freeNav::Pointi<2> &pt) -> bool { return tl.isOccupied(pt); };
    is_occupied_func = is_occupied;
    auto set_occupied = [&tl](const freeNav::Pointi<2> &pt) { tl.setOccupied(pt); };
    set_occupied_func = set_occupied;

    const auto& dim = tl.getDimensionInfo();

    InstanceDeserializer<2> deserializer;
    if(deserializer.loadInstanceFromFile(map_test_config.at("la_ins_path"), dim)) {
        std::cout << "load from path " << map_test_config.at("la_ins_path") << " success" << std::endl;
    } else {
        std::cout << "load from path " << map_test_config.at("la_ins_path") << " failed" << std::endl;
        return false;
    }
    assert(!deserializer.getAgents().empty());

    auto agent_and_instances = deserializer.getTestInstance(agent_in_instances, count_of_instance);


    // 2, do decomposition for each case
    OutputStreamS output_streamss;
    for(int i=0; i<agent_and_instances.size(); i++) {
        const auto& ists = agent_and_instances[i];
        output_streamss.clear();
        OutputStream ostream;
        std::cout << " start decomposition " << std::endl;


        if(!decompositionOfSingleInstanceBipartitionLAMAPF<2>(ists.second, ists.first,
                                                              dim, is_occupied_func,
                                                              ostream, timecost_limit_s, 3)) {
            std::cout << " decomposition failed " << std::endl;
            return false;
        }
        std::cout << "-- finish level 3 decomposition(" << i <<"/" << ists.second.size() << ")" << std::endl;
        output_streamss.push_back(ostream);

        if(!decompositionOfSingleInstanceBipartitionLAMAPF<2>(ists.second, ists.first,
                                                              dim, is_occupied_func,
                                                              ostream, timecost_limit_s, 4)) {
            std::cout << " decomposition failed " << std::endl;
            return false;
        }
        std::cout << "-- finish level 4 decomposition(" << i <<"/" << ists.second.size() << ")" << std::endl;
        output_streamss.push_back(ostream);

        if(!decompositionOfSingleInstanceBreakLoopLAMAPF<2>(ists.second, ists.first,
                                                            dim, is_occupied_func,
                                                            ostream, timecost_limit_s, 5)) {
            std::cout << " decomposition failed " << std::endl;
            return false;
        }
        std::cout << "-- finish level 5 decomposition(" << i <<"/" << ists.second.size() << ")" << std::endl;
        output_streamss.push_back(ostream);

        for(const auto& content : output_streamss) {
            appendToFile(map_test_config.at("la_dec_path"), content);
        }
    }

    return true;
}

/*
 *                         {MAPFTestConfig_Paris_1_256,     100, 200, 10, 10},
                           {MAPFTestConfig_empty_48_48,     100, 200, 10, 10},
                           {MAPFTestConfig_Berlin_1_256,    100, 200, 10, 10},
                           {MAPFTestConfig_maze_128_128_10, 100, 200, 10, 10},
                           {MAPFTestConfig_den520d,         100, 200, 10, 10},
                           {MAPFTestConfig_ost003d,         100, 200, 10, 10},

                           {MAPFTestConfig_Boston_2_256, 1, 200, 10, 10}, // ok
                           {MAPFTestConfig_Sydney_2_256, 1, 200, 10, 10}, // ok
                           {MAPFTestConfig_AR0044SR, 1, 200, 10, 10}, // ok
                           {MAPFTestConfig_AR0203SR, 1, 200, 10, 10}, // ok
                           {MAPFTestConfig_AR0072SR, 1, 200, 10, 10}, // ok
                           {MAPFTestConfig_Denver_2_256, 1, 200, 10, 10} // ok
 * */

int main() {
    for(int i=0; i<100; i++) {
        int count_of_instances = 1;

//        SingleMapDecompositionTestLAMAPF(MAPFTestConfig_Paris_1_256,
//                                         {40, 80, 120, 160, 200},
//                                         count_of_instances);
//
//        SingleMapDecompositionTestLAMAPF(MAPFTestConfig_empty_48_48,
//                                         {40, 80, 120, 160, 200},
//                                         count_of_instances);

        //  // 1,
        //  SingleMapDecompositionTestLAMAPF(MAPFTestConfig_Paris_1_256,
        //                                   {20, 40, 60, 80, 100, 120, 140},
        //                                   count_of_instances);

        // // 2,
        // SingleMapDecompositionTestLAMAPF(MAPFTestConfig_empty_48_48,
        //                                  {10, 20, 30, 40, 50, 60},
        //                                  count_of_instances);

        // // 3,
        // SingleMapDecompositionTestLAMAPF(MAPFTestConfig_Berlin_1_256,
        //                                  {20, 40, 60, 80, 100, 120, 140},
        //                                  count_of_instances);

        // // 4,
        // SingleMapDecompositionTestLAMAPF(MAPFTestConfig_maze_128_128_10,
        //                                  {20, 40, 60, 80, 100},
        //                                  count_of_instances);

        // // 5,
        // SingleMapDecompositionTestLAMAPF(MAPFTestConfig_den520d,
        //                                  {20, 40, 60, 80, 100, 120, 140},
        //                                  count_of_instances);

        // // 6,
        // SingleMapDecompositionTestLAMAPF(MAPFTestConfig_ost003d,
        //                                  {20, 40, 60, 80, 100},
        //                                  count_of_instances);

        // // 7,
        // SingleMapDecompositionTestLAMAPF(MAPFTestConfig_Boston_2_256,
        //                                  {20, 40, 60, 80, 100, 120, 140},
        //                                  count_of_instances);

        // // 8,
        // SingleMapDecompositionTestLAMAPF(MAPFTestConfig_Sydney_2_256,
        //                                  {20, 40, 60, 80, 100, 120, 140},
        //                                  count_of_instances);

    //     // 9,
    //     SingleMapDecompositionTestLAMAPF(MAPFTestConfig_AR0044SR,
    //                                      {10, 20, 30, 40, 50},
    //                                      count_of_instances);

    //    // 10,
    //    SingleMapDecompositionTestLAMAPF(MAPFTestConfig_AR0203SR,
    //                                     {10, 20, 30, 40, 50},
    //                                     count_of_instances);
       // 11,
       SingleMapDecompositionTestLAMAPF(MAPFTestConfig_AR0072SR,
                                        {20, 30, 40, 50, 60, 70},
                                        count_of_instances);

       // 12,
       SingleMapDecompositionTestLAMAPF(MAPFTestConfig_Denver_2_256,
                                        {20, 40, 60, 80, 100, 120, 140},
                                        count_of_instances);
    }
}