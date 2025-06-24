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

    MSTimer mst;
    auto bi_decompose = std::make_shared<MAPFInstanceDecompositionBipartition<2, HyperGraphNodeDataRaw<2> > >(dim,
            pre.connect_graphs_,
            pre.agent_sub_graphs_,
            pre.heuristic_tables_sat_,
            pre.heuristic_tables_,
            time_limit_s,// - pre.initialize_time_cost_/1e3,
            level);

    double time_cost =  mst.elapsed();// + pre.initialize_time_cost_;

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

    int max_subproblem = LA_MAPF::getMaxLevelSize(bi_decompose->all_clusters_);
    int num_of_subproblem = bi_decompose->all_clusters_.size();

    if(max_subproblem == 0) {
        max_subproblem = agents.size();
        num_of_subproblem = 1;
    }

    outputStream.clear();
    std::stringstream ss;
    ss << " " << time_cost << " "
       << max_subproblem << " "
       << agents.size() << " "
       << is_bi_valid << " "
       << level
       << " "
       << memory_usage << " "
       << num_of_subproblem << " "
       << bi_decompose->instance_decomposition_time_cost_ << " "
       << bi_decompose->cluster_bipartition_time_cost_ << " "
       << bi_decompose->level_sorting_time_cost_ << " "
       << bi_decompose->level_bipartition_time_cost_ << " ";

    outputStream = ss.str();
//    std::cout << " memory_usage = " << memory_usage << std::endl;
    std::cout << "level " << level << "/raw = " << LA_MAPF::getMaxLevelSize(bi_decompose->all_clusters_) << "/" << agents.size() << std::endl;
    return is_bi_valid;
}


// level 5 is a flag to different from bipartition decomposition
template<Dimension N>
bool decompositionOfSingleInstanceBreakLoopLAMAPF(const InstanceOrients<N> & instances,
                                                  const std::vector<LA_MAPF::AgentPtr<N> >& agents,
                                                  DimensionLength* dim,
                                                  const IS_OCCUPIED_FUNC<N> & isoc,
                                                  OutputStream& outputStream,
                                                  double time_limit_s, int level=5,
                                                  int max_continue_failure = 100) {


    memory_recorder.clear();
    sleep(1);
    float basic_usage = memory_recorder.getMaximalMemoryUsage();

    PrecomputationOfLAMAPF<2, HyperGraphNodeDataRaw<2>> pre(instances, agents, dim, isoc, false);

    MSTimer mst;
    auto ns_decompose = std::make_shared<MAPFInstanceDecompositionBreakLoop<2, HyperGraphNodeDataRaw<2>> >(dim,
            pre.connect_graphs_,
            pre.agent_sub_graphs_,
            pre.heuristic_tables_sat_,
            time_limit_s,// - pre.initialize_time_cost_/1e3,
            1e4,
            max_continue_failure,
            1);

    ns_decompose->breakMaxLoopIteratively();

    double time_cost =  mst.elapsed();// + pre.initialize_time_cost_;

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

    int max_subproblem = LA_MAPF::getMaxLevelSize(ns_decompose->all_levels_);
    int num_of_subproblem = ns_decompose->all_levels_.size();

    if(max_subproblem == 0) {
        max_subproblem = agents.size();
        num_of_subproblem = 1;
    }

    outputStream.clear();
    std::stringstream ss;
    ss << " " << time_cost << " "
       << max_subproblem << " "
       << agents.size() << " "
       << is_bi_valid << " "
       << level
       << " "
       << memory_usage << " "
       << num_of_subproblem << " "
       << 0 << " "
       << 0 << " "
       << 0 << " "
       << 0 << " ";

    outputStream = ss.str();
    std::cout << "max failure count = " << max_continue_failure << std::endl;
    std::cout << "level" << level << "(break loop)/raw = " << LA_MAPF::getMaxLevelSize(ns_decompose->all_levels_) << "/" << agents.size() << std::endl;
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
                                      double timecost_limit_s = 10,
                                      int max_continue_failure = 100) {

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
                                                            ostream, timecost_limit_s, 5, max_continue_failure)) {
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

// map name, num of agents, max continue failure for break loop
std::vector<std::tuple<SingleMapTestConfig<2>, std::vector<int>, int >> test_configs = {
        {MAPFTestConfig_Paris_1_256,     {20, 40, 60, 80, 100, 120, 140}, 100}, // 1,
        {MAPFTestConfig_empty_48_48,     {10, 20, 30, 40, 50, 60},        100}, // 2,
        {MAPFTestConfig_Berlin_1_256,    {20, 40, 60, 80, 100, 120, 140}, 100}, // 3,
        {MAPFTestConfig_maze_128_128_10, {20, 40, 60, 80, 100},           100}, // 4,
        {MAPFTestConfig_den520d,         {20, 40, 60, 80, 100, 120, 140}, 100}, // 5,
        {MAPFTestConfig_ost003d,         {20, 40, 60, 80, 100},           100}, // 6,
        {MAPFTestConfig_Boston_2_256,    {20, 40, 60, 80, 100, 120, 140}, 100}, // 7,
        {MAPFTestConfig_Sydney_2_256,    {20, 40, 60, 80, 100, 120, 140}, 100}, // 8,
        {MAPFTestConfig_AR0044SR,        {10, 20, 30, 40, 50},            100}, // 9
        {MAPFTestConfig_AR0203SR,        {10, 20, 30, 40, 50},            100}, // 10,
        {MAPFTestConfig_AR0072SR,        {20, 30, 40, 50, 60, 70},        100}, // 11,
        {MAPFTestConfig_Denver_2_256,    {20, 40, 60, 80, 100, 120, 140}, 100}, // 12,
};

std::vector<std::tuple<SingleMapTestConfig<2>, std::vector<int>, int> > test_configs_demo = {
        {MAPFTestConfig_Paris_1_256,     {20, 140}, 100}, // 1,
        {MAPFTestConfig_empty_48_48,     {10, 60},  100}, // 2,
        {MAPFTestConfig_Berlin_1_256,    {20,140},  100}, // 3,
        {MAPFTestConfig_maze_128_128_10, {20,100},  100}, // 4,
        {MAPFTestConfig_den520d,         {20,140},  100}, // 5,
        {MAPFTestConfig_ost003d,         {20,100},  100}, // 6,
        {MAPFTestConfig_Boston_2_256,    {20,140},  100}, // 7,
        {MAPFTestConfig_Sydney_2_256,    {20,140},  100}, // 8,
        {MAPFTestConfig_AR0044SR,        {10,50},   100}, // 9
        {MAPFTestConfig_AR0203SR,        {10,50},   100}, // 10,
        {MAPFTestConfig_AR0072SR,        {20,70},   100}, // 11,
        {MAPFTestConfig_Denver_2_256,    {20,140},  100}, // 12,
};


std::vector<std::tuple<SingleMapTestConfig<2>, std::vector<int>, int> > test_configs_single = {
//        {MAPFTestConfig_empty_48_48,     {10, 20, 30, 40, 50, 60},  100}, // 2,
//        {MAPFTestConfig_empty_48_48,     {10, 20, 30, 40, 50, 60},  200}, // 2,
//        {MAPFTestConfig_empty_48_48,     {10, 20, 30, 40, 50, 60},  400}, // 2,
//        {MAPFTestConfig_empty_48_48,     {10, 20, 30, 40, 50, 60},  800}, // 2,
        {MAPFTestConfig_empty_48_48,     {10, 20, 30, 40, 50, 60},  200}, // 2,
        {MAPFTestConfig_empty_48_48,     {10, 20, 30, 40, 50, 60},  400}, // 2,
};

// do decomposition test
int main() {

    auto test_configs_copy = test_configs; // test_configs, test_configs_demo,test_configs_single

    int interval = 6; // test_configs_copy.size()
    int repeat_times = 100;
    int num_threads = test_configs_copy.size()/interval + 1;
    std::vector<bool> finished(num_threads, false);
    for(int j=0; j<num_threads; j++) {
        auto lambda_func = [&]() {
            int thread_id = j; // save to avoid change during planning
            for(int i=0; i<repeat_times; i++) {
                int count_of_instances = 1;
                for (int k = 0; k < interval; k++) {
                    //std::cout << "thread_id = " << thread_id << std::endl;
                    int map_id = thread_id * interval + k;
                    std::cout << "map_id = " << map_id << std::endl;
                    if (map_id >= test_configs_copy.size()) { break; }
                    SingleMapDecompositionTestLAMAPF(std::get<0>(test_configs_copy[map_id]),
                                                   std::get<1>(test_configs_copy[map_id]),
                                                    count_of_instances,
                                                    30,
                                                     std::get<2>(test_configs_copy[map_id]));
                }
            }
            finished[thread_id] = true;
            std::cout << "thread " << thread_id << " finished" << std::endl;
        };
        std::thread t(lambda_func);
        t.detach();
        sleep(1);
    }
    while(finished != std::vector<bool>(num_threads, true)) {
        sleep(1);
    }
    return 0;
}



