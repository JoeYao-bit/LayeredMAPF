//
// Created by yaozhuo on 2024/1/26.
//

#include "../test/test_data.h"
//#include "multi-agent-path-finding/general_mapf_scene.h"
#include "../algorithm/constraint_table_CBS/common.h"

#include "../freeNav-base/dependencies/2d_grid/text_map_loader.h"
#include "../freeNav-base/dependencies/memory_analysis.h"

#include "EECBS/inc/driver.h"
#include "PBS/inc/driver.h"
#include "CBSH2-RTC/inc/driver.h"
#include "MAPF-LNS2/inc/driver.h"
#include "lacam/include/planner.hpp"
#include "lacam2/include/lacam2.hpp"
#include "pibt2/include/driver.h"

#include "../algorithm/layered_mapf.h"
#include "../algorithm/break_loop_decomposition/break_loop_decomposition.h"
#include "../algorithm/break_loop_decomposition/biparition_decomposition.h"
#include "../algorithm/precomputation_for_decomposition.h"

#include "../algorithm/LA-MAPF/common.h"

#include <sys/resource.h>

//ThreadPool viewer_thread(1);

using namespace freeNav;
using namespace freeNav::LayeredMAPF;

struct timezone tz;
struct timeval tv_pre;
struct timeval tv_after;

bool plan_finish = false;

// MAPFTestConfig_random_32_32_20 413.8 ms / layered slower, but faster after 160 agent
// MAPFTestConfig_maze_32_32_2 25.42 ms / layered faster, after 58 agent
// MAPFTestConfig_maze_32_32_4 54.914 ms / layered faster
// MAPFTestConfig_den312d  314.776 ms / layered faster，after 250 agent
// MAPFTestConfig_Berlin_1_256 56.385 ms / layered faster
// MAPFTestConfig_Paris_1_256 1005.82 ms / layered faster
// MAPFTestConfig_warehouse_10_20_10_2_1  5726.96 ms / layered faster， after 500 agent
// MAPFTestConfig_den520d 237.842 ms / layered faster， after 150 agent
// MAPFTestConfig_empty_32_32 2872.3 ms / layered faster

auto map_test_config = freeNav::LayeredMAPF::MAPFTestConfig_ost003d;

auto is_char_occupied1 = [](const char& value) -> bool {
    if (value == '.') return false;
    return true;
};

TextMapLoader loader(map_test_config.at("map_path"), is_char_occupied1);

auto is_occupied = [](const Pointi<2> & pt) -> bool { return loader.isOccupied(pt); };

auto set_occupied = [](const Pointi<2> & pt) { loader.setOccupied(pt); };

IS_OCCUPIED_FUNC<2> is_occupied_func = is_occupied;

SET_OCCUPIED_FUNC<2> set_occupied_func = set_occupied;


int main1() {

    std::cout << " map name " << map_test_config.at("map_path") << std::endl;
    // load mapf scene
    //GeneralMAPFScenePtr<2> scene_ptr;
    const auto& dim = loader.getDimensionInfo();
    freeNav::Instances<2> ists;
    ScenarioLoader2D sl(map_test_config.at("scene_path").c_str());
    int max_count_of_case = atoi((map_test_config.at("agent_num")).c_str());
    int count_of_experiments = sl.GetNumExperiments();

    // load experiment data
    for (int i = 0; i < std::min(max_count_of_case, count_of_experiments); i++) {
        const auto &experiment = sl.GetNthExperiment(i);
        Pointi<2> pt1({experiment.GetStartX(), experiment.GetStartY()});
        Pointi<2> pt2({experiment.GetGoalX(), experiment.GetGoalY()});
        freeNav::Instance<2> ist = {pt1, pt2};
        //std::cout << " start, target = " << pt1 << ", " << pt2 << std::endl;
        ists.push_back(ist);
    }
    std::cout << "get " << ists.size() << " instances" << std::endl << std::endl;

    // CBS_Li::eecbs_MAPF // no leak
    // PBS_Li::pbs_MAPF   // no leak
    // LaCAM::lacam_MAPF  // no leak
    // LaCAM2::lacam2_MAPF // no leak
    // MAPF_LNS::LNS2_MAPF // no leak
    // MAPF_LNS::AnytimeBCBS_MAPF // no leak
    // MAPF_LNS::AnytimeEECBS_MAPF // no leak
    // CBSH2_RTC::CBSH2_RTC_MAPF // no leak
    // PIBT_2::pibt_MAPF // no leak
    // PIBT_2::pibt2_MAPF // no leak
    // PIBT_2::hca_MAPF // no leak
    // PIBT_2::push_and_swap_MAPF // no leak
    auto mapf_func = LaCAM::lacam_MAPF;

    Paths<2> multiple_paths;
    MemoryRecorder memory_recorder(1);
    float base_usage = 0, maximal_usage = 0;
    memory_recorder.clear();
    sleep(1);
    base_usage = memory_recorder.getCurrentMemoryUsage();
    gettimeofday(&tv_pre, &tz);

    auto pre =
            std::make_shared<PrecomputationOfMAPFDecomposition<2, LA_MAPF::HyperGraphNodeDataRaw<2>> >(ists, dim, is_occupied);

    auto ns_decompose = std::make_shared<MAPFInstanceDecompositionBreakLoop<2, LA_MAPF::HyperGraphNodeDataRaw<2>, Pointi<2> > >(dim,
                                                                                                                                pre->connect_graphs_,
                                                                                                                                pre->agent_sub_graphs_,
                                                                                                                                pre->heuristic_tables_sat_,
                                                                                                                                pre->heuristic_tables_,
                                                                                                                                60 - pre->initialize_time_cost_/1e3,
                                                                                                                                1e4,
                                                                                                                                100,
                                                                                                                                1);

    multiple_paths = freeNav::LayeredMAPF::layeredMAPF<2>(ists, dim, is_occupied, mapf_func, CBS_Li::eecbs_MAPF,
                                                          ns_decompose->all_levels_,  false, 60);
    gettimeofday(&tv_after, &tz);
    double layered_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
    std::cout << multiple_paths.size() << " paths " << " / agents " << ists.size() << std::endl;
    std::cout << "-- layered is solution valid ? " << validateSolution<2>(multiple_paths) << std::endl;
    int total_cost = 0, maximum_single_cost = 0;
    for(const auto& path : multiple_paths) {
        //std::cout << path << std::endl;
        total_cost += path.size();
        maximum_single_cost = std::max(maximum_single_cost, (int)path.size());
    }
    std::cout << "-- layered mapf end in " << layered_cost << "ms" << std::endl;
    std::cout << "-- layered sum of cost = " << total_cost << std::endl;
    std::cout << "-- layered makespan    = " << maximum_single_cost << std::endl;
    sleep(2);
    maximal_usage = memory_recorder.getMaximalMemoryUsage();
    std::cout << "-- layered mapf maximal usage = " << maximal_usage - base_usage << " MB" << std::endl << std::endl;

    memory_recorder.clear();
    sleep(2);
    base_usage = memory_recorder.getCurrentMemoryUsage();
    gettimeofday(&tv_pre, &tz);
    multiple_paths = mapf_func(dim, is_occupied_func, ists, nullptr, 60);
    gettimeofday(&tv_after, &tz);

    total_cost = 0;
    maximum_single_cost = 0;
    for(const auto& path : multiple_paths) {
        //std::cout << path << std::endl;
        total_cost += path.size();
        maximum_single_cost = std::max(maximum_single_cost, (int)path.size());
    }
    double build_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
    std::cout << multiple_paths.size() << " paths " << " / agents " << ists.size() << std::endl;
    std::cout << "-- raw is solution valid ? " << validateSolution<2>(multiple_paths) << std::endl;
    std::cout << "-- raw mapf end in " << build_cost << "ms" << std::endl;
    std::cout << "-- raw sum of cost = " << total_cost << std::endl;
    std::cout << "-- raw makespan    = " << maximum_single_cost << std::endl;
    sleep(2);
    maximal_usage = memory_recorder.getMaximalMemoryUsage();
    std::cout << "-- raw mapf maximal usage = " << maximal_usage - base_usage << " MB" << std::endl;
    return 0;

}

MemoryRecorder memory_recorder(1);


template<Dimension N>
bool decompositionOfSingleInstance(const freeNav::Instances<N>& ists, DimensionLength* dim,
                                   const IS_OCCUPIED_FUNC<N>& isoc, OutputStream& outputStream, int level=3) {

    struct timezone tz;
    struct timeval tv_pre;
    struct timeval tv_after;

    memory_recorder.clear();
    sleep(1);
    float basic_usage = memory_recorder.getMaximalMemoryUsage();
    gettimeofday(&tv_pre, &tz);
    freeNav::LayeredMAPF::MAPFInstanceDecompositionPtr<N> instance_decompose
            = std::make_shared<freeNav::LayeredMAPF::MAPFInstanceDecomposition<N> >(ists, dim, isoc, level);
    gettimeofday(&tv_after, &tz);
    sleep(1);
    float peak_usage = memory_recorder.getMaximalMemoryUsage();
    float memory_usage = peak_usage - basic_usage;
    double time_cost = (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3;
    bool is_legal = instance_decompose->decompositionValidCheck(instance_decompose->all_levels_);

    int max_cluster_size = 0, total_count = 0;
    for(int i=0; i<instance_decompose->all_levels_.size(); i++) {
        total_count += instance_decompose->all_levels_[i].size();
        if(instance_decompose->all_levels_[i].size() > max_cluster_size) {
            max_cluster_size = instance_decompose->all_levels_[i].size();
        }
        //if(all_clusters_[i].size() == 1) { continue; }
        //std::cout << "-- clusters " << i << " size " << all_clusters_[i].size() << ": " << all_clusters_[i] << std::endl;
    }
    assert(total_count == ists.size());
    outputStream.clear();
    std::stringstream ss;
    ss << " " << time_cost << " " << max_cluster_size << " " << ists.size() << " " << is_legal << " " << level << " " << memory_usage << " "
       << instance_decompose->all_levels_.size() << " "
    << instance_decompose->instance_decomposition_time_cost_ << " "
    << instance_decompose->cluster_decomposition_time_cost_ << " "
    << instance_decompose->sort_level_time_cost_ << " ";

    outputStream = ss.str();
    std::cout << " memory_usage = " << memory_usage << std::endl;
    return is_legal;
}


// level 5 is a flag to different from bipartition decomposition
template<Dimension N>
bool decompositionOfSingleInstanceBreakLoop(const freeNav::Instances<N>& ists, DimensionLength* dim,
                                            const IS_OCCUPIED_FUNC<N>& isoc, OutputStream& outputStream,
                                            double time_limit_s, int level=5) {


    memory_recorder.clear();
    sleep(1);
    float basic_usage = memory_recorder.getMaximalMemoryUsage();
    MSTimer mst;

    auto pre =
            std::make_shared<PrecomputationOfMAPFDecomposition<N, LA_MAPF::HyperGraphNodeDataRaw<2>> >(ists, dim, isoc);

    auto ns_decompose = std::make_shared<MAPFInstanceDecompositionBreakLoop<2, LA_MAPF::HyperGraphNodeDataRaw<2>, Pointi<2> > >(dim,
            pre->connect_graphs_,
            pre->agent_sub_graphs_,
            pre->heuristic_tables_sat_,
            pre->heuristic_tables_,
            time_limit_s - pre->initialize_time_cost_/1e3,
            1e4,
            10,
            1);

    double time_cost =  mst.elapsed()/1e3;

    sleep(1);
    float peak_usage = memory_recorder.getMaximalMemoryUsage();
    float memory_usage = peak_usage - basic_usage;
    bool is_legal = LA_MAPF::MAPF_DecompositionValidCheckGridMap<2>(ists, ns_decompose->all_levels_, dim, isoc);

    int max_subproblem = LA_MAPF::getMaxLevelSize(ns_decompose->all_levels_);
    int num_of_subproblem = ns_decompose->all_levels_.size();

    if(max_subproblem == 0) {
        max_subproblem = ists.size();
        num_of_subproblem = 1;
    }

    outputStream.clear();
    std::stringstream ss;
    ss << " " << time_cost << " "
       << max_subproblem << " "
       << ists.size() << " "
       << is_legal << " "
       << level
       << " "
       << memory_usage << " "
       << num_of_subproblem << " "
       << 0 << " "
       << 0 << " "
       << 0 << " "
       << 0 << " ";

    outputStream = ss.str();
    std::cout << " memory_usage = " << memory_usage << std::endl;
    return is_legal;
}

// level 5 is a flag to different from bipartition decomposition
template<Dimension N>
bool decompositionOfSingleInstanceBipartition(const freeNav::Instances<N>& ists, DimensionLength* dim,
                                              const IS_OCCUPIED_FUNC<N>& isoc, OutputStream& outputStream,
                                              double time_limit_s, int level=3) {


    memory_recorder.clear();
    sleep(1);
    float basic_usage = memory_recorder.getMaximalMemoryUsage();
    MSTimer mst;

    auto pre =
            std::make_shared<PrecomputationOfMAPFDecomposition<N, LA_MAPF::HyperGraphNodeDataRaw<2>> >(ists, dim, isoc);

    auto bi_decompose = std::make_shared<MAPFInstanceDecompositionBipartition<2, LA_MAPF::HyperGraphNodeDataRaw<2>, Pointi<2>> >(dim,
                        pre->connect_graphs_,
                        pre->agent_sub_graphs_,
                        pre->heuristic_tables_sat_,
                        pre->heuristic_tables_,
                        time_limit_s - pre->initialize_time_cost_/1e3,
                        level);

    double time_cost =  mst.elapsed()/1e3;

    sleep(1);
    float peak_usage = memory_recorder.getMaximalMemoryUsage();
    float memory_usage = peak_usage - basic_usage;
    bool is_legal = LA_MAPF::MAPF_DecompositionValidCheckGridMap<2>(ists, bi_decompose->all_levels_, dim, isoc);

    int max_subproblem = LA_MAPF::getMaxLevelSize(bi_decompose->all_levels_);
    int num_of_subproblem = bi_decompose->all_levels_.size();

    if(max_subproblem == 0) {
        max_subproblem = ists.size();
        num_of_subproblem = 1;
    }

    outputStream.clear();
    std::stringstream ss;
    ss << " " << time_cost << " "
       << max_subproblem << " "
       << ists.size() << " "
       << is_legal << " "
       << level
       << " "
       << memory_usage << " "
       << num_of_subproblem << " "
       << bi_decompose->instance_decomposition_time_cost_ << " "
       << bi_decompose->cluster_bipartition_time_cost_ << " "
       << bi_decompose->level_sorting_time_cost_ << " "
       << bi_decompose->level_bipartition_time_cost_ << " ";

    outputStream = ss.str();
    std::cout << " memory_usage = " << memory_usage << std::endl;
    return is_legal;
}

bool getProcessResourceUsage()
{
    FILE* fp = fopen("/proc/self/status", "r");
    char line[128];
    while(fgets(line, 128,fp) != NULL) {
        if(strncmp(line, "VmRSS:", 6) == 0) {
            std::cout << "mem size " << atoi(line + 6) << std::endl;
            break;
        }
    }
    return true;
}


int count_of_instance_total = 0;

bool SingleMapDecompositionTestMAPF(const SingleMapTestConfig <2> &map_test_config,
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
    ScenarioLoader2D sl(map_test_config.at("scene_path").c_str());
    int count_of_experiments = sl.GetNumExperiments();

    // 1, get all test instances
    const auto& insts_ids = freeNav::LayeredMAPF::pickCasesFromScene<2>(count_of_experiments, agent_in_instances, count_of_instance);

    InstancesS<2> istss;
    for(const auto& ids : insts_ids) {
        freeNav::Instances<2> ists;
        for(const int& id : ids) {
            const auto &experiment = sl.GetNthExperiment(id);
            Pointi<2> pt1({experiment.GetStartX(), experiment.GetStartY()});
            Pointi<2> pt2({experiment.GetGoalX(), experiment.GetGoalY()});
            freeNav::Instance<2> ist = {pt1, pt2};
            ists.push_back(ist);
        }
        istss.push_back(ists);
    }

    // 2, do decomposition for each case
    OutputStreamS output_streamss;
    for(int i=1; i<=istss.size(); i++) {
        const auto& ists = istss[i-1];
        output_streamss.clear();
        OutputStream ostream;
        std::cout << " start decomposition " << std::endl;

//        if(!decompositionOfSingleInstance<2>(ists, dim, is_occupied_func, ostream, 1)) {
//            std::cout << " decomposition failed " << std::endl;
//            return false;
//        }
//        std::cout << "-- finish level 0 decomposition(" << i <<"/" << istss.size() << ")" << std::endl;
//
//        output_streamss.push_back(ostream);
//        if(!decompositionOfSingleInstance<2>(ists, dim, is_occupied_func, ostream, 2)) {
//            std::cout << " decomposition failed " << std::endl;
//            return false;
//        }
//        std::cout << "-- finish level 1 decomposition(" << i <<"/" << istss.size() << ")" << std::endl;
//        output_streamss.push_back(ostream);
//
//        if(!decompositionOfSingleInstance<2>(ists, dim, is_occupied_func, ostream, 3)) {
//            std::cout << " decomposition failed " << std::endl;
//            return false;
//        }
//        std::cout << "-- finish level 2 decomposition(" << i <<"/" << istss.size() << ")" << std::endl;

//        if(!decompositionOfSingleInstanceBipartition<2>(ists, dim, is_occupied_func, ostream, timecost_limit_s, 3)) {
//            std::cout << " decomposition failed " << std::endl;
//            return false;
//        }
//        std::cout << "-- finish level 3 decomposition(" << i <<"/" << istss.size() << ")" << std::endl;
//        output_streamss.push_back(ostream);

        if(!decompositionOfSingleInstanceBipartition<2>(ists, dim, is_occupied_func, ostream, timecost_limit_s, 4)) {
            std::cout << " decomposition failed " << std::endl;
            return false;
        }
        std::cout << "-- finish level 4 decomposition(" << i <<"/" << istss.size() << ")" << std::endl;
        output_streamss.push_back(ostream);

        if(!decompositionOfSingleInstanceBreakLoop<2>(ists, dim, is_occupied_func, ostream, timecost_limit_s, 5)) {
            std::cout << " decomposition failed " << std::endl;
            return false;
        }
        std::cout << "-- finish level 5 decomposition(" << i <<"/" << istss.size() << ")" << std::endl;
        output_streamss.push_back(ostream);

        for(const auto& content : output_streamss) {
            appendToFile(map_test_config.at("decomposition_output_path"), content);
        }
    }

    return true;
}


std::vector<std::tuple<SingleMapTestConfig<2>, std::vector<int> > > test_configs = {
    {MAPFTestConfig_empty_32_32,            {10, 40, 80, 120, 160, 200, 240, 280, 320, 360, 400}}, // 1,
    {MAPFTestConfig_empty_16_16,            {10, 20, 40, 60, 80, 100, 120}}, // 2,
    {MAPFTestConfig_maze_32_32_2,           {20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120}}, // 3,
    {MAPFTestConfig_maze_32_32_4,           {60, 80, 100, 120, 140, 160, 180, 200, 220, 240, 260}}, // 4,
    {MAPFTestConfig_maze_128_128_2,         {100, 200, 300, 400, 500, 600, 700}}, // 5,
    {MAPFTestConfig_maze_128_128_10,        {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000}}, // 6,
    {MAPFTestConfig_den312d,                {100, 200, 300, 400, 500, 600, 700, 800}}, // 7,
    {MAPFTestConfig_den520d,                {100, 200, 300, 400, 500, 600, 700, 800, 900}}, // 8,
    {MAPFTestConfig_Berlin_1_256,           {100, 200, 300, 400, 500, 600, 700, 800, 900}}, // 9,
    {MAPFTestConfig_Paris_1_256,            {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000}}, // 10,
    {MAPFTestConfig_ht_chantry,             {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000}}, // 11,
    {MAPFTestConfig_lak303d,                {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000}}, // 12,
    {MAPFTestConfig_random_32_32_20,        {20, 40, 80, 120, 160, 200, 240}}, // 13,
    {MAPFTestConfig_random_64_64_20,        {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000}}, // 14,
    {MAPFTestConfig_room_32_32_4,           {10, 20, 40, 60, 80, 120, 160, 200}}, // 15,
    {MAPFTestConfig_room_64_64_8,           {100, 200, 300, 400, 500, 600, 700}}, // 16
    {MAPFTestConfig_room_64_64_16,          {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000}}, // 17,
    {MAPFTestConfig_warehouse_10_20_10_2_1, {100, 200, 300, 400, 500, 600, 700, 800}}, // 18,
    {MAPFTestConfig_warehouse_10_20_10_2_2, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000}}, // 19,
    {MAPFTestConfig_warehouse_20_40_10_2_1, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000}}, // 20,
    {MAPFTestConfig_warehouse_20_40_10_2_2, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000}}, // 21,
    {MAPFTestConfig_Boston_0_256,           {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000}}, // 22,
    {MAPFTestConfig_lt_gallowstemplar_n,    {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000}}, // 23,
    {MAPFTestConfig_ost003d,                {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000}} // 24,
};

std::vector<std::tuple<SingleMapTestConfig<2>, std::vector<int> > > test_configs_demo = {
        {MAPFTestConfig_empty_32_32,            {10}}, // 1,
        {MAPFTestConfig_empty_16_16,            {10}}, // 2,
        {MAPFTestConfig_maze_32_32_2,           {20}}, // 3,
        {MAPFTestConfig_maze_32_32_4,           {60}}, // 4,
        {MAPFTestConfig_maze_128_128_2,         {100}}, // 5,
        {MAPFTestConfig_maze_128_128_10,        {100}}, // 6,
        {MAPFTestConfig_den312d,                {100}}, // 7,
        {MAPFTestConfig_den520d,                {100}}, // 8,
        {MAPFTestConfig_Berlin_1_256,           {100}}, // 9,
        {MAPFTestConfig_Paris_1_256,            {100}}, // 10,
        {MAPFTestConfig_ht_chantry,             {100}}, // 11,
        {MAPFTestConfig_lak303d,                {100}}, // 12,
        {MAPFTestConfig_random_32_32_20,        {20}}, // 13,
        {MAPFTestConfig_random_64_64_20,        {100}}, // 14,
        {MAPFTestConfig_room_32_32_4,           {10}}, // 15,
        {MAPFTestConfig_room_64_64_8,           {100}}, // 16
        {MAPFTestConfig_room_64_64_16,          {100}}, // 17,
        {MAPFTestConfig_warehouse_10_20_10_2_1, {100}}, // 18,
        {MAPFTestConfig_warehouse_10_20_10_2_2, {100}}, // 19,
        {MAPFTestConfig_warehouse_20_40_10_2_1, {100}}, // 20,
        {MAPFTestConfig_warehouse_20_40_10_2_2, {100}}, // 21,
        {MAPFTestConfig_Boston_0_256,           {100}}, // 22,
        {MAPFTestConfig_lt_gallowstemplar_n,    {100}}, // 23,
        {MAPFTestConfig_ost003d,                {100}} // 24,
};

// do decomposition test
int main() {

    auto test_configs_copy = test_configs; // test_configs, test_configs_demo
    for(int i=0; i<10; i++) {
        int count_of_instances = 1;
        for (int map_id = 0; map_id < test_configs_copy.size(); map_id++) {
            SingleMapDecompositionTestMAPF(std::get<0>(test_configs_copy[map_id]),
                                           std::get<1>(test_configs_copy[map_id]),
                                           count_of_instances);
        }
    }
//    int interval = 2;
//    int repeat_times = 10;
//    int num_threads = (int)std::ceil((double)test_configs_copy.size()/interval);
//    std::vector<bool> finished(num_threads, false);
//    for(int j=0; j<num_threads; j++) {
//        auto lambda_func = [&]() {
//            int thread_id = j; // save to avoid change during planning
//            for(int i=0; i<repeat_times; i++) {
//                int count_of_instances = 1;
//                for (int k = 0; k < interval; k++) {
//                    //std::cout << "thread_id = " << thread_id << std::endl;
//                    int map_id = thread_id * interval + k;
//                    std::cout << "map_id = " << map_id << std::endl;
//                    if (thread_id * interval + k >= test_configs_copy.size()) { break; }
//                    SingleMapDecompositionTestMAPF(std::get<0>(test_configs_copy[map_id]),
//                                                   std::get<1>(test_configs_copy[map_id]),
//                                                   count_of_instances);
//                }
//            }
//            finished[thread_id] = true;
//            std::cout << "thread " << thread_id << " finished" << std::endl;
//        };
//        std::thread t(lambda_func);
//        t.detach();
//        sleep(1);
//    }
//    while(finished != std::vector<bool>(num_threads, true)) {
//        sleep(1);
//    }

    return 0;
}

