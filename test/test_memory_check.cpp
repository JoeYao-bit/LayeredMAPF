//
// Created by yaozhuo on 2024/1/26.
//

#include "../test/test_data.h"
//#include "multi-agent-path-finding/general_mapf_scene.h"

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

auto map_test_config = freeNav::LayeredMAPF::MAPFTestConfig_empty_32_32;

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
    // MAPF_LNS::LNS_MAPF // no leak
    // MAPF_LNS::AnytimeBCBS_MAPF // no leak
    // MAPF_LNS::AnytimeEECBS_MAPF // no leak
    // CBSH2_RTC::CBSH2_RTC_MAPF // no leak
    // PIBT_2::pibt_MAPF // no leak
    // PIBT_2::pibt2_MAPF // no leak
    // PIBT_2::hca_MAPF // no leak
    // PIBT_2::push_and_swap_MAPF // no leak
    auto mapf_func = PIBT_2::hca_MAPF;

    Paths<2> multiple_paths;
    MemoryRecorder memory_recorder(50);
    float base_usage = 0, maximal_usage = 0;
    memory_recorder.clear();
    sleep(1);
    base_usage = memory_recorder.getCurrentMemoryUsage();
    gettimeofday(&tv_pre, &tz);
    multiple_paths = freeNav::LayeredMAPF::layeredMAPF<2>(ists, dim, is_occupied, mapf_func, CBS_Li::eecbs_MAPF, false, 60);
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

template<Dimension N>
bool decompositionOfSingleInstance(const freeNav::Instances<N>& ists, DimensionLength* dim,
                                   const IS_OCCUPIED_FUNC<N>& isoc, OutputStream& outputStream) {

    struct timezone tz;
    struct timeval tv_pre;
    struct timeval tv_after;
    gettimeofday(&tv_pre, &tz);
    freeNav::LayeredMAPF::MAPFInstanceDecompositionPtr<N> instance_decompose
            = std::make_shared<freeNav::LayeredMAPF::MAPFInstanceDecomposition<N> >(ists, dim, isoc);
    gettimeofday(&tv_after, &tz);
    double time_cost = (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3;
    bool is_legal = instance_decompose->decompositionValidCheck(instance_decompose->all_clusters_);

    int max_cluster_size = 0, total_count = 0;
    for(int i=0; i<instance_decompose->all_clusters_.size(); i++) {
        total_count += instance_decompose->all_clusters_[i].size();
        if(instance_decompose->all_clusters_[i].size() > max_cluster_size) {
            max_cluster_size = instance_decompose->all_clusters_[i].size();
        }
        //if(all_clusters_[i].size() == 1) { continue; }
        //std::cout << "-- clusters " << i << " size " << all_clusters_[i].size() << ": " << all_clusters_[i] << std::endl;
    }
    assert(total_count == ists.size());
    outputStream.clear();
    std::stringstream ss;
    ss << time_cost << " " << max_cluster_size << " " << ists.size() << " " << is_legal;
    outputStream = ss.str();
    return is_legal;
}

bool SingleMapDecompositionTest(const SingleMapTestConfig <2> &map_test_config,
                                  const std::vector<int>& agent_in_instances,
                                  int count_of_instance) {
    // 0, load scenerio
    TextMapLoader tl(map_test_config.at("map_path"), is_char_occupied1);
    std::cout << "start SingleMapTest from map " << map_test_config.at("map_path") << std::endl;
    IS_OCCUPIED_FUNC<2> is_occupied_func;
    SET_OCCUPIED_FUNC<2> set_occupied_func;
    auto is_occupied = [&tl](const freeNav::Pointi<2> &pt) -> bool { return tl.isOccupied(pt); };
    is_occupied_func = is_occupied;
    auto set_occupied = [&tl](const freeNav::Pointi<2> &pt) { tl.setOccupied(pt); };
    set_occupied_func = set_occupied;
    //std::cout << " map name " << map_test_config.at("map_path") << std::endl;
    // load mapf scene
    //GeneralMAPFScenePtr<2> scene_ptr;
    const auto& dim = tl.getDimensionInfo();
//    freeNav::Instances<2> ists;
    ScenarioLoader2D sl(map_test_config.at("scene_path").c_str());
//    int max_count_of_case = atoi((map_test_config.at("agent_num")).c_str());
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
            //std::cout << " start, target = " << pt1 << ", " << pt2 << std::endl;
            ists.push_back(ist);
        }
        istss.push_back(ists);
    }

    // 2, do decomposition for each case
    OutputStreamS output_streamss;
    for(const auto& ists : istss) {
        OutputStream ostream;
        //std::cout << " start decomposition " << std::endl;
        if(!decompositionOfSingleInstance<2>(ists, dim, is_occupied_func, ostream)) {
            std::cout << " decomposition failed " << std::endl;
            return false;
        }
        std::cout << ostream << std::endl;
        output_streamss.push_back(ostream);
    }
    // 3, save result to file
    std::ofstream os(map_test_config.at("decomposition_output_path"), std::ios_base::out);
    //os << "TYPE START TARGET TIME_COST MAKE_SPAN TOTAL_LENGTH " << std::endl;
    for (const auto &output_streams : output_streamss) {
        os << output_streams << std::endl;
    }
    os.close();
    return true;
}



// do decomposition test
int main() {
    int count_of_instances = 100;
    SingleMapDecompositionTest(MAPFTestConfig_empty_32_32, {120, 140, 160}, count_of_instances); // layered better

    //SingleMapDecompositionTest(MAPFTestConfig_empty_32_32, {10, 40, 80, 120, 160, 200, 240, 280, 320, 360, 400}, count_of_instances); // good range
    //SingleMapDecompositionTest(MAPFTestConfig_random_32_32_20, {20, 40, 80, 120, 160, 200, 240, 240}, count_of_instances);  // good range
    //SingleMapDecompositionTest(MAPFTestConfig_warehouse_10_20_10_2_1, {100, 200, 300, 400, 500, 600, 700, 800}, count_of_instances); // good range
    //SingleMapDecompositionTest(MAPFTestConfig_maze_32_32_2, {20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120}, count_of_instances); // good range
    //SingleMapDecompositionTest(MAPFTestConfig_maze_32_32_4, {60, 80, 100, 120, 140, 160, 180, 200, 220, 240, 260}, count_of_instances); // good range
    //SingleMapDecompositionTest(MAPFTestConfig_den312d, {100, 200, 300, 400, 500, 600, 700, 800}, count_of_instances); //  good range
    //SingleMapDecompositionTest(MAPFTestConfig_Berlin_1_256, {100, 200, 300, 400, 500, 600, 700, 800, 900}, count_of_instances); // layered better

    //SingleMapDecompositionTest(MAPFTestConfig_Paris_1_256, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000}, count_of_instances); // layered better

    //SingleMapDecompositionTest(MAPFTestConfig_den520d, {100, 200, 300, 400, 500, 600, 700, 800, 900}, count_of_instances); // layered better
    return 0;
}

























