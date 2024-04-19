//
// Created by yaozhuo on 2023/12/19.
//

#include "../freeNav-base/dependencies/color_table.h"
#include "../freeNav-base/dependencies/2d_grid/text_map_loader.h"
#include "../freeNav-base/dependencies/memory_analysis.h"

#include "../algorithm/general_mapf_scene.h"
#include "../algorithm/layered_mapf.h"

#include "../test/test_data.h"
#include "../third_party/EECBS/inc/driver.h"
#include "../third_party/PBS/inc/driver.h"
#include "../third_party/CBSH2-RTC/inc/driver.h"
#include "../third_party/MAPF-LNS2/inc/driver.h"
#include "../third_party/lacam/include/planner.hpp"
#include "../third_party/lacam2/include/lacam2.hpp"
#include "../third_party/pibt2/include/driver.h"


using namespace freeNav;
using namespace freeNav::LayeredMAPF;


// MAPFTestConfig_random_32_32_20 413.8 ms / layered slower, but faster after 160 agent
// MAPFTestConfig_maze_32_32_2 25.42 ms / layered faster, after 58 agent
// MAPFTestConfig_maze_32_32_4 54.914 ms / layered faster
// MAPFTestConfig_den312d  314.776 ms / layered faster，after 250 agent
// MAPFTestConfig_Berlin_1_256 56.385 ms / layered faster
// MAPFTestConfig_Paris_1_256 1005.82 ms / layered faster
// MAPFTestConfig_warehouse_10_20_10_2_1  5726.96 ms / layered faster， after 500 agent
// MAPFTestConfig_den520d 237.842 ms / layered faster， after 150 agent
// MAPFTestConfig_empty_32_32 2872.3 ms / layered faster
auto map_test_config = MAPFTestConfig_random_32_32_20;

auto is_char_occupied = [](const char& value) -> bool {
    if (value == '.') return false;
    return true;
};

struct timezone tz;
struct timeval tv_pre, tv_cur;
struct timeval tv_after;

MemoryRecorder memory_recorder(1000);

// test_count: the total count of start and target pair in the scenario file
// required_count: required
std::vector<std::set<int> > pickCasesFromScene(int test_count,
                                               const std::vector<int>& required_counts,
                                               int instance_count) {
    std::vector<std::set<int> > retv;
    for(int i=0; i<instance_count; i++) {
        for(const int& required_count : required_counts) {
            std::set<int> instance;
            while(1) {
                int current_pick = rand() % test_count;
                if(instance.find(current_pick) == instance.end()) {
                    instance.insert(current_pick);
                    if(instance.size() == required_count) {
                        retv.push_back(instance);
                        break;
                    }
                }
            }
        }
    }
    return retv;
}

#define getMassiveTextMAPFFunc(name, mapf_func, dim, cutoff_time_cost)  \
       [&](DimensionLength*, const IS_OCCUPIED_FUNC<2> & isoc, const Instances<2> & ists, \
                     Paths<2>& paths, Statistic& statistic, OutputStream& outputStream) { \
        memory_recorder.clear(); \
        sleep(1);                                                                \
        float base_usage = memory_recorder.getCurrentMemoryUsage(); \
        statistic.clear(); \
        outputStream.clear(); \
        gettimeofday(&tv_pre, &tz); \
        paths = mapf_func(dim, isoc, ists, nullptr, cutoff_time_cost); \
        gettimeofday(&tv_after, &tz); \
        int total_cost = 0, maximum_single_cost = 0; \
        for(const auto& path : paths) { \
            total_cost += path.size(); \
            maximum_single_cost = std::max(maximum_single_cost, (int)path.size()); \
        } \
        double time_cost = (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3; \
        sleep(1);                 \
        float maximal_usage = memory_recorder.getMaximalMemoryUsage(); \
        if(1){ \
            std::cout << name << " maximal usage = " << maximal_usage - base_usage << " MB" << " with data size " << memory_recorder.getAllUsedMemory().size() << std::endl; \
        }                                                               \
        if(1) { \
            std::cout << name << " time_cost = " << time_cost << " ms" << std::endl; \
        } \
        std::stringstream ss; \
        ss << name << " " << ists.size() << " " << time_cost << " " \
           << total_cost << " " << maximum_single_cost << " " << !paths.empty() << " " << maximal_usage - base_usage << " "; \
        outputStream = ss.str();                                        \
        std::cout << name << " finish" << std::endl; \
    } \


#define getLayeredMassiveTextMAPFFuncDebug(name, mapf_func, dim, cutoff_time_cost, use_path_constraint)  \
       [&](DimensionLength*, const IS_OCCUPIED_FUNC<2> & isoc, const Instances<2> & ists, \
                     Paths<2>& paths, Statistic& statistic, OutputStream& outputStream) { \
        memory_recorder.clear(); \
        sleep(1);                                                                \
        float base_usage = memory_recorder.getCurrentMemoryUsage(); \
        statistic.clear(); \
        outputStream.clear(); \
        gettimeofday(&tv_pre, &tz); \
        paths = mapf_func(dim, isoc, ists, nullptr, cutoff_time_cost);                             \
        paths = layeredMAPF<2>(ists, dim, isoc, mapf_func, CBS_Li::eecbs_MAPF, use_path_constraint, cutoff_time_cost, false, true);                                                                                           \
        gettimeofday(&tv_after, &tz); \
        int total_cost = 0, maximum_single_cost = 0; \
        for(const auto& path : paths) { \
            total_cost += path.size(); \
            maximum_single_cost = std::max(maximum_single_cost, (int)path.size()); \
        } \
        double time_cost = (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3; \
        float maximal_usage = memory_recorder.getMaximalMemoryUsage(); \
        { \
            std::cout << name << " maximal usage = " << maximal_usage - base_usage << " MB" << " with data " << memory_recorder.getAllUsedMemory().size() << std::endl; \
        }                                                                                                \
        std::stringstream ss; \
        ss << name << " " << ists.size() << " " << time_cost << " " \
           << total_cost << " " << maximum_single_cost << " " << !paths.empty() << " " << maximal_usage << " "; \
        outputStream = ss.str();                                                                         \
        std::cout << name << " finish" << std::endl;        \
    } \
//
////        std::cout << "-- " << name << ": " << std::endl;
////        std::cout << "agents = " << ists.size() << std::endl;
////        std::cout << "time_cost = " << time_cost << " ms" << std::endl;
////        std::cout << "maximum_single_cost = " << maximum_single_cost << std::endl;
////        std::cout << "total_cost = " << total_cost << std::endl << std::endl;
//
#define getLayeredMassiveTextMAPFFunc(name, mapf_func, dim, cutoff_time_cost, use_path_constraint) \
                                     [&](DimensionLength*, const IS_OCCUPIED_FUNC<2> & isoc, \
                                     const Instances<2> & instances, \
                                     Paths<2>& paths, Statistic& statistic, OutputStream& outputStream) { \
        memory_recorder.clear();                                                                   \
        sleep(1);                                                                                           \
        float base_usage = memory_recorder.getCurrentMemoryUsage(); \
        statistic.clear(); \
        outputStream.clear(); \
        gettimeofday(&tv_pre, &tz); \
        MAPFInstanceDecompositionPtr<2> \
                instance_decompose = std::make_shared<MAPFInstanceDecomposition<2> >(instances, dim, isoc); \
        assert(instance_decompose->all_clusters_.size() >= 1);                \
        Paths<2> retv; \
        std::vector<Paths<2> > pathss;                                                             \
        CBS_Li::ConstraintTable* layered_ct = new CBS_Li::ConstraintTable(dim[0], dim[0]*dim[1]);  \
        for(int i=0; i<instance_decompose->all_clusters_.size(); i++) { \
            std::set<int> current_id_set = instance_decompose->all_clusters_[i]; \
            Instances<2> ists; \
            for(const int& id : current_id_set) { \
                ists.push_back({instances[id].first, instances[id].second}); \
            }                                                                                      \
            if (use_path_constraint && !pathss.empty()) { \
                for (const auto &previous_path : pathss.back()) { \
                    CBS_Li::MAPFPath path_eecbs; \
                    for (int i = 0; i < previous_path.size(); i++) { \
                        path_eecbs.push_back( \
                            CBS_Li::PathEntry(dim[0] * previous_path[i][1] + previous_path[i][0])); \
                    } \
                    layered_ct->insert2CT(path_eecbs); \
                } \
            } \
            std::vector<std::vector<bool> > avoid_locs(dim[1], std::vector<bool>(dim[0], false));        \
            for(int j = (use_path_constraint ? i+1 : 0); j<instance_decompose->all_clusters_.size(); j++) \
            { \
                if(j == i) continue; \
                const auto& current_cluster = instance_decompose->all_clusters_[j]; \
                for(const int& agent_id : current_cluster) { \
                    if(j < i) { \
                        avoid_locs[instances[agent_id].second[1]][instances[agent_id].second[0]] = true; \
                    } else { \
                        avoid_locs[instances[agent_id].first[1]][instances[agent_id].first[0]] = true; \
                    } \
                } \
            } \
            auto new_isoc = [&](const Pointi<2> & pt) -> bool {                                    \
                if(pt[0] < 0 || pt[0] >= dim[0] || pt[1] < 0 || pt[1] >= dim[1]) { return true; }      \
                return isoc(pt) || avoid_locs[pt[1]][pt[0]];                                           \
            };   \
            double remaining_time = cutoff_time_cost - (tv_after.tv_sec - tv_pre.tv_sec) + (tv_after.tv_usec - tv_pre.tv_usec)/1e6; \
            if(remaining_time < 0) {                                                               \
                if(layered_ct != nullptr) {                                                                                   \
                    delete layered_ct; \
                    layered_ct = nullptr;                                                              \
                }                                                        \
                retv.clear();                                                                    \
                break; \
            } \
            gettimeofday(&tv_after, &tz); \
            Paths<2> next_paths = mapf_func(dim, new_isoc, ists, layered_ct, remaining_time); \
            if(next_paths.empty()) { \
                std::cout << " layered MAPF failed " << i << " th cluster: " << current_id_set << std::endl; \
                if(layered_ct != nullptr) {                                                                                   \
                    delete layered_ct; \
                    layered_ct = nullptr;                                                              \
                }                                                        \
                retv.clear();      \
                break; \
            } \
            retv.insert(retv.end(), next_paths.begin(), next_paths.end()); \
            pathss.push_back(next_paths); \
        }                                                                                          \
        if(layered_ct != nullptr) {                                                                                   \
            delete layered_ct; \
            layered_ct = nullptr;                                                              \
        }          \
        if(pathss.empty())  {                                                                                         \
                                                                                                   \
                              }                                                                     \
        if(!use_path_constraint && !retv.empty()) { retv = multiLayerCompress(dim, pathss); } \
        paths = retv; \
        if(!paths.empty()) { \
            if(instances.size() != paths.size()) { \
                paths.clear(); \
            } \
        } \
        gettimeofday(&tv_after, &tz); \
        double time_cost = (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3; \
        int total_cost = 0, maximum_single_cost = 0; \
        for(const auto& path : paths) { \
            total_cost += path.size(); \
            maximum_single_cost = std::max(maximum_single_cost, (int)path.size()); \
        }                                                                                          \
        sleep(1);         \
        float maximal_usage = memory_recorder.getMaximalMemoryUsage(); \
        if(1){ \
            std::cout << name << " maximal usage = " << maximal_usage - base_usage << " MB" << " with data size " << memory_recorder.getAllUsedMemory().size()<< std::endl; \
        }                                                                                          \
        if(1) { \
            std::cout << name << " time_cost = " << time_cost << " ms" << std::endl; \
        }\
        std::stringstream ss; \
        ss << name << " " << instances.size() << " " << time_cost << " " \
           << total_cost << " " << maximum_single_cost << " " << !paths.empty() << " " << maximal_usage - base_usage << " " \
           << instance_decompose->cluster_decomposition_time_cost_ << " " \
           << instance_decompose->sort_level_time_cost_ << " "; \
        outputStream = ss.str();                                                                   \
        std::cout << name << " finish" << std::endl;\
    } \
//
//
////        std::cout << "-- EECBS_LAYERED: " << std::endl;
////        std::cout << "agents = " << ists.size() << std::endl;
////        std::cout << "time_cost = " << time_cost << " ms" << std::endl;
////        std::cout << "maximum_single_cost = " << maximum_single_cost << std::endl;
////        std::cout << "total_cost = " << total_cost << std::endl;
////        std::cout << "cluster_decomposition_time_cost = " << instance_decompose->cluster_decomposition_time_cost_ << " ms" << std::endl;
////        std::cout << "sort_level_time_cost = " << instance_decompose->sort_level_time_cost_ << " ms" << std::endl << std::endl;
//
//
//// 1, std::vector<int>: number of agents in instance
//// such as 10, 20, 30, 40, 50, 60
//// TODO: the number for each map is different
//// 2, and the number of combination is required too
//// 3, whether choose those agent randomly
//// cutoff_time_cost is in seconds

bool
SingleMapMAPFTest(const SingleMapTestConfig <2> &map_test_config,
                  const std::vector<int>& agent_in_instances,
                  int count_of_instance,
                  int cutoff_time_cost = 60,
                  bool prune=false) {


    TextMapLoader tl(map_test_config.at("map_path"), is_char_occupied);
    std::cout << "start SingleMapTest from map " << map_test_config.at("map_path") << std::endl;
    auto dimension = tl.getDimensionInfo();

    IS_OCCUPIED_FUNC<2> is_occupied_func;

    SET_OCCUPIED_FUNC<2> set_occupied_func;

    auto is_occupied = [&tl](const Pointi<2> &pt) -> bool { return tl.isOccupied(pt); };
    is_occupied_func = is_occupied;

    auto set_occupied = [&tl](const Pointi<2> &pt) { tl.setOccupied(pt); };
    set_occupied_func = set_occupied;

    //std::cout << " map name " << map_test_config.at("map_path") << std::endl;
    // load mapf scene
    //GeneralMAPFScenePtr<2> scene_ptr;
    const auto& dim = tl.getDimensionInfo();
//    Instances<2> ists;
    ScenarioLoader2D sl(map_test_config.at("scene_path").c_str());
//    int max_count_of_case = atoi((map_test_config.at("agent_num")).c_str());
    int count_of_experiments = sl.GetNumExperiments();
//
//    // load experiment data
//    for (int i = 0; i < std::min(max_count_of_case, count_of_experiments); i++) {
//        const auto &experiment = sl.GetNthExperiment(i);
//        Pointi<2> pt1({experiment.GetStartX(), experiment.GetStartY()});
//        Pointi<2> pt2({experiment.GetGoalX(), experiment.GetGoalY()});
//        Instance<2> ist = {pt1, pt2};
//        //std::cout << " start, target = " << pt1 << ", " << pt2 << std::endl;
//        ists.push_back(ist);
//    }
//    std::cout << "get " << ists.size() << " instances" << std::endl;
    // prepare instance for test
    //InstancesS<2> istss = { ists, ists, ists };

    const auto& insts_ids = pickCasesFromScene(count_of_experiments, agent_in_instances, count_of_instance);
//    std::cout << "RANDOM INSTANCE: " << std::endl;
//    for(const auto& ids : insts_ids) {
//        std::cout << ids << std::endl;
//    }

    InstancesS<2> istss;
    for(const auto& ids : insts_ids) {
        Instances<2> ists;
        for(const int& id : ids) {
            const auto &experiment = sl.GetNthExperiment(id);
            Pointi<2> pt1({experiment.GetStartX(), experiment.GetStartY()});
            Pointi<2> pt2({experiment.GetGoalX(), experiment.GetGoalY()});
            Instance<2> ist = {pt1, pt2};
            //std::cout << " start, target = " << pt1 << ", " << pt2 << std::endl;
            ists.push_back(ist);
        }
        istss.push_back(ists);
    }

    // load algorithms

#define RAW_TEST_TYPE getMassiveTextMAPFFunc
#define LAYERED_TEST_TYPE getLayeredMassiveTextMAPFFunc

    auto EECBS = RAW_TEST_TYPE("RAW_EECBS", CBS_Li::eecbs_MAPF, dim, cutoff_time_cost);
    // all layered mapf must start with "LAYERED_"
    auto EECBS_LAYERED = LAYERED_TEST_TYPE("LAYERED_EECBS", CBS_Li::eecbs_MAPF, dim, cutoff_time_cost, true);

    auto LaCAM = RAW_TEST_TYPE("RAW_LaCAM", LaCAM::lacam_MAPF, dim, cutoff_time_cost);
    // all layered mapf must start with "LAYERED_"
    auto LaCAM_LAYERED = LAYERED_TEST_TYPE("LAYERED_LaCAM", LaCAM::lacam_MAPF, dim, cutoff_time_cost, false);

    auto PBS = RAW_TEST_TYPE("RAW_PBS", PBS_Li::pbs_MAPF, dim, cutoff_time_cost);
    // all layered mapf must start with "LAYERED_"
    auto PBS_LAYERED = LAYERED_TEST_TYPE("LAYERED_PBS", PBS_Li::pbs_MAPF, dim, cutoff_time_cost, true);

    auto LaCAM2 = RAW_TEST_TYPE("RAW_LaCAM2", LaCAM2::lacam2_MAPF, dim, cutoff_time_cost);
    // all layered mapf must start with "LAYERED_"
    auto LaCAM2_LAYERED = LAYERED_TEST_TYPE("LAYERED_LaCAM2", LaCAM2::lacam2_MAPF, dim, cutoff_time_cost, false);

    auto LNS = RAW_TEST_TYPE("RAW_LNS", MAPF_LNS::LNS_MAPF, dim, cutoff_time_cost);
    // all layered mapf must start with "LAYERED_"
    auto LNS_LAYERED = LAYERED_TEST_TYPE("LAYERED_LNS", MAPF_LNS::LNS_MAPF, dim, cutoff_time_cost, true);

    auto AnytimeBCBS = RAW_TEST_TYPE("RAW_AnytimeBCBS", MAPF_LNS::AnytimeBCBS_MAPF, dim, cutoff_time_cost);
    // all layered mapf must start with "LAYERED_"
    auto AnytimeBCBS_LAYERED = LAYERED_TEST_TYPE("LAYERED_AnytimeBCBS", MAPF_LNS::AnytimeBCBS_MAPF, dim, cutoff_time_cost, true);

    auto AnytimeEECBS = RAW_TEST_TYPE("RAW_AnytimeEECBS", MAPF_LNS::AnytimeEECBS_MAPF, dim, cutoff_time_cost);
    // all layered mapf must start with "LAYERED_"
    auto AnytimeEECBS_LAYERED = LAYERED_TEST_TYPE("LAYERED_AnytimeEECBS", MAPF_LNS::AnytimeEECBS_MAPF, dim, cutoff_time_cost, true);

    auto CBSH2_RTC = RAW_TEST_TYPE("RAW_CBSH2_RTC", CBSH2_RTC::CBSH2_RTC_MAPF, dim, cutoff_time_cost);
    // all layered mapf must start with "LAYERED_"
    auto CBSH2_RTC_LAYERED = LAYERED_TEST_TYPE("LAYERED_CBSH2_RTC", CBSH2_RTC::CBSH2_RTC_MAPF, dim, cutoff_time_cost, true);

    auto PIBT = RAW_TEST_TYPE("RAW_PIBT", PIBT_2::pibt_MAPF, dim, cutoff_time_cost);
    // all layered mapf must start with "LAYERED_"
    auto PIBT_LAYERED = LAYERED_TEST_TYPE("LAYERED_PIBT", PIBT_2::pibt_MAPF, dim, cutoff_time_cost, false);

    auto PIBT2 = RAW_TEST_TYPE("RAW_PIBT2", PIBT_2::pibt2_MAPF, dim, cutoff_time_cost);
    // all layered mapf must start with "LAYERED_"
    auto PIBT2_LAYERED = LAYERED_TEST_TYPE("LAYERED_PIBT2", PIBT_2::pibt2_MAPF, dim, cutoff_time_cost, false);

    auto HCA = RAW_TEST_TYPE("RAW_HCA", PIBT_2::hca_MAPF, dim, cutoff_time_cost);
    // all layered mapf must start with "LAYERED_"
    auto HCA_LAYERED = LAYERED_TEST_TYPE("LAYERED_HCA", PIBT_2::hca_MAPF, dim, cutoff_time_cost, true);

    auto PushAndSwap = RAW_TEST_TYPE("RAW_PushAndSwap", PIBT_2::push_and_swap_MAPF, dim, cutoff_time_cost);
    // all layered mapf must start with "LAYERED_"
    auto PushAndSwap_LAYERED = LAYERED_TEST_TYPE("LAYERED_PushAndSwap", PIBT_2::push_and_swap_MAPF, dim, cutoff_time_cost, false);

    bool all_success = SingleMapMAPFPathPlanningsTest<2>(dim, is_occupied_func, istss,
                                                         {
//                                                          EECBS,
//                                                          EECBS_LAYERED,
//
//                                                          PBS,
//                                                          PBS_LAYERED,
//
                                                          LNS,
                                                          LNS_LAYERED, // unexccepted assert failed abut MDD

//                                                          AnytimeBCBS,
//                                                          AnytimeBCBS_LAYERED,
//
//                                                          AnytimeEECBS,
//                                                          AnytimeEECBS_LAYERED,

//                                                          CBSH2_RTC,            // cause unexpected exit
//                                                          CBSH2_RTC_LAYERED,    // cause unexpected exit

//                                                          LaCAM,               // need lots storage
//                                                          LaCAM_LAYERED,       // need lots storage

//                                                          LaCAM2,              // added massive test
//                                                          LaCAM2_LAYERED,

//                                                          PIBT,            // need lots storage
//                                                          PIBT_LAYERED,    // need lots storage

//                                                          PIBT2,
//                                                          PIBT2_LAYERED,

//                                                          HCA,             // need lots storage
//                                                          HCA_LAYERED,     // need lots storage

//                                                          PushAndSwap,
//                                                          PushAndSwap_LAYERED
                                                          },
                                                         map_test_config.at("output_path"),
                                                         prune);

//    std::ofstream os(map_test_config.at("output_path"), std::ios_base::out);
//    //os << "TYPE START TARGET TIME_COST MAKE_SPAN TOTAL_LENGTH " << std::endl;
//    for (const auto &multi_method_output : output_streamss) {
//        for (const auto method_output : multi_method_output) {
//            os << method_output << std::endl;
//        }
//    }
//    os.close();
    return all_success;
}

//
////MAPFTestConfig_random_32_32_20        {45 ~ 150}
////MAPFTestConfig_maze_32_32_2           { ~ }
////MAPFTestConfig_maze_32_32_4           { ~ }
////MAPFTestConfig_den312d                {105 ~ 210}
////MAPFTestConfig_Berlin_1_256           { ~ }
////MAPFTestConfig_Paris_1_256            {300 ~ 1000}
////MAPFTestConfig_warehouse_10_20_10_2_1 {150 ~ 360}
////MAPFTestConfig_den520d                {150 ~ 500}
////MAPFTestConfig_empty_32_32            {180 ~ 320}
//
//SingleMapTestConfigs<2> configs = {
//     MAPFTestConfig_random_32_32_20, // 413.8 ms // layered slower, but faster after 160 agent
//     MAPFTestConfig_maze_32_32_2,// 25.42 ms / layered faster, after 58 agent
//     MAPFTestConfig_maze_32_32_4, // 54.914 ms / layered faster
//     MAPFTestConfig_den312d, //  314.776 ms / layered faster，after 250 agent
//     MAPFTestConfig_Berlin_1_256, // 56.385 ms / layered faster
//     MAPFTestConfig_Paris_1_256, // 1005.82 ms / layered faster
//     MAPFTestConfig_warehouse_10_20_10_2_1, //  5726.96 ms / layered faster， after 500 agent
//     MAPFTestConfig_den520d, // 237.842 ms / layered faster， after 150 agent
//     MAPFTestConfig_empty_32_32 // 2872.3 ms / layered faster
//};
//

// each method have a common range of agents
int main(void) {
////    configs = {
////            MAPFTestConfig_empty_32_32
////    };
//    SingleMapMAPFTest(MAPFTestConfig_empty_32_32, {10, 20, 30}, 2, 60); // layered better

    int cut_off_time = 30;
    int repeat_times = 5;
    for(int i=0; i<1; i++) {
        //SingleMapMAPFTest(MAPFTestConfig_empty_16_16, {10, 20, 40, 60, 80, 100, 120}, 10, 60); // layered better

//        SingleMapMAPFTest(MAPFTestConfig_empty_16_16, {10, 20, 40, 60, 80, 100, 120},
//                          10, 60); // good range
//        SingleMapMAPFTest(MAPFTestConfig_empty_32_32, {10, 40, 80, 120, 160, 200, 240, 280, 320, 360, 400},
//                          repeat_times, 60); // good range

        SingleMapMAPFTest(MAPFTestConfig_maze_32_32_2, {20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120},
                          repeat_times, cut_off_time); // good range 40, 60, 80,
        SingleMapMAPFTest(MAPFTestConfig_maze_32_32_4, {20, 40, 80, 120, 160, 200, 240},
                          repeat_times, cut_off_time); // good range / PushAndSwapFailed here
//        SingleMapMAPFTest(MAPFTestConfig_maze_128_128_2, {100, 200, 300, 400, 500, 600, 700},
//                          repeat_times, cut_off_time);
////      warn@ PushAndSwap: invalid move due to clear operation
////error@PushAndSwap: vertex conflict
//
//        SingleMapMAPFTest(MAPFTestConfig_maze_128_128_10, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000},
//                          repeat_times, cut_off_time); // good range
//
//
//        SingleMapMAPFTest(MAPFTestConfig_den312d, {100, 200, 300, 400, 500, 600, 700, 800},
//                          repeat_times, cut_off_time); //  good range
//        SingleMapMAPFTest(MAPFTestConfig_den520d, {100, 200, 300, 400, 500, 600, 700, 800, 900},
//                          repeat_times, cut_off_time); // layered better
//
//        SingleMapMAPFTest(MAPFTestConfig_Berlin_1_256, {100, 200, 300, 400, 500, 600, 700, 800, 900},
//                          repeat_times, cut_off_time); // layered better
//        SingleMapMAPFTest(MAPFTestConfig_Paris_1_256, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000},
//                          repeat_times, cut_off_time); // layered better
//
//        SingleMapMAPFTest(MAPFTestConfig_ht_chantry, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000},
//                          repeat_times, cut_off_time);
//        SingleMapMAPFTest(MAPFTestConfig_lak303d, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000},
//                          repeat_times, cut_off_time); // good range
//
//        SingleMapMAPFTest(MAPFTestConfig_random_32_32_20, {20, 40, 80, 120, 160, 200, 240},
//                          repeat_times, cut_off_time);  // good range
//
//        SingleMapMAPFTest(MAPFTestConfig_random_64_64_10, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000},
//                          repeat_times, cut_off_time); // good range // decomposition maximum 8s
//
//        SingleMapMAPFTest(MAPFTestConfig_random_64_64_20, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000},
//                          repeat_times, cut_off_time); // good range
//////                          / PushAndSwapFailed here
//////                          warn@ PushAndSwap: invalid move due to clear operation
//////                          error@PushAndSwap: vertex conflict
////
//        SingleMapMAPFTest(MAPFTestConfig_room_64_64_16, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000},
//                          repeat_times, cut_off_time); // good range
//        SingleMapMAPFTest(MAPFTestConfig_room_64_64_8, {100, 200, 300, 400, 500, 600, 700},
//                          repeat_times, cut_off_time);
//        SingleMapMAPFTest(MAPFTestConfig_room_32_32_4, {10, 20, 40, 60, 80, 120, 160, 200}, repeat_times, cut_off_time);
//
//        //  100, 200, 300, 400,
//        SingleMapMAPFTest(MAPFTestConfig_warehouse_10_20_10_2_1, {100, 200, 300, 400, 500, 600, 700, 800}, repeat_times, cut_off_time); // good range
//        //                          / PushAndSwapFailed here
//        //                          warn@ PushAndSwap: invalid move due to clear operation
//        //                          error@PushAndSwap: vertex conflict
//
//        SingleMapMAPFTest(MAPFTestConfig_warehouse_10_20_10_2_2, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000}, repeat_times, cut_off_time); // good range
//
//        SingleMapMAPFTest(MAPFTestConfig_warehouse_20_40_10_2_1, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000}, repeat_times, cut_off_time); // good range
//        SingleMapMAPFTest(MAPFTestConfig_warehouse_20_40_10_2_2, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000}, repeat_times, cut_off_time); // good range
//
//
//        SingleMapMAPFTest(MAPFTestConfig_Boston_0_256, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000}, repeat_times, cut_off_time); // good range
//
//        SingleMapMAPFTest(MAPFTestConfig_lt_gallowstemplar_n, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000}, repeat_times, cut_off_time); // good range
//        //MAPFTestConfig_ost003d
//        SingleMapMAPFTest(MAPFTestConfig_ost003d, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000}, repeat_times, cut_off_time); // good range

    }

    //
    return 0;
}




bool SingleMapAndMethodMAPFTest(const MAPF_FUNC<2>& mapf_func, const std::string& raw_name, const std::string& layered_name,
                                const SingleMapTestConfig <2> &map_test_config,
                                const std::vector<int>& agent_config,
                                bool use_path_constraint = false,
                                int cutoff_time_cost = 60,
                                bool prune=false) {
    assert(agent_config.size() == 4);
    TextMapLoader tl(map_test_config.at("map_path"), is_char_occupied);
    std::cout << "start SingleMapTest from map " << map_test_config.at("map_path") << std::endl;
    auto dimension = tl.getDimensionInfo();

    IS_OCCUPIED_FUNC<2> is_occupied_func;

    SET_OCCUPIED_FUNC<2> set_occupied_func;

    auto is_occupied = [&tl](const Pointi<2> &pt) -> bool { return tl.isOccupied(pt); };
    is_occupied_func = is_occupied;

    auto set_occupied = [&tl](const Pointi<2> &pt) { tl.setOccupied(pt); };
    set_occupied_func = set_occupied;

    const auto& dim = tl.getDimensionInfo();
    ScenarioLoader2D sl(map_test_config.at("scene_path").c_str());
    int count_of_experiments = sl.GetNumExperiments();

    std::vector<int> agent_in_instances;
    int base_agent_count = agent_config[0];
    while(true) {
        if(base_agent_count < agent_config[1]) {
            agent_in_instances.push_back(base_agent_count);
            base_agent_count = base_agent_count + agent_config[2];
        } else { break; }
    }

    const auto& insts_ids = pickCasesFromScene(count_of_experiments, agent_in_instances, agent_config[3]);
    InstancesS<2> istss;
    for(const auto& ids : insts_ids) {
        Instances<2> ists;
        for(const int& id : ids) {
            const auto &experiment = sl.GetNthExperiment(id);
            Pointi<2> pt1({experiment.GetStartX(), experiment.GetStartY()});
            Pointi<2> pt2({experiment.GetGoalX(), experiment.GetGoalY()});
            Instance<2> ist = {pt1, pt2};
            //std::cout << " start, target = " << pt1 << ", " << pt2 << std::endl;
            ists.push_back(ist);
        }
        istss.push_back(ists);
    }

    auto RAW_MAPF = getMassiveTextMAPFFunc(raw_name, mapf_func, dim, cutoff_time_cost);
    // all layered mapf must start with "LAYERED_"
    auto LAYERED_MAPF = getLayeredMassiveTextMAPFFunc(layered_name, mapf_func, dim, cutoff_time_cost, use_path_constraint);

    bool all_success = SingleMapMAPFPathPlanningsTest<2>(dim, is_occupied_func, istss,
                                                         {RAW_MAPF, LAYERED_MAPF},
                                                         map_test_config.at("output_path"),
                                                          prune);

    return all_success;
}

// method name and respective min/max num of agents, interval, repeat times
MethodAndAgentConfigs MethodAndAgentConfigs_empty_16_16 =
        {
          {CBS_Li::eecbs_MAPF, {{10, 120, 20, 5},"RAW_EECBS", "LAYERED_EECBS", true}},
          {PBS_Li::pbs_MAPF, {{10, 120, 20, 5},"RAW_PBS", "LAYERED_PBS", false}},
          {MAPF_LNS::LNS_MAPF, {{10, 120, 20, 10},"RAW_LNS", "LAYERED_LNS", true}},  // unexpected exit result by mdd level size < path size
          {MAPF_LNS::AnytimeBCBS_MAPF,{{10, 120, 20, 5},"RAW_AnytimeBCBS", "LAYERED_AnytimeBCBS", true}},
          {MAPF_LNS::AnytimeEECBS_MAPF, {{10, 120, 20, 5},"RAW_AnytimeEECBS", "LAYERED_AnytimeEECBS", true}},
          //{CBSH2_RTC::CBSH2_RTC_MAPF, {{10, 100, 10, 5},"RAW_AnytimeEECBS", "LAYERED_AnytimeEECBS", true}}, // unexpected exit
          //{PIBT_2::pibt_MAPF, {{10, 100, 10, 10},"RAW_PIBT", "LAYERED_PIBT", false}}, // need lots storage
          {PIBT_2::pibt2_MAPF, {{10, 120, 20, 5},"RAW_PIBT2", "LAYERED_PIBT2", false}},
          {PIBT_2::hca_MAPF, {{10, 100, 10, 10},"RAW_HCA", "LAYERED_HCA", false}}, // need storage
          {PIBT_2::push_and_swap_MAPF, {{10, 120, 20, 5},"RAW_PushAndSwap", "LAYERED_PushAndSwap", false}},
          //{LaCAM::lacam_MAPF, {{10, 100, 10, 10},"RAW_LaCAM", "LAYERED_LaCAM"}, false}, // need lots storage
          {LaCAM2::lacam2_MAPF, {{10, 120, 20, 5},"RAW_LaCAM2", "LAYERED_LaCAM2", false}},
          };

MapMAPFTestConfig MAPFConfig_empty_16_16 =
        {{MAPFTestConfig_empty_16_16, MethodAndAgentConfigs_empty_16_16}};

// each method have it own range of agents

int main2(void) {

    MapMAPFTestConfigs all_MapMAPFConfig = {MAPFConfig_empty_16_16};

    for(const auto& map_mapf_config : all_MapMAPFConfig) {
        for(const auto& mapf_and_config : map_mapf_config) {
            const SingleMapTestConfig<2>& map_config = mapf_and_config.first;
            for(const  auto& method_and_agent_config : mapf_and_config.second) {
                const MAPF_FUNC<2>& mapf_func         = method_and_agent_config.first;
                const std::string& raw_name           = method_and_agent_config.second.raw_name_;
                const std::string& layered_name       = method_and_agent_config.second.layered_name_;
                const std::vector<int>& agent_configs = method_and_agent_config.second.agent_configs_;
                const bool& use_path_constraint       = method_and_agent_config.second.use_path_constraint_;
                bool all_success = SingleMapAndMethodMAPFTest(mapf_func, raw_name, layered_name,
                                                              map_config, agent_configs,
                                                              use_path_constraint, 60, false);
            }
        }
    }

    return 0;
}


