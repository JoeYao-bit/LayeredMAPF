//
// Created by yaozhuo on 2023/12/19.
//

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

MemoryRecorder memory_recorder(100);

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
        float maximal_usage = memory_recorder.getMaximalMemoryUsage(); \
        if(0){ \
            std::cout << name << " maximal usage = " << maximal_usage - base_usage << " MB" << " with data " << memory_recorder.getAllUsedMemory().size() << std::endl; \
        }                                                               \
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
            auto new_isoc = [&](const Pointi<2> & pt) -> bool { return isoc(pt) || avoid_locs[pt[1]][pt[0]]; };   \
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
        } \
        float maximal_usage = memory_recorder.getMaximalMemoryUsage(); \
        if(0){ \
            std::cout << name << " maximal usage = " << maximal_usage - base_usage << " MB" << " with data " << memory_recorder.getAllUsedMemory().size()<< std::endl; \
        }                                                                                          \
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
                  int cutoff_time_cost = 60) {


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
    auto PBS_LAYERED = LAYERED_TEST_TYPE("LAYERED_PBS", PBS_Li::pbs_MAPF, dim, cutoff_time_cost, false);

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
    auto HCA_LAYERED = LAYERED_TEST_TYPE("LAYERED_HCA", PIBT_2::hca_MAPF, dim, cutoff_time_cost, false);

    auto PushAndSwap = RAW_TEST_TYPE("RAW_PushAndSwap", PIBT_2::push_and_swap_MAPF, dim, cutoff_time_cost);
    // all layered mapf must start with "LAYERED_"
    auto PushAndSwap_LAYERED = LAYERED_TEST_TYPE("LAYERED_PushAndSwap", PIBT_2::push_and_swap_MAPF, dim, cutoff_time_cost, false);


    StatisticSS statisticss;
    OutputStreamSS output_streamss;

    bool all_success = SingleMapMAPFPathPlanningsTest<2>(dim, is_occupied_func, istss,
                                                         {
                                                          EECBS,
                                                          EECBS_LAYERED,
//                                                          PBS,
//                                                          PBS_LAYERED,

//                                                          LNS,
//                                                          LNS_LAYERED,
//                                                          AnytimeBCBS,
//                                                          AnytimeBCBS_LAYERED,
//                                                          AnytimeEECBS,
//                                                          AnytimeEECBS_LAYERED,
//                                                          CBSH2_RTC,
//                                                          CBSH2_RTC_LAYERED,

//                                                          LaCAM,
//                                                          LaCAM_LAYERED,
//                                                          LaCAM2,
//                                                          LaCAM2_LAYERED,

//                                                          PIBT,
//                                                          PIBT_LAYERED,
//                                                          PIBT2,
//                                                          PIBT2_LAYERED,

//                                                          HCA,
//                                                          HCA_LAYERED,
//                                                          PushAndSwap,
//                                                          PushAndSwap_LAYERED
                                                          },
                                                          statisticss,
                                                          output_streamss);

    std::ofstream os(map_test_config.at("output_path"), std::ios_base::out);
    //os << "TYPE START TARGET TIME_COST MAKE_SPAN TOTAL_LENGTH " << std::endl;
    for (const auto &multi_method_output : output_streamss) {
        for (const auto method_output : multi_method_output) {
            os << method_output << std::endl;
        }
    }
    os.close();
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
int main(void) {
////    configs = {
////            MAPFTestConfig_empty_32_32
////    };
//    SingleMapMAPFTest(MAPFTestConfig_empty_32_32, {10, 20, 30}, 2, 60); // layered better
    SingleMapMAPFTest(MAPFTestConfig_empty_32_32, {200, 240, 280, 320, 360}, 1, 60); // layered better

//    SingleMapMAPFTest(MAPFTestConfig_random_32_32_20, {80, 100, 130, 150, 180}, 10, 10); // layered better
//    SingleMapMAPFTest(MAPFTestConfig_warehouse_10_20_10_2_1, {300, 350, 400, 450, 500}, 10, 60); //  layered worse
//    SingleMapMAPFTest(MAPFTestConfig_maze_32_32_2, {30, 40, 50, 60, 70}, 10, 60); // layered better
//    SingleMapMAPFTest(MAPFTestConfig_maze_32_32_4, {40, 50, 60, 70, 80, 90}, 10, 60); // layered better
//    SingleMapMAPFTest(MAPFTestConfig_den312d, {200, 220, 240, 260, 280}, 10, 60); // layered better
//    SingleMapMAPFTest(MAPFTestConfig_Berlin_1_256, {500, 600, 700, 800, 900}, 10, 60); // layered better
//    SingleMapMAPFTest(MAPFTestConfig_Paris_1_256, {600, 700, 800, 900, 1000}, 10, 60); // layered better
//    SingleMapMAPFTest(MAPFTestConfig_den520d, {400, 500, 600, 700, 800, 900}, 10, 60); // layered better
//
    return 0;
}
