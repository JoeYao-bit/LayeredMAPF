////
//// Created by yaozhuo on 2023/12/19.
////
//
//#include "3d_viewer/3d_viewer.h"
//#include "dependencies/test_data.h"
//#include "multi-agent-path-finding/general_mapf_scene.h"
//#include "dependencies/color_table.h"
//#include <2d_grid/text_map_loader.h>
//#include "EECBS/inc/driver.h"
//#include "my_eecbs/driver_my.h"
//#include "my_cbs/space_time_astar.h"
//#include "second_EECBS/driver_second.h"
//#include "layered_mapf/layered_mapf.h"
//#include "dependencies/memory_analysis.h"
//#include "lacam/include/planner.hpp"
//
//using namespace freeNav;
//using namespace freeNav::RimJump;
//using namespace freeNav::CBS;
//using namespace freeNav::TCBS;
//
//// MAPFTestConfig_random_32_32_20 413.8 ms / layered slower, but faster after 160 agent
//// MAPFTestConfig_maze_32_32_2 25.42 ms / layered faster, after 58 agent
//// MAPFTestConfig_maze_32_32_4 54.914 ms / layered faster
//// MAPFTestConfig_den312d  314.776 ms / layered faster，after 250 agent
//// MAPFTestConfig_Berlin_1_256 56.385 ms / layered faster
//// MAPFTestConfig_Paris_1_256 1005.82 ms / layered faster
//// MAPFTestConfig_warehouse_10_20_10_2_1  5726.96 ms / layered faster， after 500 agent
//// MAPFTestConfig_den520d 237.842 ms / layered faster， after 150 agent
//// MAPFTestConfig_empty_32_32 2872.3 ms / layered faster
//auto map_test_config = freeNav::RimJump::MAPFTestConfig_maze_32_32_2;
//
//auto is_char_occupied = [](const char& value) -> bool {
//    if (value == '.') return false;
//    return true;
//};
//
//struct timezone tz;
//struct timeval tv_pre, tv_cur;
//struct timeval tv_after;
//
//MemoryRecorder memory_recorder(1000);
//
//// test_count: the total count of start and target pair in the scenario file
//// required_count: required
//std::vector<std::set<int> > pickCasesFromScene(int test_count,
//                                               const std::vector<int>& required_counts,
//                                               int instance_count) {
//    std::vector<std::set<int> > retv;
//    for(int i=0; i<instance_count; i++) {
//        for(const int& required_count : required_counts) {
//            std::set<int> instance;
//            while(1) {
//                int current_pick = rand() % test_count;
//                if(instance.find(current_pick) == instance.end()) {
//                    instance.insert(current_pick);
//                    if(instance.size() == required_count) {
//                        retv.push_back(instance);
//                        break;
//                    }
//                }
//            }
//        }
//    }
//    return retv;
//}
//
//#define getMassiveTextMAPFFunc(name, mapf_func, dim, cutoff_time_cost)  \
//       [&](DimensionLength*, const IS_OCCUPIED_FUNC<2> & isoc, const freeNav::Instances<2> & ists, \
//                     Paths<2>& paths, Statistic& statistic, OutputStream& outputStream) { \
//        memory_recorder.clear(); \
//        float base_usage = memory_recorder.getCurrentMemoryUsage(); \
//        statistic.clear(); \
//        outputStream.clear(); \
//        gettimeofday(&tv_pre, &tz); \
//        paths = mapf_func(dim, isoc, ists, {}, cutoff_time_cost); \
//        gettimeofday(&tv_after, &tz); \
//        int total_cost = 0, maximum_single_cost = 0; \
//        for(const auto& path : paths) { \
//            total_cost += path.size(); \
//            maximum_single_cost = std::max(maximum_single_cost, (int)path.size()); \
//        } \
//        double time_cost = (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3; \
//        std::stringstream ss; \
//        ss << name << " " << ists.size() << " " << time_cost << " " \
//           << total_cost << " " << maximum_single_cost << " " << !paths.empty() << " "; \
//        outputStream = ss.str(); \
//        float maximal_usage = memory_recorder.getMaximalMemoryUsage(); \
//        { \
//            std::cout << name << " maximal usage = " << maximal_usage - base_usage << " MB" << std::endl; \
//        }                                                               \
//    } \
//
//
////        std::cout << "-- " << name << ": " << std::endl;
////        std::cout << "agents = " << ists.size() << std::endl;
////        std::cout << "time_cost = " << time_cost << " ms" << std::endl;
////        std::cout << "maximum_single_cost = " << maximum_single_cost << std::endl;
////        std::cout << "total_cost = " << total_cost << std::endl << std::endl;
//
//#define getLayeredMassiveTextMAPFFunc(name, mapf_func, dim, cutoff_time_cost) \
//    [&](DimensionLength*, const IS_OCCUPIED_FUNC<2> & isoc, \
//                                     const freeNav::Instances<2> & ists, \
//                                     Paths<2>& paths, Statistic& statistic, OutputStream& outputStream) { \
//        memory_recorder.clear(); \
//        float base_usage = memory_recorder.getCurrentMemoryUsage(); \
//        statistic.clear(); \
//        outputStream.clear(); \
//        gettimeofday(&tv_pre, &tz); \
//        freeNav::TCBS::MAPFInstanceDecompositionPtr<2> \
//                instance_decompose = std::make_shared<freeNav::TCBS::MAPFInstanceDecomposition<2> >(ists, dim, isoc); \
//        assert(instance_decompose->all_clusters_.size() >= 1); \
//        Paths<2> retv; \
//        for(int i=0; i<instance_decompose->all_clusters_.size(); i++) { \
//            std::set<int> current_id_set = instance_decompose->all_clusters_[i]; \
//            freeNav::Instances<2> current_ists; \
//            for(const int& id : current_id_set) { \
//                current_ists.push_back({ists[id].first, ists[id].second}); \
//            } \
//            gettimeofday(&tv_cur, &tz); \
//            double accumulate_cost = (tv_cur.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_cur.tv_usec - tv_pre.tv_usec) / 1e3; \
//            if(accumulate_cost >= cutoff_time_cost*1e3) { \
//                paths.clear(); \
//                break; \
//            } \
//            Paths<2> next_paths = mapf_func(dim, isoc, current_ists, retv, cutoff_time_cost-accumulate_cost/1e3); \
//            if(next_paths.empty()) { \
//                paths.clear(); \
//                break; \
//            } \
//            retv.insert(retv.end(), next_paths.begin(), next_paths.end()); \
//        } \
//        paths = retv; \
//        if(!paths.empty()) { \
//            if(ists.size() != paths.size()) { \
//                paths.clear(); \
//            } \
//        } \
//        gettimeofday(&tv_after, &tz); \
//        double time_cost = (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3; \
//        int total_cost = 0, maximum_single_cost = 0; \
//        for(const auto& path : paths) { \
//            total_cost += path.size(); \
//            maximum_single_cost = std::max(maximum_single_cost, (int)path.size()); \
//        } \
//        std::stringstream ss; \
//        ss << name << " " << ists.size() << " " << time_cost << " " \
//           << total_cost << " " << maximum_single_cost << " " << !paths.empty() << " " \
//           << instance_decompose->cluster_decomposition_time_cost_ << " " \
//           << instance_decompose->sort_level_time_cost_ << " "; \
//        outputStream = ss.str(); \
//       float maximal_usage = memory_recorder.getMaximalMemoryUsage(); \
//        { \
//            std::cout << name << " maximal usage = " << maximal_usage - base_usage << " MB" << std::endl; \
//        } \
//    } \
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
//bool
//SingleMapMAPFTest(const SingleMapTestConfig <2> &map_test_config,
//                  const std::vector<int>& agent_in_instances,
//                  int count_of_instance,
//                  int cutoff_time_cost = 60) {
//
//
//    TextMapLoader tl(map_test_config.at("map_path"), is_char_occupied);
//    std::cout << "start SingleMapTest from map " << map_test_config.at("map_path") << std::endl;
//    auto dimension = tl.getDimensionInfo();
//
//    IS_OCCUPIED_FUNC<2> is_occupied_func;
//
//    SET_OCCUPIED_FUNC<2> set_occupied_func;
//
//    auto is_occupied = [&tl](const freeNav::Pointi<2> &pt) -> bool { return tl.isOccupied(pt); };
//    is_occupied_func = is_occupied;
//
//    auto set_occupied = [&tl](const freeNav::Pointi<2> &pt) { tl.setOccupied(pt); };
//    set_occupied_func = set_occupied;
//
//    //std::cout << " map name " << map_test_config.at("map_path") << std::endl;
//    // load mapf scene
//    //GeneralMAPFScenePtr<2> scene_ptr;
//    const auto& dim = tl.getDimensionInfo();
////    freeNav::Instances<2> ists;
//    ScenarioLoader2D sl(map_test_config.at("scene_path").c_str());
////    int max_count_of_case = atoi((map_test_config.at("agent_num")).c_str());
//    int count_of_experiments = sl.GetNumExperiments();
////
////    // load experiment data
////    for (int i = 0; i < std::min(max_count_of_case, count_of_experiments); i++) {
////        const auto &experiment = sl.GetNthExperiment(i);
////        Pointi<2> pt1({experiment.GetStartX(), experiment.GetStartY()});
////        Pointi<2> pt2({experiment.GetGoalX(), experiment.GetGoalY()});
////        freeNav::Instance<2> ist = {pt1, pt2};
////        //std::cout << " start, target = " << pt1 << ", " << pt2 << std::endl;
////        ists.push_back(ist);
////    }
////    std::cout << "get " << ists.size() << " instances" << std::endl;
//    // prepare instance for test
//    //InstancesS<2> istss = { ists, ists, ists };
//
//    const auto& insts_ids = pickCasesFromScene(count_of_experiments, agent_in_instances, count_of_instance);
////    std::cout << "RANDOM INSTANCE: " << std::endl;
////    for(const auto& ids : insts_ids) {
////        std::cout << ids << std::endl;
////    }
//
//    InstancesS<2> istss;
//    for(const auto& ids : insts_ids) {
//        freeNav::Instances<2> ists;
//        for(const int& id : ids) {
//            const auto &experiment = sl.GetNthExperiment(id);
//            Pointi<2> pt1({experiment.GetStartX(), experiment.GetStartY()});
//            Pointi<2> pt2({experiment.GetGoalX(), experiment.GetGoalY()});
//            freeNav::Instance<2> ist = {pt1, pt2};
//            //std::cout << " start, target = " << pt1 << ", " << pt2 << std::endl;
//            ists.push_back(ist);
//        }
//        istss.push_back(ists);
//    }
//
//    // load algorithms
//
//    auto EECBS = getMassiveTextMAPFFunc("RAW_EECBS", CBS_Li::eecbs_MAPF, dim, cutoff_time_cost);
//
//    // all layered mapf must start with "LAYERED_"
//    auto EECBS_LAYERED = getLayeredMassiveTextMAPFFunc("LAYERED_EECBS", CBS_Li::eecbs_MAPF, dim, cutoff_time_cost);
//
//    auto LaCAM = getMassiveTextMAPFFunc("RAW_LaCAM", LaCAM::lacam_MAPF, dim, cutoff_time_cost);
//
//    // all layered mapf must start with "LAYERED_"
//    auto LaCAM_LAYERED = getLayeredMassiveTextMAPFFunc("LAYERED_LaCAM", LaCAM::lacam_MAPF, dim, cutoff_time_cost);
//
//
//    StatisticSS statisticss;
//    OutputStreamSS output_streamss;
//
//    bool all_success = SingleMapMAPFPathPlanningsTest<2>(dim, is_occupied_func, istss,
//                                                         {
//                                                          EECBS_LAYERED,
//                                                          EECBS,
//                                                          LaCAM,
//                                                          //LaCAM_LAYERED
//                                                          },
//                                                          statisticss,
//                                                          output_streamss);
//
//    std::ofstream os(map_test_config.at("output_path"), std::ios_base::out);
//    //os << "TYPE START TARGET TIME_COST MAKE_SPAN TOTAL_LENGTH " << std::endl;
//    for (const auto &multi_method_output : output_streamss) {
//        for (const auto method_output : multi_method_output) {
//            os << method_output << std::endl;
//        }
//    }
//    os.close();
//    return all_success;
//}
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
//
//    //SingleMapMAPFTest(MAPFTestConfig_empty_32_32, {200, 240, 280, 320, 360}, 5, 60); // layered better
//    SingleMapMAPFTest(MAPFTestConfig_random_32_32_20, {80, 100, 130, 150, 180}, 5, 10); // layered better
//    //SingleMapMAPFTest(MAPFTestConfig_warehouse_10_20_10_2_1, {300, 350, 400, 450, 500}, 10, 60); //  layered worse
//    //SingleMapMAPFTest(MAPFTestConfig_maze_32_32_2, {30, 40, 50, 60, 70}, 10, 60); // layered better
//    //SingleMapMAPFTest(MAPFTestConfig_maze_32_32_4, {40, 50, 60, 70, 80, 90}, 10, 60); // layered better
//    //SingleMapMAPFTest(MAPFTestConfig_den312d, {200, 220, 240, 260, 280}, 10, 60); // layered better
//    //SingleMapMAPFTest(MAPFTestConfig_Berlin_1_256, {500, 600, 700, 800, 900}, 10, 60); // layered better
//    //SingleMapMAPFTest(MAPFTestConfig_Paris_1_256, {600, 700, 800, 900, 1000}, 10, 60); // layered better
//    //SingleMapMAPFTest(MAPFTestConfig_den520d, {400, 500, 600, 700, 800, 900}, 1, 60); // layered better
//
    return 0;
}