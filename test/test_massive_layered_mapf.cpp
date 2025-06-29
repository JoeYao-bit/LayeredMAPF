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
#include "../algorithm/independence_detection.h"

#include "../third_party/Hybrid_MAPF/ID.h"

#define MAKESPAN 1
#define SOC 2

#define FULL_ID 1
#define SIMPLE_ID 0

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
           << total_cost << " " << maximum_single_cost << " " << !paths.empty() << " " << max_size_of_stack << " "; \
        outputStream = ss.str();                                        \
        std::cout << name << " finish" << std::endl; \
    } \


//            << total_cost << " " << maximum_single_cost << " " << !paths.empty() << " " << maximal_usage - base_usage << " "; \

// mapf_func == PIBT_2::pibt2_MAPF || mapf_func == PIBT_2::hca_MAPF || mapf_func == PIBT_2::hca_MAPF


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
        assert(instance_decompose->all_levels_.size() >= 1);                \
        Paths<2> retv; \
        std::vector<Paths<2> > pathss;                                                             \
        CBS_Li::ConstraintTable* layered_ct = new CBS_Li::ConstraintTable(dim[0], dim[0]*dim[1]);  \
        max_size_of_stack_layered = 0;                                                             \
        path_pathfinding::Grid * raw_grid_ptr = nullptr;                    \
        if(mapf_func == PIBT_2::pibt2_MAPF || mapf_func == PIBT_2::hca_MAPF || mapf_func == PIBT_2::push_and_swap_MAPF)          \
        { raw_grid_ptr = PIBT_2::Problem::generateGridPtr(dim, is_occupied); }                     \
        LaCAM2::Graph* raw_graph_ptr = nullptr;                                                    \
        if(mapf_func == LaCAM2::lacam2_MAPF) {  raw_graph_ptr = new LaCAM2::Graph(dim, is_occupied); } \
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
            std::vector<std::vector<bool> > avoid_locs(dim[1], std::vector<bool>(dim[0], false));  \
            Pointis<2> occ_grids = {}; \
            for(int j = (use_path_constraint ? i+1 : 0); j<instance_decompose->all_clusters_.size(); j++) \
            { \
                if(j == i) continue; \
                const auto& current_cluster = instance_decompose->all_clusters_[j]; \
                for(const int& agent_id : current_cluster) { \
                    if(j < i) { \
                        avoid_locs[instances[agent_id].second[1]][instances[agent_id].second[0]] = true;  \
                        occ_grids.push_back(instances[agent_id].second); \
                    } else { \
                        avoid_locs[instances[agent_id].first[1]][instances[agent_id].first[0]] = true;    \
                        occ_grids.push_back(instances[agent_id].first); \
                    } \
                } \
            } \
            auto new_isoc = [&](const Pointi<2> & pt) -> bool {                                    \
                if(pt[0] < 0 || pt[0] >= dim[0] || pt[1] < 0 || pt[1] >= dim[1]) { return true; }      \
                return isoc(pt) || avoid_locs[pt[1]][pt[0]];                                           \
            };   \
            double remaining_time = cutoff_time_cost - ((tv_after.tv_sec - tv_pre.tv_sec) + (tv_after.tv_usec - tv_pre.tv_usec)/1e6); \
            if(remaining_time < 0) {                                                               \
                if(layered_ct != nullptr) {                                                                                   \
                    delete layered_ct; \
                    layered_ct = nullptr;                                                              \
                }                                                        \
                retv.clear();                                                                    \
                break; \
            } \
            gettimeofday(&tv_after, &tz);                                                          \
            if(mapf_func == PIBT_2::pibt2_MAPF || mapf_func == PIBT_2::hca_MAPF || mapf_func == PIBT_2::push_and_swap_MAPF) {           \
                PIBT_2::setStatesToOccupied(raw_grid_ptr, occ_grids, isoc, new_isoc);            \
                PIBT_2::external_grid_ptr = raw_grid_ptr;/*PIBT_2::Problem::generateGridPtr(dim, new_isoc)*/                                            \
            }                                                                                      \
            if(mapf_func == LaCAM2::lacam2_MAPF) {                                                 \
                LaCAM2::setStatesToOccupied(raw_graph_ptr, occ_grids, isoc, new_isoc);             \
                LaCAM2::external_graph_ptr = raw_graph_ptr;\
            } \
            Paths<2> next_paths = mapf_func(dim, new_isoc, ists, layered_ct, remaining_time);      \
            if(mapf_func == PIBT_2::pibt2_MAPF || mapf_func == PIBT_2::hca_MAPF || mapf_func == PIBT_2::push_and_swap_MAPF)      \
            {                                                                                      \
                PIBT_2::restoreStatesToPassable(raw_grid_ptr, occ_grids, isoc, new_isoc);/*restore removed nodes and edges*/;           \
                PIBT_2::external_grid_ptr = nullptr;                                               \
              }                                                 \
            if(mapf_func == LaCAM2::lacam2_MAPF) {                                                 \
                LaCAM2::restoreStatesToPassable(raw_graph_ptr, occ_grids, isoc, new_isoc);    \
                LaCAM2::external_graph_ptr = nullptr;                                              \
            }                                                                                       \
            max_size_of_stack_layered = std::max(max_size_of_stack_layered, max_size_of_stack);    \
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
        if(raw_grid_ptr != nullptr) { delete raw_grid_ptr; raw_grid_ptr = nullptr; }               \
        if(raw_graph_ptr != nullptr) { delete raw_graph_ptr; raw_graph_ptr = nullptr; } \
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
           << total_cost << " " << maximum_single_cost << " " << !paths.empty() << " " << max_size_of_stack_layered << " " \
           << instance_decompose->getMaximalSubProblem() << " " \
           << instance_decompose->getNumberOfSubProblem() << " "; \
        outputStream = ss.str();                                                                   \
        std::cout << name << " finish" << std::endl;                                               \
    } \

#define getIDMassiveTextMAPFFunc(name, mapf_func, dim, cutoff_time_cost)  \
       [&](DimensionLength*, const IS_OCCUPIED_FUNC<2> & isoc, const Instances<2> & ists, \
                     Paths<2>& paths, Statistic& statistic, OutputStream& outputStream) { \
        memory_recorder.clear(); \
        sleep(1);                                                                \
        float base_usage = memory_recorder.getCurrentMemoryUsage(); \
        statistic.clear(); \
        outputStream.clear(); \
        gettimeofday(&tv_pre, &tz);                                       \
        Hybird_MAPF::Instance* inst = new Hybird_MAPF::Instance(dim, is_occupied, ists);  \
        Hybird_MAPF::ID* Solver = new Hybird_MAPF::ID(inst, FULL_ID, SOC);                \
        Solver->mapf_func_ = mapf_func;  Solver->runtime = cutoff_time_cost*1e3;            \
        auto ret_val = Solver->SolveProblem(std::vector<bool> {true,false,false});  \
        if(Solver->solved) { paths = Solver->paths_fr; }                  \
        gettimeofday(&tv_after, &tz);                                     \
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
           << total_cost << " " << maximum_single_cost << " " << !paths.empty() << " " << max_size_of_stack_layered << " " \
           << Solver->getMaximalSubProblem() << " " << Solver->getNumberOfSubProblem(); \
        outputStream = ss.str();                                          \
        delete inst; delete Solver; \
        std::cout << name << " finish" << std::endl; \
    } \


bool
SingleMapMAPFTest(const SingleMapTestConfig <2> &map_test_config,
                  const std::vector<int>& agent_in_instances,
                  int count_of_instance,
                  int cutoff_time_cost = 30,
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

    const auto& dim = tl.getDimensionInfo();
    ScenarioLoader2D sl(map_test_config.at("scene_path").c_str());
    int count_of_experiments = sl.GetNumExperiments();


    const auto& insts_ids = pickCasesFromScene(count_of_experiments, agent_in_instances, count_of_instance);
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

    auto CBS = RAW_TEST_TYPE("RAW_CBS", CBS_Li::cbs_MAPF, dim, cutoff_time_cost);
    // all layered mapf must start with "LAYERED_"
    auto CBS_LAYERED = LAYERED_TEST_TYPE("LAYERED_CBS", CBS_Li::cbs_MAPF, dim, cutoff_time_cost, true);

    auto EECBS = RAW_TEST_TYPE("RAW_EECBS", CBS_Li::eecbs_MAPF, dim, cutoff_time_cost);
    // all layered mapf must start with "LAYERED_"
    auto EECBS_LAYERED = LAYERED_TEST_TYPE("LAYERED_EECBS", CBS_Li::eecbs_MAPF, dim, cutoff_time_cost, true);

    auto LaCAM = RAW_TEST_TYPE("RAW_LaCAM", LaCAM::lacam_MAPF, dim, cutoff_time_cost);
    // all layered mapf must start with "LAYERED_"
    auto LaCAM_LAYERED = LAYERED_TEST_TYPE("LAYERED_LaCAM", LaCAM::lacam_MAPF, dim, cutoff_time_cost, false);

    auto LaCAM2 = RAW_TEST_TYPE("RAW_LaCAM", LaCAM2::lacam2_MAPF, dim, cutoff_time_cost);
    // all layered mapf must start with "LAYERED_"
    auto LaCAM2_LAYERED = LAYERED_TEST_TYPE("LAYERED_LaCAM", LaCAM2::lacam2_MAPF, dim, cutoff_time_cost, false);

    auto PBS = RAW_TEST_TYPE("RAW_PBS", PBS_Li::pbs_MAPF, dim, cutoff_time_cost);
    // all layered mapf must start with "LAYERED_"
    auto PBS_LAYERED = LAYERED_TEST_TYPE("LAYERED_PBS", PBS_Li::pbs_MAPF, dim, cutoff_time_cost, true);

    auto LNS = RAW_TEST_TYPE("RAW_LNS", MAPF_LNS2::LNS2_MAPF, dim, cutoff_time_cost);
    // all layered mapf must start with "LAYERED_"
    auto LNS_LAYERED = LAYERED_TEST_TYPE("LAYERED_LNS", MAPF_LNS2::LNS2_MAPF, dim, cutoff_time_cost, true);

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
                                                            CBS,
                                                            CBS_LAYERED,
//                                                          EECBS, // local test ok
//                                                          EECBS_LAYERED,

//                                                          PBS, // local test ok
//                                                          PBS_LAYERED,

//                                                          LNS,
//                                                          LNS_LAYERED,

//                                                          LaCAM2,
//                                                          LaCAM2_LAYERED,

//                                                         PIBT2,
//                                                         PIBT2_LAYERED,
//
//                                                         HCA,
//                                                         HCA_LAYERED,
//
//                                                          PushAndSwap,
//                                                          PushAndSwap_LAYERED

                                                          },
                                                         map_test_config.at("output_path"),
                                                         prune);

    return all_success;
}

// each method have a common range of agents
int main(void) {
    int cut_off_time = 30;
    int repeat_times = 1;
    int flag = 1;
    bool all_in = true;
    for(int i=0; i<100; i++) {
        // 1,
        if(flag == 1 || all_in) {
            SingleMapMAPFTest(MAPFTestConfig_empty_16_16, {10, 20, 40, 60, 80, 100, 120},
                                repeat_times, cut_off_time); // layered better

            SingleMapMAPFTest(MAPFTestConfig_empty_32_32, {10, 40, 80, 120, 160, 200, 240, 280, 320, 360, 400},
                                repeat_times, cut_off_time);
        }
        // // 2,
        if(flag == 2 || all_in) {

            SingleMapMAPFTest(MAPFTestConfig_maze_32_32_2, {20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120},
                                repeat_times, cut_off_time);

            SingleMapMAPFTest(MAPFTestConfig_maze_32_32_4, {20, 40, 80, 120, 160, 200, 240},
                                repeat_times, cut_off_time);
        }
        // // 3,
        if(flag == 3 || all_in) {

            SingleMapMAPFTest(MAPFTestConfig_maze_128_128_2, {100, 200, 300, 400, 500, 600, 700},
                                repeat_times, cut_off_time);

            SingleMapMAPFTest(MAPFTestConfig_maze_128_128_10, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000},
                                repeat_times, cut_off_time);
        }
        // // 4,
        if(flag == 4 || all_in) {

            SingleMapMAPFTest(MAPFTestConfig_den312d, {100, 200, 300, 400, 500, 600, 700, 800},
                                repeat_times, cut_off_time);

            SingleMapMAPFTest(MAPFTestConfig_den520d, {100, 200, 300, 400, 500, 600, 700, 800, 900},
                                repeat_times, cut_off_time);
        }
        // // 5,
        if(flag == 5 || all_in) {

            SingleMapMAPFTest(MAPFTestConfig_Berlin_1_256, {100, 200, 300, 400, 500, 600, 700, 800, 900},
                                repeat_times, cut_off_time);

            SingleMapMAPFTest(MAPFTestConfig_Paris_1_256, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000},
                                repeat_times, cut_off_time);
        }
        // // 6,
        if(flag == 6 || all_in) {

            SingleMapMAPFTest(MAPFTestConfig_ht_chantry, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000},
                                repeat_times, cut_off_time);

            SingleMapMAPFTest(MAPFTestConfig_lak303d, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000},
                                repeat_times, cut_off_time);
        }
        // // 7,
        if(flag == 7 || all_in) {

            SingleMapMAPFTest(MAPFTestConfig_random_32_32_20, {20, 40, 80, 120, 160, 200, 240},
                                repeat_times, cut_off_time);

            SingleMapMAPFTest(MAPFTestConfig_random_64_64_20, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000},
                                repeat_times, cut_off_time);
        }
        // // 8,
        if(flag == 8 || all_in) {

            SingleMapMAPFTest(MAPFTestConfig_room_64_64_16, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000},
                                repeat_times, cut_off_time);

            SingleMapMAPFTest(MAPFTestConfig_room_64_64_8, {100, 200, 300, 400, 500, 600, 700},
                                repeat_times, cut_off_time);
        }
        // // 9,
        if(flag == 9 || all_in) {

            SingleMapMAPFTest(MAPFTestConfig_room_32_32_4, {10, 20, 40, 60, 80, 120, 160, 200},
                                repeat_times, cut_off_time);

            SingleMapMAPFTest(MAPFTestConfig_warehouse_10_20_10_2_1, {100, 200, 300, 400, 500, 600, 700, 800},
                                repeat_times, cut_off_time);
        }
        // // 10,
        if(flag == 10 || all_in) {

            SingleMapMAPFTest(MAPFTestConfig_warehouse_10_20_10_2_2,
                                {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000},
                                repeat_times, cut_off_time);

            SingleMapMAPFTest(MAPFTestConfig_warehouse_20_40_10_2_1,
                                {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000},
                                repeat_times, cut_off_time);
        }
        // // 11,
        if(flag == 11 || all_in) {

            SingleMapMAPFTest(MAPFTestConfig_warehouse_20_40_10_2_2,
                                {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000},
                                repeat_times, cut_off_time);

            SingleMapMAPFTest(MAPFTestConfig_Boston_0_256, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000},
                                repeat_times, cut_off_time);
        }
        // // 12,
        if(flag == 12 || all_in) {

            SingleMapMAPFTest(MAPFTestConfig_lt_gallowstemplar_n, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000},
                                repeat_times, cut_off_time);

            SingleMapMAPFTest(MAPFTestConfig_ost003d, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000},
                                repeat_times, cut_off_time);
        }
    }
    return 0;
}


bool
SingleMapMAPFTestID(const SingleMapTestConfig <2> &map_test_config,
                    const std::vector<int>& agent_in_instances,
                    int count_of_instance,
                    int cutoff_time_cost = 30,
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

    const auto& dim = tl.getDimensionInfo();
    ScenarioLoader2D sl(map_test_config.at("scene_path").c_str());
    int count_of_experiments = sl.GetNumExperiments();


    const auto& insts_ids = pickCasesFromScene(count_of_experiments, agent_in_instances, count_of_instance);
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

#define ID_TEST_TYPE getIDMassiveTextMAPFFunc

    auto EECBS = RAW_TEST_TYPE("RAW_EECBS", CBS_Li::eecbs_MAPF, dim, cutoff_time_cost);
    // all layered mapf must start with "LAYERED_"
    auto EECBS_LAYERED = LAYERED_TEST_TYPE("LAYERED_EECBS", CBS_Li::eecbs_MAPF, dim, cutoff_time_cost, true);

    auto EECBS_ID = ID_TEST_TYPE("ID_EECBS", CBS_Li::eecbs_MAPF, dim, cutoff_time_cost);

    auto CBS = RAW_TEST_TYPE("RAW_CBS", CBS_Li::cbs_MAPF, dim, cutoff_time_cost);
    // all layered mapf must start with "LAYERED_"
    auto CBS_LAYERED = LAYERED_TEST_TYPE("LAYERED_CBS", CBS_Li::cbs_MAPF, dim, cutoff_time_cost, true);

    auto CBS_ID = ID_TEST_TYPE("ID_CBS", CBS_Li::cbs_MAPF, dim, cutoff_time_cost);

//    auto PBS = RAW_TEST_TYPE("RAW_PBS", PBS_Li::pbs_MAPF, dim, cutoff_time_cost);
//    // all layered mapf must start with "LAYERED_"
//    auto PBS_LAYERED = LAYERED_TEST_TYPE("LAYERED_PBS", PBS_Li::pbs_MAPF, dim, cutoff_time_cost, true);
//
//    auto HCA = RAW_TEST_TYPE("RAW_HCA", PIBT_2::hca_MAPF, dim, cutoff_time_cost);
//    // all layered mapf must start with "LAYERED_"
//    auto HCA_LAYERED = LAYERED_TEST_TYPE("LAYERED_HCA", PIBT_2::hca_MAPF, dim, cutoff_time_cost, true);


    bool all_success = SingleMapMAPFPathPlanningsTest<2>(dim, is_occupied_func, istss,
                                                         {
                                                                 CBS,
                                                                 CBS_LAYERED,
                                                                 CBS_ID,
//                                                                 EECBS,
//                                                                 EECBS_LAYERED,
//                                                                 EECBS_ID
                                                        },
                                                         map_test_config.at("output_path_id"),
                                                         prune);

    return all_success;
}

#define FLAG 1

// each method have a common range of agents
int main1(void) {
    int cut_off_time = 30;
    int repeat_times = 1;
    int flag = 1;
    bool all_in = false;
    for(int i=0; i<1; i++) {
        // 1,
        if(flag == 1 || all_in) {
             SingleMapMAPFTestID(MAPFTestConfig_empty_16_16, {10, 20, 40, 60, 80, 100, 120},
                               repeat_times, cut_off_time); // layered better

             SingleMapMAPFTestID(MAPFTestConfig_empty_32_32, {10, 40, 80, 120, 160, 200, 240, 280, 320, 360, 400},
                               repeat_times, cut_off_time);
        }
        // // 2,
        if(flag == 2 || all_in) {

            SingleMapMAPFTestID(MAPFTestConfig_maze_32_32_2, {20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120},
                                repeat_times, cut_off_time);

            SingleMapMAPFTestID(MAPFTestConfig_maze_32_32_4, {20, 40, 80, 120, 160, 200, 240},
                                repeat_times, cut_off_time);
        }
        // // 3,
        if(flag == 3 || all_in) {

            SingleMapMAPFTestID(MAPFTestConfig_maze_128_128_2, {100, 200, 300, 400, 500, 600, 700},
                                repeat_times, cut_off_time);

            SingleMapMAPFTestID(MAPFTestConfig_maze_128_128_10, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000},
                                repeat_times, cut_off_time);
        }
        // // 4,
        if(flag == 4 || all_in) {

            SingleMapMAPFTestID(MAPFTestConfig_den312d, {100, 200, 300, 400, 500, 600, 700, 800},
                                repeat_times, cut_off_time);

            SingleMapMAPFTestID(MAPFTestConfig_den520d, {100, 200, 300, 400, 500, 600, 700, 800, 900},
                                repeat_times, cut_off_time);
        }
        // // 5,
        if(flag == 5 || all_in) {

            SingleMapMAPFTestID(MAPFTestConfig_Berlin_1_256, {100, 200, 300, 400, 500, 600, 700, 800, 900},
                                repeat_times, cut_off_time);

            SingleMapMAPFTestID(MAPFTestConfig_Paris_1_256, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000},
                                repeat_times, cut_off_time);
        }
        // // 6,
        if(flag == 6 || all_in) {

            SingleMapMAPFTestID(MAPFTestConfig_ht_chantry, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000},
                                repeat_times, cut_off_time);

            SingleMapMAPFTestID(MAPFTestConfig_lak303d, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000},
                                repeat_times, cut_off_time);
        }
        // // 7,
        if(flag == 7 || all_in) {

            SingleMapMAPFTestID(MAPFTestConfig_random_32_32_20, {20, 40, 80, 120, 160, 200, 240},
                                repeat_times, cut_off_time);

            SingleMapMAPFTestID(MAPFTestConfig_random_64_64_20, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000},
                                repeat_times, cut_off_time);
        }
        // // 8,
        if(flag == 8 || all_in) {

            SingleMapMAPFTestID(MAPFTestConfig_room_64_64_16, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000},
                                repeat_times, cut_off_time);

            SingleMapMAPFTestID(MAPFTestConfig_room_64_64_8, {100, 200, 300, 400, 500, 600, 700},
                                repeat_times, cut_off_time);
        }
        // // 9,
        if(flag == 9 || all_in) {

            SingleMapMAPFTestID(MAPFTestConfig_room_32_32_4, {10, 20, 40, 60, 80, 120, 160, 200},
                                repeat_times, cut_off_time);

            SingleMapMAPFTestID(MAPFTestConfig_warehouse_10_20_10_2_1, {100, 200, 300, 400, 500, 600, 700, 800},
                                repeat_times, cut_off_time);
        }
        // // 10,
        if(flag == 10 || all_in) {

            SingleMapMAPFTestID(MAPFTestConfig_warehouse_10_20_10_2_2,
                                {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000},
                                repeat_times, cut_off_time);

            SingleMapMAPFTestID(MAPFTestConfig_warehouse_20_40_10_2_1,
                                {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000},
                                repeat_times, cut_off_time);
        }
        // // 11,
        if(flag == 11 || all_in) {

            SingleMapMAPFTestID(MAPFTestConfig_warehouse_20_40_10_2_2,
                                {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000},
                                repeat_times, cut_off_time);

            SingleMapMAPFTestID(MAPFTestConfig_Boston_0_256, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000},
                                repeat_times, cut_off_time);
        }
        // // 12,
        if(flag == 12 || all_in) {

            SingleMapMAPFTestID(MAPFTestConfig_lt_gallowstemplar_n, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000},
                                repeat_times, cut_off_time);

            SingleMapMAPFTestID(MAPFTestConfig_ost003d, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000},
                                repeat_times, cut_off_time);
        }
    }

}