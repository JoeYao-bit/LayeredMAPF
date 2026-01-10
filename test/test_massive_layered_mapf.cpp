//
// Created by yaozhuo on 6/30/25.
//

#include "../freeNav-base/dependencies/2d_grid/text_map_loader.h"
#include "../freeNav-base/dependencies/memory_analysis.h"
#include "../algorithm/general_mapf_scene.h"
#include "../test/test_data.h"

#include "common_interfaces.h"
#include "large_agent_test/common_interfaces.h"
#include "../third_party/EECBS/inc/driver.h"
#include "../third_party/lacam/include/lacam.hpp"
#include "../third_party/CBSH2-RTC/inc/driver.h"

#include "common_interfaces_raw.h"

using namespace freeNav;
using namespace freeNav::LayeredMAPF;


void multiLoadAgentAndCompare(const SingleMapTestConfig<2>& map_file,
                              const std::vector<int>& agent_in_instances,
                              int count_of_instance,
                              double time_limit = 60) {

    TextMapLoader loader = TextMapLoader(map_file.at("map_path"), is_char_occupied1);

    auto dim = loader.getDimensionInfo();
    auto is_occupied = [&loader](const freeNav::Pointi<2> &pt) -> bool { return loader.isOccupied(pt); };
    IS_OCCUPIED_FUNC<2> is_occupied_func = is_occupied;

    ScenarioLoader2D sl(map_file.at("scene_path").c_str());
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

    std::cout << " istss size " << istss.size() << std::endl;

    for (int i = 0; i < istss.size(); i++) {

        const auto& instances_local = istss[i];

        std::vector<std::string> strs;
        std::string str;



        // str = BIPARTITION_MAPF_RAW<2>(
        //         instances_local,
        //         dim,
        //         is_occupied,
        //         //CBS_Li::cbs_MAPF,
        //         MAPF::CBSH2_RTC::CBSH2_RTC_MAPF,
        //         "CBS",
        //         time_limit); // ok
        // strs.push_back(str);

        // str = BIPARTITION_MAPF_RAW<2>(
        //         instances_local,
        //         dim,
        //         is_occupied,
        //         LaCAM::lacam_MAPF,
        //         "LaCAM",
        //         time_limit); // ok
        // strs.push_back(str);

        // str = BREAKLOOP_MAPF_RAW<2>(
        //         instances_local,
        //         dim,
        //         is_occupied,
        //         //CBS_Li::cbs_MAPF,
        //         MAPF::CBSH2_RTC::CBSH2_RTC_MAPF,                "CBS",
        //         time_limit); // ok
        // strs.push_back(str);

        // str = BREAKLOOP_INIT_MAPF_RAW<2>(
        //         instances_local,
        //         dim,
        //         is_occupied,
        //         //CBS_Li::cbs_MAPF,
        //         MAPF::CBSH2_RTC::CBSH2_RTC_MAPF,
        //         "CBS",
        //         time_limit); // ok
        // strs.push_back(str);

        str = BREAKLOOP_MAPF_RAW<2>(
                instances_local,
                dim,
                is_occupied,
                LaCAM::lacam_MAPF,
                "LaCAM",
                time_limit); // ok
        strs.push_back(str);

        // str = BREAKLOOP_INIT_MAPF_RAW<2>(
        //         instances_local,
        //         dim,
        //         is_occupied,
        //         LaCAM::lacam_MAPF,
        //         "LaCAM",
        //         time_limit); // ok
        // strs.push_back(str);

        // str = RAW_MAPF_RAW<2>(
        //         instances_local,
        //         dim,
        //         is_occupied,
        //         //CBS_Li::cbs_MAPF,
        //         MAPF::CBSH2_RTC::CBSH2_RTC_MAPF,
        //         "CBS",
        //         time_limit); // ok
        // strs.push_back(str);

        // str = RAW_MAPF_RAW<2>(
        //         instances_local,
        //         dim,
        //         is_occupied,
        //         LaCAM::lacam_MAPF,
        //         "LaCAM",
        //         time_limit); // ok
        // strs.push_back(str);

        // str = ID_MAPF_RAW<2>(
        //         instances_local,
        //         dim,
        //         is_occupied,
        //         //CBS_Li::cbs_MAPF,
        //         MAPF::CBSH2_RTC::CBSH2_RTC_MAPF,
        //         "CBS",
        //         time_limit); // ok
        // strs.push_back(str);

        for(const auto& str : strs) {
            std::cout << str << std::endl;
        }

//        IDLAMAPF<2>();
//        RAWLAMAPF<2>();

        writeStrsToEndOfFile(strs, map_file.at("output_path"));

        //break;
    }

}



//TEST(Multi_Generate_Agent_And_Compare, test) {
int main() {
    // file_path, count_of_test, max_agent_count, min_agent_count, interval, max_sample
    std::vector<std::tuple<SingleMapTestConfig<2>, std::vector<int>> > map_configs = {
        //  {MAPFTestConfig_empty_16_16, {10, 20, 40, 60, 80, 100, 120}}, // 10, 20, 40, 60, 80, 100, 120
        //  {MAPFTestConfig_empty_32_32, {10,40, 80, 120, 160, 200, 240, 280, 320, 360, 400}}, // 10,40, 80, 120, 160, 200, 240, 280, 320, 360, 400
        //  {MAPFTestConfig_random_32_32_20, {20, 40, 80, 120, 160, 200, 240}},
        //  {MAPFTestConfig_random_64_64_20, {100, 200, 300, 00, 500, 600, 700, 800, 900, 1000}},
        // {MAPFTestConfig_room_64_64_16, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000}},
        // {MAPFTestConfig_room_64_64_8, {100, 200, 300, 400, 500, 600, 700}},

        // {MAPFTestConfig_maze_32_32_2, {20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120}},
        // {MAPFTestConfig_maze_32_32_4, {20, 40, 80, 120, 160, 200, 240}},
        // {MAPFTestConfig_maze_128_128_2, {100, 200, 300, 400, 500, 600, 700}},
        // {MAPFTestConfig_maze_128_128_10, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000}},
        // {MAPFTestConfig_den312d, {100, 200, 300, 400, 500, 600, 700, 800}},
        // {MAPFTestConfig_den520d, {100, 200, 300, 400, 500, 600, 700, 800, 900}},
//           {MAPFTestConfig_Paris_1_256, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000}},
//           {MAPFTestConfig_room_32_32_4, {10, 20, 40, 60, 80, 120, 160, 200}},

        //   {MAPFTestConfig_Berlin_1_256, {100, 200, 300, 400, 500, 600, 700, 800, 900}}, // in pub
        //   {MAPFTestConfig_ht_chantry, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000}},  // in pub
        //    {MAPFTestConfig_lak303d, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000}},  // in pub
        //    {MAPFTestConfig_warehouse_10_20_10_2_1, {100, 200, 300, 400, 500, 600, 700, 800}},  // in pub
        //    {MAPFTestConfig_warehouse_10_20_10_2_2, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000}},  // in pub
        //    {MAPFTestConfig_warehouse_20_40_10_2_1, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000}},   // in pub
        //    {MAPFTestConfig_warehouse_20_40_10_2_2, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000}},  // in pub
        //    {MAPFTestConfig_Boston_0_256, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000}}, // in pub
            {MAPFTestConfig_lt_gallowstemplar_n, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000}},  // in pub
            {MAPFTestConfig_ost003d, {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000}}  // in pub
    };
    std::vector<bool> finished(map_configs.size(), false);
    std::mutex lock_1, lock_2;
    int map_id = 0;
    for(int i=0; i<20;i++)
    {
        std::cout << "global layered" << i << std::endl;
        for(int j=0; j<map_configs.size(); j++) {
            const auto& file_config = map_configs[j];
            multiLoadAgentAndCompare(std::get<0>(file_config),
                                     std::get<1>(file_config),
                                     1,
                                     60);
        }
    }
    // for(int i=0; i<map_configs.size(); i++) {
    //     auto lambda_func = [&]() {
    //         lock_1.lock();
    //         int file_config_id = map_id;
    //         map_id ++;
    //         lock_1.unlock();
    //         for(int j=0; j<100;j++)
    //         {
    //             std::cout << "global layered" << j << std::endl;
    //
    //             const auto& file_config = map_configs[file_config_id];
    //             multiLoadAgentAndCompare(std::get<0>(file_config),
    //                                      std::get<1>(file_config),
    //                                      1,
    //                                      60);
    //         }
    //         lock_2.lock();
    //         finished[file_config_id] = true;
    //         lock_2.unlock();
    //     };
    //     std::thread t(lambda_func);
    //     t.detach();
    //     sleep(1);
    // }
    // while(finished != std::vector<bool>(map_configs.size(), true)) {
    //     sleep(1);
    // }
    return 0;
}

