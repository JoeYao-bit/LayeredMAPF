//
// Created by yaozhuo on 2022/9/6.
//

#ifndef FREENAV_TEST_DATA_H
#define FREENAV_TEST_DATA_H
#include <iostream>
#include <map>
#include "../freeNav-base/basic_elements/point.h"
#include "../freeNav-base/dependencies/massive_test_interfaces.h"


namespace freeNav::LayeredMAPF{

    SingleMapTestConfig<2> MAPFTestConfig_random_32_32_20 =

            {
                    {"map_name",     "random-32-32-20"},
                    {"map_path",     "../test/test_data/random-32-32-20.map"},
                    {"scene_path",   "../test/test_data/random-32-32-20-random-1.scen"},
                    {"ct_path",   "../test/test_data/random-32-32-20-random-1.ct"},
                    {"output_path", "../test/test_data/layered_mapf/random-32-32-20-random-1.txt"},
                    {"decomposition_output_path", "../test/test_data/decomposition/random-32-32-20-random-1_de.txt"},
                    {"agent_num",    "650"},
                    {"cut_off_time", "200"},
                    {"max_run_time", "200"} // in second

            };

    SingleMapTestConfig<2> MAPFTestConfig_den312d =

            {
                    {"map_name",     "den312d"},
                    {"map_path",     "../test/test_data/den312d.map"},
                    {"scene_path",   "../test/test_data/den312d-random-1.scen"},
                    {"ct_path",   "../test/test_data/den312d.ct"},
                    {"output_path", "../test/test_data/layered_mapf/den312d-random-1.txt"},
                    {"decomposition_output_path", "../test/test_data/decomposition/den312d-random-1_de.txt"},
                    {"agent_num",    "200"},
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"} // in second
            };

    SingleMapTestConfig<2> MAPFTestConfig_maze_32_32_2 =

            {
                    {"map_name",     "maze-32-32-2"},
                    {"map_path",     "../test/test_data/maze-32-32-2.map"},
                    {"scene_path",   "../test/test_data/maze-32-32-2-random-1.scen"},
                    {"ct_path",   "../test/test_data/maze-32-32-2.ct"},
                    {"output_path", "../test/test_data/layered_mapf/maze-32-32-2-random-1.txt"},
                    {"decomposition_output_path", "../test/test_data/decomposition/maze-32-32-2-random-1_de.txt"},
                    {"agent_num",    "40"},
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"} // in second
            };

    SingleMapTestConfig<2> MAPFTestConfig_maze_32_32_4 =

            {
                    {"map_name",     "maze-32-32-4"},
                    {"map_path",     "../test/test_data/maze-32-32-4.map"},
                    {"scene_path",   "../test/test_data/maze-32-32-4-random-1.scen"},
                    {"ct_path",   "../test/test_data/maze-32-32-4.ct"},
                    {"output_path", "../test/test_data/layered_mapf/maze-32-32-4-random-1.txt"},
                    {"decomposition_output_path", "../test/test_data/decomposition/maze-32-32-4-random-1_de.txt"},
                    {"agent_num",    "50"},
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"} // in second
            };

    SingleMapTestConfig<2> MAPFTestConfig_Berlin_1_256 =

            {
                    {"map_name",     "Berlin_1_256"},
                    {"map_path",     "../test/test_data/Berlin_1_256.map"},
                    {"scene_path",   "../test/test_data/Berlin_1_256-random-1.scen"},
                    {"ct_path",   "../test/test_data/Berlin_1_256.ct"},
                    {"output_path", "../test/test_data/layered_mapf/Berlin_1_256-random-1.txt"},
                    {"decomposition_output_path", "../test/test_data/decomposition/Berlin_1_256-random-1_de.txt"},
                    {"agent_num",    "700"}, // 600
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"} // in second
            };

    SingleMapTestConfig<2> MAPFTestConfig_den520d =

            {
                    {"map_name",     "den520d"},
                    {"map_path",     "../test/test_data/den520d.map"},
                    {"scene_path",   "../test/test_data/den520d-random-1.scen"},
                    {"ct_path",   "../test/test_data/den520d.ct"},
                    {"output_path", "../test/test_data/layered_mapf/den520d-random-1.txt"},
                    {"decomposition_output_path", "../test/test_data/decomposition/den520d-random-1_de.txt"},
                    {"agent_num",    "500"}, // up to 500
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"} // in second
            };

    SingleMapTestConfig<2> MAPFTestConfig_Paris_1_256 =

            {
                    {"map_name",     "Paris_1_256"},
                    {"map_path",     "../test/test_data/Paris_1_256.map"},
                    {"scene_path",   "../test/test_data/Paris_1_256-random-1.scen"},
                    {"ct_path",   "../test/test_data/Paris_1_256.ct"},
                    {"output_path", "../test/test_data/layered_mapf/Paris_1_256-random-1.txt"},
                    {"decomposition_output_path", "../test/test_data/decomposition/Paris_1_256-random-1_de.txt"},
                    {"agent_num",    "500"},
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"} // in second
            };

    // warehouse-10-20-10-2-1
    SingleMapTestConfig<2> MAPFTestConfig_warehouse_10_20_10_2_1 =

            {
                    {"map_name",     "warehouse-10-20-10-2-1"},
                    {"map_path",     "../test/test_data/warehouse-10-20-10-2-1.map"},
                    {"scene_path",   "../test/test_data/warehouse-10-20-10-2-1-random-1.scen"},
                    {"ct_path",   "../test/test_data/warehouse-10-20-10-2-1.ct"},
                    {"output_path", "../test/test_data/layered_mapf/warehouse-10-20-10-2-1.txt"},
                    {"decomposition_output_path", "../test/test_data/decomposition/warehouse-10-20-10-2-1_de.txt"},
                    {"agent_num",    "500"},
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"} // in second
            };

    // empty-32-32
    SingleMapTestConfig<2> MAPFTestConfig_empty_32_32 =

            {
                    {"map_name",     "empty-32-32"},
                    {"map_path",     "../test/test_data/empty-32-32.map"},
                    {"scene_path",   "../test/test_data/empty-32-32-random-1.scen"},
                    {"ct_path",   "../test/test_data/empty-32-32.ct"},
                    {"output_path", "../test/test_data/layered_mapf/empty-32-32.txt"},
                    {"decomposition_output_path", "../test/test_data/decomposition/empty-32-32_de.txt"},
                    {"agent_num",    "260"}, // 330
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"} // in second
            };

    // empty-32-32
    SingleMapTestConfig<2> MAPFTestConfig_empty_16_16 =

            {
                    {"map_name",     "empty_16_16"},
                    {"map_path",     "../test/test_data/empty-16-16.map"},
                    {"scene_path",   "../test/test_data/empty-16-16-random-1.scen"},
                    {"ct_path",   "../test/test_data/empty-16-16.ct"},
                    {"output_path", "../test/test_data/layered_mapf/empty-16-16.txt"},
                    {"decomposition_output_path", "../test/test_data/decomposition/empty-16-16_de.txt"},
                    {"agent_num",    "120"}, // 330
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"} // in second
            };

    SingleMapTestConfig<2> MAPFTestConfig_empty_48_48 =

            {
                    {"map_name",     "empty-48-48"},
                    {"map_path",     "../test/test_data/empty-48-48.map"},
                    {"scene_path",   "../test/test_data/empty-48-48-random-1.scen"},
                    {"ct_path",   "../test/test_data/empty-48-48.ct"},
                    {"output_path", "../test/test_data/layered_mapf/empty-48-48.txt"},
                    {"decomposition_output_path", "../test/test_data/decomposition/empty-48-48_de.txt"},
                    {"agent_num",    "260"}, // 330
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"} // in second
            };

    // empty-32-32
    SingleMapTestConfig<2> MAPFTestConfig_simple =

            {
                    {"map_name",     "simple"},
                    {"map_path",     "../test/test_data/simple.map"},
                    {"scene_path",   "../test/test_data/simple.scen"},
                    {"ct_path",   "../test/test_data/simple.ct"},
                    {"output_path", "../test/test_data/layered_mapf/simple.txt"},
                    {"decomposition_output_path", "../test/test_data/decomposition/simple_de.txt"},
                    {"agent_num",    "3"}, // 330
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"} // in second
            };

    //
    SingleMapTestConfig<2> MAPFTestConfig_ht_chantry =

            {
                    {"map_name",     "ht_chantry"},
                    {"map_path",     "../test/test_data/ht_chantry.map"},
                    {"scene_path",   "../test/test_data/ht_chantry-random-1.scen"},
                    {"ct_path",   "../test/test_data/ht_chantry.ct"},
                    {"output_path", "../test/test_data/layered_mapf/ht_chantry.txt"},
                    {"decomposition_output_path", "../test/test_data/decomposition/ht_chantry_de.txt"},
                    {"agent_num",    "260"}, // 330
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"} // in second
            };

    SingleMapTestConfig<2> MAPFTestConfig_lak303d =

            {
                    {"map_name",     "lak303d"},
                    {"map_path",     "../test/test_data/lak303d.map"},
                    {"scene_path",   "../test/test_data/lak303d-random-1.scen"},
                    {"ct_path",   "../test/test_data/lak303d.ct"},
                    {"output_path", "../test/test_data/layered_mapf/lak303d.txt"},
                    {"decomposition_output_path", "../test/test_data/decomposition/lak303d_de.txt"},
                    {"agent_num",    "260"}, // 330
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"} // in second
            };
}
#endif //FREENAV_TEST_DATA_H
