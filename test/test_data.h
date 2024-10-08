//
// Created by yaozhuo on 2022/9/6.
//

#ifndef FREENAV_TEST_DATA_H
#define FREENAV_TEST_DATA_H
#include <iostream>
#include <map>
#include "../freeNav-base/basic_elements/point.h"
#include "../freeNav-base/dependencies/massive_test_interfaces.h"
#include "../algorithm/layered_mapf.h"

namespace freeNav::LayeredMAPF{

    SingleMapTestConfig<2> MAPFTestConfig_random_32_32_20 =

            {
                    {"map_name",     "random-32-32-20"},
                    {"map_path",     "../test/test_data/random-32-32-20.map"},
                    {"scene_path",   "../test/test_data/random-32-32-20-random-1.scen"},
                    {"ct_path",   "../test/test_data/random-32-32-20-random-1.ct"},
                    {"output_path", "../test/test_data/layered_mapf/random-32-32-20.txt"},
                    {"decomposition_output_path", "../test/test_data/decomposition/random-32-32-20_de.txt"},
                    {"agent_num",    "240"},
                    {"cut_off_time", "200"},
                    {"max_run_time", "200"} // in second

            };

    SingleMapTestConfig<2> MAPFTestConfig_den312d =

            {
                    {"map_name",     "den312d"},
                    {"map_path",     "../test/test_data/den312d.map"},
                    {"scene_path",   "../test/test_data/den312d-random-1.scen"},
                    {"ct_path",   "../test/test_data/den312d.ct"},
                    {"output_path", "../test/test_data/layered_mapf/den312d.txt"},
                    {"decomposition_output_path", "../test/test_data/decomposition/den312d_de.txt"},
                    {"agent_num",    "800"},
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"} // in second
            };

    SingleMapTestConfig<2> MAPFTestConfig_maze_32_32_2 =

            {
                    {"map_name",     "maze-32-32-2"},
                    {"map_path",     "../test/test_data/maze-32-32-2.map"},
                    {"scene_path",   "../test/test_data/maze-32-32-2-random-1.scen"},
                    {"ct_path",   "../test/test_data/maze-32-32-2.ct"},
                    {"output_path", "../test/test_data/layered_mapf/maze-32-32-2.txt"},
                    {"decomposition_output_path", "../test/test_data/decomposition/maze-32-32-2_de.txt"},
                    {"agent_num",    "120"},
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"} // in second
            };

    SingleMapTestConfig<2> MAPFTestConfig_maze_32_32_4 =

            {
                    {"map_name",     "maze-32-32-4"},
                    {"map_path",     "../test/test_data/maze-32-32-4.map"},
                    {"scene_path",   "../test/test_data/maze-32-32-4-random-1.scen"},
                    {"ct_path",   "../test/test_data/maze-32-32-4.ct"},
                    {"output_path", "../test/test_data/layered_mapf/maze-32-32-4.txt"},
                    {"decomposition_output_path", "../test/test_data/decomposition/maze-32-32-4_de.txt"},
                    {"agent_num",    "240"},
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"}, // in second
                    {"la_ins_path", "../test/test_data/large_agent_instance/maze-32-32-4_la.txt"},
                    {"la_comp_path", "../test/test_data/large_agent_instance/maze-32-32-4_la_comp.txt"},
                    {"la_dec_path", "../test/test_data/large_agent_instance/maze-32-32-4_la_dec.txt"}

            };

    SingleMapTestConfig<2> MAPFTestConfig_Berlin_1_256 =

            {
                    {"map_name",     "Berlin_1_256"},
                    {"map_path",     "../test/test_data/Berlin_1_256.map"},
                    {"scene_path",   "../test/test_data/Berlin_1_256-random-1.scen"},
                    {"ct_path",   "../test/test_data/Berlin_1_256.ct"},
                    {"output_path", "../test/test_data/layered_mapf/Berlin_1_256.txt"},
                    {"decomposition_output_path", "../test/test_data/decomposition/Berlin_1_256_de.txt"},
                    {"agent_num",    "900"}, // 600
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"}, // in second
                    {"la_ins_path", "../test/test_data/large_agent_instance/Berlin_1_256_la.txt"},
                    {"la_comp_path", "../test/test_data/large_agent_instance/Berlin_1_256_la_comp.txt"},
                    {"la_dec_path", "../test/test_data/large_agent_instance/Berlin_1_256_la_dec.txt"}

            };

    SingleMapTestConfig<2> MAPFTestConfig_den520d =

            {
                    {"map_name",     "den520d"},
                    {"map_path",     "../test/test_data/den520d.map"},
                    {"scene_path",   "../test/test_data/den520d-random-1.scen"},
                    {"ct_path",   "../test/test_data/den520d.ct"},
                    {"output_path", "../test/test_data/layered_mapf/den520d.txt"},
                    {"decomposition_output_path", "../test/test_data/decomposition/den520d_de.txt"},
                    {"agent_num",    "900"}, // up to 500
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"}, // in second
                    {"la_ins_path", "../test/test_data/large_agent_instance/den520d_la.txt"},
                    {"la_comp_path", "../test/test_data/large_agent_instance/den520d_la_comp.txt"},
                    {"la_dec_path", "../test/test_data/large_agent_instance/den520d_la_dec.txt"}
            };

    SingleMapTestConfig<2> MAPFTestConfig_Paris_1_256 =

            {
                    {"map_name",     "Paris_1_256"},
                    {"map_path",     "../test/test_data/Paris_1_256.map"},
                    {"scene_path",   "../test/test_data/Paris_1_256-random-1.scen"},
                    {"ct_path",   "../test/test_data/Paris_1_256.ct"},
                    {"output_path", "../test/test_data/layered_mapf/Paris_1_256.txt"},
                    {"decomposition_output_path", "../test/test_data/decomposition/Paris_1_256_de.txt"},
                    {"agent_num",    "1000"},
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"}, // in second
                    {"la_ins_path", "../test/test_data/large_agent_instance/Paris_1_256_la.txt"},
                    {"la_comp_path", "../test/test_data/large_agent_instance/Paris_1_256_la_comp.txt"},
                    {"la_dec_path", "../test/test_data/large_agent_instance/Paris_1_256_la_dec.txt"}
            };

    SingleMapTestConfig<2> MAPFTestConfig_Boston_0_256 =

            {
                    {"map_name",     "Boston_0_256"},
                    {"map_path",     "../test/test_data/Boston_0_256.map"},
                    {"scene_path",   "../test/test_data/Boston_0_256-random-1.scen"},
                    {"ct_path",   "../test/test_data/Boston_0_256.ct"},
                    {"output_path", "../test/test_data/layered_mapf/Boston_0_256.txt"},
                    {"decomposition_output_path", "../test/test_data/decomposition/Boston_0_256_de.txt"},
                    {"agent_num",    "1000"},
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
                    {"agent_num",    "800"},
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"}, // in second
                    {"la_ins_path", "../test/test_data/large_agent_instance/warehouse-10-20-10-2-1_la.txt"},
                    {"la_comp_path", "../test/test_data/large_agent_instance/warehouse-10-20-10-2-1_la_comp.txt"},
                    {"la_dec_path", "../test/test_data/large_agent_instance/warehouse-10-20-10-2-1_la_dec.txt"}
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
                    {"agent_num",    "200"}, // 330
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
                    {"max_run_time", "60"}, // in second
                    {"la_ins_path", "../test/test_data/large_agent_instance/empty-48-48_la.txt"},
                    {"la_comp_path", "../test/test_data/large_agent_instance/empty-48-48_la_comp.txt"},
                    {"la_dec_path", "../test/test_data/large_agent_instance/empty-48-48_la_dec.txt"}

            };

    SingleMapTestConfig<2> MAPFTestConfig_simple_10_10 =

            {
                    {"map_name",     "simple-10-10"},
                    {"map_path",     "../test/test_data/simple-10-10.map"},
                    //{"scene_path",   "../test/test_data/empty-48-48-random-1.scen"}, // no available
                    {"ct_path",   "../test/test_data/simple-10-10.ct"},
                    {"output_path", "../test/test_data/layered_mapf/simple-10-10.txt"},
                    {"decomposition_output_path", "../test/test_data/decomposition/simple-10-10_de.txt"},
                    {"agent_num",    "10"}, // 330
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"}, // in second
                    {"la_ins_path", "../test/test_data/large_agent_instance/simple-10-10_la.txt"},
                    {"la_comp_path", "../test/test_data/large_agent_instance/simple-10-10_la_comp.txt"},
                    {"la_dec_path", "../test/test_data/large_agent_instance/simple-10-10_la_dec.txt"}

            };

    // simple
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
                    {"max_run_time", "60"}, // in second
                    {"la_ins_path", "../test/test_data/large_agent_instance/simple_la.txt"},
                    {"la_comp_path", "../test/test_data/large_agent_instance/simple_la_comp.txt"},
                    {"la_dec_path", "../test/test_data/large_agent_instance/simple_la_dec.txt"}
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
                    {"agent_num",    "1000"}, // 330
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
                    {"agent_num",    "1000"}, // 330
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"} // in second
            };

    // new added maps
    // maze-128-128-2
    SingleMapTestConfig<2> MAPFTestConfig_maze_128_128_2 =

            {
                    {"map_name",     "maze-128-128-2"},
                    {"map_path",     "../test/test_data/maze-128-128-2.map"},
                    {"scene_path",   "../test/test_data/maze-128-128-2-random-1.scen"},
                    {"ct_path",   "../test/test_data/maze_128_128_2.ct"},
                    {"output_path", "../test/test_data/layered_mapf/maze-128-128-2.txt"},
                    {"decomposition_output_path", "../test/test_data/decomposition/maze-128-128-2_de.txt"},
                    {"agent_num",    "700"}, // 330
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"} // in second
            };

    SingleMapTestConfig<2> MAPFTestConfig_maze_128_128_10 =

            {
                    {"map_name",     "maze-128-128-10"},
                    {"map_path",     "../test/test_data/maze-128-128-10.map"},
                    {"scene_path",   "../test/test_data/maze-128-128-10-random-1.scen"},
                    {"ct_path",   "../test/test_data/maze-128-128-10.ct"},
                    {"output_path", "../test/test_data/layered_mapf/maze-128-128-10.txt"},
                    {"decomposition_output_path", "../test/test_data/decomposition/maze-128-128-10_de.txt"},
                    {"agent_num",    "1000"}, // 330
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"}, // in second
                    {"la_ins_path", "../test/test_data/large_agent_instance/maze-128-128-10_la.txt"},
                    {"la_comp_path", "../test/test_data/large_agent_instance/maze-128-128-10_la_comp.txt"},
                    {"la_dec_path", "../test/test_data/large_agent_instance/maze-128-128-10_la_dec.txt"}
            };

    // random-64-64-10

    SingleMapTestConfig<2> MAPFTestConfig_random_64_64_10 =

            {
                    {"map_name",     "random-64-64-10"},
                    {"map_path",     "../test/test_data/random-64-64-10.map"},
                    {"scene_path",   "../test/test_data/random-64-64-10-random-1.scen"},
                    {"ct_path",   "../test/test_data/random-64-64-10.ct"},
                    {"output_path", "../test/test_data/layered_mapf/random-64-64-10.txt"},
                    {"decomposition_output_path", "../test/test_data/decomposition/random-64-64-10_de.txt"},
                    {"agent_num",    "1000"}, // 330
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"} // in second
            };

    // random-64-64-20
    SingleMapTestConfig<2> MAPFTestConfig_random_64_64_20 =

            {
                    {"map_name",     "random-64-64-20"},
                    {"map_path",     "../test/test_data/random-64-64-20.map"},
                    {"scene_path",   "../test/test_data/random-64-64-20-random-1.scen"},
                    {"ct_path",   "../test/test_data/random-64-64-20.ct"},
                    {"output_path", "../test/test_data/layered_mapf/random-64-64-20.txt"},
                    {"decomposition_output_path", "../test/test_data/decomposition/random-64-64-20_de.txt"},
                    {"agent_num",    "1000"}, // 330
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"} // in second
            };

    // room-64-64-16
    SingleMapTestConfig<2> MAPFTestConfig_room_64_64_16 =

            {
                    {"map_name",     "room-64-64-16"},
                    {"map_path",     "../test/test_data/room-64-64-16.map"},
                    {"scene_path",   "../test/test_data/room-64-64-16-random-1.scen"},
                    {"ct_path",   "../test/test_data/room-64-64-16.ct"},
                    {"output_path", "../test/test_data/layered_mapf/room-64-64-16.txt"},
                    {"decomposition_output_path", "../test/test_data/decomposition/room-64-64-16_de.txt"},
                    {"agent_num",    "1000"}, // 330
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"} // in second
            };

    // room-64-64-8
    SingleMapTestConfig<2> MAPFTestConfig_room_64_64_8 =

            {
                    {"map_name",     "room-64-64-8"},
                    {"map_path",     "../test/test_data/room-64-64-8.map"},
                    {"scene_path",   "../test/test_data/room-64-64-8-random-1.scen"},
                    {"ct_path",   "../test/test_data/room-64-64-8.ct"},
                    {"output_path", "../test/test_data/layered_mapf/room-64-64-8.txt"},
                    {"decomposition_output_path", "../test/test_data/decomposition/room-64-64-8_de.txt"},
                    {"agent_num",    "200"}, // 330
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"}, // in second
                    {"la_ins_path", "../test/test_data/large_agent_instance/room-64-64-8_la.txt"},
                    {"la_comp_path", "../test/test_data/large_agent_instance/room-64-64-8_la_comp.txt"},
                    {"la_dec_path", "../test/test_data/large_agent_instance/room-64-64-8_la_dec.txt"}
            };

    // room-32-32-4
    SingleMapTestConfig<2> MAPFTestConfig_room_32_32_4 =

            {
                    {"map_name",     "room-32-32-4"},
                    {"map_path",     "../test/test_data/room-32-32-4.map"},
                    {"scene_path",   "../test/test_data/room-32-32-4-random-1.scen"},
                    {"ct_path",   "../test/test_data/room-32-32-4.ct"},
                    {"output_path", "../test/test_data/layered_mapf/room-32-32-4.txt"},
                    {"decomposition_output_path", "../test/test_data/decomposition/room-32-32-4_de.txt"},
                    {"agent_num",    "330"}, // 330
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"}, // in second
                    {"la_ins_path", "../test/test_data/large_agent_instance/room-32-32-4_la.txt"},
                    {"la_comp_path", "../test/test_data/large_agent_instance/room-32-32-4_la_comp.txt"},
                    {"la_dec_path", "../test/test_data/large_agent_instance/room-32-32-4_la_dec.txt"}
            };

    // warehouse-10-20-10-2-2
    SingleMapTestConfig<2> MAPFTestConfig_warehouse_10_20_10_2_2 =

            {
                    {"map_name",     "warehouse-10-20-10-2-2"},
                    {"map_path",     "../test/test_data/warehouse-10-20-10-2-2.map"},
                    {"scene_path",   "../test/test_data/warehouse-10-20-10-2-2-random-1.scen"},
                    {"ct_path",   "../test/test_data/warehouse-10-20-10-2-2.ct"},
                    {"output_path", "../test/test_data/layered_mapf/warehouse-10-20-10-2-2.txt"},
                    {"decomposition_output_path", "../test/test_data/decomposition/warehouse-10-20-10-2-2_de.txt"},
                    {"agent_num",    "1000"}, // 330
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"}, // in second
                    {"la_ins_path", "../test/test_data/large_agent_instance/warehouse-10-20-10-2-2_la.txt"},
                    {"la_comp_path", "../test/test_data/large_agent_instance/warehouse-10-20-10-2-2_la_comp.txt"},
                    {"la_dec_path", "../test/test_data/large_agent_instance/warehouse-10-20-10-2-2_la_dec.txt"}
            };

    // warehouse-20-40-10-2-1
    SingleMapTestConfig<2> MAPFTestConfig_warehouse_20_40_10_2_1 =

            {
                    {"map_name",     "warehouse-20-40-10-2-1"},
                    {"map_path",     "../test/test_data/warehouse-20-40-10-2-1.map"},
                    {"scene_path",   "../test/test_data/warehouse-20-40-10-2-1-random-1.scen"},
                    {"ct_path",   "../test/test_data/warehouse-20-40-10-2-1.ct"},
                    {"output_path", "../test/test_data/layered_mapf/warehouse-20-40-10-2-1.txt"},
                    {"decomposition_output_path", "../test/test_data/decomposition/warehouse-20-40-10-2-1_de.txt"},
                    {"agent_num",    "1000"}, // 330
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"}, // in second
                    {"la_ins_path", "../test/test_data/large_agent_instance/warehouse-20-40-10-2-1_la.txt"},
                    {"la_comp_path", "../test/test_data/large_agent_instance/warehouse-20-40-10-2-1_la_comp.txt"},
                    {"la_dec_path", "../test/test_data/large_agent_instance/warehouse-20-40-10-2-1_la_dec.txt"}
            };

    // warehouse-20-40-10-2-2
    SingleMapTestConfig<2> MAPFTestConfig_warehouse_20_40_10_2_2 =

            {
                    {"map_name",     "warehouse-20-40-10-2-2"},
                    {"map_path",     "../test/test_data/warehouse-20-40-10-2-2.map"},
                    {"scene_path",   "../test/test_data/warehouse-20-40-10-2-2-random-1.scen"},
                    {"ct_path",   "../test/test_data/warehouse-20-40-10-2-2.ct"},
                    {"output_path", "../test/test_data/layered_mapf/warehouse-20-40-10-2-2.txt"},
                    {"decomposition_output_path", "../test/test_data/decomposition/warehouse-20-40-10-2-2_de.txt"},
                    {"agent_num",    "1000"}, // 330
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"}, // in second
                    {"la_ins_path", "../test/test_data/large_agent_instance/warehouse-20-40-10-2-2_la.txt"},
                    {"la_comp_path", "../test/test_data/large_agent_instance/warehouse-20-40-10-2-2_la_comp.txt"},
                    {"la_dec_path", "../test/test_data/large_agent_instance/warehouse-20-40-10-2-2_la_dec.txt"}
            };

    // lt_gallowstemplar_n
    SingleMapTestConfig<2> MAPFTestConfig_lt_gallowstemplar_n =

            {
                    {"map_name",     "lt_gallowstemplar_n"},
                    {"map_path",     "../test/test_data/lt_gallowstemplar_n.map"},
                    {"scene_path",   "../test/test_data/lt_gallowstemplar_n-random-1.scen"},
                    {"ct_path",   "../test/test_data/lt_gallowstemplar_n.ct"},
                    {"output_path", "../test/test_data/layered_mapf/lt_gallowstemplar_n.txt"},
                    {"decomposition_output_path", "../test/test_data/decomposition/lt_gallowstemplar_n_de.txt"},
                    {"agent_num",    "1000"}, // 330
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"}, // in second
                    {"la_ins_path", "../test/test_data/large_agent_instance/lt_gallowstemplar_n_la.txt"},
                    {"la_comp_path", "../test/test_data/large_agent_instance/lt_gallowstemplar_n_la_comp.txt"},
                    {"la_dec_path", "../test/test_data/large_agent_instance/lt_gallowstemplar_n_la_dec.txt"}
            };

    // ost003d
    SingleMapTestConfig<2> MAPFTestConfig_ost003d =

            {
                    {"map_name",     "ost003d"},
                    {"map_path",     "../test/test_data/ost003d.map"},
                    {"scene_path",   "../test/test_data/ost003d-random-1.scen"},
                    {"ct_path",   "../test/test_data/ost003d.ct"},
                    {"output_path", "../test/test_data/layered_mapf/ost003d.txt"},
                    {"decomposition_output_path", "../test/test_data/decomposition/ost003d_de.txt"},
                    {"agent_num",    "1000"}, // 330
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"}, // in second
                    {"la_ins_path", "../test/test_data/large_agent_instance/ost003d_la.txt"},
                    {"la_comp_path", "../test/test_data/large_agent_instance/ost003d_la_comp.txt"},
                    {"la_dec_path", "../test/test_data/large_agent_instance/ost003d_la_dec.txt"}
            };

    // AR0011SR
    SingleMapTestConfig<2> MAPFTestConfig_AR0011SR =

            {
                    {"map_name",     "AR0011SR"},
                    {"map_path",     "../test/test_data/AR0011SR.map"},
                    {"scene_path",   "../test/test_data/AR0011SR.scen"},
                    {"ct_path",   "../test/test_data/AR0011SR.ct"},
                    {"output_path", "../test/test_data/layered_mapf/AR0011SR.txt"},
                    {"decomposition_output_path", "../test/test_data/decomposition/AR0011SR_de.txt"},
                    {"agent_num",    "10"}, // 330
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"}, // in second
                    {"la_ins_path", "../test/test_data/large_agent_instance/AR0011SR_la.txt"},
                    {"la_comp_path", "../test/test_data/large_agent_instance/AR0011SR_la_comp.txt"},
                    {"la_dec_path", "../test/test_data/large_agent_instance/AR0011SR_la_dec.txt"}

            };

    // AR0012SR
    SingleMapTestConfig<2> MAPFTestConfig_AR0012SR =

            {
                    {"map_name",     "AR0012SR"},
                    {"map_path",     "../test/test_data/AR0012SR.map"},
                    {"scene_path",   "../test/test_data/AR0012SR.scen"},
                    {"ct_path",   "../test/test_data/AR0012SR.ct"},
                    {"output_path", "../test/test_data/layered_mapf/AR0012SR.txt"},
                    {"decomposition_output_path", "../test/test_data/decomposition/AR0012SR_de.txt"},
                    {"agent_num",    "10"}, // 330
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"}, // in second
                    {"la_ins_path", "../test/test_data/large_agent_instance/AR0012SR_la.txt"},
                    {"la_comp_path", "../test/test_data/large_agent_instance/AR0012SR_la_comp.txt"},
                    {"la_dec_path", "../test/test_data/large_agent_instance/AR0012SR_la_dec.txt"}

            };

    // AR0013SR
    SingleMapTestConfig<2> MAPFTestConfig_AR0013SR =

            {
                    {"map_name",     "AR0013SR"},
                    {"map_path",     "../test/test_data/AR0013SR.map"},
                    {"scene_path",   "../test/test_data/AR0013SR.scen"},
                    {"ct_path",   "../test/test_data/AR0013SR.ct"},
                    {"output_path", "../test/test_data/layered_mapf/AR0013SR.txt"},
                    {"decomposition_output_path", "../test/test_data/decomposition/AR0013SR_de.txt"},
                    {"agent_num",    "10"}, // 330
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"}, // in second
                    {"la_ins_path", "../test/test_data/large_agent_instance/AR0013SR_la.txt"},
                    {"la_comp_path", "../test/test_data/large_agent_instance/AR0013SR_la_comp.txt"},
                    {"la_dec_path", "../test/test_data/large_agent_instance/AR0013SR_la_dec.txt"}

            };

    // AR0014SR
    SingleMapTestConfig<2> MAPFTestConfig_AR0014SR =

            {
                    {"map_name",     "AR0013SR"},
                    {"map_path",     "../test/test_data/AR0014SR.map"},
                    {"scene_path",   "../test/test_data/AR0014SR.scen"},
                    {"ct_path",   "../test/test_data/AR0014SR.ct"},
                    {"output_path", "../test/test_data/layered_mapf/AR0014SR.txt"},
                    {"decomposition_output_path", "../test/test_data/decomposition/AR0014SR_de.txt"},
                    {"agent_num",    "10"}, // 330
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"}, // in second
                    {"la_ins_path", "../test/test_data/large_agent_instance/AR0014SR_la.txt"},
                    {"la_comp_path", "../test/test_data/large_agent_instance/AR0014SR_la_comp.txt"},
                    {"la_dec_path", "../test/test_data/large_agent_instance/AR0014SR_la_dec.txt"}

            };

    // AR0015SR
    SingleMapTestConfig<2> MAPFTestConfig_AR0015SR =

            {
                    {"map_name",     "AR0015SR"},
                    {"map_path",     "../test/test_data/AR0015SR.map"},
                    {"scene_path",   "../test/test_data/AR0015SR.scen"},
                    {"ct_path",   "../test/test_data/AR0015SR.ct"},
                    {"output_path", "../test/test_data/layered_mapf/AR0015SR.txt"},
                    {"decomposition_output_path", "../test/test_data/decomposition/AR0015SR_de.txt"},
                    {"agent_num",    "10"}, // 330
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"}, // in second
                    {"la_ins_path", "../test/test_data/large_agent_instance/AR0015SR_la.txt"},
                    {"la_comp_path", "../test/test_data/large_agent_instance/AR0015SR_la_comp.txt"},
                    {"la_dec_path", "../test/test_data/large_agent_instance/AR0015SR_la_dec.txt"}

            };

    // AR0016SR
    SingleMapTestConfig<2> MAPFTestConfig_AR0016SR =

            {
                    {"map_name",     "AR0016SR"},
                    {"map_path",     "../test/test_data/AR0016SR.map"},
                    {"scene_path",   "../test/test_data/AR0016SR.scen"},
                    {"ct_path",   "../test/test_data/AR0016SR.ct"},
                    {"output_path", "../test/test_data/layered_mapf/AR0016SR.txt"},
                    {"decomposition_output_path", "../test/test_data/decomposition/AR0016SR_de.txt"},
                    {"agent_num",    "10"}, // 330
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"}, // in second
                    {"la_ins_path", "../test/test_data/large_agent_instance/AR0016SR_la.txt"},
                    {"la_comp_path", "../test/test_data/large_agent_instance/AR0016SR_la_comp.txt"},
                    {"la_dec_path", "../test/test_data/large_agent_instance/AR0016SR_la_dec.txt"}

            };

    // TODO: add map from Baldurs Gate II (published September, 2000), like AR0011SR.map

    struct MethodConfigs {
        std::vector<int> agent_configs_;
        std::string raw_name_;
        std::string layered_name_;
        bool use_path_constraint_=false;
    };

    typedef std::vector<std::pair<MAPF_FUNC<2>, MethodConfigs> > MethodAndAgentConfigs;
    typedef std::vector<std::pair< SingleMapTestConfig<2>, MethodAndAgentConfigs> > MapMAPFTestConfig;
    typedef std::vector<MapMAPFTestConfig> MapMAPFTestConfigs;
}
#endif //FREENAV_TEST_DATA_H
