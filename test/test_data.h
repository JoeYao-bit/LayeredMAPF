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

// full BFS edge pass
    SingleMapTestConfig<2> MapTestConfig_maze512_4_8 =

            {
                    {"map_name",    "maze512-4-8"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/maze512-4-8.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/maze512-4-8_ENLSVG.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/maze512-4-8.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/maze512-4-8.txt"}
            };

// tarjan scc search segment fault
    SingleMapTestConfig<2> MapTestConfig_random512_10_0 =

            {
                    {"map_name",    "random512-10-0"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/random512-10-0.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/random512-10-0_ENLSVG.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/random512-10-0.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/random512-10-0.txt"}
            };

    //random512-35-2
    SingleMapTestConfig<2> MapTestConfig_random512_35_2 =

            {
                    {"map_name",    "random512-10-0"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/random512_35_2.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/random512_35_2_ENLSVG.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/random512_35_2.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/random512_35_2.txt"}
            };



    SingleMapTestConfig<2> MapTestConfig_AR0018SR =

            {
                    {"map_name",    "AR0018SR"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/AR0018SR.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/AR0018SR_ENLSVG.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/AR0018SR.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/AR0018SR.txt"}
            };

    SingleMapTestConfig<2> MapTestConfig_AR0205SR =

            {
                    {"map_name",    "AR0205SR"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/AR0205SR.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/AR0205SR_ENLSVG.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/AR0205SR.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/AR0205SR.txt"}
            };

    SingleMapTestConfig<2> MapTestConfig_AR0013SR =

            {
                    {"map_name",    "AR0013SR"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/AR0013SR.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/AR0013SR_ENLSVG.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/AR0013SR.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/AR0013SR.txt"}
            };

    SingleMapTestConfig<2> MapTestConfig_AR0014SR =

            {
                    {"map_name",    "AR0014SR"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/AR0014SR.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/AR0014SR_ENLSVG.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/AR0014SR.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/AR0014SR.txt"}
            };

// full BFS edge pass
    SingleMapTestConfig<2> MapTestConfig_TheFrozenSea =

            {
                    {"map_name",    "TheFrozenSea"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/TheFrozenSea.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/TheFrozenSea_ENLSVG.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/TheFrozenSea.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/TheFrozenSea.txt"}
            };

    // full BFS edge pass
    SingleMapTestConfig<2> MapTestConfig_Entanglement =

            {
                    {"map_name",    "Entanglement"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/Entanglement.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/Entanglement_ENLSVG.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/Entanglement.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/Entanglement.txt"}
            };

    // EbonLakes

    SingleMapTestConfig<2> MapTestConfig_EbonLakes =

            {
                    {"map_name",    "EbonLakes"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/EbonLakes.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/EbonLakes_ENLSVG.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/EbonLakes.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/EbonLakes.txt"}
            };

    SingleMapTestConfig<2> MapTestConfig_Enigma =

            {
                    {"map_name",    "Enigma"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/Enigma.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/Enigma_ENLSVG.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/Enigma.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/Enigma.txt"}
            };

// full BFS edge pass
    SingleMapTestConfig<2> MapTestConfig_Boston_0_1024 =

            {
                    {"map_name",    "Boston-0-1024"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/Boston_0_1024.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/Boston_0_1024_ENLSVG.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/Boston_0_1024.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/Boston_0_1024.txt"}
            };

    SingleMapTestConfig<2> MapTestConfig_Boston_2_256 =

            {
                    {"map_name",    "Boston_2_256"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/Boston_2_256.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/Boston_2_256_ENLSVG.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/Boston_2_256.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/Boston_2_256.txt"},
                    {"sat_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/Boston_2_256.sat"}

            };

    SingleMapTestConfig<2> MapTestConfig_Boston_2_512 =

            {
                    {"map_name",    "Boston_2_256"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/Boston_2_512.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/Boston_2_512_ENLSVG.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/Boston_2_512.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/Boston_2_512.txt"}
            };

    SingleMapTestConfig<2> MapTestConfig_Boston_2_1024 =

            {
                    {"map_name",    "Boston_2_1024"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/Boston_2_1024.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/Boston_2_1024_ENLSVG.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/Boston_2_1024.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/Boston_2_1024.txt"}
            };

    SingleMapTestConfig<2> MapTestConfig_Milan_1_256 =
            {
                    {"map_name",    "Milan_1_256"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/Milan_1_256.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/Milan_1_512_ENLSVG.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/Milan_1_256.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/Milan_1_256.txt"}
            };

    // Milan_1_512.map
    SingleMapTestConfig<2> MapTestConfig_Milan_1_512 =
            {
                    {"map_name",    "Milan_1_512"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/Milan_1_512.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/Milan_1_512_ENLSVG.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/Milan_1_512.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/Milan_1_512.txt"}
            };

    SingleMapTestConfig<2> MapTestConfig_Milan_1_1024 =
            {
                    {"map_name",    "Milan_1_1024"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/Milan_1_1024.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/Milan_1_1024_ENLSVG.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/Milan_1_1024.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/Milan_1_1024.txt"}
            };

    // Milan_2_256.map
    SingleMapTestConfig<2> MapTestConfig_Milan_2_256 =
            {
                    {"map_name",    "Milan_1_512"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/Milan_2_256.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/Milan_2_256_ENLSVG.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/Milan_2_256.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/Milan_2_256.txt"},
                    {"sat_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/Milan_2_256.sat"}
            };
    // Moscow_2_256.map
    SingleMapTestConfig<2> MapTestConfig_Moscow_2_256 =
            {
                    {"map_name",    "Moscow_2_256"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/Moscow_2_256.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/Moscow_2_256_ENLSVG.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/Moscow_2_256.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/Moscow_2_256.txt"},
                    {"sat_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/Moscow_2_256.sat"}
            };

    SingleMapTestConfig<2> MapTestConfig_Moscow_2_512 =
            {
                    {"map_name",    "Moscow_2_512"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/Moscow_2_512.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/Moscow_2_512_ENLSVG.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/Moscow_2_512.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/Moscow_2_512.txt"}
            };

    // Moscow_2_1024.map
    SingleMapTestConfig<2> MapTestConfig_Moscow_2_1024 =

            {
                    {"map_name",    "Moscow_2_1024"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/MapTestConfig_Moscow_2_1024.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/MapTestConfig_Moscow_2_1024_ENLSVG.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/MapTestConfig_Moscow_2_1024.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/MapTestConfig_Moscow_2_1024.txt"}
            };

    // Shanghai_0_512.map
    SingleMapTestConfig<2> MapTestConfig_Shanghai_0_512 =
            {
                    {"map_name",    "Shanghai_0_512"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/Shanghai_0_512.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/Shanghai_0_512_ENLSVG.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/Shanghai_0_512.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/Shanghai_0_512.txt"}
            };

    SingleMapTestConfig<2> MapTestConfig_London_2_256 =
            {
                    {"map_name",    "London_2_256"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/London_2_256.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/London_2_256_ENLSVG.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/London_2_256.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/London_2_256.txt"},
                    {"sat_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/London_2_256.sat"}
            };

    SingleMapTestConfig<2> MapTestConfig_London_2_512 =
            {
                    {"map_name",    "London_2_512"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/London_2_512.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/London_2_512_ENLSVG.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/London_2_512.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/London_2_512.txt"}
            };

    SingleMapTestConfig<2> MapTestConfig_London_2_1024 =
            {
                    {"map_name",    "London_2_1024"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/London_2_1024.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/London_2_1024_ENLSVG.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/London_2_1024.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/London_2_1024.txt"}
            };

    // Sydney_1_256.map
    SingleMapTestConfig<2> MapTestConfig_Sydney_1_256 =
            {
                    {"map_name",    "Sydney_1_256"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/Sydney_1_256.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/Sydney_1_256_ENLSVG.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/Sydney_1_256.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/Sydney_1_256.txt"},
                    {"sat_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/Sydney_1_256.sat"}
            };

    SingleMapTestConfig<2> MapTestConfig_Sydney_1_512 =
            {
                    {"map_name",    "Sydney_1_512"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/Sydney_1_512.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/Sydney_1_512_ENLSVG.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/Sydney_1_512.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/Sydney_1_512.txt"}
            };

    SingleMapTestConfig<2> MapTestConfig_Sydney_1_1024 =
            {
                    {"map_name",    "Sydney_1_1024"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/Sydney_1_1024.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/Sydney_1_1024_ENLSVG.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/Sydney_1_1024.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/Sydney_1_1024.txt"}
            };

    SingleMapTestConfig<2> MapTestConfig_Paris_0_256 =
            {
                    {"map_name",    "Paris_0_256"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/Paris_0_256.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/Paris_0_256_ENLSVG.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/Paris_0_256.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/Paris_0_256.txt"},
                    {"sat_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/Paris_0_256.sat"}

            };

    // Paris_0_512
    SingleMapTestConfig<2> MapTestConfig_Paris_0_512 =
            {
                    {"map_name",    "Paris_0_512"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/Paris_0_512.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/Paris_0_512_ENLSVG.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/Paris_0_512.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/Paris_0_512.txt"}
            };

    SingleMapTestConfig<2> MapTestConfig_Paris_0_1024 =
            {
                    {"map_name",    "Paris_0_1024"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/Paris_0_1024.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/Paris_0_1024_ENLSVG.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/Paris_0_1024.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/Paris_0_1024.txt"}
            };

    SingleMapTestConfig<2> MapTestConfig_Denver_2_256 =

            {
                    {"map_name",    "Denver_2_256"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/Denver_2_256.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/Denver_2_256_ENLSVG.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/Denver_2_256.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/Denver_2_256.txt"},
                    {"sat_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/Denver_2_256.sat"}

            };

    //Denver_2_512.map
    SingleMapTestConfig<2> MapTestConfig_Denver_2_512 =

            {
                    {"map_name",    "Denver_2_512"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/Denver_2_512.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/Denver_2_512_ENLSVG.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/Denver_2_512.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/Denver_2_512.txt"}
            };

    SingleMapTestConfig<2> MapTestConfig_Denver_2_1024 =

            {
                    {"map_name",    "Denver_2_1024"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/Denver_2_1024.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/Denver_2_1024_ENLSVG.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/Denver_2_1024.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/Denver_2_1024.txt"}
            };

// some problem with map parse
    SingleMapTestConfig<2> MapTestConfig_dustwallowkeys =

            {
                    {"map_name",    "dustwallowkeys"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/dustwallowkeys.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/dustwallowkeys_ENLSVG.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/dustwallowkeys.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/dustwallowkeys.txt"}
            };

// full BFS edge pass
    SingleMapTestConfig<2> MapTestConfig_FloodedPlains =

            {
                    {"map_name",    "FloodedPlains"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/FloodedPlains.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/FloodedPlains_ENLSVG.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/FloodedPlains.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/FloodedPlains.txt"}
            };

    SingleMapTestConfig<2> MapTestConfig_Berlin_0_256 =

            {
                    {"map_name",    "Berlin_0_256"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/Berlin_0_256.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/Berlin_0_256_ENLSVG.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/Berlin_0_256.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/Berlin_0_256.txt"}
            };


    //full BFS edge pass
    SingleMapTestConfig<2> MapTestConfig_Berlin_1_256 =

            {
                    {"map_name",    "Berlin_1_256"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/Berlin_1_256.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/Berlin_1_256_ENLSVG.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/Berlin_1_256.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/Berlin_1_256.txt"},
                    {"sat_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/Berlin_1_256.sat"}

            };

    SingleMapTestConfig<2> MapTestConfig_Berlin_1_512 =

            {
                    {"map_name",    "Berlin_1_512"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/Berlin_1_512.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/Berlin_1_512_ENLSVG.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/Berlin_1_512.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/Berlin_1_512.txt"}
            };

    SingleMapTestConfig<2> MapTestConfig_Berlin_1_1024 =

            {
                    {"map_name",    "Berlin_1_1024"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/Berlin_1_1024.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/Berlin_1_1024_ENLSVG.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/Berlin_1_1024.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/Berlin_1_1024.txt"}
            };

    // London_0_256.map
    SingleMapTestConfig<2> MapTestConfig_London_0_256 =

            {
                    {"map_name",    "London_0_256"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/London_0_256.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/London_0_256_ENLSVG.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/London_0_256.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/London_0_256.txt"}
            };



// maze512-4-0.map
// full BFS edge pass, without ETC_NoUselessPoint3, otherwise with thousands of failure
    SingleMapTestConfig<2> MapTestConfig_maze512_4_0 =

            {
                    {"map_name",    "maze-512-4"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/maze512-4-0.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/maze512-4-0_ENLSVG.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/maze512-4-0.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/maze512-4-0.txt"}
            };

    // maze512-16-3
    SingleMapTestConfig<2> MapTestConfig_maze512_16_3 =

            {
                    {"map_name",    "maze-512-4"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/maze512-16-3.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/maze512-16-3.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/maze512-16-3.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/maze512-16-3.txt"}
            };

    // maze512-4-6.map
    SingleMapTestConfig<2> MapTestConfig_maze512_4_6 =

            {
                    {"map_name",    "maze-512-4"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/maze512-4-6.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/maze512-4-6.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/maze512-4-6.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/maze512-4-6.txt"}
            };

    // maze512-8-6.map
    SingleMapTestConfig<2> MapTestConfig_maze512_8_6 =

            {
                    {"map_name",    "maze-512-4"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/maze512-8-6.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/maze512-8-6.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/maze512-8-6.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/maze512-8-6.txt"}
            };


//8room_002.map
// error in _Rb_tree_key_compare
    SingleMapTestConfig<2> MapTestConfig_8room_002 =

            {
                    {"map_name",    "8room-002"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/8room_002.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/8room_002_ENLSVG.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/8room_002.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/8room_002.txt"}
            };

//16room_001.map
// error in _Rb_tree_key_compare
    SingleMapTestConfig<2> MapTestConfig_16room_001 =

            {
                    {"map_name",    "16room_001"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/16room_001.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/16room_001.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/16room_001.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/16room_001.txt"}
            };

    // 8room_009.map
    SingleMapTestConfig<2> MapTestConfig_8room_009 =

            {
                    {"map_name",    "16room_001"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/8room_009.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/8room_009.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/8room_009.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/8room_009.txt"}
            };

    //32room_003.map
    SingleMapTestConfig<2> MapTestConfig_32room_003 =

            {
                    {"map_name",    "16room_001"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/32room_003.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/32room_003.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/32room_003.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/32room_003.txt"}
            };
//Aurora.map
    SingleMapTestConfig<2> MapTestConfig_Aurora =

            {
                    {"map_name",    "Aurora"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/Aurora.map"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/Aurora_ENLSVG.vis"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/Aurora.map.scen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/Aurora.txt"}
            };

    SingleMapTestConfig<3> MapTestConfig_Simple =

            {
                    {"map_name",    "Simple"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/Simple.3dmap"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/Simple.vis"},
                    {"block_path",  "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/Simple.block"},
                    {"block_path_oc",  "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/Simple_oc.block"},
                    {"minimum_block_width", "28"},
                    {"los_output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/Simple-los.txt"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/Simple.3dmap.3dscen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/Simple.txt"},
                    {"shrink_level", "1"}
            };

    SingleMapTestConfig<3> MapTestConfig_DC1 =

            {
                    {"map_name",    "DC1"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/DC1.3dmap"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/DC1.vis"},
                    {"block_path",  "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/DC1.block"},
                    {"block_path_oc",  "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/DC1_oc.block"},
                    {"minimum_block_width", "45"},
                    {"los_output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/DC1-los.txt"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/DC1.3dmap.3dscen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/DC1.txt"},
                    {"shrink_level", "1"}
            };

    SingleMapTestConfig<3> MapTestConfig_DC2 =

            {
                    {"map_name",    "DC2"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/DC2.3dmap"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/DC2.vis"},
                    {"block_path",  "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/DC2.block"},
                    {"block_path_oc",  "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/DC2_oc.block"},
                    {"minimum_block_width", "52"},
                    {"los_output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/DC2-los.txt"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/DC2.3dmap.3dscen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/DC2.txt"},
                    {"shrink_level", "1"}
            };

    SingleMapTestConfig<3> MapTestConfig_EB1 =

            {
                    {"map_name",    "EB1"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/EB1.3dmap"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/EB1.vis"},
                    {"block_path",  "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/EB1.block"},
                    {"block_path_oc",  "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/EB1_oc.block"},
                    {"minimum_block_width", "54"},
                    {"los_output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/EB1-los.txt"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/EB1.3dmap.3dscen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/EB1.txt"},
                    {"shrink_level", "1"}
            };

    SingleMapTestConfig<3> MapTestConfig_EB2 =

            {
                    {"map_name",    "EB2"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/EB2.3dmap"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/EB2.vis"},
                    {"block_path",  "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/EB2.block"},
                    {"block_path_oc",  "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/EB2_oc.block"},
                    {"minimum_block_width", "60"},
                    {"los_output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/EB2-los.txt"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/EB2.3dmap.3dscen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/EB2.txt"},
                    {"shrink_level", "5"}
            };

    SingleMapTestConfig<3> MapTestConfig_EC1 =

            {
                    {"map_name",    "EC1"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/EC1.3dmap"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/EC1.vis"},
                    {"block_path",  "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/EC1.block"},
                    {"block_path_oc",  "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/EC1_oc.block"},
                    {"minimum_block_width", "37"},
                    {"los_output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/EC1-los.txt"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/EC1.3dmap.3dscen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/EC1.txt"},
                    {"shrink_level", "1"}
            };

    SingleMapTestConfig<3> MapTestConfig_EC2 =

            {
                    {"map_name",    "EC2"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/EC2.3dmap"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/EC2.vis"},
                    {"block_path",  "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/EC2.block"},
                    {"block_path_oc",  "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/EC2_oc.block"},
                    {"minimum_block_width", "40"}, // 33 pre
                    {"los_output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/EC2-los.txt"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/EC2.3dmap.3dscen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/EC2.txt"},
                    {"shrink_level", "1"}
            };

    SingleMapTestConfig<3> MapTestConfig_Complex =

            {
                    {"map_name",    "Complex"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/Complex.3dmap"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/Complex.vis"},
                    {"block_path",  "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/Complex.block"},
                    {"block_path_oc",  "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/Complex_oc.block"},
                    {"minimum_block_width", "10"}, // optimal 26
                    {"los_output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/Complex-los.txt"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/Complex.3dmap.3dscen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/Complex.txt"},
                    {"shrink_level", "1"}
            };

    // run out of storage space when init
    SingleMapTestConfig<3> MapTestConfig_Full4 =

            {
                    {"map_name",    "Full4"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/Full4.3dmap"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/Full4.vis"},
                    {"block_path",  "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/Full4.block"},
                    {"block_path_oc",  "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/Full4_oc.block"},
                    {"minimum_block_width", "40"},
                    {"los_output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/Full4-los.txt"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/Full4.3dmap.3dscen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/Full4.txt"},
                    {"shrink_level", "1"}
            };

    // DA2.3dmap
    SingleMapTestConfig<3> MapTestConfig_DA2 =

            {
                    {"map_name",    "DA2"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/DA2.3dmap"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/DA2.vis"},
                    {"block_path",  "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/DA2.block"},
                    {"block_path_oc",  "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/DA2_oc.block"},
                    {"minimum_block_width", "35"},
                    {"los_output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/DA2-los.txt"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/DA2.3dmap.3dscen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/DA2.txt"},
                    {"shrink_level", "1"}
            };

    // DA2.3dmap
    SingleMapTestConfig<3> MapTestConfig_DA1 =

            {
                    {"map_name",    "DA1"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/DA1.3dmap"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/DA1.vis"},
                    {"block_path",  "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/DA1.block"},
                    {"block_path_oc",  "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/DA1_oc.block"},
                    {"minimum_block_width", "34"}, // pre 37
                    {"los_output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/DA1-los.txt"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/DA1.3dmap.3dscen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/DA1.txt"},
                    {"shrink_level", "1"}
            };

    // DB1.3dmap
    SingleMapTestConfig<3> MapTestConfig_DB1 =

            {
                    {"map_name",    "DB1"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/DB1.3dmap"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/DB1.vis"},
                    {"block_path",  "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/DB1.block"},
                    {"block_path_oc",  "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/DB1_oc.block"},
                    {"minimum_block_width", "36"}, // pre 23
                    {"los_output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/DB1-los.txt"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/DB1.3dmap.3dscen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/DB1.txt"},
                    {"shrink_level", "1"}
            };

    // DB2.3dmap
    SingleMapTestConfig<3> MapTestConfig_DB2 =

            {
                    {"map_name",    "DB2"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/DB2.3dmap"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/DB2.vis"},
                    {"block_path",  "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/DB2.block"},
                    {"block_path_oc",  "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/DB2_oc.block"},
                    {"minimum_block_width", "40"},
                    {"los_output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/DB2-los.txt"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/DB2.3dmap.3dscen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/DB2.txt"},
                    {"shrink_level", "1"}
            };

    // BC1.3dmap
    SingleMapTestConfig<3> MapTestConfig_BC1 =

            {
                    {"map_name",    "BC1"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/BC1.3dmap"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/BC1.vis"},
                    {"block_path",  "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/BC1.block"},
                    {"block_path_oc",  "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/BC1_oc.block"},
                    {"minimum_block_width", "38"},
                    {"los_output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/BC1-los.txt"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/BC1.3dmap.3dscen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/BC1.txt"},
                    {"shrink_level", "1"}
            };

    // BC2.3dmap
    SingleMapTestConfig<3> MapTestConfig_BC2 =

            {
                    {"map_name",    "BC2"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/BC2.3dmap"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/BC2.vis"},
                    {"block_path",  "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/BC2.block"},
                    {"block_path_oc",  "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/BC2_oc.block"},
                    {"minimum_block_width", "33"},
                    {"los_output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/BC2-los.txt"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/BC2.3dmap.3dscen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/BC2.txt"},
                    {"shrink_level", "1"}
            };

    // A1.3dmap
    SingleMapTestConfig<3> MapTestConfig_A1 =

            {
                    {"map_name",    "A1"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/A1.3dmap"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/A1.vis"},
                    {"block_path",  "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/A1.block"},
                    {"block_path_oc",  "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/A1_oc.block"},
                    {"minimum_block_width", "3"},
                    {"los_output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/A1-los.txt"},{"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/A1.3dmap.3dscen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/A1.txt"},
                    {"shrink_level", "3"}
            };

    // A2.3dmap
    SingleMapTestConfig<3> MapTestConfig_A2 =

            {
                    {"map_name",    "A2"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/A2.3dmap"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/A2.vis"},
                    {"block_path",  "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/A2.block"},
                    {"block_path_oc",  "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/A2_oc.block"},
                    {"minimum_block_width", "36"},
                    {"los_output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/A2-los.txt"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/A2.3dmap.3dscen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/A2.txt"},
                    {"shrink_level", "1"}
            };

    // A3.3dmap
    SingleMapTestConfig<3> MapTestConfig_A3 =

            {
                    {"map_name",    "A3"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/A3.3dmap"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/A3.vis"},
                    {"block_path",  "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/A3.block"},
                    {"block_path_oc",  "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/A3_oc.block"},
                    {"minimum_block_width", "32"}, // pre 32, 36
                    {"los_output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/A3-los.txt"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/A3.3dmap.3dscen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/A3.txt"},
                    {"shrink_level", "1"}
            };

    // A4.3dmap
    SingleMapTestConfig<3> MapTestConfig_A4 =

            {
                    {"map_name",    "A4"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/A4.3dmap"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/A4.vis"},
                    {"block_path",  "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/A4.block"},
                    {"block_path_oc",  "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/A4_oc.block"},
                    {"minimum_block_width", "38"},
                    {"los_output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/A4-los.txt"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/A4.3dmap.3dscen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/A4.txt"},
                    {"shrink_level", "1"}
            };

    // A5.3dmap
    SingleMapTestConfig<3> MapTestConfig_A5=

            {
                    {"map_name",    "A5"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/A5.3dmap"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/A5.vis"},
                    {"block_path",  "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/A5.block"},
                    {"block_path_oc",  "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/A5_oc.block"},
                    {"minimum_block_width", "31"},
                    {"los_output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/A5-los.txt"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/A5.3dmap.3dscen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/A5.txt"},
                    {"shrink_level", "1"}
            };

    // FA2.3dmap too big to load
    SingleMapTestConfig<3> MapTestConfig_FA2 =

            {
                    {"map_name",    "FA2"},
                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/FA2.3dmap"},
                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/FA2.vis"},
                    {"block_path",  "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/FA2.block"},
                    {"block_path_oc",  "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/FA2_oc.block"},
                    {"minimum_block_width", "15"},
                    {"los_output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/FA2-los.txt"},
                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/FA2.3dmap.3dscen"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/FA2.txt"},
                    {"shrink_level", "1"}
            };

    SingleMapTestConfig<2> MAPFTestConfig_random_32_32_20 =

            {
                    {"map_name",     "random-32-32-20"},
                    {"map_path",     "/home/yaozhuo/code/free-nav/third_party/EECBS/random-32-32-20.map"},
                    {"scene_path",   "/home/yaozhuo/code/free-nav/third_party/EECBS/random-32-32-20-random-1.scen"},
                    {"ct_path",   "/home/yaozhuo/code/free-nav/third_party/EECBS/random-32-32-20-random-1.ct"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/MAPF/random-32-32-20-random-1.txt"},
                    {"decomposition_output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/MAPF/random-32-32-20-random-1_de.txt"},
                    {"agent_num",    "100"},
                    {"cut_off_time", "200"},
                    {"max_run_time", "200"} // in second

            };

    SingleMapTestConfig<2> MAPFTestConfig_den312d =

            {
                    {"map_name",     "den312d"},
                    {"map_path",     "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/Multi-Agent Path-Finding (MAPF) Benchmarks/mapf-map/den312d.map"},
                    {"scene_path",   "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/Multi-Agent Path-Finding (MAPF) Benchmarks/mapf-scen-random/scen-random/den312d-random-1.scen"},
                    {"ct_path",   "/home/yaozhuo/code/free-nav/third_party/EECBS/den312d.ct"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/MAPF/den312d-random-1.txt"},
                    {"decomposition_output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/MAPF/den312d-random-1_de.txt"},
                    {"agent_num",    "100"},
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"} // in second
            };

    SingleMapTestConfig<2> MAPFTestConfig_maze_32_32_2 =

            {
                    {"map_name",     "maze-32-32-2"},
                    {"map_path",     "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/Multi-Agent Path-Finding (MAPF) Benchmarks/mapf-map/maze-32-32-2.map"},
                    {"scene_path",   "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/Multi-Agent Path-Finding (MAPF) Benchmarks/mapf-scen-random/scen-random/maze-32-32-2-random-1.scen"},
                    {"ct_path",   "/home/yaozhuo/code/free-nav/third_party/EECBS/maze-32-32-2.ct"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/MAPF/maze-32-32-2-random-1.txt"},
                    {"decomposition_output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/MAPF/maze-32-32-2-random-1_de.txt"},
                    {"agent_num",    "40"},
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"} // in second
            };

    SingleMapTestConfig<2> MAPFTestConfig_maze_32_32_4 =

            {
                    {"map_name",     "maze-32-32-4"},
                    {"map_path",     "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/Multi-Agent Path-Finding (MAPF) Benchmarks/mapf-map/maze-32-32-4.map"},
                    {"scene_path",   "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/Multi-Agent Path-Finding (MAPF) Benchmarks/mapf-scen-random/scen-random/maze-32-32-4-random-1.scen"},
                    {"ct_path",   "/home/yaozhuo/code/free-nav/third_party/EECBS/maze-32-32-4.ct"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/MAPF/maze-32-32-4-random-1.txt"},
                    {"decomposition_output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/MAPF/maze-32-32-4-random-1_de.txt"},
                    {"agent_num",    "50"},
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"} // in second
            };

    SingleMapTestConfig<2> MAPFTestConfig_Berlin_1_256 =

            {
                    {"map_name",     "Berlin_1_256"},
                    {"map_path",     "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/Multi-Agent Path-Finding (MAPF) Benchmarks/mapf-map/Berlin_1_256.map"},
                    {"scene_path",   "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/Multi-Agent Path-Finding (MAPF) Benchmarks/mapf-scen-random/scen-random/Berlin_1_256-random-1.scen"},
                    {"ct_path",   "/home/yaozhuo/code/free-nav/third_party/EECBS/Berlin_1_256.ct"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/MAPF/Berlin_1_256-random-1.txt"},
                    {"decomposition_output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/MAPF/Berlin_1_256-random-1_de.txt"},
                    {"agent_num",    "400"}, // 600
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"} // in second
            };

    SingleMapTestConfig<2> MAPFTestConfig_den520d =

            {
                    {"map_name",     "den520d"},
                    {"map_path",     "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/Multi-Agent Path-Finding (MAPF) Benchmarks/mapf-map/den520d.map"},
                    {"scene_path",   "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/Multi-Agent Path-Finding (MAPF) Benchmarks/mapf-scen-random/scen-random/den520d-random-1.scen"},
                    {"ct_path",   "/home/yaozhuo/code/free-nav/third_party/EECBS/den520d.ct"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/MAPF/den520d-random-1.txt"},
                    {"decomposition_output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/MAPF/den520d-random-1_de.txt"},
                    {"agent_num",    "500"}, // up to 500
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"} // in second
            };

    SingleMapTestConfig<2> MAPFTestConfig_Paris_1_256 =

            {
                    {"map_name",     "Paris_1_256"},
                    {"map_path",     "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/Multi-Agent Path-Finding (MAPF) Benchmarks/mapf-map/Paris_1_256.map"},
                    {"scene_path",   "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/Multi-Agent Path-Finding (MAPF) Benchmarks/mapf-scen-random/scen-random/Paris_1_256-random-1.scen"},
                    {"ct_path",   "/home/yaozhuo/code/free-nav/third_party/EECBS/Paris_1_256.ct"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/MAPF/Paris_1_256-random-1.txt"},
                    {"decomposition_output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/MAPF/Paris_1_256-random-1_de.txt"},
                    {"agent_num",    "500"},
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"} // in second
            };

    // warehouse-10-20-10-2-1
    SingleMapTestConfig<2> MAPFTestConfig_warehouse_10_20_10_2_1 =

            {
                    {"map_name",     "warehouse-10-20-10-2-1"},
                    {"map_path",     "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/Multi-Agent Path-Finding (MAPF) Benchmarks/mapf-map/warehouse-10-20-10-2-1.map"},
                    {"scene_path",   "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/Multi-Agent Path-Finding (MAPF) Benchmarks/mapf-scen-random/scen-random/warehouse-10-20-10-2-1-random-1.scen"},
                    {"ct_path",   "/home/yaozhuo/code/free-nav/third_party/EECBS/warehouse-10-20-10-2-1.ct"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/MAPF/warehouse-10-20-10-2-1.txt"},
                    {"decomposition_output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/MAPF/warehouse-10-20-10-2-1_de.txt"},
                    {"agent_num",    "100"},
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"} // in second
            };

    // empty-32-32
    SingleMapTestConfig<2> MAPFTestConfig_empty_32_32 =

            {
                    {"map_name",     "empty-32-32"},
                    {"map_path",     "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/Multi-Agent Path-Finding (MAPF) Benchmarks/mapf-map/empty-32-32.map"},
                    {"scene_path",   "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/Multi-Agent Path-Finding (MAPF) Benchmarks/mapf-scen-random/scen-random/empty-32-32-random-1.scen"},
                    {"ct_path",   "/home/yaozhuo/code/free-nav/third_party/EECBS/empty-32-32.ct"},
                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/MAPF/empty-32-32.txt"},
                    {"decomposition_output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/MAPF/empty-32-32_de.txt"},
                    {"agent_num",    "170"}, // 330
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"} // in second
            };

    template <Dimension N>
    struct RandomMapTestConfig {

        std::string name_;

        DimensionLength dim_[N];

        Id cubic_half_width_;

        Id cubic_number_;

        std::string random_file_path_;

        std::string block_file_path_;

        std::string test_data_path_;

        PathLen min_block_width_ = 3;

        int shrink_level_ = 3;

    };

    template <Dimension N>
    using RandomMapTestConfigs = std::vector<RandomMapTestConfig<N> >;

    RandomMapTestConfig<2> grid_2D_0 = {"grid_2D_0",
                                        {500, 500}, 10, 5,
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/random_2d_0.rm",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/random_2d_0.block",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/random_2d_0.txt",
                                        3, 3};

    RandomMapTestConfig<2> grid_2D_1 = {"grid_2D_1",
                                        {500, 500}, 10, 20,
    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/random_2d_1.rm",
    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/random_2d_1.block",
    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/random_2d_1.txt",
    3, 3};

    RandomMapTestConfig<2> grid_2D_2 = {"grid_2D_2",
                                        {500, 500}, 10, 40,
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/grid_2D_2.rm",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_2D_2.block",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_2D_2.txt",
                                        3, 3};

    RandomMapTestConfig<2> grid_2D_3 = {"grid_2D_3",
                                        {500, 500}, 10, 60,
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/grid_2D_3.rm",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_2D_3.block",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_2D_3.txt",
                                        3, 3};

    RandomMapTestConfig<2> grid_2D_4 = {"grid_2D_4",
                                        {500, 500}, 10, 80,
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/grid_2D_4.rm",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_2D_4.block",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_2D_4.txt",
                                        3, 3};

    RandomMapTestConfig<2> grid_2D_5 = {"grid_2D_5",
                                        {500, 500}, 10, 100,
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/grid_2D_4.rm",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_2D_4.block",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_2D_4.txt",
                                        3, 3};

    RandomMapTestConfig<2> grid_2D_6 = {"grid_2D_6",
                                        {500, 500}, 5, 60,
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/grid_2D_6.rm",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_2D_6.block",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_2D_6.txt",
                                        3, 3};

    RandomMapTestConfig<2> grid_2D_7 = {"grid_2D_7",
                                        {100, 100}, 5, 70,
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/grid_2D_7.rm",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_2D_7.block",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_2D_7.txt",
                                        3, 3};

    // dimension length of map and block has merely no influence on the efficiency of JOB los check
    RandomMapTestConfig<2> grid_2D_8 = {"grid_2D_8",
                                        {100, 100}, 5, 80,
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/grid_2D_8.rm",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_2D_8.block",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_2D_8.txt",
                                        3, 3};

    RandomMapTestConfig<2> grid_2D_8_1 = {"grid_2D_8_1",
                                        {200, 200}, 5, 320,
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/grid_2D_8_1.rm",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_2D_8_1.block",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_2D_8_1.txt",
                                        3, 3};

    RandomMapTestConfig<2> grid_2D_8_2 = {"grid_2D_8_2",
                                          {300, 300}, 5, 720,
                                          "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/grid_2D_8_2.rm",
                                          "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_2D_8_2.block",
                                          "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_2D_8_2.txt",
                                          3, 3};

    RandomMapTestConfig<2> grid_2D_8_3 = {"grid_2D_8_3",
                                          {400, 400}, 5, 1280,
                                          "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/grid_2D_8_3.rm",
                                          "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_2D_8_3.block",
                                          "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_2D_8_3.txt",
                                          3, 3};

    RandomMapTestConfig<2> grid_2D_8_4 = {"grid_2D_8_4",
                                          {500, 500}, 5, 2000,
                                          "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/grid_2D_8_4.rm",
                                          "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_2D_8_4.block",
                                          "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_2D_8_4.txt",
                                          3, 3};

    RandomMapTestConfig<2> grid_2D_8_5 = {"grid_2D_8_5",
                                          {600, 600}, 5, 2880,
                                          "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/grid_2D_8_5.rm",
                                          "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_2D_8_5.block",
                                          "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_2D_8_5.txt",
                                          3, 3};

    RandomMapTestConfig<2> grid_2D_8_6 = {"grid_2D_8_6",
                                          {700, 700}, 5, 3920,
                                          "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/grid_2D_8_6.rm",
                                          "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_2D_8_6.block",
                                          "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_2D_8_6.txt",
                                          3, 3};

    RandomMapTestConfig<3> grid_3D_1 = {"grid_3D_1",
                                        {100, 100, 100}, 5, 40,
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/grid_3D_1.rm",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_3D_1.block",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_3D_1.txt",
                                        3, 3};

    RandomMapTestConfig<3> grid_3D_2 = {"grid_3D_2",
                                        {100, 100, 100}, 5, 80,
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/grid_3D_2.rm",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_3D_2.block",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_3D_2.txt",
                                        3, 3};

    RandomMapTestConfig<3> grid_3D_3 = {"grid_3D_3",
                                        {100, 100, 100}, 5, 120,
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/grid_3D_3.rm",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_3D_3.block",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_3D_3.txt",
                                        3, 3};

    RandomMapTestConfig<3> grid_3D_4 = {"grid_3D_4",
                                        {100, 100, 100}, 5, 160,
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/grid_3D_4.rm",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_3D_4.block",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_3D_4.txt",
                                        3, 3};

    RandomMapTestConfig<3> grid_3D_5 = {"grid_3D_5",
                                        {100, 100, 100}, 5, 200,
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/grid_3D_5.rm",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_3D_5.block",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_3D_5.txt",
                                        3, 3};

    RandomMapTestConfig<3> grid_3D_6 = {"grid_3D_6",
                                        {100, 100, 100}, 5, 240,
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/grid_3D_6.rm",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_3D_6.block",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_3D_6.txt",
                                        3, 3};

    RandomMapTestConfig<3> grid_3D_7 = {"grid_3D_7",
                                        {100, 100, 100}, 5, 280,
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/grid_3D_7.rm",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_3D_7.block",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_3D_7.txt",
                                        3, 3};

    RandomMapTestConfig<3> grid_3D_8 = {"grid_3D_8",
                                        {100, 100, 100}, 5, 320,
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/grid_3D_8.rm",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_3D_8.block",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_3D_8.txt",
                                        3, 3};

    RandomMapTestConfig<3> grid_3D_8_1 = {"grid_3D_8_1",
                                        {150, 150, 150}, 5, 1080,
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/grid_3D_8_1.rm",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_3D_8_1.block",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_3D_8_1.txt",
                                        3, 3};

    RandomMapTestConfig<3> grid_3D_8_2 = {"grid_3D_8_2",
                                          {200, 200, 200}, 5, 2560,
                                          "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/grid_3D_8_2.rm",
                                          "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_3D_8_2.block",
                                          "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_3D_8_2.txt",
                                          3, 3};

    RandomMapTestConfig<3> grid_3D_8_3 = {"grid_3D_8_3",
                                          {250, 250, 250}, 5, 5000,
                                          "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/grid_3D_8_3.rm",
                                          "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_3D_8_3.block",
                                          "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_3D_8_3.txt",
                                          3, 3};

    RandomMapTestConfig<3> grid_3D_8_4 = {"grid_3D_8_4",
                                          {300, 300, 300}, 5, 8640,
                                          "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/grid_3D_8_4.rm",
                                          "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_3D_8_4.block",
                                          "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_3D_8_4.txt",
                                          3, 3};


#define dimen_4d 50
#define cubic_width_4d 5
#define min_block_width_4d 2
#define shrink_level_4d 2

    RandomMapTestConfig<4> grid_4D_1 = {"grid_4D_1",
                                        {dimen_4d, dimen_4d, dimen_4d, dimen_4d}, cubic_width_4d, 60,
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/grid_4D_1.rm",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_4D_1.block",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_4D_1.txt",
                                        min_block_width_4d, shrink_level_4d};

    RandomMapTestConfig<4> grid_4D_2 = {"grid_4D_2",
                                        {dimen_4d, dimen_4d, dimen_4d, dimen_4d}, cubic_width_4d, 120,
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/grid_4D_2.rm",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_4D_2.block",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_4D_2.txt",
                                        min_block_width_4d, shrink_level_4d};

    RandomMapTestConfig<4> grid_4D_3 = {"grid_4D_3",
                                        {dimen_4d, dimen_4d, dimen_4d, dimen_4d}, cubic_width_4d, 180,
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/grid_4D_3.rm",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_4D_3.block",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_4D_3.txt",
                                        min_block_width_4d, shrink_level_4d};

    RandomMapTestConfig<4> grid_4D_4 = {"grid_4D_4",
                                        {dimen_4d, dimen_4d, dimen_4d, dimen_4d}, cubic_width_4d, 240,
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/grid_4D_4.rm",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_4D_4.block",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_4D_4.txt",
                                        min_block_width_4d, shrink_level_4d};

    RandomMapTestConfig<4> grid_4D_5 = {"grid_4D_5",
                                        {dimen_4d, dimen_4d, dimen_4d, dimen_4d}, cubic_width_4d, 300,
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/grid_4D_5.rm",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_4D_5.block",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_4D_5.txt",
                                        min_block_width_4d, shrink_level_4d};

    RandomMapTestConfig<4> grid_4D_6 = {"grid_4D_6",
                                        {dimen_4d, dimen_4d, dimen_4d, dimen_4d}, cubic_width_4d, 360,
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/grid_4D_6.rm",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_4D_6.block",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_4D_6.txt",
                                        min_block_width_4d, shrink_level_4d};

    RandomMapTestConfig<4> grid_4D_7 = {"grid_4D_7",
                                        {dimen_4d, dimen_4d, dimen_4d, dimen_4d}, cubic_width_4d, 420,
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/grid_4D_7.rm",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_4D_7.block",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_4D_7.txt",
                                        min_block_width_4d, shrink_level_4d};

    RandomMapTestConfig<4> grid_4D_8 = {"grid_4D_8",
                                        {dimen_4d, dimen_4d, dimen_4d, dimen_4d}, cubic_width_4d, 480,
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/grid_4D_8.rm",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_4D_8.block",
                                        "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/grid_4D_8.txt",
                                        min_block_width_4d, shrink_level_4d};


    // random start and target
    StartAndTargets<2> Berlin_1_256_SAT = {
            {{59, 72}, {109, 214}},
            {{218, 231}, {21, 73}},
            {{53, 189}, {175, 39}},
            {{212, 228}, {61, 65}},
            {{98, 158}, {58, 9}},
            {{201, 236}, {173, 34}},
            {{211, 172}, {57, 10}},
            {{243, 250}, {195, 40}},
            {{243, 250}, {195, 40}},
            {{19, 72}, {212, 198}}
    };

    StartAndTargets<2> Denver_2_256_SAT = {
            {{59, 205},{210, 44}},
            {{214, 219},{60, 31}},
            {{25, 35},{209, 232}},
            {{21, 236},{241, 17}},
            {{208, 240},{66, 30}},
            {{246, 188},{28, 32}},
            {{203, 39},{24, 136}},
            {{209, 203},{68, 30}},
            {{122, 197},{228, 85}},
            {{31, 183},{198, 35}}
    };

    StartAndTargets<2> Boston_2_256_SAT = {
            {{36, 196},{178, 15}},
            {{69, 226},{166, 13}},
            {{212, 218},{10, 11}},
            {{9, 119},{229, 42}},
            {{66, 237},{209, 10}},
            {{197, 230},{8, 13}},
            {{51, 230},{221, 7}},
            {{177, 232},{46, 100}},
            {{243, 43},{31, 169}},
            {{58, 230},{181, 9}}
    };

    StartAndTargets<2> Milan_2_256_SAT = {
            {{13, 149},{241, 12}},
            {{12, 27},{227, 241}},
            {{10, 192},{234, 19}},
            {{4, 41},{211, 200}},
            {{20, 205},{148, 69}},
            {{229, 245},{7, 28}},
            {{239, 17},{16, 84}},
            {{251, 149},{10, 27}},
            {{239, 209},{37, 10}},
            {{8, 237},{55, 14}}
    };

    StartAndTargets<2> Moscow_2_256_SAT= {
            {{62,171}, {179,38}},
            {{218,183}, {57,60}},
            {{33,218}, {194,49}},
            {{87,227}, {154,30}},
            {{248,240}, {125,20}},
            {{50,59}, {196,112}},
            {{82,218}, {209,36}},
            {{42,105}, {242,172}},
            {{241,24}, {19,203}},
            {{201,245}, {101,33}}
    };


    StartAndTargets<2> London_2_256_SAT = {
            {{71, 196}, {160, 24}},
            {{190, 160}, {52, 44}},
            {{105, 192}, {235, 12}},
            {{14, 153}, {195, 132}},
            {{236, 211}, {102, 68}},
            {{80, 201}, {223, 83}},
            {{86, 78}, {198, 227}},
            {{18, 74}, {212, 35}},
            {{203, 232}, {66, 45}},
            {{84, 220}, {125, 7}}
    };

    StartAndTargets<2> Sydney_1_256_SAT = {
            {{38,210}, {226,43}},
            {{108,9}, {133,214}},
            {{27,74}, {243,153}},
            {{227,12}, {42,175}},
            {{147,216}, {125,8}},
            {{240,199}, {50,23}},
            {{84,203}, {197,20}},
            {{21,208}, {231,40}},
            {{121,208}, {116,10}},
            {{238,199}, {45,28}}
    };

    StartAndTargets<2> Paris_0_256_SAT = {
            {{37,237}, {174,26}},
            {{36,57}, {186,201}},
            {{183,20}, {42,184}},
            {{143,243}, {132,15}},
            {{242,158}, {8,102}},
            {{151,246}, {73,10}},
            {{250,200}, {10,196}},
            {{246,251}, {37,58}},
            {{86,238}, {244,41}},
            {{30,69}, {243,157}}
    };


    StartAndTargets<2> AR0013SR_SAT = {
            {{37,237}, {174,26}},
            {{36,57}, {186,201}},
            {{183,20}, {42,184}},
            {{143,243}, {132,15}},
            {{242,158}, {8,102}},
            {{151,246}, {73,10}},
            {{250,200}, {10,196}},
            {{246,251}, {37,58}},
            {{86,238}, {244,41}},
            {{30,69}, {243,157}}
    };

    StartAndTargets<2> AR0014SR_SAT = {
            {{75,34}, {73,130}},
            {{47,45}, {121,96}},
            {{23,83}, {125,82}},
            {{64,25}, {87,123}},
            {{37,55}, {128,85}},
            {{23,81}, {123,96}},
            {{89,125}, {58,46}},
            {{38,57}, {93,99}},
            {{49,86}, {133,80}},
            {{101,118}, {30,52}},
    };

    StartAndTargets<2> AR0018SR_SAT = {
            {{9,60}, {80,58}},
            {{38,83}, {60,24}},
            {{79,59}, {22,54}},
            {{87,51}, {38,73}},
            {{52,17}, {73,76}},
            {{13,64}, {32,81}},
            {{59,25}, {68,35}},
            {{88,53}, {52,18}},
            {{74,76}, {19,53}},
            {{53,20}, {31,64}}
    };

    StartAndTargets<2> AR00205SR_SAT = {
            {{116,26}, {49,141}},
            {{165,164}, {9,87}},
            {{160,54}, {188,134}},
            {{50,83}, {160,138}},
            {{46,135}, {138,59}},
            {{114,171}, {22,76}},
            {{78,198}, {13,116}},
            {{82,143}, {141,84}},
            {{162,165}, {57,68}},
            {{185,135}, {126,34}}
    };

}
#endif //FREENAV_TEST_DATA_H
