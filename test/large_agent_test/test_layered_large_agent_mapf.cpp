//
// Created by yaozhuo on 2024/8/15.
//

#ifndef LAYEREDMAPF_TEST_LAYERED_LARGE_AGENT_MAPF_H
#define LAYEREDMAPF_TEST_LAYERED_LARGE_AGENT_MAPF_H

#include <gtest/gtest.h>
#include "common_interfaces.h"
#include "../../algorithm/LA-MAPF/laryered_large_agent_mapf.h"
#include "../../algorithm/LA-MAPF/CBS/layered_large_agent_CBS.h"
#include "../../algorithm/LA-MAPF/CBS/large_agent_CBS.h"

//#include "../../algorithm/LA-MAPF/LaCAM/large_agent_lacam.h"
#include "../../algorithm/LA-MAPF/IndependenceDetection/independence_detection.h"
#include "../algorithm/LA-MAPF/LaCAM/layered_large_agent_LaCAM.h"

using namespace freeNav::LayeredMAPF::LA_MAPF;

void layeredLargeAgentMAPFTest(const SingleMapTestConfig<2>& file_path,
                               int num_of_agents,
                               const LA_MAPF_FUNC<2, Pose<int, 2>>& mapf_func,
                               double time_limit = 60) {

    auto map_path = file_path.at("la_ins_path");

    TextMapLoader tl(file_path.at("map_path"), is_char_occupied1);
    auto dim = tl.getDimensionInfo();
    auto is_occupied = [&tl](const freeNav::Pointi<2> &pt) -> bool { return tl.isOccupied(pt); };
    IS_OCCUPIED_FUNC<2> is_occupied_func = is_occupied;

    InstanceDeserializer<2> deserializer;
    if (deserializer.loadInstanceFromFile(file_path.at("map_path"), dim)) {
        std::cout << "load from path " << file_path.at("map_path") << " success" << std::endl;
    } else {
        std::cout << "load from path " << file_path.at("map_path") << " failed" << std::endl;
        return;
    }
    std::cout << "map scale = " << dim[0] << "*" << dim[1] << std::endl;

    auto instances = deserializer.getTestInstance({num_of_agents}, 1);

//    layered_paths = layeredLargeAgentMAPF<2, HyperGraphNodeDataRaw<2> >(instances.front().second,
//                                                                        instances.front().first,
//                                                                        dim, is_occupied,
//                                                                        CBS::LargeAgentCBS_func<2>,
//                                                                        grid_visit_count_table,
//                                                                        detect_loss_solvability,
//                                                                        20, decomposer_ptr,
//                                                                        false);


    //std::cout << " flag 0 = " << is_occupied(Pointi<2>{0,0}) << std::endl;

    MSTimer mst;
    auto pre_dec =
            std::make_shared<PrecomputationOfLAMAPFDecomposition<2, HyperGraphNodeDataRaw<2>>>(
                                                instances.front().second,
                                                instances.front().first,
                                                dim, is_occupied);

    //std::cout << " flag 1 = " << pre_dec->isoc_(Pointi<2>{0,0}) << std::endl; // ok


//    auto bi_decompose = std::make_shared<MAPFInstanceDecompositionBipartition<2, HyperGraphNodeDataRaw<2>, Pose<int, 2>> >(
//            dim,
//            pre_dec->connect_graphs_,
//            pre_dec->agent_sub_graphs_,
//            pre_dec->heuristic_tables_sat_,
//            pre_dec->heuristic_tables_,
//            time_limit);

    auto bi_decompose = std::make_shared<MAPFInstanceDecompositionBreakLoop<2, HyperGraphNodeDataRaw<2>, Pose<int, 2>> >(
            dim,
            pre_dec->connect_graphs_,
            pre_dec->agent_sub_graphs_,
            pre_dec->heuristic_tables_sat_,
            pre_dec->heuristic_tables_,
            time_limit);

    //std::cout << " flag 2 = " << pre_dec->isoc_(Pointi<2>{0,0}) << std::endl; // not ok

    LAMAPF_Paths layered_paths;
    bool detect_loss_solvability;
    std::vector<std::vector<int> > grid_visit_count_table;
    layered_paths = layeredLargeAgentMAPF<2, Pose<int, 2>>(bi_decompose->all_levels_,
                                                           mapf_func, //
                                                           grid_visit_count_table,
                                                           detect_loss_solvability,
                                                           pre_dec,
                                                           time_limit - mst.elapsed()/1e3,
                                                           false);

//    auto id_solver = ID::ID<2>(instances.front().second, instances.front().first,
//              dim, is_occupied, CBS::LargeAgentCBS_func<2>);
//
//    if(id_solver.solve()) {
//        layered_paths = id_solver.getSolution();
//        std::cout << "max subproblem / total = " << id_solver.getMaximalSubProblem() << " / " << instances.front().first.size() << std::endl;
//        std::cout << "num of subproblem = " << id_solver.getNumberOfSubProblem() << std::endl;
//        std::cout << "is solution valid ? " << isSolutionValid<2>(layered_paths, instances.front().firs t,id_solver.all_poses_) << std::endl;
//    }

    std::cout << (layered_paths.size() == instances.front().first.size() ? "success" : "failed")
              << " layered large agent mapf in " << mst.elapsed()/1e3 << "s " << std::endl;
    std::cout << std::endl;

    mst.reset();
    auto raw_paths = mapf_func(instances.front().second,
                               instances.front().first,
                               dim, is_occupied,
                               nullptr,
                               grid_visit_count_table,
                               time_limit,
                               pre_dec->instance_node_ids_,
                               pre_dec->all_poses_,
                               pre_dec->distance_map_updater_,
                               pre_dec->agent_sub_graphs_,
                               pre_dec->agents_heuristic_tables_,
                               pre_dec->agents_heuristic_tables_ignore_rotate_,
                               nullptr); // default null config for layered MAPF


    std::cout << (raw_paths.size() == instances.front().first.size() ? "success" : "failed")
              << " raw large agent mapf in " << mst.elapsed()/1e3 << "s " << std::endl;

//    gettimeofday(&tv_pre, &tz);
//    CBS::LargeAgentCBS<2, CircleAgent<2> > solver(deserializer.getInstances(), deserializer.getAgents(),
//                                                  dim, is_occupied);
//    gettimeofday(&tv_after, &tz);
//    double time_cost1 = (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3;
//    std::vector<LAMAPF_Path> raw_path;
//    if(solver.solve(60)) {
//        raw_path = solver.getSolution();
//    }
//    std::cout << (raw_path.size() == deserializer.getAgents().size() ? "success" : "failed")
//              << " raw large agent mapf in " << time_cost1 << "ms " << std::endl;

}

//TEST(test, layered_large_agent_CBS) {
int main() {
    //layeredLargeAgentMAPFTest(7, CBS::LargeAgentCBS_func<2, Pose<int, 2> >, 20);
    //(MAPFTestConfig_empty_48_48, 7, LargeAgentLaCAMPose_func<2, Pose<int, 2> >, 20);
    return 0;
}

#endif //LAYEREDMAPF_TEST_LAYERED_LARGE_AGENT_MAPF_H
