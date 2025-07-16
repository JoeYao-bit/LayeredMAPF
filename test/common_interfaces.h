//
// Created by yaozhuo on 6/30/25.
//

#ifndef LAYEREDMAPF_COMMON_INTERFACES_H
#define LAYEREDMAPF_COMMON_INTERFACES_H

#include <gtest/gtest.h>
#include <sstream>
#include <string>
#include "../algorithm/LA-MAPF/circle_shaped_agent.h"
#include "../algorithm/LA-MAPF/block_shaped_agent.h"

#include "../algorithm/LA-MAPF/CBS/large_agent_CBS.h"
#include "../algorithm/LA-MAPF/LaCAM/large_agent_lacam.h"

#include "../algorithm/LA-MAPF/large_agent_instance_generator.h"
#include "../algorithm/LA-MAPF/instance_serialize_and_deserialize.h"
#include "../algorithm/precomputation_for_decomposition.h"

#include "../algorithm/LA-MAPF/laryered_large_agent_mapf.h"

//#include "../../freeNav-base/visualization/canvas/canvas.h"
#include "../freeNav-base/dependencies/2d_grid/text_map_loader.h"
#include "test_data.h"
#include "../algorithm/LA-MAPF/CBS/layered_large_agent_CBS.h"

#include "../algorithm/LA-MAPF/laryered_large_agent_mapf.h"
#include "../freeNav-base/dependencies/memory_analysis.h"

#include "../algorithm/LA-MAPF/IndependenceDetection/independence_detection.h"

#include "../algorithm/break_loop_decomposition/biparition_decomposition.h"
#include "../algorithm/break_loop_decomposition/break_loop_decomposition.h"


#include "../algorithm/LA-MAPF/LaCAM/layered_large_agent_LaCAM.h"
#include "../algorithm/LA-MAPF/CBS/layered_large_agent_CBS.h"

namespace freeNav::LayeredMAPF::LA_MAPF {


    template<Dimension N>
    std::string BIPARTITION_MAPF(const std::vector<std::pair<Pointi<N>, Pointi<N>>> &instances,
                                 DimensionLength *dim,
                                 const IS_OCCUPIED_FUNC<N> &isoc,
                                 const LA_MAPF_FUNC<N, Pointi<N>> &mapf_func,
                                 const std::string &func_identifer,
                                 double time_limit) {
        memory_recorder.clear();
        sleep(1);
        double basic_usage = memory_recorder.getCurrentMemoryUsage();
        MSTimer mst;
        auto pre_dec =
                std::make_shared<PrecomputationOfMAPFDecomposition<N, HyperGraphNodeDataRaw<N>>>(
                        instances,
                        dim, isoc, true);

        auto bi_decompose = std::make_shared<MAPFInstanceDecompositionBipartition<N, HyperGraphNodeDataRaw<N>, Pointi<N>>>(
                dim,
                pre_dec->connect_graphs_,
                pre_dec->agent_sub_graphs_,
                pre_dec->heuristic_tables_sat_,
                pre_dec->heuristic_tables_,
                time_limit);

        LAMAPF_Paths layered_paths;
        bool detect_loss_solvability;
        std::vector<std::vector<int> > grid_visit_count_table;
        layered_paths = layeredLargeAgentMAPF<N, Pointi<N>>(bi_decompose->all_levels_,
                                                            mapf_func, //
                                                            grid_visit_count_table,
                                                            detect_loss_solvability,
                                                            pre_dec,
                                                            time_limit - mst.elapsed() / 1e3,
                                                            false);

        double total_time_cost = mst.elapsed() / 1e3;

        std::cout << "instance has " << instances.size() << " agents, bipartition " << func_identifer
                  << " find solution ? " << !layered_paths.empty()
                  << " in " << total_time_cost << "s " << std::endl;

        std::cout << "bipartition: max subproblem / total = " << getMaxLevelSize(bi_decompose->all_levels_) << " / "
                  << instances.size() << std::endl;
        std::cout << "bipartition: num of subproblem = " << bi_decompose->all_levels_.size() << std::endl;

        double max_usage = memory_recorder.getCurrentMemoryUsage();

        // agents size / time cost / success / SOC / makespan / success / memory usage / init time cost / decom time cost / max subproblem / num of subproblems
        std::stringstream ss_layered;
        ss_layered << "BP_" << func_identifer << " " << instances.size() << " "
                   << total_time_cost << " "
                   << getSOC(layered_paths) << " " << getMakeSpan(layered_paths) << " "
                   << !layered_paths.empty() << " " << max_usage - basic_usage << " "
                   << 0 << " "
                   << 0 << " "
                   << getMaxLevelSize(bi_decompose->all_levels_) << " "
                   << bi_decompose->all_levels_.size() << " "
                   << detect_loss_solvability;

        return ss_layered.str();
    }

    template<Dimension N>
    std::string BREAKLOOP_MAPF(const std::vector<std::pair<Pointi<N>, Pointi<N>>> &instances,
                               DimensionLength *dim,
                               const IS_OCCUPIED_FUNC<N> &isoc,
                               const LA_MAPF_FUNC<N, Pointi<N>> &mapf_func,
                               const std::string &func_identifer,
                               double time_limit) {
        memory_recorder.clear();
        sleep(1);
        double basic_usage = memory_recorder.getCurrentMemoryUsage();
        MSTimer mst;
        auto pre_dec =
                std::make_shared<PrecomputationOfMAPFDecomposition<N, HyperGraphNodeDataRaw<N>>>(
                        instances,
                        dim, isoc, true);

        auto bi_decompose = std::make_shared<MAPFInstanceDecompositionBipartition<N, HyperGraphNodeDataRaw<N>, Pointi<N>>>(
                dim,
                pre_dec->connect_graphs_,
                pre_dec->agent_sub_graphs_,
                pre_dec->heuristic_tables_sat_,
                pre_dec->heuristic_tables_,
                time_limit);

        LAMAPF_Paths layered_paths;
        bool detect_loss_solvability;
        std::vector<std::vector<int> > grid_visit_count_table;
        layered_paths = layeredLargeAgentMAPF<N, Pointi<N>>(bi_decompose->all_levels_,
                                                            mapf_func, //
                                                            grid_visit_count_table,
                                                            detect_loss_solvability,
                                                            pre_dec,
                                                            time_limit - mst.elapsed() / 1e3,
                                                            false);

        double total_time_cost = mst.elapsed() / 1e3;

        std::cout << "instance has " << instances.size() << " agents, layered " << func_identifer << " find solution ? "
                  << !layered_paths.empty()
                  << " in " << total_time_cost << "s " << std::endl;

        std::cout << "BreakLoop: max subproblem / total = " << getMaxLevelSize(bi_decompose->all_levels_) << " / "
                  << instances.size() << std::endl;
        std::cout << "BreakLoop: num of subproblem = " << bi_decompose->all_levels_.size() << std::endl;

        double max_usage = memory_recorder.getCurrentMemoryUsage();

        // agents size / time cost / success / SOC / makespan / success / memory usage / init time cost / decom time cost / max subproblem / num of subproblems
        std::stringstream ss_layered;
        ss_layered << "BL_" << func_identifer << " " << instances.size() << " "
                   << total_time_cost << " "
                   << getSOC(layered_paths) << " " << getMakeSpan(layered_paths) << " "
                   << !layered_paths.empty() << " " << max_usage - basic_usage << " "
                   << 0 << " "
                   << 0 << " "
                   << getMaxLevelSize(bi_decompose->all_levels_) << " "
                   << bi_decompose->all_levels_.size() << " "
                   << detect_loss_solvability;

        return ss_layered.str();
    }

    template<Dimension N>
    std::string ID_MAPF(const std::vector<std::pair<Pointi<N>, Pointi<N>>> &instances,
                        DimensionLength *dim,
                        const IS_OCCUPIED_FUNC<N> &isoc,
                        const LA_MAPF_FUNC<N, Pointi<N>> &mapf_func,
                        const std::string &func_identifer,
                        double time_limit) {
        memory_recorder.clear();
        sleep(1);
        double basic_usage = memory_recorder.getCurrentMemoryUsage();
        MSTimer mst;
        auto pre =
                std::make_shared<PrecomputationOfMAPF<2>>(instances,
                                                          dim, isoc);

        auto id_solver = ID::ID<N, Pointi<N>>(mapf_func, pre, (time_limit - mst.elapsed() / 1e3));
        LAMAPF_Paths id_paths;
        if (id_solver.solve()) {
            id_paths = id_solver.getSolution();
        }
        double total_time_cost = mst.elapsed() / 1e3;

        if (!id_paths.empty()) {
            std::cout << "ID: max subproblem / total = " << id_solver.getMaximalSubProblem() << " / "
                      << instances.size() << std::endl;
            std::cout << "ID: num of subproblem = " << id_solver.getNumberOfSubProblem() << std::endl;
            //std::cout << "ID: is solution valid ? " << isSolutionValid<2>(id_paths, agents, id_solver.pre_->all_poses_) << std::endl;
        } else {
            std::cout << "ID: failed when there are " << instances.size() << " agents" << std::endl;
        }

        double max_usage = memory_recorder.getCurrentMemoryUsage();

        // agents size / time cost / success / SOC / makespan / success / memory usage / init time cost / decom time cost / max subproblem / num of subproblems
        std::stringstream ss_id;
        // agents size / time cost / success / SOC / makespan / success / memory usage / max subproblem / num of subproblems
        ss_id << "ID_" << func_identifer << " " << instances.size() << " "
              << total_time_cost << " "
              << getSOC(id_paths) << " " << getMakeSpan(id_paths) << " "
              << !id_paths.empty() << " " << max_usage - basic_usage << " "

              << id_solver.getMaximalSubProblem() << " "
              << id_solver.getNumberOfSubProblem();

        return ss_id.str();
    }

    template<Dimension N>
    std::string RAW_MAPF(const std::vector<std::pair<Pointi<N>, Pointi<N>>> &instances,
                         DimensionLength *dim,
                         const IS_OCCUPIED_FUNC<N> &isoc,
                         const LA_MAPF_FUNC<N, Pointi<N>> &mapf_func,
                         const std::string &func_identifer,
                         double time_limit) {

        memory_recorder.clear();
        sleep(1);
        double basic_usage = memory_recorder.getCurrentMemoryUsage();
        MSTimer mst;
        auto pre =
                std::make_shared<PrecomputationOfMAPF<N>>(instances,
                                                          dim, isoc);
        std::vector<std::vector<int> > grid_visit_count_table;



        auto raw_paths = mapf_func(instances,
                                   pre->agents_,
                                   dim, isoc,
                                   nullptr,
                                   grid_visit_count_table,
                                   time_limit,
                                   pre->instance_node_ids_,
                                   pre->all_poses_,
                                   pre->distance_map_updater_,
                                   pre->agent_sub_graphs_,
                                   pre->agents_heuristic_tables_,
                                   pre->agents_heuristic_tables_ignore_rotate_,
                                   nullptr); // default null config for layered MAPF

        double total_time_cost = mst.elapsed() / 1e3;

        double max_usage = memory_recorder.getCurrentMemoryUsage();

        // agents size / time cost / success / SOC / makespan / success / memory usage / init time cost / decom time cost / max subproblem / num of subproblems
        std::cout << "instance has " << instances.size() << " agents, raw " << func_identifer << " find solution ? "
                  << !raw_paths.empty()
                  << " in " << total_time_cost << "s " << std::endl;

        // agents size / time cost / success / SOC / makespan / success / memory usage
        std::stringstream ss_raw;
        ss_raw << "RAW_" << func_identifer << " " << instances.size() << " "
               << total_time_cost << " "
               << getSOC(raw_paths) << " " << getMakeSpan(raw_paths) << " "
               << !raw_paths.empty() << " " << max_usage - basic_usage;

        return ss_raw.str();
    }

}
#endif //LAYEREDMAPF_COMMON_INTERFACES_H
