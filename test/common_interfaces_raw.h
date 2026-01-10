//
// Created by yaozhuo on 7/4/25.
//

#ifndef LAYEREDMAPF_COMMON_INTERFACES_RAW_H
#define LAYEREDMAPF_COMMON_INTERFACES_RAW_H

#include <Hybrid_MAPF/instance.h>
#include "../algorithm/layered_mapf.h"
#include "../algorithm/break_loop_decomposition/break_loop_decomposition.h"
#include "../algorithm/break_loop_decomposition/biparition_decomposition.h"
#include "../algorithm/precomputation_for_decomposition.h"
#include "../third_party/EECBS/inc/driver.h"
#include "../algorithm/LA-MAPF/common.h"
#include "../third_party/Hybrid_MAPF/ID.h"

namespace freeNav::LayeredMAPF {

    template<Dimension N>
    std::string BIPARTITION_MAPF_RAW(const std::vector<std::pair<Pointi<N>, Pointi<N>>> &instances,
                                 DimensionLength *dim,
                                 const IS_OCCUPIED_FUNC<N> &isoc,
                                 const MAPF_FUNC<N> & mapf_func,
                                 const std::string &func_identifer,
                                 double time_limit) {

        MSTimer mst;

        auto pre =
                std::make_shared<PrecomputationOfMAPFDecomposition<2, LA_MAPF::HyperGraphNodeDataRaw<2>>>(instances, dim,
                                                                                                          isoc);

        auto ns_decompose = std::make_shared<MAPFInstanceDecompositionBipartition<2, LA_MAPF::HyperGraphNodeDataRaw<2>, Pointi<2> > >(
                dim,
                pre->connect_graphs_,
                pre->agent_sub_graphs_,
                pre->heuristic_tables_sat_,
                pre->heuristic_tables_,
                time_limit - mst.elapsed() / 1e3,
                3);

        auto multiple_paths = layeredMAPF<2>(instances, dim, isoc, mapf_func, mapf_func,
                                        ns_decompose->all_levels_, false, time_limit - mst.elapsed() / 1e3);

        double total_time_cost = mst.elapsed() / 1e3;

        std::cout << "instance has " << instances.size() << " agents, bipartition " << func_identifer
                  << " find solution ? " << !multiple_paths.empty()
                  << " in " << total_time_cost << "s " << std::endl;

        std::cout << "bipartition: max subproblem / total = " << getMaxLevelSize(ns_decompose->all_levels_) << " / "
                  << instances.size() << std::endl;
        std::cout << "bipartition: num of subproblem = " << ns_decompose->all_levels_.size() << std::endl;


        // agents size / time cost / success / SOC / makespan / success / memory usage / init time cost / decom time cost / max subproblem / num of subproblems
        std::stringstream ss_layered;
        ss_layered << "BP_" << func_identifer << " " << instances.size() << " "
                   << total_time_cost << " "
                   << getSOC(multiple_paths) << " " << getMakeSpan(multiple_paths) << " "
                   << !multiple_paths.empty() << " " << 0 << " "
                   << 0 << " "
                   << 0 << " "
                   << getMaxLevelSize(ns_decompose->all_levels_) << " "
                   << ns_decompose->all_levels_.size() << " "
                   << 0;

        return ss_layered.str();
    }



    template<Dimension N>
    std::string BREAKLOOP_MAPF_RAW(const std::vector<std::pair<Pointi<N>, Pointi<N>>> &instances,
                                     DimensionLength *dim,
                                     const IS_OCCUPIED_FUNC<N> &isoc,
                                     const MAPF_FUNC<N> & mapf_func,
                                     const std::string &func_identifer,
                                     double time_limit) {

        MSTimer mst;

        auto pre =
                std::make_shared<PrecomputationOfMAPFDecomposition<2, LA_MAPF::HyperGraphNodeDataRaw<2>>>(instances, dim,
                                                                                                          isoc);

        auto ns_decompose = std::make_shared<MAPFInstanceDecompositionBreakLoop<2, LA_MAPF::HyperGraphNodeDataRaw<2>, Pointi<2> > >(
                dim,
                pre->connect_graphs_,
                pre->agent_sub_graphs_,
                pre->heuristic_tables_sat_,
                pre->heuristic_tables_,
                time_limit - mst.elapsed() / 1e3,
                1e4,
                100,
                1);

        auto multiple_paths = layeredMAPF<2>(instances, dim, isoc, mapf_func, mapf_func,
                                             ns_decompose->all_levels_, false, time_limit - mst.elapsed() / 1e3);

        double total_time_cost = mst.elapsed() / 1e3;

        std::cout << "instance has " << instances.size() << " agents, breakloop " << func_identifer
                  << " find solution ? " << !multiple_paths.empty()
                  << " in " << total_time_cost << "s " << std::endl;

        std::cout << "breakloop: max subproblem / total = " << getMaxLevelSize(ns_decompose->all_levels_) << " / "
                  << instances.size() << std::endl;
        std::cout << "breakloop: num of subproblem = " << ns_decompose->all_levels_.size() << std::endl;


        // agents size / time cost / success / SOC / makespan / success / memory usage / init time cost / decom time cost / max subproblem / num of subproblems
        std::stringstream ss_layered;
        ss_layered << "BL_" << func_identifer << " " << instances.size() << " "
                   << total_time_cost << " "
                   << getSOC(multiple_paths) << " " << getMakeSpan(multiple_paths) << " "
                   << !multiple_paths.empty() << " " << 0 << " "
                   << 0 << " "
                   << 0 << " "
                   << getMaxLevelSize(ns_decompose->all_levels_) << " "
                   << ns_decompose->all_levels_.size() << " "
                   << 0;

        return ss_layered.str();
    }



    template<Dimension N>
    std::string BREAKLOOP_INIT_MAPF_RAW(const std::vector<std::pair<Pointi<N>, Pointi<N>>> &instances,
                                   DimensionLength *dim,
                                   const IS_OCCUPIED_FUNC<N> &isoc,
                                   const MAPF_FUNC<N> & mapf_func,
                                   const std::string &func_identifer,
                                   double time_limit) {

        MSTimer mst;

        auto pre =
                std::make_shared<PrecomputationOfMAPFDecomposition<2, LA_MAPF::HyperGraphNodeDataRaw<2>>>(instances, dim,
                                                                                                          isoc);

        auto ns_decompose = std::make_shared<MAPFInstanceDecompositionBreakLoop<2, LA_MAPF::HyperGraphNodeDataRaw<2>, Pointi<2> > >(
                dim,
                pre->connect_graphs_,
                pre->agent_sub_graphs_,
                pre->heuristic_tables_sat_,
                pre->heuristic_tables_,
                time_limit - mst.elapsed() / 1e3,
                false,
                1e4,
                100,
                1);

        auto multiple_paths = layeredMAPF<2>(instances, dim, isoc, mapf_func, mapf_func,
                                             ns_decompose->all_levels_, false, time_limit - mst.elapsed() / 1e3);

        double total_time_cost = mst.elapsed() / 1e3;

        std::cout << "instance has " << instances.size() << " agents, breakloop init " << func_identifer
                  << " find solution ? " << !multiple_paths.empty()
                  << " in " << total_time_cost << "s " << std::endl;

        std::cout << "breakloop: max subproblem / total = " << getMaxLevelSize(ns_decompose->all_levels_) << " / "
                  << instances.size() << std::endl;
        std::cout << "breakloop: num of subproblem = " << ns_decompose->all_levels_.size() << std::endl;


        // agents size / time cost / success / SOC / makespan / success / memory usage / init time cost / decom time cost / max subproblem / num of subproblems
        std::stringstream ss_layered;
        ss_layered << "BL_INIT_" << func_identifer << " " << instances.size() << " "
                   << total_time_cost << " "
                   << getSOC(multiple_paths) << " " << getMakeSpan(multiple_paths) << " "
                   << !multiple_paths.empty() << " " << 0 << " "
                   << 0 << " "
                   << 0 << " "
                   << getMaxLevelSize(ns_decompose->all_levels_) << " "
                   << ns_decompose->all_levels_.size() << " "
                   << 0;

        return ss_layered.str();
    }



    template<Dimension N>
    std::string RAW_MAPF_RAW(const std::vector<std::pair<Pointi<N>, Pointi<N>>> &instances,
                         DimensionLength *dim,
                         const IS_OCCUPIED_FUNC<N> &isoc,
                         const MAPF_FUNC<N> & mapf_func,
                         const std::string &func_identifer,
                         double time_limit) {

        memory_recorder.clear();
        sleep(1);
        MSTimer mst;

        auto raw_paths = mapf_func(dim, isoc, instances, nullptr, time_limit); // default null config for layered MAPF

        double total_time_cost = mst.elapsed() / 1e3;


        // agents size / time cost / success / SOC / makespan / success / memory usage / init time cost / decom time cost / max subproblem / num of subproblems
        std::cout << "instance has " << instances.size() << " agents, raw " << func_identifer << " find solution ? "
                  << !raw_paths.empty()
                  << " in " << total_time_cost << "s " << std::endl;

        // agents size / time cost / success / SOC / makespan / success / memory usage
        std::stringstream ss_raw;
        ss_raw << "RAW_" << func_identifer << " " << instances.size() << " "
               << total_time_cost << " "
               << getSOC(raw_paths) << " " << getMakeSpan(raw_paths) << " "
               << !raw_paths.empty() << " " << 0;

        return ss_raw.str();
    }


#define MAKESPAN 1
#define SOC 2

#define FULL_ID 1
#define SIMPLE_ID 0

    template<Dimension N>
    std::string ID_MAPF_RAW(const std::vector<std::pair<Pointi<N>, Pointi<N>>> &instances,
                             DimensionLength *dim,
                             const IS_OCCUPIED_FUNC<N> &isoc,
                             const MAPF_FUNC<N> & mapf_func,
                             const std::string &func_identifer,
                             double time_limit) {
        memory_recorder.clear();

        sleep(1);

        MSTimer mst;

        auto inst = new Hybird_MAPF::Instance(dim, isoc, instances);
        auto Solver = std::make_shared<Hybird_MAPF::ID>(inst, FULL_ID, SOC);

        Solver->mapf_func_ = mapf_func;  Solver->runtime = time_limit*1e3;
        auto ret_val = Solver->SolveProblem(std::vector<bool> {true,false,false});
        Paths<2> paths;
        if(Solver->solved) {
            std::cout << "ID with " << instances.size() << " agents success" << std::endl;
            paths = Solver->paths_fr;
        } else {
            std::cout << "ID with " << instances.size() << " agents failed" << std::endl;
        }

        int total_cost = 0, maximum_single_cost = 0;
        for(const auto& path : paths) {
            total_cost += path.size();
            maximum_single_cost = std::max(maximum_single_cost, (int)path.size());
        }
        double time_cost = mst.elapsed() / 1e3;
        sleep(1);

//        if(1){
//            std::cout << name << " maximal usage = " << maximal_usage - base_usage << " MB" << " with data size " << memory_recorder.getAllUsedMemory().size() << std::endl; \
//        }
//        if(1) {
//            std::cout << name << " time_cost = " << time_cost << " ms" << std::endl; \
//        }
//        std::stringstream ss;
//        ss << name << " " << ists.size() << " " << time_cost << " "
//           << total_cost << " " << maximum_single_cost << " " << !paths.empty() << " " << max_size_of_stack_layered << " " \
//           << Solver->getMaximalSubProblem() << " " << Solver->getNumberOfSubProblem();


        std::stringstream ss_id;
        // agents size / time cost / success / SOC / makespan / success / memory usage / max subproblem / num of subproblems
        ss_id << "ID_" << func_identifer << " " << instances.size() << " "
              << time_cost << " "
              << getSOC(paths) << " " << getMakeSpan(paths) << " "
              << !paths.empty() << " " << 0 << " "

              << Solver->getMaximalSubProblem() << " "
              << Solver->getNumberOfSubProblem();
        delete inst;
        return ss_id.str();
    }


}


#endif //LAYEREDMAPF_COMMON_INTERFACES_RAW_H
