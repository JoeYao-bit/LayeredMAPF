//
// Created by yaozhuo on 2024/9/6.
//

#ifndef LAYEREDMAPF_LAYERED_LARGE_AGENT_LACAM_H
#define LAYEREDMAPF_LAYERED_LARGE_AGENT_LACAM_H

#include "large_agent_lacam.h"

namespace freeNav::LayeredMAPF::LA_MAPF::LaCAM {

    template<Dimension N>
    std::vector<LAMAPF_Path> LargeAgentLaCAM_func(const InstanceOrients<N> & instances,
                                                  const std::vector<AgentPtr<N> >& agents,
                                                  DimensionLength* dim,
                                                  const IS_OCCUPIED_FUNC<N> & isoc,
                                                  const LargeAgentStaticConstraintTablePtr<N>& path_constraint,
                                                  std::vector<std::vector<int> >& grid_visit_count_table,
                                                  double cutoff_time = 30,

                                                  const std::vector<PosePtr<int, N> >& all_poses = {},
                                                  const DistanceMapUpdaterPtr<N>& distance_map_updater = nullptr,
                                                  const std::vector<SubGraphOfAgent<N> >& agent_sub_graphs = {},
                                                  const std::vector<std::vector<int> >& agents_heuristic_tables = {},
                                                  const std::vector<std::vector<int> >& agents_heuristic_tables_ignore_rotate = {},
                                                  ConnectivityGraph* connect_graph = nullptr) {

        LargeAgentLaCAM<N, LaCAM::LargeAgentConstraints<N> > solver(instances, agents,
                                                                                dim, isoc,
                                                                                path_constraint,
                                                                                all_poses,
                                                                                distance_map_updater,
                                                                                agent_sub_graphs,
                                                                                agents_heuristic_tables,
                                                                                agents_heuristic_tables_ignore_rotate,
                                                                                cutoff_time);


        if(solver.solve()) {
            return solver.getSolution();
        } else {
            return {};
        }
    }

}

#endif //LAYEREDMAPF_LAYERED_LARGE_AGENT_LACAM_H
