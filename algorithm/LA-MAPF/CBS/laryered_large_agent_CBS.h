//
// Created by yaozhuo on 2024/8/15.
//

#ifndef LAYEREDMAPF_LARYERED_LARGE_AGENT_CBS_H
#define LAYEREDMAPF_LARYERED_LARGE_AGENT_CBS_H

#include "large_agent_CBS.h"

namespace freeNav::LayeredMAPF::LA_MAPF::CBS {

    template<Dimension N, typename AgentType>
    std::vector<LAMAPF_Path> LargeAgentCBS_func(const InstanceOrients<N> & instances,
                                           const std::vector<AgentType>& agents,
                                           DimensionLength* dim,
                                           const IS_OCCUPIED_FUNC<N> & isoc,
                                           const LargeAgentPathConstraintTablePtr<N, AgentType>& path_constraint,
                                           std::vector<std::vector<int> >& grid_visit_count_table,
                                           double cutoff_time = 30) {
        LargeAgentCBS<N, AgentType> solver(instances, agents, dim, isoc, path_constraint);
        grid_visit_count_table = solver.grid_visit_count_tables_;
        if(solver.solve(cutoff_time)) {
            return solver.getSolution();
        } else {
            return {};
        }
    }

}

#endif //LAYEREDMAPF_LARYERED_LARGE_AGENT_CBS_H
