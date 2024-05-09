//
// Created by yaozhuo on 2024/5/5.
//

#ifndef LAYEREDMAPF_LA_CBS_H
#define LAYEREDMAPF_LA_CBS_H

#include "large_agent_mapf.h"
#include "space_time_astar.h"

namespace freeNav::LayeredMAPF::LA_MAPF {

    template <Dimension N, typename AgentType>
    class LargeAgentCBS : public LargeAgentMAPF<N, AgentType> {
    public:
        LargeAgentCBS(const InstanceOrients<N> & instances,
                      const std::vector<AgentType>& agents,
                      DimensionLength* dim,
                      const IS_OCCUPIED_FUNC<N> & isoc) : LargeAgentMAPF<N, AgentType>(instances, agents, dim, isoc) {
            // 1, initial paths
            for(int agent=0; agent<this->instance_node_ids_.size(); agent++) {
                ConstraintTable<N, AgentType> constraint_table(agent, this->agents_, this->all_poses_, this->dim_, this->isoc_);
                const size_t& start_node_id = this->instance_node_ids_[agent].first,
                              target_node_id = this->instance_node_ids_[agent].second;
                SpaceTimeAstar<N, AgentType> astar(start_node_id, target_node_id,
                                                   this->agents_heuristic_tables_[agent],
                                                   this->agent_sub_graphs_[agent],
                                                   constraint_table);
                LAMAPF_Path solution = astar.solve();
                if(solution.empty()) {
                    std::cerr << " agent " << agent << " search path failed " << std::endl;
                } else {
                    std::cout << instances[agent].first << "->" << instances[agent].second << std::endl;
                    for(int t=0; t<solution.size(); t++) {
                        std::cout << *(this->all_poses_[solution[t]]) << "->";
                    }
                    std::cout << std::endl;
                    solutions_.push_back(solution);
                }
            }
        }

        virtual bool solve() override {
            return false;
        }

        std::vector<LAMAPF_Path> solutions_;

    };


}

// when both move, do edge-2-edge check
// when only one move, do edge-2-vertex check

#endif //LAYEREDMAPF_LA_CBS_H
