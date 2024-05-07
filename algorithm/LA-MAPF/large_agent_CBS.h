//
// Created by yaozhuo on 2024/5/5.
//

#ifndef LAYEREDMAPF_LA_CBS_H
#define LAYEREDMAPF_LA_CBS_H

#include "large_agent_mapf.h"

namespace freeNav::LayeredMAPF::LA_MAPF {

    template <Dimension N>
    class LargeAgentCBS : public LargeAgentMAPF<N> {
    public:
        LargeAgentCBS(const InstanceOrients<N> & instances,
                      const Agents<N>& agents,
                      DimensionLength* dim,
                      const IS_OCCUPIED_FUNC<N> & isoc) : LargeAgentMAPF<N>(instances, agents, dim, isoc) {
        }

        virtual bool solve() override {
            return false;
        }


    };


}

// when both move, do edge-2-edge check
// when only one move, do edge-2-vertex check

#endif //LAYEREDMAPF_LA_CBS_H
