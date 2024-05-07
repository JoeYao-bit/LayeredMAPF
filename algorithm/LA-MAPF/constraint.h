//
// Created by yaozhuo on 2024/5/7.
//

#ifndef LAYEREDMAPF_CONSTRAINT_H
#define LAYEREDMAPF_CONSTRAINT_H

#include "../freeNav-base/basic_elements/point.h"

namespace freeNav::LayeredMAPF::LA_MAPF {

    // <agent id, node from, node to, time range start, time range end>
    typedef std::tuple<int, int, int, int, int> Constraint;

    // conflict resulted by two agents, and constraint store what move and when cause conflict
    typedef std::pair<Constraint, Constraint> Conflict;

    struct ConstraintTable {
        //
    };

}

#endif //LAYEREDMAPF_CONSTRAINT_H
