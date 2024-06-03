//
// Created by yaozhuo on 2024/6/1.
//

#include "common.h"

namespace freeNav::LayeredMAPF::LA_MAPF {

    // yz: determine priority of conflicts
    bool operator<(const Conflict &conflict1, const Conflict &conflict2) // return true if conflict2 has higher priority
    {
        // always the same priority under current mode
//    if(conflict1.priority == CARDINAL || conflict2.priority == CARDINAL) {
//        std::cout << " find cardinal conflict " << std::endl;
//    }
//    if(conflict1.priority != conflict2.priority) {
//        std::cout << " find not same conflict " << std::endl;
//    }
        if (conflict1.priority == conflict2.priority) {
            if (conflict1.type == conflict2.type) {
                if (conflict1.secondary_priority == conflict2.secondary_priority) {
                    return rand() % 2;
                }
                return conflict1.secondary_priority > conflict2.secondary_priority;
            }
            return conflict1.type > conflict2.type;
        }
        return conflict1.priority > conflict2.priority;
    }

}