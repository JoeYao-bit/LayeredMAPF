//
// Created by yaozhuo on 2024/6/1.
//

#include "common.h"

namespace freeNav::LayeredMAPF::LA_MAPF {

    // yz: determine priority of conflicts
    bool operator<(const Conflict &conflict1, const Conflict &conflict2) // return true if conflict2 has higher priority
    {
//        if (conflict1.priority == conflict2.priority) {
//            if (conflict1.type == conflict2.type) {
//                if (conflict1.secondary_priority == conflict2.secondary_priority) {
//                    return rand() % 2;
//                }
//                return conflict1.secondary_priority > conflict2.secondary_priority;
//            }
//            return conflict1.type > conflict2.type;
//        }
//        return conflict1.priority > conflict2.priority;
         return conflict1.t1 > conflict2.t2;
    }

    bool isSamePath(const std::vector<size_t>& path1, const std::vector<size_t>& path2) {
        if(path1.size() != path2.size()) { return false; }
        for(int t = 0; t < path1.size(); t++) {
            if(path1[t] != path2[t]) {
                return false;
            }
        }
        return true;
    }

}