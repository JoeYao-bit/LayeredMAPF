//
// Created by yaozhuo on 2024/5/19.
//

#ifndef LAYEREDMAPF_COMMON_H
#define LAYEREDMAPF_COMMON_H

#include <boost/geometry.hpp>
#include <boost/unordered_map.hpp>
#include <boost/heap/pairing_heap.hpp>

#include "../freeNav-base/basic_elements/distance_map_update.h"
#include "../../freeNav-base/basic_elements/point.h"
#include "../freeNav-base/basic_elements/point.h"

namespace freeNav::LayeredMAPF::LA_MAPF {

#define MAX_TIMESTEP INT_MAX / 2
#define MAX_COST INT_MAX / 2
#define MAX_NODES INT_MAX / 2

    // <agent id, node from, node to, time range start, time range end>
    typedef std::tuple<int, size_t, size_t, int, int> Constraint;

    typedef std::list<std::shared_ptr<Constraint> > Constraints;

    // conflict resulted by two agents, and constraint store what move and when cause conflict
    //typedef std::pair<Constraint, Constraint> Conflict;

    enum conflict_type {
        STANDARD, TYPE_COUNT
    };

    enum conflict_priority {
        CARDINAL, PSEUDO_CARDINAL, SEMI, NON, UNKNOWN, PRIORITY_COUNT
    };

    struct Conflict {
        Conflict(int a1, int a2, const Constraints& cs1, const Constraints& cs2)
        : a1(a1), a2(a2), cs1(cs1), cs2(cs2) {
            int agent; size_t from, to;
            int start_t, end_t;
            // get start time index of conflict 1
            for(const auto& cs : cs1) {
                std::tie(agent, from, to, start_t, end_t) = *cs;
                t1 = start_t < t1 ? start_t : t1;
                t2 = end_t > t2   ? end_t   : t2;
            }
            for(const auto& cs : cs2) {
                std::tie(agent, from, to, start_t, end_t) = *cs;
                t1 = start_t < t1 ? start_t : t1;
                t2 = end_t > t2   ? end_t   : t2;
            }
        }

        int a1, a2; // agent id
        int t1 = MAX<int>, t2 = 0; // time range of constraints
        Constraints cs1;
        Constraints cs2;

        conflict_type type;
        conflict_priority priority = conflict_priority::UNKNOWN;
        double secondary_priority = 0; // used as the tie-breaking creteria for conflict selection
    };

    typedef std::vector<std::shared_ptr<Conflict> > Conflicts;

    bool operator<(const Conflict &conflict1, const Conflict &conflict2);


    typedef std::vector<size_t> LAMAPF_Path; // node id sequence

    bool isSamePath(const std::vector<size_t>& path1, const std::vector<size_t>& path2);

    template <Dimension N>
    struct Agent {

        explicit Agent(float excircle_radius, float incircle_radius, int id) : excircle_radius_(excircle_radius), incircle_radius_(incircle_radius),id_(id) {}

        virtual bool isCollide(const Pose<N>& pose,
                               DimensionLength* dim,
                               const IS_OCCUPIED_FUNC<N>& isoc,
                               const DistanceMapUpdater<N>& distance_table) const = 0;

        virtual bool isCollide(const Pose<N>& edge_from,
                               const Pose<N>& edge_to,
                               DimensionLength* dim,
                               const IS_OCCUPIED_FUNC<N>& isoc,
                               const DistanceMapUpdater<N>& distance_table) const = 0;

        float excircle_radius_, incircle_radius_;

        int id_;

    };

    template <Dimension N>
    using Agents = std::vector<Agent<N> >;

    class HighLvNode;
}

#endif //LAYEREDMAPF_COMMON_H
