//
// Created by yaozhuo on 2024/5/19.
//

#ifndef LAYEREDMAPF_COMMON_H
#define LAYEREDMAPF_COMMON_H

#include <boost/geometry.hpp>
#include <boost/unordered_map.hpp>
#include "../freeNav-base/basic_elements/distance_map_update.h"
#include "../../freeNav-base/basic_elements/point.h"
#include "../freeNav-base/basic_elements/point.h"

namespace freeNav::LayeredMAPF::LA_MAPF {

    // <agent id, node from, node to, time range start, time range end>
    typedef std::tuple<int, size_t, size_t, int, int> Constraint;

    // conflict resulted by two agents, and constraint store what move and when cause conflict
    typedef std::pair<Constraint, Constraint> Conflict;

    typedef std::vector<size_t> LAMAPF_Path; // node id sequence



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

}

#endif //LAYEREDMAPF_COMMON_H
