/*
 * instance definition
 */
#pragma once
#include <random>

#include "graph.hpp"
#include "utils.hpp"
#include "../../../freeNav-base/basic_elements/point.h"
#include "EECBS/inc/ConstraintTable.h" // yz: use EECBS's constraint table as static constraint during each expansion of LaCAM

namespace LaCAM2 {
    struct Instance {
        const Graph G;
        Config starts;
        Config goals;
        const uint N;  // number of agents

        // for testing
        Instance(const std::string &map_filename,
                 const std::vector<uint> &start_indexes,
                 const std::vector<uint> &goal_indexes);

        // for MAPF benchmark
        Instance(const std::string &scen_filename, const std::string &map_filename,
                 const uint _N = 1);

        // random instance generation
        Instance(const std::string &map_filename, std::mt19937 *MT,
                 const uint _N = 1);

        // yz: for freeNav style interfaces
        Instance(freeNav::DimensionLength* dim, const freeNav::IS_OCCUPIED_FUNC<2> & isoc,
                 const freeNav::Instances<2> & instance_sat);

        ~Instance() {}

        // simple feasibility check of instance
        bool is_valid(const int verbose = 0) const;

        CBS_Li::ConstraintTable* ct = nullptr;

    };

// solution: a sequence of configurations
    using Solution = std::vector<Config>;

    std::ostream &operator<<(std::ostream &os, const Solution &solution);
}