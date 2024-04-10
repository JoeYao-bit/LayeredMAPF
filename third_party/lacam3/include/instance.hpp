/*
 * instance definition
 */
#pragma once
#include <random>

#include "graph.hpp"
#include "utils.hpp"
#include "../../../freeNav-base/basic_elements/point.h"
#include "EECBS/inc/ConstraintTable.h" // yz: use EECBS's constraint table as static constraint during each expansion of LaCAM

namespace LaCAM3 {

    struct Instance {
        Graph *G;       // graph
        Config starts;  // initial configuration
        Config goals;   // goal configuration
        const uint N;   // number of agents
        bool delete_graph_after_used;

        Instance(Graph *_G, const Config &_starts, const Config &_goals, uint _N);

        Instance(const std::string &map_filename,
                 const std::vector<int> &start_indexes,
                 const std::vector<int> &goal_indexes);

        // for MAPF benchmark
        Instance(const std::string &scen_filename, const std::string &map_filename,
                 const int _N = 1);

        // random instance generation
        Instance(const std::string &map_filename, const int _N = 1,
                 const int seed = 0);


        // yz: for freeNav style interfaces
        Instance(freeNav::DimensionLength* dim, const freeNav::IS_OCCUPIED_FUNC<2> & isoc,
                 const freeNav::Instances<2> & instance_sat);

        ~Instance();

        // simple feasibility check of instance
        bool is_valid(const int verbose = 0) const;
    };

// solution: a sequence of configurations
    using Solution = std::vector<Config>;
}