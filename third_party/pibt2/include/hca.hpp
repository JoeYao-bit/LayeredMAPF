/*
 * Implementation of Hierarchical Cooperative A* (HCA*)
 *
 * - ref
 * Silver, D. (2005).
 * Cooperative pathfinding.
 * In AIIDE’05 Proceedings of the First AAAI Conference on Artificial
 * Intelligence and Interactive Digital Entertainment (pp. 117–122).
 *
 * Initial priorities are determined such that far agents have high priorities.
 * If you dislike it, use -d [--disable-dist-init] option.
 * - ref
 * Berg, J. P. van den, & Overmars, M. H. (2005).
 * Prioritized motion planning for multiple robots.
 * In 2005 IEEE/RSJ International Conference on Intelligent Robots and Systems
 * (pp. 430–435).
 *
 */

#pragma once
#include "solver.hpp"
#include "../algorithm/layered_mapf.h"
namespace PIBT_2 {

    class HCA : public MAPF_Solver {
    public:
        static const std::string SOLVER_NAME;

    private:

        // get one agent path
        path_pathfinding::Path getPrioritizedPath(int id, const Paths &paths);

        void run();

        // used for tie-break
        std::vector<bool> table_starts;
        std::vector<bool> table_goals;

    public:
        HCA(MAPF_Instance *_P, CBS_Li::ConstraintTable *ct=nullptr);

        ~HCA() {};

        void setParams(int argc, char *argv[]);

        static void printHelp();

        bool disable_dist_init = false;  // option

    };
}