#pragma once

#include "dist_table.hpp"
#include "graph.hpp"
#include "instance.hpp"
#include "planner.hpp"
#include "post_processing.hpp"
#include "utils.hpp"
#include "lacam2//include/post_processing.hpp"

namespace LaCAM2 {
// main function
    Solution solve(const Instance &ins, std::string &additional_info,
                   const int verbose = 0, const Deadline *deadline = nullptr,
                   std::mt19937 *MT = nullptr, const Objective objective = OBJ_NONE,
                   const float restart_rate = 0.001);


    freeNav::Paths<2> lacam2_MAPF(freeNav::DimensionLength *dim,
                                  const freeNav::IS_OCCUPIED_FUNC<2> &isoc,
                                  const freeNav::Instances<2> &instance_sat,
                                  CBS_Li::ConstraintTable *ct,
                                  int cutoff_time);

}