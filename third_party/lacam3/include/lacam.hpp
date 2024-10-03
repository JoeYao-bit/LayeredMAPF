#pragma once

#include "dist_table.hpp"
#include "graph.hpp"
#include "instance.hpp"
#include "planner.hpp"
#include "post_processing.hpp"
#include "sipp.hpp"
#include "utils.hpp"
#include <argparse/argparse.hpp>

namespace LaCAM3 {

    Solution solve(const Instance &ins, const int verbose = 0,
                   const Deadline *deadline = nullptr, int seed = 0);

    freeNav::Paths<2> lacam3_MAPF(freeNav::DimensionLength *dim,
                                  const freeNav::IS_OCCUPIED_FUNC<2> &isoc,
                                  const freeNav::Instances<2> &instance_sat,
                                  CBS_Li::ConstraintTable *ct,
                                  int cutoff_time);

}