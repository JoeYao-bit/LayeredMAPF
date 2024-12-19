#pragma once
#include "Instance.h"
#include "EECBS/inc/ConstraintTable.h"
#include "../../../freeNav-base/basic_elements/point.h"
#include "EECBS/inc/ECBS.h"
#include "EECBS/inc/common.h"

namespace MAPF_LNS2 {

    class AnytimeEECBS {
    public:
        std::vector<CBS_Li::MAPFPath> solution;
        std::list<IterationStats> iteration_stats; //stats about each iteration
        double preprocessing_time = 0;
        double runtime = 0;
        int sum_of_costs = MAX_COST;
        int sum_of_costs_lowerbound = 0;
        int sum_of_distances = -1;

        AnytimeEECBS(const CBS_Li::Instance &instance, double time_limit, int screen, CBS_Li::ConstraintTable *ct=nullptr) :
                instance(instance), time_limit(time_limit), screen(screen),ct_(ct) {}

        void run();

        void validateSolution() const;

        void writeIterStatsToFile(string file_name) const;

        void writeResultToFile(string file_name) const;

        string getSolverName() const { return "AnytimeEECBS"; }

        void savePaths(); // yz: save result path in freeNav style

        freeNav::Paths<2> getfreeNavPath() { return fr_paths_; }

        CBS_Li::ConstraintTable *ct_ = nullptr;

    private:
        // intput params
        const CBS_Li::Instance &instance; // avoid making copies of this variable as much as possible
        double time_limit;
        int screen;

        freeNav::Paths<2> fr_paths_;

    };
}