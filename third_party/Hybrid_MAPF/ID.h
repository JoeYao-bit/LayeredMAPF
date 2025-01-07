#include <thread>

#include "instance.h"
#include "Solver.h"
#include "Picat.h"
#include "CBS.h"
#include "ICTS.h"
#include "Dijkstra.h"

#ifndef ID_H
#define ID_H
namespace Hybird_MAPF {

    class ID {
    public:
        ID(Instance *, int, int);

        ~ID();

        int SolveProblem(const std::vector<bool> & = {true, true, true});


        // statistic variables
        std::vector<int> solver_computed;
        std::vector<int> solver_used;
        std::vector<std::vector<long long> > solver_time;
        int final_makespan;
        int final_soc;

    private:
        Instance *inst;
        Dijkstra *single_path;
        std::vector<Solver *> solvers;
        int cost_function; // 1 - Makespan, 2 - Sum of Costs
        int full_ID; // 0 - simple ID, 1 - full ID

        std::vector<std::vector<int> > groups;
        std::vector<int> agent_to_group;
        std::vector<std::pair<int, int> > conflicted_groups;
        std::vector<std::vector<int> > current_plan;

        long long runtime;

        bool CheckForConflicts(int &, int &);

        bool CheckPastConflicts(int, int);

        void MergeGroups(int, int);

        int ComputeGroupCost(int);

        void FixPlan(std::vector<int> &, std::vector<std::vector<int> > &);

        int PlanForGroups(int, int, int);

        int CleanUp(int);
    };
}
#endif /* ID_H */