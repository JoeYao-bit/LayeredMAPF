#ifndef HYBIRD_MAPF_ID_H
#define HYBIRD_MAPF_ID_H
#include <thread>

#include "instance.h"
#include "Solver.h"
#include "Picat.h"
#include "CBS.h"
#include "ICTS.h"
#include "Dijkstra.h"

#include "../algorithm/layered_mapf.h"

namespace Hybird_MAPF {

/*
 * Bibtex:
 *
  @inproceedings{surynek2018variants,
  title={Variants of independence detection in sat-based optimal multi-agent path finding},
  author={Surynek, Pavel and {\v{S}}vancara, Ji{\v{r}}{\'\i} and Felner, Ariel and Boyarski, Eli},
  booktitle={Agents and Artificial Intelligence: 9th International Conference, ICAART 2017, Porto, Portugal, February 24--26, 2017, Revised Selected Papers 9},
  pages={116--136},
  year={2018},
  organization={Springer}
}

 * */

    class ID {
    public:
        ID(Instance *, int, int);

        ~ID();

        int SolveProblem(const std::vector<bool> & = {true, true, true});

        size_t getMaximalSubProblem() const {
            size_t max_subproblem_size = 0;
            for(const auto& g : groups) {
                max_subproblem_size = std::max(g.size(), max_subproblem_size);
            }
            return max_subproblem_size;
        }

        size_t getNumberOfSubProblem() const {
            int count_of_group = 0;
            for(int i=0; i<groups.size(); i++) {
                if(!groups[i].empty()) { count_of_group++; }
            }
            return count_of_group;
        }

        freeNav::LayeredMAPF::MAPF_FUNC<2> mapf_func_;

        // statistic variables
        std::vector<int> solver_computed;
        std::vector<int> solver_used;
        std::vector<std::vector<long long> > solver_time;
        int final_makespan;
        int final_soc;

        //private:
        Instance *inst; // yz: grid map and graph about mapf
        Dijkstra *single_path; // yz: single agent path planner
        std::vector<Solver *> solvers; // yz: multiple mapf path planner
        int cost_function; // 1 - Makespan, 2 - Sum of Costs
        int full_ID; // 0 - simple ID, 1 - full ID

        std::vector<std::vector<int> > groups; // yz: which agents in groups
        std::vector<int> agent_to_group; // yz: mapping from agent to group id
        std::vector<std::pair<int, int> > conflicted_groups;
        std::vector<std::vector<int> > current_plan; // yz: all agent's paths, store node id

        bool solved = false; // yz: whether current instance is solved

        freeNav::Paths<2> paths_fr; // yz: freeNav style paths

        long long runtime; // yz: in ms

        struct timezone tz;
        struct timeval tv_pre;
        struct timeval tv_after; // yz: start time of planning

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