#include "instance.h"
#include "Solver.h"

#ifndef ICTS_H
#define ICTS_H
namespace Hybird_MAPF {

    class ICTSSolver : public Solver {
    public:
        ICTSSolver(Instance *, int);

        void Solve(std::vector<int> &, std::vector<std::vector<int> > &, int, long long);

        int ReadResults(std::vector<std::vector<int> > &, int);

        long long total_time;
        int makespan;
        std::vector<std::vector<int> > plan;
    private:
        Instance *inst;
        std::string solver_input;
        std::string solver_output;
        std::string solver_name;

        int cost_function;

        std::pair<int, int> FindCoordinates(std::vector<std::vector<int> >, int);
    };
}
#endif /* ICTS_H */