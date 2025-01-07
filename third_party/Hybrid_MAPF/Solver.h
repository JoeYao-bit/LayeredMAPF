#include "instance.h"

#ifndef SOLVER_H
#define SOLVER_H
namespace Hybird_MAPF {

    class Solver {
    public:
        virtual void Solve(std::vector<int> &, std::vector<std::vector<int> > &, int, long long) = 0;

        // 0 - ok, 1 - no solution, x - timeout or error
        virtual int ReadResults(std::vector<std::vector<int> > &, int) = 0;
    };
}
#endif /* SOLVER_H */