#include <set>
#include "instance.h"

#ifndef DIJKSTRA_H
#define DIJKSTRA_H

namespace Hybird_MAPF {

// yz: Dijkstra path planning
    class Dijkstra {
    public:
        Dijkstra(Instance *);

        // yz: search path from start node to target node, return solution in sequence of node id
        void ShortestPath(int, int, std::vector<int> &);

    private:
        Instance *inst;
    };
}
#endif /* DIJKSTRA_H */