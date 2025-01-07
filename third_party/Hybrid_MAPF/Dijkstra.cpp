#include "Dijkstra.h"

using namespace std;
namespace Hybird_MAPF {

    Dijkstra::Dijkstra(Instance *in) {
        inst = in;
    }

    void Dijkstra::ShortestPath(int from, int to, std::vector<int> &solution) {
        solution.clear();
        vector<int> distance(inst->nodes);
        vector<int> path(inst->nodes);

        for (int i = 0; i < inst->nodes; i++) {
            distance[i] = inst->nodes + 1;
            path[i] = i;
        }

        distance[from] = 0;

        set<pair<int, int> > active_vertices;
        active_vertices.insert({0, from});

        while (!active_vertices.empty()) {
            int where = active_vertices.begin()->second;
            if (where == to)
                break;
            active_vertices.erase(active_vertices.begin());
            for (auto neib : inst->graph[where])
                if (distance[neib] > distance[where] + 1) {
                    active_vertices.erase({distance[neib], neib});
                    distance[neib] = distance[where] + 1;
                    active_vertices.insert({distance[neib], neib});
                    path[neib] = where;
                }
        }

        int tmp = to;

        while (tmp != from) {
            solution.push_back(tmp);
            tmp = path[tmp];
        }
        solution.push_back(from);

        reverse(solution.begin(), solution.end());

        return;
    }
}