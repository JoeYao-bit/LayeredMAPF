/*
 * Implementation of Push & Swap
 *
 * - ref
 * Luna, R., & Bekris, K. E. (2011, July).
 * Push and swap: Fast cooperative path-finding with completeness guarantees.
 * In IJCAI (pp. 294-300).
 */

#pragma once
#include "solver.hpp"
namespace PIBT_2 {

    class PushAndSwap : public MAPF_Solver {
    public:
        static const std::string SOLVER_NAME;

    private:

        Nodes nodes_with_many_neighbors;  // nodes of degree >= 3
        bool emergency_stop;

        // used in occupancy
        static constexpr int NIL = -1;

        // main
        void run();

        // push operation
        bool push(Plan &plan, const int i, Nodes &U, std::vector<int> &occupied_now);

        // swap operation
        bool swap(Plan &plan, const int i, Nodes &U, std::vector<int> &occupied_now);

        bool swap(Plan &plan, const int i, Nodes &U, std::vector<int> &occupied_now,
                  std::vector<int> &recursive_list);

        // improve solution quality, see
        Plan compress(const Plan &plan);

        // ---------------------------------------
        // sub procedures

        // push several agents simultaneously
        bool multiPush(Plan &plan, const int r, const int s, const path_pathfinding::Path &p,
                       std::vector<int> &occupied_now);

        // clear operation
        bool clear(Plan &plan, Node *v, const int r, const int s,
                   std::vector<int> &occupied_now);

        // execute swap operation
        void executeSwap(Plan &plan, const int r, const int s,
                         std::vector<int> &occupied_now);

        // resolve operation
        bool resolve(Plan &plan, const int r, const int s, Nodes &U,
                     std::vector<int> &occupied_now);

        // ---------------------------------------
        // utilities

        // get nearest empty location
        Node *getNearestEmptyNode(Node *v, std::vector<int> &occupied_now,
                                  const Nodes &obs);

        // get the shortest path towards goals
        path_pathfinding::Path getShortestPath(const int id, Node *s, std::vector<int> &occupied_now);

        // push toward empty node
        bool pushTowardEmptyNode(Node *v, Plan &plan, std::vector<int> &occupied_now,
                                 const Nodes &obs);

        // update plan
        void updatePlan(const int id, Node *next_node, Plan &plan,
                        std::vector<int> &occupied_now);

        // get all vertices of degree >= 3 on G
        void findNodesWithManyNeighbors();

        // error check
        void checkConsistency(Plan &plan, std::vector<int> &occupied_now);

    public:
        PushAndSwap(MAPF_Instance *_P);

        ~PushAndSwap() {}

        void setParams(int argc, char *argv[]);

        static void printHelp();

        bool flg_compress = false;                // whether to compress solution
        bool disable_dist_init = false;           // prioritization depending on distance
    };
}