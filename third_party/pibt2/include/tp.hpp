#pragma once
#include "solver.hpp"
namespace PIBT_2 {

    class TP : public MAPD_Solver {
    public:
        static const std::string SOLVER_NAME;

    private:
        struct Agent {
            int id;
            Node *v_now;  // current location
            Task *task;
            bool load_task;
        };
        using Agents = std::vector<Agent *>;

        void updatePath1(int i, Task *task, std::vector<path_pathfinding::Path> &TOKEN);

        void updatePath2(int i, std::vector<path_pathfinding::Path> &TOKEN, Tasks &unassigned_tasks);

        void updatePath(int i, Node *g, std::vector<path_pathfinding::Path> &TOKEN);

        std::vector<std::vector<int>> CONFLICT_TABLE;  // time, node -> agent
        static constexpr int NIL = -1;

        // main
        void run();

    public:
        TP(MAPD_Instance *_P, bool _use_distance_table = false);

        ~TP() {}

        static void printHelp();
    };
}