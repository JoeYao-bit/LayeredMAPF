#pragma once
#include <getopt.h>

#include <chrono>
#include <functional>
#include <memory>
#include <queue>
#include <unordered_map>

#include "paths.hpp"
#include "plan.hpp"
#include "problem.hpp"
#include "util.hpp"
#include "../../../freeNav-base/basic_elements/point.h" // yz: transform to freeNav style path
namespace PIBT_2 {

    extern std::set<int> visited_grid_; // yz: visited grid during path search

    class MinimumSolver {
    protected:
        std::string solver_name;  // solver name
        path_pathfinding::Graph *const G;           // graph
        std::mt19937 *const MT;   // seed for randomness
        const int max_timestep;   // maximum makespan
        const int max_comp_time;  // time limit for computation, ms
        Plan solution;            // solution
        bool solved;              // success -> true, failed -> false (default)

    private:
        int comp_time;             // computation time
        Time::time_point t_start;  // when to start solving

    protected:
        bool verbose;    // true -> print additional info
        bool log_short;  // true -> cannot visualize the result, default: false

        // -------------------------------
        // utilities for time
    public:
        int getRemainedTime() const;  // get remained time
        bool overCompTime() const;    // check time limit

        freeNav::Paths<2> getResultPath(freeNav::DimensionLength* dim, const freeNav::Instances<2> &instance_sat) {
            //std::cout << " solution.size() " << solution.size() << " solution.front().size() " << solution.last().size() << std::endl;
            // yz: transform to freeNav style path
            freeNav::Paths<2> retv(instance_sat.size());
            for(int i=0; i<instance_sat.size(); i++) {
                const auto& copy_of_path = solution.getPath(i);
                for(int t=0; t<copy_of_path.size(); t++) {
                    retv[i].push_back(freeNav::IdToPointi<2>(copy_of_path[t]->id, dim));
                    //assert(retv[agent].size()-1 == t);
                }
                //std::cout << " path " << i << ":" << retv[i] << std::endl;
            }
            // yz: remove way point when agent is stop
            for(int agent=0; agent<instance_sat.size(); agent++) {
                const freeNav::Pointi<2>& target = retv[agent].back();
                auto& path = retv[agent];
//            std::cout << "before prune" << path << std::endl;
                for(auto iter = path.end(); iter != path.begin(); ) {
                    if(*(iter-2) == target) {
                        iter = path.erase(iter-1);
                    } else {
                        break;
                    }
                }
                //std::cout << " path " << agent << ":" << retv[agent] << std::endl;
                assert(path.front() == instance_sat[agent].first);
                assert(path.back() == instance_sat[agent].second);
            }
            return retv;
        }

        CBS_Li::ConstraintTable *ct_ = nullptr;

        // -------------------------------
        // utilities for debug
    protected:
        // print debug info (only when verbose=true)
        void info() const;

        template<class Head, class... Tail>
        void info(Head &&head, Tail &&... tail) const {
            if (!verbose) return;
            std::cout << head << " ";
            info(std::forward<Tail>(tail)...);
        }

        void halt(const std::string &msg) const;  // halt program
        void warn(const std::string &msg) const;  // just printing msg

        // -------------------------------
        // utilities for solver options
    public:
        virtual void setParams(int argc, char *argv[]) {};

        void setVerbose(bool _verbose) { verbose = _verbose; }

        void setLogShort(bool _log_short) { log_short = _log_short; }

        // -------------------------------
        // print help
    protected:
        static void printHelpWithoutOption(const std::string &solver_name);

        // -------------------------------
        // utilities for computing path

        // space-time A*
        struct AstarNode {
            Node *v;           // location
            int g;             // time
            int f;             // f-value
            AstarNode *p;      // parent
            std::string name;  // name
            AstarNode(Node *_v, int _g, int _f, AstarNode *_p);

            static std::string getName(Node *_v, int _g);
        };

        using CompareAstarNode = std::function<bool(AstarNode *, AstarNode *)>;
        using CheckAstarFin = std::function<bool(AstarNode *)>;
        using CheckInvalidAstarNode = std::function<bool(AstarNode *)>;
        using AstarHeuristics = std::function<int(AstarNode *)>;
        using AstarNodes = std::vector<AstarNode *>;

        /*
         * Template of Space-Time A*.
         * See the following reference.
         *
         * Cooperative Pathﬁnding.
         * D. Silver.
         * AI Game Programming Wisdom 3, pages 99–111, 2006.
         */
        static path_pathfinding::Path getPathBySpaceTimeAstar(
                Node *const s,                 // start
                Node *const g,                 // goal
                AstarHeuristics &fValue,       // func: f-value
                CompareAstarNode &compare,     // func: compare two nodes
                CheckAstarFin &checkAstarFin,  // func: check goal
                CheckInvalidAstarNode &
                checkInvalidAstarNode,  // func: check invalid nodes
                const int time_limit = -1   // time limit
        );

        // typical functions
        static CompareAstarNode compareAstarNodeBasic;

    public:
        virtual void solve();  // call start -> run -> end
    protected:
        void start();

        void end();

        virtual void exec() {};  // main

    public:
        MinimumSolver(Problem *_P, CBS_Li::ConstraintTable *ct=nullptr);

        virtual ~MinimumSolver() {};

        // getter
        Plan getSolution() const { return solution; };

        bool succeed() const { return solved; };

        std::string getSolverName() const { return solver_name; };

        int getMaxTimestep() const { return max_timestep; };

        int getCompTime() const { return comp_time; }

        int getSolverElapsedTime() const;  // get elapsed time from start
    };

// -----------------------------------------------
// base class with utilities
// -----------------------------------------------
    class MAPF_Solver : public MinimumSolver {
    protected:
        MAPF_Instance *const P;  // problem instance

    private:
        // useful info
        int LB_soc;       // lower bound of soc
        int LB_makespan;  // lower bound of makespan

        // distance to goal
    protected:
        using DistanceTable = std::vector<std::vector<int>>;  // [agent][node_id]
        DistanceTable distance_table;                         // distance table
        DistanceTable *distance_table_p;  // pointer, used in nested solvers
        int preprocessing_comp_time;      // computation time

        // -------------------------------
        // main
    private:
        void exec();

    protected:
        virtual void run() {}  // main

        // -------------------------------
        // utilities for problem instance
    public:
        int getLowerBoundSOC();       // get trivial lower bound of sum-of-costs
        int getLowerBoundMakespan();  // get trivial lower bound of makespan
    private:
        void computeLowerBounds();  // compute lb_soc and lb_makespan

        // -------------------------------
        // utilities for solution representation
    public:
        static Paths planToPaths(const Plan &plan);   // plan -> paths
        static Plan pathsToPlan(const Paths &paths);  // paths -> plan

        // -------------------------------
        // log

        // yz: add all static paths into path table
        void updateStaticPaths() {

            if(ct_ != nullptr) {
                for(const auto& static_path : ct_->static_paths) {
                    updateStaticPath(static_path);
                }
            }
        }

        // yz: add single CBS style path into path table
        void updateStaticPath(const CBS_Li::MAPFPath& path) {
            int makespan = path.size();
            const int nodes_size = G->getNodesSize();
            // extend PATH_TABLE
            while ((int) PATH_TABLE.size() < makespan + 1)
                PATH_TABLE.push_back(std::vector<int>(nodes_size, NIL));
            // update locations
            for (int t = 0; t < makespan; t++) PATH_TABLE[t][path[t].location] = P->getNum() + 1;
        }


    public:
        virtual void makeLog(const std::string &logfile = "./result.txt");

    protected:
        virtual void makeLogBasicInfo(std::ofstream &log);

        virtual void makeLogSolution(std::ofstream &log);

        // -------------------------------
        // params
    protected:
        // used for set underlying solver options
        static void setSolverOption(std::shared_ptr<MAPF_Solver> solver,
                                    const std::vector<std::string> &option);

        // -------------------------------
        // print
    public:
        void printResult();

        // -------------------------------
        // utilities for distance
    public:
        int pathDist(Node *const s, Node *const g) const { return G->pathDist(s, g); }

        int pathDist(const int i,
                     Node *const s) const;  // get path distance between s -> g_i
        int pathDist(const int i) const;    // get path distance between s_i -> g_i
        void createDistanceTable();         // compute distance table
        void setDistanceTable(DistanceTable *p) {
            distance_table_p = p;
        }  // used in nested solvers

        // -------------------------------
        // utilities for getting path
    public:
        // use grid-pathfinding
        path_pathfinding::Path getPath(Node *const s, Node *const g, bool cache = false) const {
            return G->getPath(s, g, cache);
        }

        // prioritized planning
        path_pathfinding::Path getPrioritizedPath(
                const int id,                // agent id
                const Paths &paths,          // already reserved paths
                const int time_limit = -1,   // time limit
                const int upper_bound = -1,  // upper bound of timesteps
                const std::vector<std::tuple<Node *, int>> &constraints =
                        {},  // additional constraints, space-time
                CompareAstarNode &compare = compareAstarNodeBasic,  // compare two nodes
                const bool manage_path_table =
                true  // manage path table automatically, conflict check
        );

    protected:
        // used for checking conflicts
        void updatePathTable(const Paths &paths, const int id);

        void clearPathTable(const Paths &paths);

        void updatePathTableWithoutClear(const int id, const path_pathfinding::Path &p,
                                         const Paths &paths);

        static constexpr int NIL = -1;
        std::vector<std::vector<int>> PATH_TABLE;

    public:
        MAPF_Solver(MAPF_Instance *_P, CBS_Li::ConstraintTable *ct=nullptr);

        virtual ~MAPF_Solver();

        MAPF_Instance *getP() { return P; }
    };

// ====================================================

    class MAPD_Solver : public MinimumSolver {
    protected:
        MAPD_Instance *const P;  // problem instance

        std::vector<Nodes> hist_targets;  // time, agent -> current target
        std::vector<Tasks> hist_tasks;    // time, agent -> assigned_task

    public:
        void printResult();

        // -------------------------------
        // log
    public:
        virtual void makeLog(const std::string &logfile = "./result.txt");

    protected:
        virtual void makeLogBasicInfo(std::ofstream &log);

        virtual void makeLogSolution(std::ofstream &log);

        // -------------------------------
        // distance
    protected:
        bool use_distance_table;
        int preprocessing_comp_time;                          // computation time
        using DistanceTable = std::vector<std::vector<int>>;  // [node_id][node_id]
        DistanceTable distance_table;                         // distance table
        int pathDist(Node *const s, Node *const g) const;

    private:
        void createDistanceTable();

        // -------------------------------
        // metric
    public:
        float getTotalServiceTime();

        float getAverageServiceTime();

        // -------------------------------
        // main
    public:
        void solve();

    private:
        void exec();

    protected:
        virtual void run() {}  // main

    public:
        MAPD_Solver(MAPD_Instance *_P, bool _use_distance_table = false);

        virtual ~MAPD_Solver();

        MAPD_Instance *getP() { return P; }
    };
}