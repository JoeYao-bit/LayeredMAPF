#pragma once
#include "PBSNode.h"
#include "SingleAgentSolver.h"
#include "EECBS/inc/ConstraintTable.h"
#include "../../../algorithm/basic.h"

namespace PBS_Li {


    class PBS {
    public:
        /////////////////////////////////////////////////////////////////////////////////////
        // stats
        double runtime = 0;
        double runtime_generate_child = 0; // runtimr of generating child nodes
        double runtime_build_CT = 0; // runtimr of building constraint table
        double runtime_build_CAT = 0; // runtime of building conflict avoidance table
        double runtime_path_finding = 0; // runtime of finding paths for single agents
        double runtime_detect_conflicts = 0;
        double runtime_preprocessing = 0; // runtime of building heuristic table for the low level

        uint64_t num_HL_expanded = 0;
        uint64_t num_HL_generated = 0;
        uint64_t num_LL_expanded = 0;
        uint64_t num_LL_generated = 0;

        PBSNode *dummy_start = nullptr;
        PBSNode *goal_node = nullptr;


        bool solution_found = false;
        int solution_cost = -2;

        /////////////////////////////////////////////////////////////////////////////////////////
        // set params
        void setConflictSelectionRule(conflict_selection c) { conflict_seletion_rule = c; }

        void setNodeLimit(int n) { node_limit = n; }

        ////////////////////////////////////////////////////////////////////////////////////////////
        // Runs the algorithm until the problem is solved or time is exhausted
        bool solve(double time_limit);

        PBS(const CBS_Li::Instance &instance, const CBS_Li::ConstraintTable &static_constraint, bool sipp, int screen);

        void clearSearchEngines();

        ~PBS();

        // Save results
        void saveResults(const std::string &fileName, const std::string &instanceName) const;

        void saveCT(const std::string &fileName) const; // write the CT to a file
        void savePaths(const std::string &fileName) const; // write the paths to a file
        void savePaths();
        void clear(); // used for rapid random  restart

        freeNav::Paths<2> getfreeNavPath() { return fr_paths_; }

        const CBS_Li::ConstraintTable &static_constraint; // yz: use only to save space

        freeNav::Paths<2> fr_paths_;

    private:
        conflict_selection conflict_seletion_rule;

        std::stack<PBSNode *> open_list;
        std::list<PBSNode *> allNodes_table;


        std::list<int> ordered_agents;
        std::vector<std::vector<bool>> priority_graph; // [i][j] = true indicates that i is lower than j

        std::string getSolverName() const;

        int screen;

        double time_limit;
        int node_limit = MAX_NODES;

        clock_t start;

        int num_of_agents;



        std::vector<CBS_Li::MAPFPath *> paths;
        std::vector<SingleAgentSolver *> search_engines;  // used to find (single) agents' paths and mdd

        bool generateChild(int child_id, PBSNode *parent, int low, int high);

        bool hasConflicts(int a1, int a2) const;

        bool hasConflicts(int a1, const std::set<int> &agents) const;

        std::shared_ptr<Conflict> chooseConflict(const PBSNode &node) const;

        int getSumOfCosts() const;

        inline void releaseNodes();

        // print and save
        void printResults() const;

        static void printConflicts(const PBSNode &curr);

        void printPriorityGraph() const;

        bool validateSolution() const;

        inline int getAgentLocation(int agent_id, size_t timestep) const;

        std::vector<int> shuffleAgents() const;  //generate random permuattion of agent indices
        bool terminate(PBSNode *curr); // check the stop condition and return true if it meets

        void getHigherPriorityAgents(const std::list<int>::reverse_iterator &p1, std::set<int> &agents);

        void getLowerPriorityAgents(const std::list<int>::iterator &p1, std::set<int> &agents);

        bool hasHigherPriority(int low, int high) const; // return true if agent low is lower than agent high

        // node operators
        void pushNode(PBSNode *node);

        void pushNodes(PBSNode *n1, PBSNode *n2);

        PBSNode *selectNode();

        // high level search
        bool generateRoot();

        bool findPathForSingleAgent(PBSNode &node, const std::set<int> &higher_agents, int a, CBS_Li::MAPFPath &new_path);

        void classifyConflicts(PBSNode &parent);

        void update(PBSNode *node);

        void printPaths() const;

        void topologicalSort(std::list<int> &stack);

        void topologicalSortUtil(int v, std::vector<bool> &visited, std::list<int> &stack);
    };

}