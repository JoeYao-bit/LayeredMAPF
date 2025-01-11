//
// Created by yaozhuo on 2025/1/10.
//

#ifndef LAYEREDMAPF_CBSNODE_H
#define LAYEREDMAPF_CBSNODE_H

#include "Conflict.h"

namespace CBS_Li {
    // constraint tree node
    class HLNode // a virtual base class for high-level node
    {
    public:
        list<Constraint> constraints; // new constraints // yz: constraint add in current node ?

        // yz: predicted time step (path length) to target
        int g_val = 0; // sum of costs for CBS, and sum of min f for ECBS
        // yz: have finished path time step (path length)
        int h_val = 0; // admissible h
        int cost_to_go = 0; // informed but inadmissible h
        int distance_to_go = 0; // distance to the goal state
        // yz: time index of waypoint, start from zero
        size_t depth = 0; // depath of this CT node
        size_t makespan = 0; // makespan over all paths
        bool h_computed = false;

        uint64_t time_expanded = 0;
        uint64_t time_generated = 0;

        // For debug
        string chosen_from = "none"; // chosen from the open/focal/cleanup least
        int f_of_best_in_cleanup = 0;
        int f_hat_of_best_in_cleanup = 0;
        int d_of_best_in_cleanup = 0;
        int f_of_best_in_open = 0;
        int f_hat_of_best_in_open = 0;
        int d_of_best_in_open = 0;
        int f_of_best_in_focal = 0;
        int f_hat_of_best_in_focal = 0;
        int d_of_best_in_focal = 0;

        // conflicts in the current paths
        list<shared_ptr<Conflict> > conflicts;
        list<shared_ptr<Conflict> > unknownConf; // yz: unclassified conflict

        // The chosen conflict
        shared_ptr<Conflict> conflict;
        // unordered_map<int, pair<int, int> > conflictGraph; //<edge index, <weight, num of CT nodes> >

        // online learning
        int distance_error = 0;
        int cost_error = 0;
        bool fully_expanded = false;

        // yz: tree format
        HLNode *parent;
        list<HLNode *> children;

        inline int getFVal() const { return g_val + h_val; }

        virtual inline int getFHatVal() const = 0;

        virtual inline int getNumNewPaths() const = 0;

        virtual list<int> getReplannedAgents() const = 0;

        virtual inline string getName() const = 0;

        void clear();

        // void printConflictGraph(int num_of_agents) const;
        void updateDistanceToGo();

        void printConstraints(int id) const;

        virtual ~HLNode() {}

        list<pair<int, MAPFPath> > paths; // new paths // yz: replanned new path

    };

    std::ostream &operator<<(std::ostream &os, const HLNode &node);
}

#endif //LAYEREDMAPF_CBSNODE_H
