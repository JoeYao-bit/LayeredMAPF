//
// Created by yaozhuo on 2024/5/8.
//

#ifndef LAYEREDMAPF_SINGLE_AGENT_PATH_SEARCH_H
#define LAYEREDMAPF_SINGLE_AGENT_PATH_SEARCH_H

#include "../freeNav-base/basic_elements/point.h"
#include "constraint.h"
#include "../large_agent_mapf.h"
#include <boost/heap/pairing_heap.hpp>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>

namespace freeNav::LayeredMAPF::LA_MAPF::CBS {

    // yz: low level is for isolated single agent path planning

    class LowLvNode
    {
    public:

        LowLvNode *parent;

        size_t node_id;
        int g_val;
        int h_val = 0;
        int h_val_second = 0;
        int timestep = 0;
        int num_of_conflicts = 0;
        bool in_openlist = false;
        bool in_focallist = false; // yz: add by me
        bool wait_at_goal; // the action is to wait at the goal vertex or not. This is used for >length constraints
        bool is_goal = false;

        // the following is used to compare nodes in the OPEN list // yz: for hasher ?
        struct compare_node {
            // returns true if n1 > n2 (note -- this gives us *min*-heap).
            bool operator()(const LowLvNode *n1, const LowLvNode *n2) const {
                if (n1->g_val + n1->h_val == n2->g_val + n2->h_val) {
                    if (n1->h_val == n2->h_val) {
                        if(n1->h_val_second == n2->h_val_second) {
                            return rand() % 2 == 0;   // break ties randomly
                        }
                        return n1->h_val_second >= n2->h_val_second;
                    }
                    return n1->h_val >= n2->h_val;  // break ties towards smaller h_vals (closer to goal location)
                }
                return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
            }
        };  // used by OPEN (heap) to compare nodes (top of the heap has min f-val, and then highest g-val)

        // the following is used to compare nodes in the FOCAL list
        struct secondary_compare_node {
            bool operator()(const LowLvNode *n1, const LowLvNode *n2) const // returns true if n1 > n2
            {
                if (n1->num_of_conflicts == n2->num_of_conflicts) {
                    if (n1->g_val + n1->h_val == n2->g_val + n2->h_val) {
                        if (n1->h_val == n2->h_val) {
                            if(n1->h_val_second == n2->h_val_second) {
                                return rand() % 2 == 0;   // break ties randomly
                            }
                            return n1->h_val_second >= n2->h_val_second;
                        }
                        return n1->h_val >= n2->h_val;  // break ties towards smaller h_vals (closer to goal location)
                    }
                    return n1->g_val + n1->h_val >=
                           n2->g_val + n2->h_val;  // break ties towards smaller f_vals (prefer shorter solutions)
                }
                return n1->num_of_conflicts >= n2->num_of_conflicts;  // n1 > n2 if it has more conflicts
            }
        };  // used by FOCAL (heap) to compare nodes (top of the heap has min number-of-conflicts)


        LowLvNode() : node_id(0), g_val(0), h_val(0), parent(nullptr), timestep(0), num_of_conflicts(0),
                   in_openlist(false), wait_at_goal(false) {}

        LowLvNode(size_t location, int g_val, int h_val, int h_val_second, LowLvNode *parent, int timestep, int num_of_conflicts = 0,
               bool in_openlist = false) :
                node_id(location), g_val(g_val), h_val(h_val), h_val_second(h_val_second), parent(parent), timestep(timestep),
                num_of_conflicts(num_of_conflicts), in_openlist(in_openlist), wait_at_goal(false) {}

        inline int getFVal() const { return g_val + h_val; }

        void copy(const LowLvNode &other) {
            node_id = other.node_id;
            g_val = other.g_val;
            h_val = other.h_val;
            parent = other.parent;
            timestep = other.timestep;
            num_of_conflicts = other.num_of_conflicts;
            wait_at_goal = other.wait_at_goal;
            is_goal = other.is_goal;
        }
    };

// yz: find optimal path under constraints
    template <Dimension N>
    class SingleAgentSolver {
    public:

        SingleAgentSolver(const size_t& start_node_id,
                          const size_t& target_node_id,
                          const std::vector<int>& heuristic,
                          const std::vector<int>& heuristic_ignore_rotate,
                          const SubGraphOfAgent<N>& sub_graph,
                          const ConstraintTable<N>& constraint_table = nullptr,
                          const ConstraintAvoidanceTablePtr<N>& constraint_avoidance_table = nullptr,
                          const LargeAgentStaticConstraintTablePtr<N> & path_constraint = nullptr
                          ) : start_node_id_(start_node_id), target_node_id_(target_node_id),
                          heuristic_(heuristic),
                          heuristic_ignore_rotate_(heuristic_ignore_rotate),
                          sub_graph_(sub_graph),
                          constraint_table_(constraint_table),
                          constraint_avoidance_table_(constraint_avoidance_table),
                          path_constraint_(path_constraint) {
        }

        virtual LAMAPF_Path solve() = 0;

        virtual ~SingleAgentSolver() = default;

        const size_t& start_node_id_;
        const size_t& target_node_id_;
        const std::vector<int>& heuristic_;  // this is the precomputed heuristic for this agent
        const std::vector<int>& heuristic_ignore_rotate_;  // this is the precomputed heuristic for this agent
        const SubGraphOfAgent<N>& sub_graph_;

    //protected:

        int min_f_val_; // minimal f value in OPEN
        int lower_bound_ = 0; // Threshold for FOCAL
        double w_ = 1.2; // suboptimal bound

        LAMAPF_Path solution_;

        const ConstraintTable<N>& constraint_table_; // vertex and edge constraint, hard constraint

        const ConstraintAvoidanceTablePtr<N>& constraint_avoidance_table_; // try to avoid, take as soft constraint

        const LargeAgentStaticConstraintTablePtr<N>& path_constraint_; // take external path as obstacles, hard constraints

    };

}

#endif //LAYEREDMAPF_SINGLE_AGENT_PATH_SEARCH_H
