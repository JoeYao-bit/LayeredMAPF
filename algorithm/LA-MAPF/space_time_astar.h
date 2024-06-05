//
// Created by yaozhuo on 2024/5/10.
//

#ifndef LAYEREDMAPF_SPACE_TIME_ASTAR_H
#define LAYEREDMAPF_SPACE_TIME_ASTAR_H

#include "single_agent_path_search.h"

namespace freeNav::LayeredMAPF::LA_MAPF {

    class AStarNode : public LowLvNode {
    public:
        // define a typedefs for handles to the heaps (allow up to quickly update a node in the heap)
        typedef boost::heap::pairing_heap<AStarNode *, boost::heap::compare<LowLvNode::compare_node> >::handle_type open_handle_t;
        typedef boost::heap::pairing_heap<AStarNode *, boost::heap::compare<LowLvNode::secondary_compare_node> >::handle_type focal_handle_t;
        open_handle_t open_handle;
        focal_handle_t focal_handle;


        AStarNode() : LowLvNode() {}

        AStarNode(int loc, int g_val, int h_val, LowLvNode *parent, int timestep, int num_of_conflicts = 0,
                  bool in_openlist = false) :
                LowLvNode(loc, g_val, h_val, parent, timestep, num_of_conflicts, in_openlist) {}


        ~AStarNode() {}

        // The following is used by for generating the hash value of a nodes
        struct NodeHasher {
            size_t operator()(const AStarNode *n) const {
                size_t loc_hash = std::hash<int>()(n->node_id);
                size_t timestep_hash = std::hash<int>()(n->timestep);
                return (loc_hash ^ (timestep_hash << 1)); // yz: 按位异或
            }
        };

        // The following is used for checking whether two nodes are equal
        // we say that two nodes, s1 and s2, are equal if
        // both are non-NULL and agree on the id and timestep
        struct eqnode {
            bool operator()(const AStarNode *s1, const AStarNode *s2) const {
                // yz: the same pointer or the same value
                return (s1 == s2) || (s1 && s2 &&
                                      s1->node_id == s2->node_id &&
                                      s1->timestep == s2->timestep &&
                                      s1->wait_at_goal == s2->wait_at_goal);
            }
        };
    };


    template <Dimension N, typename AgentType>
    class SpaceTimeAstar : public SingleAgentSolver<N, AgentType> {
    public:
        SpaceTimeAstar(const size_t& start_pose_id,
                       const size_t& target_pose_id,
                       const std::vector<int>& heuristic,
                       const SubGraphOfAgent<N>& sub_graph,
                       const ConstraintTable<N, AgentType>& constraint_table
        ) : SingleAgentSolver<N, AgentType>(start_pose_id, target_pose_id, heuristic, sub_graph, constraint_table) {
            //
        }

        virtual LAMAPF_Path solve() override {

            LAMAPF_Path path;

            // generate start and add it to the OPEN & FOCAL list
            auto start = new AStarNode(this->start_node_id_, 0, std::max(this->lower_bound_, this->heuristic_[this->start_node_id_]), nullptr, 0, 0);

            start->open_handle = open_list.push(start);
//            start->focal_handle = focal_list.push(start);
            start->in_openlist = true;
            allNodes_table.insert(start); // yz: visited vertex
            // lower_bound = int(w * min_f_val));

            auto holding_time = this->constraint_table_.getHoldingTime(this->target_node_id_, this->constraint_table_.length_min_);
            this->lower_bound_ = std::max(holding_time, this->lower_bound_); // yz: considering minimum time stamp to target

            auto static_timestep = this->constraint_table_.getMaxTimestep() + 1; // everything is static after this timestep

            while (!open_list.empty()) {
                //std::cout << "open, focal size = " << open_list.size() << ", " << focal_list.size() << std::endl;
                //updateFocalList(); // update FOCAL if min f-val increased
                auto *curr = popNode();
                assert(curr->node_id >= 0);
                // check if the popped node is a goal
                if (curr->node_id == this->target_node_id_ && // arrive at the goal location
                    !curr->wait_at_goal && // not wait at the goal location
                    curr->timestep >= holding_time) // the agent can hold the goal location afterward
                {
                    // yz: if find path, update node connection in LLNode
                    updatePath(curr, path);
                    break;
                }

                auto next_locations = this->sub_graph_.all_edges_[curr->node_id];//instance.getNeighbors(curr->location);

                next_locations.emplace_back(curr->node_id); // considering wait
                for (const size_t& next_node_id : next_locations) {
                    int next_timestep = curr->timestep + 1;
                    if (static_timestep <
                        next_timestep) { // now everything is static, so switch to space A* where we always use the same timestep
                        // yz: no need to wait after no constraint is applied
                        if (next_node_id == curr->node_id) {
                            continue;
                        }
                        next_timestep--;
                    }
                    // yz: check whether satisfied all constraint, including vertex constraint and edge constraint
                    if (this->constraint_table_.constrained(next_node_id, next_timestep) ||
                            this->constraint_table_.constrained(curr->node_id, next_node_id, next_timestep))
                        continue;

                    // compute cost to next_id via curr node
                    int next_g_val = curr->g_val + 1;
                    int next_h_val = std::max(this->lower_bound_ - next_g_val, this->heuristic_[next_node_id]);
                    if (next_g_val + next_h_val > this->constraint_table_.length_max_)
                        continue;
                    int next_internal_conflicts = curr->num_of_conflicts +
                            this->constraint_table_.getNumOfConflictsForStep(this->sub_graph_.all_nodes_[curr->node_id],
                                                                             this->sub_graph_.all_nodes_[next_node_id],
                                                                             next_timestep);

                    // generate (maybe temporary) node
                    auto next = new AStarNode(next_node_id, next_g_val, next_h_val,
                                              curr, next_timestep, next_internal_conflicts);

                    if (next_node_id == this->target_node_id_ && curr->node_id == this->target_node_id_)
                        next->wait_at_goal = true;

                    // try to retrieve it from the hash table
                    auto it = allNodes_table.find(next);
                    if (it == allNodes_table.end()) {
                        pushNode(next);
                        allNodes_table.insert(next); // yz: add to hash table
                        continue;
                    }
                    // update existing node's if needed (only in the open_list)

                    auto existing_next = *it;
                    if (existing_next->getFVal() > next->getFVal() || // if f-val decreased through this new path
                        (existing_next->getFVal() == next->getFVal() &&
                         existing_next->num_of_conflicts >
                         next->num_of_conflicts)) // or it remains the same but there's fewer conflicts
                    {
                        if (!existing_next->in_openlist) // if it is in the closed list (reopen)
                        {
                            existing_next->copy(*next);
                            pushNode(existing_next);
                        } else {
//                            bool add_to_focal = false;  // check if it was above the focal bound before and now below (thus need to be inserted)
//                            bool update_in_focal = false;  // check if it was inside the focal and needs to be updated (because f-val changed)
                            bool update_open = false;
//                            if ((next_g_val + next_h_val) <= this->w_ * this->min_f_val_) {  // if the new f-val qualify to be in FOCAL
//                                if (existing_next->getFVal() > this->w_ * this->min_f_val_)
//                                    add_to_focal = true;  // and the previous f-val did not qualify to be in FOCAL then add
//                                else
//                                    update_in_focal = true;  // and the previous f-val did qualify to be in FOCAL then update
//                            }
                            if (existing_next->getFVal() > next_g_val + next_h_val)
                                update_open = true;

                            existing_next->copy(*next);    // update existing node

                            if (update_open)
                                open_list.increase(existing_next->open_handle);  // increase because f-val improved
//                            if (add_to_focal)
//                                existing_next->focal_handle = focal_list.push(existing_next);
//                            if (update_in_focal) {
//                                // should we do update? yes, because number of conflicts may go up or down
//                                focal_list.update(existing_next->focal_handle);
//                            }
                        }
                    }

                    delete (next);  // not needed anymore -- we already generated it before
                }  // end for loop that generates successors
            }  // end while loop
            return path;
        }

    private:

        void updateFocalList() {
//            auto open_head = open_list.top();
//            if (open_head->getFVal() > this->min_f_val_) {
//                int new_min_f_val = (int) open_head->getFVal();
//                for (auto n : open_list) {
//                    if (n->getFVal() > this->w_ * this->min_f_val_ && n->getFVal() <= this->w_ * new_min_f_val)
//                        n->focal_handle = focal_list.push(n);
//                }
//                this->min_f_val_ = new_min_f_val;
//            }
        }

        inline AStarNode *popNode() {
//            std::cout << "open, focal size = " << open_list.size() << std::endl;
//            std::cout << "open, focal size = " << open_list.size() << ", " << focal_list.size() << std::endl;
//
//            assert(!focal_list.empty() && !open_list.empty());
//
//            auto node = focal_list.top();
//            focal_list.pop();
            auto node = open_list.top();
            open_list.erase(node->open_handle);
            node->in_openlist = false;
            return node;
        }

        inline void pushNode(AStarNode *node) {
            node->open_handle = open_list.push(node);
            node->in_openlist = true;
//            if (node->getFVal() <= this->w_ * this->min_f_val_)
//                node->focal_handle = focal_list.push(node);
        }

        void updatePath(const LowLvNode *goal, LAMAPF_Path &path) {
            const LowLvNode *curr = goal;
            if (curr->is_goal)
                curr = curr->parent;
            path.reserve(curr->g_val + 1);
            while (curr != nullptr) {
                path.emplace_back(curr->node_id);
                curr = curr->parent;
            }
            std::reverse(path.begin(), path.end());
        }

        // define typedefs and handles for heap
        typedef boost::heap::pairing_heap<AStarNode *, boost::heap::compare<AStarNode::compare_node> > heap_open_t;
        typedef boost::heap::pairing_heap<AStarNode *, boost::heap::compare<AStarNode::secondary_compare_node> > heap_focal_t;
        heap_open_t open_list;
        //heap_focal_t focal_list;

        // define typedef for hash_map
        typedef boost::unordered_set<AStarNode *, AStarNode::NodeHasher, AStarNode::eqnode> hashtable_t;
        hashtable_t allNodes_table;


    };

}

#endif //LAYEREDMAPF_SPACE_TIME_ASTAR_H
