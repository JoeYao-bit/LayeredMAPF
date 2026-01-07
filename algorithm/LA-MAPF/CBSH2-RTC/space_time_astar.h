//
// Created by yaozhuo on 2024/5/10.
//

#ifndef LAYEREDMAPF_SPACE_TIME_ASTAR_CBSH2_RTC_H
#define LAYEREDMAPF_SPACE_TIME_ASTAR_CBSH2_RTC_H

#include "single_agent_path_search.h"

namespace freeNav::LayeredMAPF::LA_MAPF::CBSH2_RTC {

    class AStarNode : public LowLvNode {
    public:
        // define a typedefs for handles to the heaps (allow up to quickly update a node in the heap)
        typedef boost::heap::pairing_heap<AStarNode *, boost::heap::compare<LowLvNode::compare_node> >::handle_type open_handle_t;
        typedef boost::heap::pairing_heap<AStarNode *, boost::heap::compare<LowLvNode::secondary_compare_node> >::handle_type focal_handle_t;
        open_handle_t open_handle;
        focal_handle_t focal_handle;


        AStarNode() : LowLvNode() {}

        AStarNode(int loc, int g_val, int h_val, int h_val_second, LowLvNode *parent, int timestep, int num_of_conflicts = 0,
                  bool in_openlist = false) :
                LowLvNode(loc, g_val, h_val, h_val_second, parent, timestep, num_of_conflicts, in_openlist) {}


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


    // debug: record all visited node during expansion and visualize,
    //       to find out why in some cases 10^7 nodes are created
    template <Dimension N, typename State>
    class SpaceTimeAstar : public SingleAgentSolver<N, State> {
    public:
        SpaceTimeAstar(const size_t& start_pose_id,
                       const size_t& target_pose_id,
                       const std::vector<int>& heuristic,
                       const std::vector<int>& heuristic_ignore_rotate,
                       const SubGraphOfAgent<N, State>& sub_graph,

                       const ConstraintTable<N, State>& constraint_table,
                       const ConstraintAvoidanceTablePtr<N, State>& constraint_avoidance_table,
                       const LargeAgentStaticConstraintTablePtr<N, State>& path_constraint,
                       ConnectivityGraph* connect_graph = nullptr
        ) : SingleAgentSolver<N,  State>(start_pose_id, target_pose_id, heuristic, heuristic_ignore_rotate, sub_graph,
                                         constraint_table, constraint_avoidance_table, path_constraint),
            connect_graph_(connect_graph) {
            //
//            std::cout << "space time search agent " <<  this->sub_graph_.agent_ << std::endl;
            grid_visit_count_table_.resize(sub_graph.data_ptr_->all_nodes_.size() / (2*N));
        }

        virtual std::string getName() const override { return "AStar"; }

        virtual LAMAPF_Path solve() override {

//            std::cout << "space time search agent in solve " <<  this->sub_graph_.agent_
//                      << ", from " << *this->sub_graph_.all_nodes_[this->start_node_id_]
//                      << ", to " << *this->sub_graph_.all_nodes_[this->target_node_id_] << std::endl;

            int static_time_step = this->constraint_table_.getMaxTimestep();

            if(this->path_constraint_ != nullptr) {
                static_time_step = std::max(static_time_step, this->path_constraint_->static_time_);
            }

//            std::cout << "static_time_step = " << static_time_step << std::endl;

            new_nodes_in_open.clear();
            LAMAPF_Path path;

            // generate start and add it to the OPEN & FOCAL list
            auto start = new AStarNode(this->start_node_id_, 0, std::max(this->lower_bound_,
                                                                         this->heuristic_[this->start_node_id_]),
                                       (this->heuristic_ignore_rotate_.empty() ? 0 : this->heuristic_ignore_rotate_[this->start_node_id_/(2*N)]),
                                       nullptr, 0, 0);

            start->open_handle = open_list.push(start);
            start->in_openlist = true;
            new_nodes_in_open.push_back(start);
            allNodes_table.insert(start); // yz: visited vertex
            // lower_bound = int(w * min_f_val));

//            auto holding_time = this->constraint_table_.getHoldingTime(this->target_node_id_, this->constraint_table_.length_min_);
//
//            auto static_timestep = this->constraint_table_.getMaxTimestep() + 1; // everything is static after this timestep
//            if(this->path_constraint_ != nullptr) {
//                static_timestep = std::max(static_timestep, this->path_constraint_->getMaxTimeStamp());
//                holding_time = std::max(holding_time,
//                                        this->path_constraint_->getEarliestTime(agents_[this->sub_graph_.agent_id_]));
//            }

//            this->lower_bound_ = std::max(holding_time, this->lower_bound_); // yz: considering minimum time stamp to target
            auto start_t = clock();

            int count = 0;
            while (!open_list.empty()) {
//                if(this->sub_graph_.agent_.id_ == 9) {
//                    std::cout << count << " th step, open / focal size = " << open_list.size() << " / " << focal_list.size() << std::endl;
//                }
                // check time cost to now every 50000 times
                if(count % 30000 == 0 && this->time_limit_ > 0) {
                    auto now_t = clock();
                    auto sum_s = (double) (clock() - start_t) / CLOCKS_PER_SEC;
//                    std::cout << "sum_s = " << sum_s << ", this->time_limit_ = " << this->time_limit_ << std::endl;
                    if(sum_s*1e3 > this->time_limit_) { break; }
                }
                //assert(count <= 1000);
                count++;
                updateFocalList(); // update FOCAL if min f-val increased
                new_nodes_in_open.clear();
                auto *curr = popNode();
                // yz: used in independence detection, set upbound of path length,
                // maximum_length_of_paths_ is empty in other place,
                if(this->path_constraint_ != nullptr) {
                    if (!this->path_constraint_->maximum_length_of_paths_.empty()) {
                        int agent = this->sub_graph_.agent_->id_; // using global id
                        assert(agent + 1 <= this->path_constraint_->maximum_length_of_paths_.size()); // store all path length
                        int path_length = 0; // calculated path length
                        LowLvNode* buffer = curr;
                        while (buffer != nullptr) {
                            path_length++;
                            buffer = buffer->parent;
                        }
                        if (path_length > this->path_constraint_->maximum_length_of_paths_[agent]) {
//                    std::cout << " path length " << path_length << " over max length " << initial_constraints.maximum_length_of_paths_[agent] << std::endl;
                            continue;
                        }
                    }
                }
//                if(this->sub_graph_.agent_.id_ == 4) {
//                    std::cout << " SpaceTimeAstar pop " << *(this->sub_graph_.all_nodes_[curr->node_id])
//                          << ", id = " << curr->node_id << ", t = " << curr->timestep << ", h = " << curr->h_val;
//                    if(connect_graph_ != nullptr
//                    //&& connect_graph_->hyper_node_id_map_[curr->node_id] != 212
//                    //   && connect_graph_->hyper_node_id_map_[curr->node_id] != 220
//                    ) {
//                        std::cout << ", hyper id = " << connect_graph_->hyper_node_id_map_[curr->node_id];
//                    }
//                    std::cout << std::endl;
//                }
                grid_visit_count_table_[curr->node_id/(2*N)] ++;
//                assert(curr->node_id >= 0);
//                std::cout << "before check reach goal this->path_constraint_ = " << this->path_constraint_ << std::endl;

                // check if the popped node is a goal
                if (curr->node_id == this->target_node_id_ // && // arrive at the goal location
                    //!curr->wait_at_goal && // not wait at the goal location
                    //curr->timestep >= holding_time
                    ) // the agent can hold the goal location afterward
                {
                    // yz: if find path, update node connection in LLNode
                    updatePath(curr, path);
                    // yz: used in independence detection, set upbound of path length,
                    // maximum_length_of_paths_ is empty in other place,

                    if(this->path_constraint_ != nullptr) {
                        if (!this->path_constraint_->maximum_length_of_paths_.empty()) {
//                            std::cout << " use path length limitation " << std::endl;
                            int agent = this->sub_graph_.agent_->id_; // using global id
//                            std::cout << " agent = " << agent << " / total size = " <<
//                            this->path_constraint_->maximum_length_of_paths_.size() <<
//                            " / empty ? " << this->path_constraint_->maximum_length_of_paths_.empty() << std::endl;
//                            std::cout << "start/end = " << (this->path_constraint_->maximum_length_of_paths_.begin() ==
//                            this->path_constraint_->maximum_length_of_paths_.end()) << std::endl;
                            assert(agent + 1 <= this->path_constraint_->maximum_length_of_paths_.size());
                            assert(path.size() <= this->path_constraint_->maximum_length_of_paths_[agent]);
                        }
                    }
                    break;
                }

                int next_timestep = // curr->timestep + 1
                        curr->timestep >= static_time_step ? curr->timestep : curr->timestep + 1;

                auto next_locations = this->sub_graph_.data_ptr_->all_edges_[curr->node_id];//instance.getNeighbors(curr->location);
                if(next_timestep < static_time_step) {
                    next_locations.emplace_back(curr->node_id); // considering wait before every obstacle is static
                }
                //std::reverse(next_locations.begin(), next_locations.end());
                std::random_shuffle(next_locations.begin(), next_locations.end()); // shuffle to make agent move in all direction equally

                for (const size_t& next_node_id : next_locations) {
                    // debug
                    //if(this->sub_graph_.agent_.id_ == 4)
//                    {
//                        std::cout << " next_node_id = " << *this->sub_graph_.all_nodes_[next_node_id]
//                                  << "{" << next_node_id << "}" ;
//
//                        if(connect_graph_ != nullptr
//                            //&& connect_graph_->hyper_node_id_map_[curr->node_id] != 212
//                            //   && connect_graph_->hyper_node_id_map_[curr->node_id] != 220
//                                ) {
//                            std::cout << ", hyper id = " << connect_graph_->hyper_node_id_map_[next_node_id];
//                        }
//
//                        std::cout << std::endl;
//                    }
                    //debug
                    //if(curr->node_id == 1261 || curr->node_id == 1262)
//                    {
//                        std::cout << " flag1 next_node_id = " << next_node_id << ", h = " << this->heuristic_[next_node_id] << std::endl;
//                    }
//                    if (static_timestep <
//                        next_timestep) { // now everything is static, so switch to space A* where we always use the same timestep
//                        // yz: no need to wait after no constraint is applied
//                        if (next_node_id == curr->node_id) {
//                            continue;
//                        }
//                        next_timestep--;
//                    }
//                    std::cout << "next_node " << *this->sub_graph_.all_nodes_[next_node_id] << std::endl;
                    maximum_t_ = std::max(maximum_t_, next_timestep);
//                    if(this->sub_graph_.agent_.id_ == 9 && next_node_id == this->target_node_id_) {
//                        std::cout << " reach target flag 1 " << std::endl;
//                    }
                    // yz: check whether satisfied all constraint, including vertex constraint and edge constraint
                    if (this->constraint_table_.constrained(next_node_id, next_timestep) ||
                            this->constraint_table_.constrained(curr->node_id, next_node_id, next_timestep))
                    { continue; }
                    //if(curr->node_id == 1261 || curr->node_id == 1262)
//                    {
//                        std::cout << " flag2 next_node_id = " << next_node_id << ", h = " << this->heuristic_[next_node_id] << std::endl;
//                    }
//                    if(this->sub_graph_.agent_.id_ == 9 && next_node_id == this->target_node_id_) {
//                        std::cout << " reach target flag 2 " << std::endl;
//                    }

//                    std::cout << " flag 1 " << std::endl;
//                    if(this->path_constraint_ == nullptr) {
//                        std::cout << "this->path_constraint_ == nullptr" << std::endl;
//                    }

//                    std::cout << " this->sub_graph_.agent_id_ = " << this->sub_graph_.agent_id_
//                              << ", this->path_constraint_ = " << this->path_constraint_ << std::endl;
                    // avoid conflict     with external paths
                    //std::cout << " reach target = " << (next_node_id == this->target_node_id_) << std::endl;
                    if(this->path_constraint_ != nullptr &&
                        this->path_constraint_->hasCollide(this->sub_graph_.agent_->id_, curr->timestep,
                                                           curr->node_id, next_node_id, next_node_id == this->target_node_id_)) {
                        continue;
                    }
                    //if(curr->node_id == 1261 || curr->node_id == 1262)
//                    {
//                        std::cout << " flag3 next_node_id = " << next_node_id << ", h = " << this->heuristic_[next_node_id] << std::endl;
//                    }
//                    if(this->sub_graph_.agent_.id_ == 9 && next_node_id == this->target_node_id_) {
//                        std::cout << " reach target flag 3 " << std::endl;
//                    }
//                    std::cout << " flag 2 " << std::endl;

                    // compute cost to next_id via curr node
                    int next_g_val = curr->g_val + 1;

                    int next_h_val = this->heuristic_[next_node_id];
                    if(next_h_val == MAX<int>) {
                        continue;
                    }
                    //if(curr->node_id == 1261 || curr->node_id == 1262)
//                    {
//                        std::cout << " flag4 next_node_id = " << next_node_id << ", h = " << this->heuristic_[next_node_id] << std::endl;
//                    }
//                    if(next_node_id == 4737) {
//                        std::cout << " reach target flag 4 " << std::endl;
//                    }

//                    std::cout << " flag 3 " << std::endl;
//                    assert(next_h_val != MAX<int>);

                    // getNumOfConflictsForStep is very time consuming for large agents
                    // resulting no getNumOfConflictsForStep might be faster than use it
                    int next_internal_conflicts = curr->num_of_conflicts +
                            (this->constraint_avoidance_table_ == nullptr ? 0 :
                             this->constraint_avoidance_table_->getNumOfConflictsForStep(*this->sub_graph_.data_ptr_->all_nodes_[curr->node_id],
                                                                                         *this->sub_graph_.data_ptr_->all_nodes_[next_node_id],
                                                                                         curr->timestep));

                    // generate (maybe temporary) node
                    auto next = new AStarNode(next_node_id, next_g_val, next_h_val,
                                              (this->heuristic_ignore_rotate_.empty() ? 0 : this->heuristic_ignore_rotate_[this->start_node_id_/(2*N)]),
                                              curr, next_timestep, next_internal_conflicts);

                    // try to retrieve it from the hash table
                    auto it = allNodes_table.find(next);
                    if (it == allNodes_table.end()) {
                        pushNode(next);
                        new_nodes_in_open.push_back(next);
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
                            new_nodes_in_open.push_back(existing_next);
                        } else {
                            bool update_open = false;
                            if (existing_next->getFVal() > next_g_val + next_h_val)
                                update_open = true;

                            if(!existing_next->in_openlist) {
                                std::cout << "copy !existing_next->in_openlist " << next << std::endl;
                            }

                            existing_next->copy(*next);    // update existing node

                            if (update_open) {
                                open_list.increase(existing_next->open_handle);  // increase because f-val improved
                                new_nodes_in_open.push_back(existing_next);
                            }
                        }
                    }

                    delete (next);  // not needed anymore -- we already generated it before
                }  // end for loop that generates successors
            }  // end while loop
            releaseNodes();
            return path;
        }

        Path findPath() override {
            LAMAPF_Path solution = solve();
            if(solution.empty()) { return {}; }
            return LAMAPF_Path2Path(solution);
        }

        // for debug only, record how many times each grid are visited during low lever search
        std::vector<int> grid_visit_count_table_;
        ConnectivityGraph* connect_graph_ = nullptr;

    private:

        void updateFocalList() {
            auto open_head = open_list.top();
            this->min_f_val_ = open_head->getFVal();
//            focal_list.clear();
//            for(auto iter = open_list.begin(); iter != open_list.end(); iter++) {
//                if(iter.get_node()->value->getFVal() <= this->min_f_val_* this->w_) {
//                    focal_list.push(iter.get_node()->value);
//                    iter.get_node()->value->in_focallist = false;
//                }
//            }
            for(const auto& new_node : new_nodes_in_open) {
                if(new_node->getFVal() <= this->min_f_val_* this->w_) {
                    if(new_node->in_focallist) {
                        focal_list.update(new_node->focal_handle);
                    } else {
                        new_node->focal_handle = focal_list.push(new_node);
                    }
                    new_node->in_focallist = true;
                }
            }
            if(focal_list.empty()) {
                for(auto& new_node : open_list) {
                    if(new_node->getFVal() <= this->min_f_val_* this->w_) {
                        new_node->in_focallist = true;
                        new_node->focal_handle = focal_list.push(new_node);
                    }
                }
            }
        }

        inline AStarNode *popNode() {
            if(open_list.empty() || focal_list.empty()) {
                std::cout << "FATAL: open | focal size = " << open_list.size() << " | " << focal_list.size() << std::endl;
            }
            assert(!open_list.empty() && !focal_list.empty());
            //auto node = open_list.top();
            auto node = focal_list.top();
            node->in_focallist = false;
            focal_list.pop();
            open_list.erase(node->open_handle);
            node->in_openlist = false;
            return node;
        }

        inline void pushNode(AStarNode *node) {
            node->open_handle = open_list.push(node);
            node->in_openlist = true;
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

        void releaseNodes() {
//            std::cout << " space time astar release " << allNodes_table.size() << " nodes " << std::endl;
//            std::cout << " maximum t = " << maximum_t_ << std::endl;
            open_list.clear();
            focal_list.clear();
            for (auto node: allNodes_table)
                delete node;
            allNodes_table.clear();
        }

        // define typedefs and handles for heap
        typedef boost::heap::pairing_heap<AStarNode *, boost::heap::compare<AStarNode::compare_node> > heap_open_t;
        typedef boost::heap::pairing_heap<AStarNode *, boost::heap::compare<AStarNode::secondary_compare_node> > heap_focal_t;
        heap_open_t open_list;
        heap_focal_t focal_list;

        // define typedef for hash_map
        typedef boost::unordered_set<AStarNode *, AStarNode::NodeHasher, AStarNode::eqnode> hashtable_t;
        hashtable_t allNodes_table;

        std::vector<AStarNode*> new_nodes_in_open;

        int maximum_t_ = 0;

    };

}

#endif //LAYEREDMAPF_SPACE_TIME_ASTAR_H
