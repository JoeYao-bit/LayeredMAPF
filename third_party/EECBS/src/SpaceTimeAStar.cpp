#include "EECBS/inc/SpaceTimeAStar.h"
namespace CBS_Li {

// yz: update node connection from LLNode goal
    void SpaceTimeAStar::updatePath(const LLNode *goal, vector<PathEntry> &path) {
        const LLNode *curr = goal;
        if (curr->is_goal)
            curr = curr->parent;
        path.reserve(curr->g_val + 1);
        while (curr != nullptr) {
            path.emplace_back(curr->location);
            curr = curr->parent;
        }
        std::reverse(path.begin(), path.end());
    }


    MAPFPath SpaceTimeAStar::findOptimalPath(const HLNode &node, const ConstraintTable &initial_constraints,
                                             const vector<MAPFPath *> &paths, int agent, int lowerbound) {
        return findSuboptimalPath(node, initial_constraints, paths, agent, lowerbound, 1).first;
    }

    AStarNode *getWaitNode(AStarNode *node, int wait_time) {
        assert(wait_time > 0);
        AStarNode *buffer_node = node;
        while (wait_time > 0) {
            auto next_node = new AStarNode(buffer_node->location, buffer_node->g_val + 1, buffer_node->h_val,
                                           buffer_node, buffer_node->timestep + 1, 0);
            buffer_node = next_node;
            wait_time--;
        }
        return buffer_node;
    }

    bool waited = false;

// find path by time-space A* search
// Returns a bounded-suboptimal path that satisfies the constraints of the give node  while
// minimizing the number of internal conflicts (that is conflicts with known_paths for other agents found so far).
// lowerbound is an underestimation of the length of the path in order to speed up the search.
    pair<MAPFPath, int>
    SpaceTimeAStar::findSuboptimalPath(const HLNode &node, const ConstraintTable &initial_constraints,
                                       const vector<MAPFPath *> &paths, int agent, int lowerbound, double w) {
        this->w = w;
        MAPFPath path;
        num_expanded = 0;
        num_generated = 0;

        // build constraint table
        auto t = clock();
        // yz: avoid the copy of initial constraint table, which is very time consuming ...
        ConstraintTable constraint_table(initial_constraints.num_col,
                                         initial_constraints.map_size);//(initial_constraints); // yz: a constraint table for a agent
        constraint_table.insert2CT(node, agent); // yz: new path must satisfied current hyper level constraint node
        runtime_build_CT = (double) (clock() - t) / CLOCKS_PER_SEC;
        // yz: if start is occupied, return empty path
        if (constraint_table.constrained(start_location, 0) or initial_constraints.constrained(start_location, 0)) {
            return {path, 0};
        }

        t = clock();
        constraint_table.insert2CAT(agent, paths);
        runtime_build_CAT = (double) (clock() - t) / CLOCKS_PER_SEC;

        // the earliest timestep that the agent can hold its goal location. The length_min is considered here.
        auto holding_time = constraint_table.getHoldingTime(goal_location, constraint_table.length_min);
        holding_time = std::max(holding_time,
                                initial_constraints.getHoldingTime(goal_location, constraint_table.length_min));

        auto static_timestep = constraint_table.getMaxTimestep() + 1; // everything is static after this timestep

        //std::cout << " static_timestep " << static_timestep << std::endl;
        //std::cout << " constraint_table.length_max " << constraint_table.length_max << " / initial_constraints.length_max " << initial_constraints.length_max << std::endl;

        static_timestep = std::max(static_timestep, initial_constraints.getMaxTimestep() + 1);
        lowerbound = max(holding_time, lowerbound); // yz: considering minimum time stamp to target

        // generate start and add it to the OPEN & FOCAL list
        auto start = new AStarNode(start_location, 0, max(lowerbound, my_heuristic[start_location]), nullptr, 0, 0);

//    if( !waited && start_location == 8503 && goal_location == 2658 ) {
//        start = getWaitNode(start, static_timestep);
//        waited = true;
//    }

        num_generated++;
        start->open_handle = open_list.push(start);
        start->focal_handle = focal_list.push(start);
        start->in_openlist = true;
        allNodes_table.insert(start); // yz: visited vertex
        min_f_val = (int) start->getFVal();
        // lower_bound = int(w * min_f_val));

        while (!open_list.empty()) {
            updateFocalList(); // update FOCAL if min f-val increased
            auto *curr = popNode();
            assert(curr->location >= 0);
            // check if the popped node is a goal
            if (curr->location == goal_location && // arrive at the goal location
                !curr->wait_at_goal && // not wait at the goal location
                curr->timestep >= holding_time) // the agent can hold the goal location afterward
            {
                // yz: if find path, update node connection in LLNode
                updatePath(curr, path);
                break;
            }
            if (curr->timestep >= constraint_table.length_max)
                continue;
            if (curr->timestep >= initial_constraints.length_max)
                continue;
            // yz: used in independence detection, set upbound of path length,
            // maximum_length_of_paths_ is empty in other place,
            if(!initial_constraints.maximum_length_of_paths_.empty()) {
                assert(agent + 1 <= initial_constraints.maximum_length_of_paths_.size());
                int path_length = 0; // calculated path length
                LLNode* buffer = curr;
                while(buffer != nullptr) {
                    path_length ++;
                    buffer = buffer->parent;
                }
                if(path_length > initial_constraints.maximum_length_of_paths_[agent]) {
//                    std::cout << " path length " << path_length << " over max length " << initial_constraints.maximum_length_of_paths_[agent] << std::endl;
                    continue;
                }
            }
            auto next_locations = instance.getNeighbors(curr->location);
            next_locations.emplace_back(curr->location); // considering wait
            for (int next_location : next_locations) {
                int next_timestep = curr->timestep + 1;
                if (static_timestep <
                    next_timestep) { // now everything is static, so switch to space A* where we always use the same timestep
                    // yz: no need to wait after no constraint is applied
                    if (next_location == curr->location) {
                        continue;
                    }
                    next_timestep--;
                }
//            if(start_location == 8503 && goal_location == 2658 ) {
//                std::cout << " next pt " << instance.getColCoordinate(next_location) << ", " << instance.getRowCoordinate(next_location) << std::endl;
//                std::cout << " pass flag: ";
//            }

                // yz: check whether satisfied all constraint, including vertex constraint and edge constraint
                if (constraint_table.constrained(next_location, next_timestep) ||
                    constraint_table.constrained(curr->location, next_location, next_timestep))
                    continue;
//            if(start_location == 8503 && goal_location == 2658 ) { std::cout << " 1 "; }
                if (initial_constraints.constrained(next_location, next_timestep) ||
                    initial_constraints.constrained(curr->location, next_location, next_timestep))
                    continue;
                // yz: avoid occupation of target caused conflict with previous path
                if(next_location == goal_location) {
                    if(next_timestep < initial_constraints.getHoldingTime(next_location, 0)) {
                        continue;
                    }
                }

//            if(start_location == 8503 && goal_location == 2658 ) { std::cout << " 2 "; }
                // compute cost to next_id via curr node
                int next_g_val = curr->g_val + 1;
                int next_h_val = max(lowerbound - next_g_val, my_heuristic[next_location]);
                if (next_g_val + next_h_val > constraint_table.length_max)
                    continue;
//            if(start_location == 8503 && goal_location == 2658 ) { std::cout << " 3 "; }

                if (next_g_val + next_h_val > initial_constraints.length_max)
                    continue;
//            if(start_location == 8503 && goal_location == 2658 ) { std::cout << " 4 "; }
                int next_internal_conflicts = curr->num_of_conflicts +
                                              constraint_table.getNumOfConflictsForStep(curr->location, next_location,
                                                                                        next_timestep);

                next_internal_conflicts += initial_constraints.getNumOfConflictsForStep(curr->location, next_location,
                                                                                        next_timestep);

                // generate (maybe temporary) node
                auto next = new AStarNode(next_location, next_g_val, next_h_val,
                                          curr, next_timestep, next_internal_conflicts);
//            if(start_location == 8503 && goal_location == 2658 ) {
//                std::cout << " next g / h " << next->g_val << " / " << next->h_val << std::endl;
//            }
                if (next_location == goal_location && curr->location == goal_location)
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
                        bool add_to_focal = false;  // check if it was above the focal bound before and now below (thus need to be inserted)
                        bool update_in_focal = false;  // check if it was inside the focal and needs to be updated (because f-val changed)
                        bool update_open = false;
                        if ((next_g_val + next_h_val) <= w * min_f_val) {  // if the new f-val qualify to be in FOCAL
                            if (existing_next->getFVal() > w * min_f_val)
                                add_to_focal = true;  // and the previous f-val did not qualify to be in FOCAL then add
                            else
                                update_in_focal = true;  // and the previous f-val did qualify to be in FOCAL then update
                        }
                        if (existing_next->getFVal() > next_g_val + next_h_val)
                            update_open = true;

                        existing_next->copy(*next);    // update existing node

                        if (update_open)
                            open_list.increase(existing_next->open_handle);  // increase because f-val improved
                        if (add_to_focal)
                            existing_next->focal_handle = focal_list.push(existing_next);
                        if (update_in_focal)
                            focal_list.update(
                                    existing_next->focal_handle);  // should we do update? yes, because number of conflicts may go up or down
                    }
                }

                delete (next);  // not needed anymore -- we already generated it before
            }  // end for loop that generates successors
        }  // end while loop
//    if(open_list.empty()) { std::cout << " SpaceTimeAstar failed " << std::endl; }
//    else {
//        if( start_location == 8503 && goal_location == 2658 ) {
//            std::cout << " agent 428 success after wait till evert thing is static " << std::endl;
//        }
//    }
        releaseNodes();
        return {path, min_f_val};
    }

// yz: get time makespan of a path from start to end under constraints
    int SpaceTimeAStar::getTravelTime(int start, int end, const ConstraintTable &constraint_table, int upper_bound) {
        int length = MAX_TIMESTEP;
        auto static_timestep = constraint_table.getMaxTimestep() + 1; // everything is static after this timestep
        auto root = new AStarNode(start, 0, compute_heuristic(start, end), nullptr, 0, 0);
        root->open_handle = open_list.push(root);  // add root to heap
        allNodes_table.insert(root);       // add root to hash_table (nodes)
        AStarNode *curr = nullptr;
        while (!open_list.empty()) {
            curr = open_list.top();
            open_list.pop();
            // yz: if reach target, exit
            if (curr->location == end) {
                length = curr->g_val;
                break;
            }
            list<int> next_locations = instance.getNeighbors(curr->location);
            next_locations.emplace_back(curr->location);
            for (int next_location : next_locations) {
                int next_timestep = curr->timestep + 1;
                int next_g_val = curr->g_val + 1;
                if (static_timestep < next_timestep) {
                    if (curr->location == next_location) {
                        continue;
                    }
                    next_timestep--;
                }
                // yz: if satisfied both vertex constraint and edge constraint
                if (!constraint_table.constrained(next_location, next_timestep) &&
                    !constraint_table.constrained(curr->location, next_location,
                                                  next_timestep)) {  // if that grid is not blocked
                    int next_h_val = compute_heuristic(next_location, end);
                    if (next_g_val + next_h_val >= upper_bound) // the cost of the path is larger than the upper bound
                        continue;
                    auto next = new AStarNode(next_location, next_g_val, next_h_val, nullptr, next_timestep, 0);
                    auto it = allNodes_table.find(next);
                    if (it == allNodes_table.end()) {  // add the newly generated node to heap and hash table
                        next->open_handle = open_list.push(next);
                        allNodes_table.insert(next);
                    } else {  // update existing node's g_val if needed (only in the heap)
                        delete (next);  // not needed anymore -- we already generated it before
                        auto existing_next = *it;
                        if (existing_next->g_val > next_g_val) {
                            existing_next->g_val = next_g_val;
                            existing_next->timestep = next_timestep;
                            open_list.increase(existing_next->open_handle);
                        }
                    }
                }
            }
        }
        releaseNodes();
        return length;
    }

// yz: pop from FOCAL set
    inline AStarNode *SpaceTimeAStar::popNode() {
        auto node = focal_list.top();
        focal_list.pop();
        open_list.erase(node->open_handle);
        node->in_openlist = false;
        num_expanded++;
        return node;
    }

// yz: insert new node to open list anf FOCAL list if in range
    inline void SpaceTimeAStar::pushNode(AStarNode *node) {
        node->open_handle = open_list.push(node);
        node->in_openlist = true;
        num_generated++;
        if (node->getFVal() <= w * min_f_val)
            node->focal_handle = focal_list.push(node);
    }


    void SpaceTimeAStar::updateFocalList() {
        auto open_head = open_list.top();
        if (open_head->getFVal() > min_f_val) {
            int new_min_f_val = (int) open_head->getFVal();
            for (auto n : open_list) {
                if (n->getFVal() > w * min_f_val && n->getFVal() <= w * new_min_f_val)
                    n->focal_handle = focal_list.push(n);
            }
            min_f_val = new_min_f_val;
        }
    }

// yz: release all pointer
    void SpaceTimeAStar::releaseNodes() {
        open_list.clear();
        focal_list.clear();
        for (auto node: allNodes_table)
            delete node;
        allNodes_table.clear();
    }

}