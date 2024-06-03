//
// Created by yaozhuo on 2024/5/5.
//

#ifndef LAYEREDMAPF_LA_CBS_H
#define LAYEREDMAPF_LA_CBS_H

#include "large_agent_mapf.h"
#include "space_time_astar.h"
#include "high_level_node.h"

namespace freeNav::LayeredMAPF::LA_MAPF {

    template <Dimension N, typename AgentType>
    class LargeAgentCBS : public LargeAgentMAPF<N, AgentType> {
    public:
        LargeAgentCBS(const InstanceOrients<N> & instances,
                      const std::vector<AgentType>& agents,
                      DimensionLength* dim,
                      const IS_OCCUPIED_FUNC<N> & isoc) : LargeAgentMAPF<N, AgentType>(instances, agents, dim, isoc) {
            // 1, initial paths
            for(int agent=0; agent<this->instance_node_ids_.size(); agent++) {
                ConstraintTable<N, AgentType> constraint_table(agent, this->agents_, this->all_poses_, this->dim_, this->isoc_);
                const size_t& start_node_id = this->instance_node_ids_[agent].first,
                              target_node_id = this->instance_node_ids_[agent].second;
                SpaceTimeAstar<N, AgentType> astar(start_node_id, target_node_id,
                                                   this->agents_heuristic_tables_[agent],
                                                   this->agent_sub_graphs_[agent],
                                                   constraint_table);
                LAMAPF_Path solution = astar.solve();
                if(solution.empty()) {
                    std::cerr << " agent " << agent << " search path failed " << std::endl;
                } else {
                    std::cout << instances[agent].first << "->" << instances[agent].second << std::endl;
                    for(int t=0; t<solution.size(); t++) {
                        std::cout << *(this->all_poses_[solution[t]]) << "->";
                    }
                    std::cout << std::endl;
                    solutions_.push_back(solution);
                }
            }
            // 2, detect conflict
            for(int i=0; i<this->instances_.size()-1; i++) {
                for(int j=i+1; j<this->instances_.size(); j++) {
                    const auto& conflicts = detectFirstConflictBetweenPaths<N>(solutions_[i], solutions_[j], this->agents_[i], this->agents_[j], this->all_poses_);
                    //break;
                }
            }
            //const auto& conflicts = detectAllConflictBetweenPaths<N>(solutions_[0], solutions_[0], this->agents_[0], this->agents_[0], this->all_poses_);
        }



        virtual bool solve(double time_limit, int cost_lowerbound = 0, int cost_upperbound = MAX_COST) override {
            this->cost_lowerbound = cost_lowerbound;
            this->cost_upperbound = cost_upperbound;
            this->time_limit = time_limit;
            // set timer
            start = clock();
            // yz: generate a init node of CT
            generateRoot();
            while (!cleanup_list.empty() && !solution_found) {
                // yz: select node with minimum heuristic value
                auto curr = selectNode();
                // yz: check whether reach terminate condition
                if (terminate(curr))
                    return solution_found;
                if (!curr->h_computed)  // heuristics has not been computed yet
                {
                    runtime = (double) (clock() - start) / CLOCKS_PER_SEC;
                    bool succ = true;
                    runtime = (double) (clock() - start) / CLOCKS_PER_SEC;
                    curr->h_val = 0;
                    if (!succ) // no solution, so prune this node
                    {
                        curr->clear();
                        continue;
                    }
                    if (reinsertNode(curr)) {
                        continue;
                    }
                }
                //Expand the node
                bool foundBypass = true;
                while (foundBypass) {
                    // yz: reach terminate condition
                    if (terminate(curr))
                        return solution_found;
                    foundBypass = false;
                    CBSNode *child[2] = {new CBSNode(), new CBSNode()};
                    // yz: pick conflict with the highest priority
                    curr->conflict = chooseConflict(*curr);
                    // yz: child 1,2 inherit constraint from curr
                    addConstraints(curr, child[0], child[1]);
                    bool solved[2] = {false, false};
                    std::vector<LAMAPF_Path> copy(solutions_);
                    // yz: split to two branches
                    for (int i = 0; i < 2; i++) {
                        if (i > 0)
                            solutions_ = copy;
                        // yz: check whether child is legal, if legal, add to parent node
                        solved[i] = generateChild(child[i], curr);
                        if (!solved[i]) {
                            delete (child[i]);
                            continue;
                        }
                    }
                    if (foundBypass) {
                        for (auto &i : child) {
                            delete i;
                            i = nullptr;
                        }
                    } else {
                        for (int i = 0; i < 2; i++) {
                            if (solved[i]) {
                                pushNode(child[i]);
                                curr->children.push_back(child[i]);
                            }
                        }
                        curr->clear();
                    }
                }
            }  // end of while loop
            return solution_found;
        }

        std::vector<LAMAPF_Path> solutions_;

    private:

        // implement
        double time_limit;
        int cost_lowerbound = 0;
        int cost_upperbound = MAX_COST;

        double suboptimality = 1.0;

        bool solution_found = false;
        int solution_cost = -2;
        clock_t start;

        HighLvNode *dummy_start = nullptr;
        HighLvNode *goal_node = nullptr;

        std::list<HighLvNode *> allNodes_table; // this is ued for both ECBS and EES

        // stats
        double runtime = 0;

        boost::heap::pairing_heap<CBSNode *, boost::heap::compare<CBSNode::compare_node_by_f> > cleanup_list; // it is called open list in ECBS
        boost::heap::pairing_heap<CBSNode *, boost::heap::compare<CBSNode::compare_node_by_inadmissible_f> > open_list; // this is used for EES
        boost::heap::pairing_heap<CBSNode *, boost::heap::compare<CBSNode::compare_node_by_d> > focal_list; // this is ued for both ECBS and EES

        // yz: child node inherit constraint from parent node
        void addConstraints(const HighLvNode *curr, HighLvNode *child1, HighLvNode *child2) const {
            {
                // yz: inherit constraint from parent node
                child1->constraints = curr->conflict->cs1;
                child2->constraints = curr->conflict->cs2;
            }
        }

        void generateRoot() {
            auto root = new CBSNode();
            root->g_val = 0;
            for (int i = 0; i < this->instances_.size(); i++) {
                root->makespan = std::max(root->makespan, solutions_[i].size() - 1); // yz: makespan
                root->g_val += (int) solutions_[i].size() - 1; // yz: sum of all current path cost
            }
            root->h_val = 0;
            root->depth = 0;
            findConflicts(*root);
        }

        // yz: check whether current result paths is legal
        bool validateSolution() const {
            // check whether the solution cost is within the bound
            if (solution_cost > cost_lowerbound * suboptimality) {
                std::cout << "Solution cost exceeds the sub-optimality bound!" << std::endl;
                return false;
            }

            // check whether the paths are feasible
            size_t soc = 0;
            for (int a1 = 0; a1 < this->instances_.size(); a1++) {
                soc += solutions_[a1].size() - 1; // yz: soc: sum of cost
                for (int a2 = a1 + 1; a2 < this->instances_.size(); a2++) {
                    size_t min_path_length = solutions_[a1].size() < solutions_[a2].size() ? solutions_[a1].size() : solutions_[a2].size();
                    // yz: check conflict in common time range
                    for (size_t timestep = 0; timestep < min_path_length; timestep++) {
                        int loc1 = solutions_[a1][timestep];
                        int loc2 = solutions_[a2][timestep];
                        // yz: vertex conflict error
                        if (loc1 == loc2) {
                            std::cout << "Agents " << a1 << " and " << a2 << " collides at " << loc1 << " at timestep "
                                 << timestep << std::endl;
                            return false;
                        } else if (timestep < min_path_length - 1
                                   && loc1 == solutions_[a2][timestep + 1]
                                   && loc2 == solutions_[a1][timestep + 1]) {
                            std::cout << "Agents " << a1 << " and " << a2 << " collides at (" <<
                                 loc1 << "-->" << loc2 << ") at timestep " << timestep << std::endl;
                            return false;
                        }
                    }
                    // yz: check conflict when one of them is stop
                    if (solutions_[a1].size() != solutions_[a2].size()) {
                        int a1_ = solutions_[a1].size() < solutions_[a2].size() ? a1 : a2;
                        int a2_ = solutions_[a1].size() < solutions_[a2].size() ? a2 : a1;
                        int loc1 = solutions_[a1_].back();
                        for (size_t timestep = min_path_length; timestep < solutions_[a2_].size(); timestep++) {
                            int loc2 = solutions_[a2_][timestep];
                            if (loc1 == loc2) {
                                std::cout << "Agents " << a1 << " and " << a2 << " collides at " << loc1 << " at timestep "
                                     << timestep << std::endl;
                                return false; // It's at least a semi conflict
                            }
                        }
                    }
                }
            }
            if ((int) soc != solution_cost) {
                std::cout << "The solution cost is wrong!" << std::endl;
                return false;
            }
            return true;
        }

        // yz: check whether current hyper node reach terminate condition
        bool terminate(HighLvNode *curr) {
            // yz: if lower bound >= upper bound, terminate with no solution
            if (cost_lowerbound >= cost_upperbound) {
                solution_cost = cost_lowerbound;
                solution_found = false;
                return true;
            }
            runtime = (double) (clock() - start) / CLOCKS_PER_SEC;
            // yz: if current node have no conflict, find a solution, terminate
            if (curr->conflicts.empty() && curr->unknownConf.empty()) //no conflicts
            {// found a solution
                solution_found = true;
                goal_node = curr;
                solution_cost = goal_node->getFHatVal() - goal_node->cost_to_go;
                if (!validateSolution()) {
                    std::cout << "Solution invalid!!!" << std::endl;
                    exit(-1);
                }
                return true;
            }
            // yz: if exceed time limit or number of high level node exceed limit
            if (runtime > time_limit) {   // time/node out
                solution_cost = -1;
                solution_found = false;
                return true;
            }
            return false;
        }

        // yz: add CBSNode to focal list
        inline void pushNode(CBSNode *node) {
            // update handles
            node->cleanup_handle = this->cleanup_list.push(node);
            this->allNodes_table.push_back(node);
        }

        // yz: add CBSNode to focal list
        inline bool reinsertNode(CBSNode *node) {
            node->cleanup_handle = cleanup_list.push(node);
            return true;
        }

        // yz: get heuristic node from open list with minimum heuristic value
//     considering high level solver type
        CBSNode* selectNode() {
            CBSNode *curr = nullptr;
            cost_lowerbound = std::max(cost_lowerbound, cleanup_list.top()->getFVal());
            curr = cleanup_list.top();
            cleanup_list.pop();
            return curr;
        }

        // yz: filter current conflict that involve with agents in excluded_agents
        void copyConflicts(const std::list<std::shared_ptr<Conflict >> &conflicts,
                                std::list<std::shared_ptr<Conflict>> &copy, const std::list<int> &excluded_agents) {
            for (auto &conflict : conflicts) {
                bool found = false;
                for (auto a : excluded_agents) {
                    if (conflict->a1 == a || conflict->a2 == a) {
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    copy.push_back(conflict);
                }
            }
        }

        // yz: add conflicts between a1's path and a2's path to curr
        void findConflicts(HighLvNode &curr, int a1, int a2) {
            const auto& conflicts = detectFirstConflictBetweenPaths<N>(
                    solutions_[a1], solutions_[a2], this->agents_[a1], this->agents_[a2], this->all_poses_);
            for(const auto & conflict : conflicts) {
                curr.unknownConf.push_front(conflict); // It's at least a semi conflict
            }
        }

        void findConflicts(HighLvNode &curr) {
            if (curr.parent != nullptr) {
                // Copy from parent
                auto new_agents = curr.getReplannedAgents();
                // yz: when agent is replanned, their conflicts are ignored
                copyConflicts(curr.parent->conflicts, curr.conflicts, new_agents);
                copyConflicts(curr.parent->unknownConf, curr.unknownConf, new_agents);

                // detect new conflicts between any pair of agent
                // yz: detect conflict between agents that replanned paths and other agent
                for (auto it = new_agents.begin(); it != new_agents.end(); ++it) {
                    int a1 = *it;
                    for (int a2 = 0; a2 < this->instances_.size(); a2++) {
                        if (a1 == a2)
                            continue;
                        bool skip = false;
                        // yz: if a2 before a1 in new_agents, skip, avoid repeat
                        for (auto it2 = new_agents.begin(); it2 != it; ++it2) {
                            if (*it2 == a2) {
                                skip = true;
                                break;
                            }
                        }
                        if (!skip)
                            findConflicts(curr, a1, a2);
                    }
                }
            } else {
                // yz: add constraints between all pair of agents to current hyper node curr
                for (int a1 = 0; a1 < this->instances_.size(); a1++) {
                    for (int a2 = a1 + 1; a2 < this->instances_.size(); a2++) {
                        findConflicts(curr, a1, a2);
                    }
                }
            }
        }

// yz: choose conflict in HLNode with highest priority constraint, keep HLNode unchanged
        std::shared_ptr<Conflict> chooseConflict(const HighLvNode &node) const {
            std::shared_ptr<Conflict> choose;
            if (node.conflicts.empty() && node.unknownConf.empty())
                // yz: if HLNode have no any constraint
                return nullptr;
            else if (!node.conflicts.empty()) {
                choose = node.conflicts.back();
                for (const auto &conflict : node.conflicts) {
                    if (*choose < *conflict)
                        choose = conflict;
                }
            } else {
                choose = node.unknownConf.back();
                for (const auto &conflict : node.unknownConf) {
                    if (*choose < *conflict)
                        choose = conflict;
                }
            }
            return choose;
        }

        // yz: get agents id that violate the first constraint (new added constraint) in list
        std::set<int> getInvalidAgents(const Constraints &constraints)  // return agents that violate the constraints
        {
            std::set<int> agents;
            int agent; size_t from, to; int start_t, end_t;
            // <agent id, node from, node to, time range start, time range end>
            assert(!constraints.empty());
            // yz: there should be only one constraint in constraints, one of the two constraint of the hyper node conflict
            //if(constraints.size() != 1) { std::cout << "constraints.size() != 1" << std::endl; } // AC
            // yz: only check the front constraint ?
            std::tie(agent, from, to, start_t, end_t) = *(constraints.front());
            agents.insert(agent);
            return agents;
        }

        // yz: update path of a agent, under node's constraint, without conflict with other path
        // return whether exist a path satisfied all this constraint
        // if there is, update to solution paths
        bool findPathForSingleAgent(CBSNode *node, int agent, int lowerbound) {
            ConstraintTable<N, AgentType> constraint_table(agent, this->agents_, this->all_poses_, this->dim_, this->isoc_);
            auto buffer_node = node;
            while(true) {
                if(buffer_node == nullptr) { break; }
                constraint_table.insertCT(buffer_node->constraints, agent);
                buffer_node = buffer_node->parent;
            }
            const size_t& start_node_id = this->instance_node_ids_[agent].first,
                    target_node_id = this->instance_node_ids_[agent].second;

            SpaceTimeAstar<N, AgentType> astar(start_node_id, target_node_id,
                                               this->agents_heuristic_tables_[agent],
                                               this->agent_sub_graphs_[agent],
                                               constraint_table);
            astar.lower_bound_ = lowerbound;
            LAMAPF_Path solution = astar.solve();
            return false;
        }

        // yz: check whether child node is legal or illegal. if legal, add
        bool generateChild(CBSNode *node, CBSNode *parent) {
            clock_t t1 = clock();
            node->parent = parent;
            node->HighLvNode::parent = parent;
            node->g_val = parent->g_val;
            node->makespan = parent->makespan;
            node->depth = parent->depth + 1;
            // yz: get agents that violates the first constraint
            auto agents = getInvalidAgents(node->constraints);
            assert(!agents.empty());
            for (auto agent : agents) {
                int lowerbound = (int) solutions_[agent].size() - 1;
                // yz: if find no path meet current constraint, the child node is illegal
                if (!findPathForSingleAgent(node, agent, lowerbound)) {
                    return false;
                }
            }

            findConflicts(*node);
            return true;
        }

    };


}

// when both move, do edge-2-edge check
// when only one move, do edge-2-vertex check

#endif //LAYEREDMAPF_LA_CBS_H
