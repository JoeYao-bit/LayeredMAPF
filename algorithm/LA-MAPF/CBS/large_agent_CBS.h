//
// Created by yaozhuo on 2024/5/5.
//

#ifndef LAYEREDMAPF_LA_CBS_H
#define LAYEREDMAPF_LA_CBS_H

#include "../large_agent_mapf.h"
#include "space_time_astar.h"
#include "high_level_node.h"
#include "constraint.h"
#include "../circle_shaped_agent.h"
#include "../block_shaped_agent.h"

namespace freeNav::LayeredMAPF::LA_MAPF::CBS {

    template <Dimension N, typename AgentType>
    class LargeAgentCBS : public LargeAgentMAPF<N, AgentType> {
    public:
        LargeAgentCBS(const InstanceOrients<N> & instances,
                      const std::vector<AgentType>& agents,
                      DimensionLength* dim,
                      const IS_OCCUPIED_FUNC<N> & isoc,
                      const LargeAgentStaticConstraintTablePtr<N, AgentType>& path_constraint = nullptr,

                      const std::vector<PosePtr<int, N> > all_poses = {},
                      const DistanceMapUpdaterPtr<N> distance_map_updater = nullptr,
                      const std::vector<SubGraphOfAgent<N> > agent_sub_graphs = {},
                      const std::vector<std::vector<int> >& agents_heuristic_tables = {},
                      const std::vector<std::vector<int> >& agents_heuristic_tables_ignore_rotate_ = {}
                      )
                      : LargeAgentMAPF<N, AgentType>(instances, agents, dim, isoc,
                                                     all_poses,
                                                     distance_map_updater,
                                                     agent_sub_graphs,
                                                     agents_heuristic_tables,
                                                     agents_heuristic_tables_ignore_rotate_),
                        constraint_avoidance_table_(nullptr), //ConstraintAvoidanceTable<N, AgentType>(dim, this->all_poses_, agents.front())),
                        path_constraint_(path_constraint){
            // 1, initial paths
            for(int agent=0; agent<this->instance_node_ids_.size(); agent++) {
                ConstraintTable<N, AgentType> constraint_table(agent, this->agents_, this->all_poses_, this->dim_, this->isoc_);

//                for(int another_agent=0; another_agent<this->instance_node_ids_.size(); another_agent++) {
//                    if(agent == another_agent) { continue; }
//                    constraint_table.insert2CT(this->instance_node_ids_[agent].first, 0, MAX_TIMESTEP);
//                }

                const size_t& start_node_id = this->instance_node_ids_[agent].first,
                              target_node_id = this->instance_node_ids_[agent].second;

                SpaceTimeAstar<N, AgentType> astar(start_node_id, target_node_id,
                                                   this->agents_heuristic_tables_[agent],
                                                   this->agents_heuristic_tables_ignore_rotate_[agent],
                                                   this->agent_sub_graphs_[agent],
                                                   constraint_table,
                                                   constraint_avoidance_table_,
                                                   path_constraint_,
                                                   this->agents_);
                LAMAPF_Path solution = astar.solve();
                //grid_visit_count_tables_.push_back(astar.grid_visit_count_table_);
                if(solution.empty()) {
                    std::cout << " agent " << agent << " search path failed " << std::endl;
                    this->solvable = false;
                } else {
//                    this->printPath(agent, solution);
                    this->initial_solutions_.push_back(solution);
//                    init_agent_occ_grids.push_back(ConstraintAvoidanceTable<N, AgentType>::getAgentPathOccGrids(this->agents_[agent],
//                                                                                                                this->initial_solutions_[agent],
//                                                                                                                this->all_poses_,
//                                                                                                                this->dim_));
                    this->solutions_.push_back(solution);
                }
            }
            // 2, detect conflict
//            for(int i=0; i<this->instances_.size()-1; i++) {
//                for(int j=i+1; j<this->instances_.size(); j++) {
//                    const auto& conflicts = detectFirstConflictBetweenPaths<N>(solutions_[i], solutions_[j], this->agents_[i], this->agents_[j], this->all_poses_);
//                    //break;
//                }
//            }
            //const auto& conflicts = detectAllConflictBetweenPaths<N>(solutions_[0], solutions_[1], this->agents_[0], this->agents_[1], this->all_poses_);
        }

        ~LargeAgentCBS() {
            releaseNodes();
        }

        virtual bool solve(double time_limit, int cost_lowerbound = 0, int cost_upperbound = MAX_COST) override {
            if(!this->solvable) {
                std::cout << "-- unsolvable instance " << std::endl;
                return false;
            }
            this->cost_lowerbound = cost_lowerbound;
            this->cost_upperbound = cost_upperbound;
            this->time_limit = time_limit;
            // yz: generate a init node of CT
            generateRoot();
//            std::cout << "-- generate root node " << std::endl;
            int count = 0;
            while (!cleanup_list.empty() && !solution_found) {
//                if(count >= 2000) { break; }
//                std::cout << "-- " << count << " iteration, open size " << cleanup_list.size() << std::endl;
                count ++;
                // yz: select node with minimum heuristic value
                auto curr = selectNode();
//                std::cout << " select node with conflicts size = " << curr->conflicts.size() << " + " <<  curr->unknownConf.size() << std::endl;
//                std::cout << " curr->g_val = " << curr->g_val << std::endl;
                // yz: check whether reach terminate condition
                if (terminate(curr)) {
//                    std::cout << "-- Layered CBS finish after " << count << " iteration" << std::endl;
                    return solution_found;
                }
                if (!curr->h_computed)  // heuristics has not been computed yet
                {
                    curr->h_val = 0;
                    if (reinsertNode(curr)) {
                        continue;
                    }
                }

//                std::cout << "-- generate children node " << std::endl;
                CBSNode *child[2] = {new CBSNode(), new CBSNode()};
                // yz: pick conflict with the highest priority
                curr->conflict = chooseConflict(*curr);
//                std::cout << " select conflict " << std::endl;
//                printConflict(*curr->conflict);
                // yz: child 1,2 inherit constraint from curr
                addConstraints(curr, child[0], child[1]);
//                bool has_target_conflict = false;
//                if(std::get<3>(*child[0]->constraints.front()) == 0 || std::get<3>(*child[1]->constraints.front()) == 0) {
//                    std::cout << " has_target_conflict " << std::endl;
//                    has_target_conflict = true;
//                }
                bool solved[2] = {false, false};
                std::vector<LAMAPF_Path> copy(this->solutions_);
                std::vector<CBSNode*> children;
                // yz: split to two branches
                for (int i = 0; i < 2; i++) {
                    if (i > 0) {
                        this->solutions_ = copy;
                    }
//                    if(has_target_conflict && std::get<3>(*child[i]->constraints.front()) != 0) {
//                        delete (child[i]);
//                        continue;
//                    }
                    // yz: check whether child is legal, if legal, add to parent node
                    solved[i] = generateChild(child[i], curr);
                    if (!solved[i]) {
                        delete (child[i]);
                        continue;
                    } else {
                        children.push_back(child[i]);
                    }
                }
                for(const auto& temp_child : children) {
                    pushNode(temp_child);
                    curr->children.push_back(temp_child);
                }
//                for (int i = 0; i < 2; i++) {
//                    if (solved[i]) {
//                        pushNode(child[i]);
//                        curr->children.push_back(child[i]);
//                    }
//                }
                curr->clear();
                //break;
            }  // end of while loop
            return solution_found;
        }

        // for debug only, record how many times each grid are visited during low lever search
        std::vector<std::vector<int> > grid_visit_count_tables_;

    private:

        // implement
        double time_limit;
        int cost_lowerbound = 0;
        int cost_upperbound = MAX_COST;

        double suboptimality = 1.0;

        bool solution_found = false;
        int solution_cost = -2;

        HighLvNode *dummy_start = nullptr;
        HighLvNode *goal_node = nullptr;

        std::list<HighLvNode *> allNodes_table; // this is ued for both ECBS and EES

        // stats
        double runtime = 0;

        boost::heap::pairing_heap<CBSNode *, boost::heap::compare<CBSNode::compare_node_by_f> > cleanup_list; // it is called open list in ECBS
        boost::heap::pairing_heap<CBSNode *, boost::heap::compare<CBSNode::compare_node_by_inadmissible_f> > open_list; // this is used for EES
        boost::heap::pairing_heap<CBSNode *, boost::heap::compare<CBSNode::compare_node_by_d> > focal_list; // this is ued for both ECBS and EES

        ConstraintAvoidanceTablePtr<N, AgentType> constraint_avoidance_table_;

        const LargeAgentStaticConstraintTablePtr<N, AgentType>& path_constraint_; // take external path as obstacles

        // store each agent's occupied grid at each time , update with this->solutions_
//        std::vector< typename ConstraintAvoidanceTable<N, AgentType>::OccGridLevels > agent_occ_grids;

        std::vector< typename ConstraintAvoidanceTable<N, AgentType>::OccGridLevels > init_agent_occ_grids;


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
            this->solutions_.resize(this->instances_.size(), {});
//            agent_occ_grids.clear();
//            agent_occ_grids.resize(this->instances_.size(), {});
            for (int agent = 0; agent < this->instances_.size(); agent++) {
                this->solutions_[agent] = this->initial_solutions_[agent];
//                agent_occ_grids[agent]  = init_agent_occ_grids[agent];
                root->makespan = std::max(root->makespan, this->solutions_[agent].size() - 1); // yz: makespan
                root->g_val += (int) this->solutions_[agent].size() - 1; // yz: sum of all current path cost
            }
            root->h_val = 0;
            root->depth = 0;
            root->h_computed = true;
            findConflicts(*root);
            pushNode(root);
//            std::cout << "root node = " << root->toString(this->all_poses_) << "\n";
        }

        // yz: check whether current result paths is legal
        bool validateSolution() const {
            // check whether the solution cost is within the bound
            if (solution_cost > cost_lowerbound * suboptimality) {
                std::cout << "Solution cost exceeds the sub-optimality bound!" << std::endl;
                return false;
            }
            size_t soc = this->getSOC();
            if ((int) soc != solution_cost) {
                std::cout << "-- The solution cost is wrong!" << std::endl;
                std::cout <<"-- soc = " << soc << " / solution_cost = " << solution_cost << std::endl;
                return false;
            }
            size_t makespan = this->getMakeSpan();
//            std::cout << "-- find solution with SOC/makespan = " << soc << " / " << makespan << std::endl;
            for(int agent=0; agent<this->instances_.size(); agent++) {
//                this->printPath(agent, this->solutions_[agent]);
            }
            return true;
        }

        // yz: check whether current hyper node reach terminate condition
        bool terminate(HighLvNode *curr) {
            //std::cout << " terminate, have " << curr->unknownConf.size() << " conflict " << std::endl;
            // yz: if lower bound >= upper bound, terminate with no solution
            if (cost_lowerbound >= cost_upperbound) {
                std::cout << " cost_lowerbound >= cost_upperbound " << std::endl;
                solution_cost = cost_lowerbound;
                solution_found = false;
                return true;
            }
            // yz: if current node have no conflict, find a solution, terminate
            if (curr->conflicts.empty() && curr->unknownConf.empty()) //no conflicts
            {// found a solution
                solution_found = true;
                goal_node = curr;
                solution_cost = goal_node->getFHatVal() - goal_node->cost_to_go;
                auto conflicts = findConflicts(*curr);
//                std::cout << "-- finish with node depth = " << curr->depth << std::endl;
                if(!conflicts.empty()) {
                    std::cout << "Solution have conflict !!!" << std::endl;
                    exit(-1);
                }
                if (!validateSolution()) {
                    std::cout << "Solution have wrong cost !!!" << std::endl;
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

        // takes the paths_found_initially and UPDATE all (constrained) paths found for agents from curr to start
        // yz: set HLNode's newest path as solution, and update replanned path in hyper node
        // yz: update paths in CBS to paths under current hyper node constraint
        inline void updatePaths(HighLvNode *curr) {
            for (int agent = 0; agent < this->instances_.size(); agent++) {
                this->solutions_[agent] = this->initial_solutions_[agent]; // yz: considering what if an agent that never have conflict with other agent
            }
            std::vector<bool> updated(this->instances_.size(), false);  // initialized for false
            while (curr != nullptr) {
                for (auto &path : curr->paths) {
                    if (!updated[path.first]) {
                        this->solutions_[path.first] = path.second;
                        updated[path.first] = true;
                    }
                }
                curr = curr->parent;
            }
        }

        // yz: get heuristic node from open list with minimum heuristic value
//     considering high level solver type
        CBSNode* selectNode() {
            CBSNode *curr = nullptr;
            cost_lowerbound = std::max(cost_lowerbound, cleanup_list.top()->getFVal());
            curr = cleanup_list.top();
            cleanup_list.pop();
            // takes the paths_found_initially and UPDATE all constrained paths found for agents from curr to dummy_start (and lower-bounds)
            updatePaths(curr);
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
        Conflicts findConflicts(HighLvNode &curr, int a1, int a2) {
            const auto& conflicts = detectAllConflictBetweenPaths(
                    this->solutions_[a1], this->solutions_[a2], this->agents_[a1], this->agents_[a2], this->all_poses_);
            for(const auto & conflict : conflicts) {
                curr.unknownConf.push_front(conflict); // It's at least a semi conflict
            }
//            std::cout << " get " << curr.unknownConf.size() << " conflicts between agent " << a1 << " and " << a2 << std::endl;
            return conflicts;
        }

        Conflicts findConflicts(HighLvNode &curr) {
            Conflicts retv;
            if (curr.parent != nullptr) {
                // Copy from parent
                auto new_agents = curr.getReplannedAgents();
//                std::cout << __FUNCTION__ << ": curr.parent != nullptr" << std::endl;
                // yz: when agent is replanned, their conflicts are ignored
                copyConflicts(curr.parent->conflicts, curr.conflicts, new_agents);
                copyConflicts(curr.parent->unknownConf, curr.unknownConf, new_agents);

                // detect new conflicts between any pair of agent
                // yz: detect conflict between agents that replanned paths and other agent
                for (auto it = new_agents.begin(); it != new_agents.end(); ++it) {
//                    std::cout << "new agent " << *it << std::endl;
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
                        if (!skip) {
//                            std::cout << " a1/a2 path size " << this->solutions_[a1].size() << "/" << this->solutions_[a2].size() << std::endl;
                            auto retv1 = findConflicts(curr, a1, a2);
                            retv.insert(retv.end(), retv1.begin(), retv1.end());
                        }
                    }
                }
            } else {
                // yz: add constraints between all pair of agents to current hyper node curr
                for (int a1 = 0; a1 < this->instances_.size(); a1++) {
                    for (int a2 = a1 + 1; a2 < this->instances_.size(); a2++) {
                        auto retv2 = findConflicts(curr, a1, a2);
                        retv.insert(retv.end(), retv2.begin(), retv2.end());
                    }
                }
            }
//            std::cout << " get " << curr.unknownConf.size() << " conflicts " << std::endl;
            return retv;
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
//            std::cout << "-- choose conflict time range [" << choose->t1 << ", " << choose->t2 << ")" << std::endl;
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
                // debug
//                std::cout << "add constraint: " << std::endl;
//                for(const auto& cs : buffer_node->constraints) {
//                    printConstraint(*cs);
//                }
                buffer_node = buffer_node->parent;
            }
            constraint_table.insert2CAT(agent, this->solutions_);

            const size_t& start_node_id = this->instance_node_ids_[agent].first,
                    target_node_id = this->instance_node_ids_[agent].second;

//            constraint_avoidance_table_.clearAllExistingOccGrids();
//            for(int a=0; a<this->agents_.size(); a++) {
//                if(a == agent) { continue; }
//                constraint_avoidance_table_.insertAgentPathOccGrids(this->agents_[a], this->solutions_[a]);
//            }
//            constraint_avoidance_table_.updateAgent(this->agents_[agent]);
            SpaceTimeAstar<N, AgentType> astar(start_node_id, target_node_id,
                                               this->agents_heuristic_tables_[agent],
                                               this->agents_heuristic_tables_ignore_rotate_[agent],
                                               this->agent_sub_graphs_[agent],
                                               constraint_table,
                                               nullptr,
                                               path_constraint_,
                                               this->agents_);
            astar.lower_bound_ = lowerbound;
            LAMAPF_Path new_path = astar.solve();
            if (!new_path.empty()) {
//                std::cout << " old path size = " << this->solutions_[agent].size() << std::endl;
//                std::cout << " new path: size = " << new_path.size() << std::endl;
//                printPath(agent, new_path);
//                if(this->solutions_[agent].size() > new_path.size()) {
//                    std::cout << " new path short than old path" << std::endl;
//                }
                assert(!isSamePath(this->solutions_[agent], new_path));
                node->paths.emplace_back(agent, new_path); // yz: add to replanned paths
                // yz: update current node's total cost (time step) of paths
//                std::cout << "old node->g_val = " << node->g_val << std::endl;
//                std::cout << "old soc = " << getSOC() << std::endl;
                node->g_val = node->g_val - (int) this->solutions_[agent].size() + (int) new_path.size();
                this->solutions_[agent] = node->paths.back().second;
                node->makespan = std::max(node->makespan, new_path.size() - 1);
//                std::cout << "new node->g_val = " << node->g_val << std::endl;
//                std::cout << "new soc = " << getSOC() << std::endl;
                return true;
            } else {
                return false;
            }
        }

        // yz: check whether child node is legal or illegal. if legal, add
        bool generateChild(CBSNode *node, CBSNode *parent) {
            node->parent = parent;
            node->HighLvNode::parent = parent;
            node->g_val = parent->g_val;
            node->makespan = parent->makespan;
            node->depth = parent->depth + 1;
            node->h_val = std::max(0, parent->h_val);
            node->h_computed = true;
            // yz: get agents that violates the first constraint
            auto agents = getInvalidAgents(node->constraints);
            assert(!agents.empty());
            for (auto agent : agents) {
                int lowerbound = (int) this->solutions_[agent].size() - 1;
                // yz: if find no path meet current constraint, the child node is illegal
                if (!findPathForSingleAgent(node, agent, lowerbound)) {
                    return false;
                }
//                std::cout << __FUNCTION__ << " update path of agent " << agent << std::endl;
            }
            findConflicts(*node);
//            std::cout << "new node = " << node->toString(this->all_poses_) << "\n";
            return true;
        }

        void printConflict(const Conflict& cf) {
            int a1, a2; size_t from1, from2, to1, to2;
            int start_t1, start_t2, end_t1, end_t2;
            std::tie(a1, from1, to1, start_t1, end_t1) = *(cf.cs1.front());
            std::tie(a2, from2, to2, start_t2, end_t2) = *(cf.cs2.front());

            std::cout << "cf agent: " << a1 << ", " << a2 << " | "
                      << *this->all_poses_[from1] << "->" << (to1 == MAX_NODES ? Pose<int, N>(Pointi<N>(), -1) : *this->all_poses_[to1])
                      << "t:{" << start_t1 << "," << end_t1 << "}" << ", "
                      << *this->all_poses_[from2] << "->" << (to2 == MAX_NODES ? Pose<int, N>(Pointi<N>(), -1) : *this->all_poses_[to2])
                      << "t:{" << start_t2 << "," << end_t2 << "}" << std::endl;

        }

        void printConstraint(const Constraint & cs) {
            int agent; size_t from, to;
            int start_t, end_t;
            std::tie(agent, from, to, start_t, end_t) = cs;

            std::cout << "cs agent: " << agent << " | {" << from << "}" << *this->all_poses_[from] << "->{" << to <<  "}" << *this->all_poses_[to] << ", "
                      << "/t:{" << start_t << "," << end_t << "}" << std::endl;
        }

        inline void releaseNodes() {
            open_list.clear();
            cleanup_list.clear();
            focal_list.clear();
            for (auto &node : allNodes_table)
                delete node;
            allNodes_table.clear();
        }

    };


}

// when both move, do edge-2-edge check
// when only one move, do edge-2-vertex check

#endif //LAYEREDMAPF_LA_CBS_H
