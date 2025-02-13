﻿#include <algorithm>    // std::shuffle
#include <random>      // std::default_random_engine
#include <chrono>       // std::chrono::system_clock
#include "PBS/inc/PBS.h"
#include "PBS/inc/SIPP.h"
#include "PBS/inc/SpaceTimeAStar.h"

namespace PBS_Li {


    PBS::PBS(const CBS_Li::Instance &instance, const CBS_Li::ConstraintTable &static_constraint, bool sipp, int screen) :
            screen(screen),
            static_constraint(static_constraint),
            num_of_agents(instance.getDefaultNumberOfAgents()) {
        clock_t t = clock();

        search_engines.resize(num_of_agents);
        for (int i = 0; i < num_of_agents; i++) {
            if (sipp)
                search_engines[i] = new SIPP(instance, i);
            else
                search_engines[i] = new SpaceTimeAStar(instance, i);
        }
        runtime_preprocessing = (double) (clock() - t) / CLOCKS_PER_SEC;

        if (screen >= 2) // print start and goals
        {
            instance.printAgents();
        }
    }


    bool PBS::solve(double _time_limit) {
        this->time_limit = _time_limit;

        if (screen > 0) // 1 or 2
        {
            std::string name = getSolverName();
            name.resize(35, ' ');
            std::cout << name << ": ";
        }
        // set timer
        start = clock();

        generateRoot();

//        freeNav::DimensionLength dim[2];
//        dim[0] = search_engines[0]->instance.num_of_cols;
//        dim[1] = search_engines[0]->instance.num_of_rows;
//        std::cout << " root node: "  << dummy_start->toString(dim) << std::endl;
        freeNav::LayeredMAPF::max_size_of_stack = 0; // yz: add for statistics

        while (!open_list.empty()) {
            auto curr = selectNode();

            // yz: all nodes size have nothing to do with how algorithm works, just for save data
            freeNav::LayeredMAPF::max_size_of_stack = std::max(allNodes_table.size(), freeNav::LayeredMAPF::max_size_of_stack);

            if (terminate(curr)) break;

            curr->conflict = chooseConflict(*curr);

            if (screen > 1)
                std::cout << "	Expand " << *curr << "	on " << *(curr->conflict) << std::endl;

            assert(!hasHigherPriority(curr->conflict->a1, curr->conflict->a2) and
                   !hasHigherPriority(curr->conflict->a2, curr->conflict->a1));
            auto t1 = clock();
            std::vector < CBS_Li::MAPFPath * > copy(paths);
            generateChild(0, curr, curr->conflict->a1, curr->conflict->a2);

//            std::cout << " new node: "  << curr->children[0]->toString(dim) << std::endl;
            paths = copy;
            generateChild(1, curr, curr->conflict->a2, curr->conflict->a1);
            runtime_generate_child += (double) (clock() - t1) / CLOCKS_PER_SEC;
            pushNodes(curr->children[0], curr->children[1]);
//            std::cout << " new node: "  << curr->children[1]->toString(dim) << std::endl;

            curr->clear();
        }  // end of while loop
//        std::cout << "PBS::max_size_of_stack = " << freeNav::LayeredMAPF::max_size_of_stack << std::endl;
        return solution_found;
    }

    bool PBS::generateChild(int child_id, PBSNode *parent, int low, int high) {
        assert(child_id == 0 or child_id == 1);
        parent->children[child_id] = new PBSNode(*parent);
        auto node = parent->children[child_id];
        node->constraint.set(low, high);
        priority_graph[high][low] = false;
        priority_graph[low][high] = true;
        if (screen > 2)
            printPriorityGraph();
        topologicalSort(ordered_agents); // yz: update sort of prioriry based on priority_graph
        if (screen > 2) {
            std::cout << "Ordered agents: ";
            for (int i : ordered_agents)
                std::cout << i << ",";
            std::cout << std::endl;
        }
        // yz: set order of each agents
        std::vector<int> topological_orders(num_of_agents); // map agent i to its position in ordered_agents
        auto i = num_of_agents - 1;
        for (const auto &a : ordered_agents) {
            topological_orders[a] = i;
            i--;
        }

        std::priority_queue<std::pair < int, int>>
        to_replan; // <position in ordered_agents, agent id>
        // yz: store whether agent have insert into replan, avoid repeat
        std::vector<bool> lookup_table(num_of_agents, false);
        to_replan.emplace(topological_orders[low], low);
        lookup_table[low] = true;
        { // find conflicts where one agent is higher than high and the other agent is lower than low
            std::set<int> higher_agents; // yz: higher agent is not update ?
            auto p = ordered_agents.rbegin();
            std::advance(p, topological_orders[high]);
            assert(*p == high);
            getHigherPriorityAgents(p, higher_agents);
            higher_agents.insert(high);

            std::set<int> lower_agents;
            auto p2 = ordered_agents.begin();
            // yz: advance() 函数用于将迭代器前进（或者后退）指定长度的距离。
            std::advance(p2, num_of_agents - 1 - topological_orders[low]);
            assert(*p2 == low);
            getLowerPriorityAgents(p2, lower_agents);

            for (const auto &conflict : node->conflicts) {
                int a1 = conflict->a1;
                int a2 = conflict->a2;
                // yz: avoid repeat of insert into replan
                if (a1 == low or a2 == low)
                    continue;
                if (topological_orders[a1] > topological_orders[a2]) {
                    std::swap(a1, a2); // yz: make a1 < a2
                }
                // yz: if a1 is lower than current and high is higher than current, then replan a1
                if (!lookup_table[a1] and lower_agents.find(a1) != lower_agents.end() and
                    higher_agents.find(a2) != higher_agents.end()) {
                    to_replan.emplace(topological_orders[a1], a1);
                    lookup_table[a1] = true;
                }
            }
        }


        while (!to_replan.empty()) {
            int a, rank;
            std::tie(rank, a) = to_replan.top();
            to_replan.pop();
            lookup_table[a] = false;
            if (screen > 2) std::cout << "Replan agent " << a << std::endl;
            // Re-plan path
            std::set<int> higher_agents;
            auto p = ordered_agents.rbegin();
            std::advance(p, rank); // yz: get current agent in priority queue
            assert(*p == a);
            getHigherPriorityAgents(p, higher_agents);
            assert(!higher_agents.empty());
            if (screen > 2) {
                std::cout << "Higher agents: ";
                for (auto i : higher_agents)
                    std::cout << i << ",";
                std::cout << std::endl;
            }
            CBS_Li::MAPFPath new_path;
            if (!findPathForSingleAgent(*node, higher_agents, a, new_path)) {
                delete node;
                parent->children[child_id] = nullptr;
                return false;
            }

            // Delete old conflicts
            // yz: if an agent is replanned, delete all conflicts about it
            for (auto c = node->conflicts.begin(); c != node->conflicts.end();) {
                if ((*c)->a1 == a or (*c)->a2 == a)
                    c = node->conflicts.erase(c);
                else
                    ++c;
            }

            // Update conflicts and to_replan
            std::set<int> lower_agents;
            auto p2 = ordered_agents.begin();
            std::advance(p2, num_of_agents - 1 - rank);
            assert(*p2 == a);
            getLowerPriorityAgents(p2, lower_agents);
            if (screen > 2 and !lower_agents.empty()) {
                std::cout << "Lower agents: ";
                for (auto i : lower_agents)
                    std::cout << i << ",";
                std::cout << std::endl;
            }

            // Find new conflicts
            // yz: after an agent is replanned, update conflict about it
            for (auto a2 = 0; a2 < num_of_agents; a2++) {
                if (a2 == a or lookup_table[a2] or
                    higher_agents.count(a2) > 0) // already in to_replan or has higher priority
                    continue;
                auto t = clock();
                if (hasConflicts(a, a2)) {
                    node->conflicts.emplace_back(new Conflict(a, a2));
                    if (lower_agents.count(a2) > 0) // has a collision with a lower priority agent
                    {
                        if (screen > 1)
                            std::cout << "\t" << a2 << " needs to be replanned due to collisions with " << a << std::endl;
                        // yz: if find new agent that lower than current agent, it need to be replan
                        to_replan.emplace(topological_orders[a2], a2);
                        lookup_table[a2] = true;
                    }
                }
                runtime_detect_conflicts += (double) (clock() - t) / CLOCKS_PER_SEC;
            }
        }
        num_HL_generated++;
        node->time_generated = num_HL_generated;
        if (screen > 1)
            std::cout << "Generate " << *node << std::endl;
        return true;
    }

    bool PBS::findPathForSingleAgent(PBSNode &node, const std::set<int> &higher_agents, int a, CBS_Li::MAPFPath &new_path) {
        clock_t t = clock();
        new_path = search_engines[a]->findOptimalPath(higher_agents, paths,
                                                      a, static_constraint);  //TODO: add runtime check to the low level
        num_LL_expanded += search_engines[a]->num_expanded;
        num_LL_generated += search_engines[a]->num_generated;
        runtime_build_CT += search_engines[a]->runtime_build_CT;
        runtime_build_CAT += search_engines[a]->runtime_build_CAT;
        runtime_path_finding += (double) (clock() - t) / CLOCKS_PER_SEC;
        if (new_path.empty())
            return false;
        assert(paths[a] != nullptr and !isSamePath(*paths[a], new_path));
        node.cost += (int) new_path.size() - (int) paths[a]->size();
        if (node.makespan >= paths[a]->size()) {
            node.makespan = std::max(node.makespan, new_path.size() - 1);
        } else {
            node.makespan = 0;
            for (int i = 0; i < num_of_agents; i++) {
                if (i == a and new_path.size() - 1 > node.makespan)
                    node.makespan = new_path.size() - 1;
                else
                    node.makespan = std::max(node.makespan, paths[i]->size() - 1);
            }
        }
        node.paths.emplace_back(a, new_path);
        paths[a] = &node.paths.back().second;
        assert(!hasConflicts(a, higher_agents));
        return true;
    }

// takes the paths_found_initially and UPDATE all (constrained) paths found for agents from curr to start
    inline void PBS::update(PBSNode *node) {
        // yz: reset path to previous updated path
        paths.assign(num_of_agents, nullptr);
        // yz: reset priority_graph and set it according to it and its parent nodes' priority constraint
        priority_graph.assign(num_of_agents, std::vector<bool>(num_of_agents, false));
        for (auto curr = node; curr != nullptr; curr = curr->parent) {
            for (auto &path : curr->paths) {
                if (paths[path.first] == nullptr) {
                    paths[path.first] = &(path.second);
                }
            }
            if (curr->parent != nullptr) // non-root node
                priority_graph[curr->constraint.low][curr->constraint.high] = true;
        }
        assert(getSumOfCosts() == node->cost);
    }

    bool PBS::hasConflicts(int a1, int a2) const {
        int min_path_length = (int) (paths[a1]->size() < paths[a2]->size() ? paths[a1]->size() : paths[a2]->size());
        for (int timestep = 0; timestep < min_path_length; timestep++) {
            int loc1 = paths[a1]->at(timestep).location;
            int loc2 = paths[a2]->at(timestep).location;
            if (loc1 == loc2 or (timestep < min_path_length - 1 and loc1 == paths[a2]->at(timestep + 1).location
                                 and loc2 == paths[a1]->at(timestep + 1).location)) // vertex or edge conflict
            {
                return true;
            }
        }
        if (paths[a1]->size() != paths[a2]->size()) {
            int a1_ = paths[a1]->size() < paths[a2]->size() ? a1 : a2;
            int a2_ = paths[a1]->size() < paths[a2]->size() ? a2 : a1;
            int loc1 = paths[a1_]->back().location;
            for (int timestep = min_path_length; timestep < (int) paths[a2_]->size(); timestep++) {
                int loc2 = paths[a2_]->at(timestep).location;
                if (loc1 == loc2) {
                    return true; // target conflict
                }
            }
        }
        return false; // conflict-free
    }

    bool PBS::hasConflicts(int a1, const std::set<int> &agents) const {
        for (auto a2 : agents) {
            if (hasConflicts(a1, a2))
                return true;
        }
        return false;
    }

    std::shared_ptr <Conflict> PBS::chooseConflict(const PBSNode &node) const {
        if (screen == 3)
            printConflicts(node);
        if (node.conflicts.empty())
            return nullptr;
        return node.conflicts.back();
    }

    int PBS::getSumOfCosts() const {
        int cost = 0;
        for (const auto &path : paths)
            cost += (int) path->size() - 1;
        return cost;
    }

    inline void PBS::pushNode(PBSNode *node) {
        // update handles
        open_list.push(node);
        allNodes_table.push_back(node);
    }

    void PBS::pushNodes(PBSNode *n1, PBSNode *n2) {
        if (n1 != nullptr and n2 != nullptr) {
            if (n1->cost < n2->cost) {
                pushNode(n2);
                pushNode(n1);
            } else {
                pushNode(n1);
                pushNode(n2);
            }
        } else if (n1 != nullptr) {
            pushNode(n1);
        } else if (n2 != nullptr) {
            pushNode(n2);
        }
    }

    PBSNode *PBS::selectNode() {
        PBSNode *curr = open_list.top();
        open_list.pop();
        update(curr);
        num_HL_expanded++;
        curr->time_expanded = num_HL_expanded;
        if (screen > 1)
            std::cout << std::endl << "Pop " << *curr << std::endl;
        return curr;
    }

    void PBS::printPaths() const {
        for (int i = 0; i < num_of_agents; i++) {
            std::cout << "Agent " << i << " (" << search_engines[i]->my_heuristic[search_engines[i]->start_location]
                 << " -->" <<
                 paths[i]->size() - 1 << "): ";
            for (const auto &t : *paths[i])
                std::cout << t.location << "->";
            std::cout << std::endl;
        }
    }

    void PBS::printPriorityGraph() const {
        std::cout << "Priority graph:";
        for (int a1 = 0; a1 < num_of_agents; a1++) {
            for (int a2 = 0; a2 < num_of_agents; a2++) {
                if (priority_graph[a1][a2])
                    std::cout << a1 << "<" << a2 << ",";
            }
        }
        std::cout << std::endl;
    }

    void PBS::printResults() const {
        if (solution_cost >= 0) // solved
            std::cout << "Succeed,";
        else if (solution_cost == -1) // time_out
            std::cout << "Timeout,";
        else if (solution_cost == -2) // no solution
            std::cout << "No solutions,";
        else if (solution_cost == -3) // nodes out
            std::cout << "Nodesout,";

        std::cout << solution_cost << "," << runtime << "," <<
             num_HL_expanded << "," << num_LL_expanded << ","
             << // HL_num_generated << "," << LL_num_generated << "," <<
             dummy_start->cost << "," << std::endl;
        /*if (solution_cost >= 0) // solved
        {
            cout << "fhat = [";
            auto curr = goal_node;
            while (curr != nullptr)
            {
                cout << curr->getFHatVal() << ",";
                curr = curr->parent;
            }
            cout << "]" << endl;
            cout << "hhat = [";
            curr = goal_node;
            while (curr != nullptr)
            {
                cout << curr->cost_to_go << ",";
                curr = curr->parent;
            }
            cout << "]" << endl;
            cout << "d = [";
            curr = goal_node;
            while (curr != nullptr)
            {
                cout << curr->distance_to_go << ",";
                curr = curr->parent;
            }
            cout << "]" << endl;
            cout << "soc = [";
            curr = goal_node;
            while (curr != nullptr)
            {
                cout << curr->getFHatVal() - curr->cost_to_go << ",";
                curr = curr->parent;
            }
            cout << "]" << endl;
        }*/
    }

    void PBS::saveResults(const std::string &fileName, const std::string &instanceName) const {
        std::ifstream infile(fileName);
        bool exist = infile.good();
        infile.close();
        if (!exist) {
            std::ofstream addHeads(fileName);
            addHeads << "runtime,#high-level expanded,#high-level generated,#low-level expanded,#low-level generated,"
                     <<
                     "solution cost,root g value," <<
                     "runtime of detecting conflicts,runtime of building constraint tables,runtime of building CATs," <<
                     "runtime of path finding,runtime of generating child nodes," <<
                     "preprocessing runtime,solver name,instance name" << std::endl;
            addHeads.close();
        }
        std::ofstream stats(fileName, std::ios::app);
        stats << runtime << "," <<
              num_HL_expanded << "," << num_HL_generated << "," <<
              num_LL_expanded << "," << num_LL_generated << "," <<

              solution_cost << "," << dummy_start->cost << "," <<

              runtime_detect_conflicts << "," << runtime_build_CT << "," << runtime_build_CAT << "," <<
              runtime_path_finding << "," << runtime_generate_child << "," <<

              runtime_preprocessing << "," << getSolverName() << "," << instanceName << std::endl;
        stats.close();
    }

    void PBS::saveCT(const std::string &fileName) const // write the CT to a file
    {
        // Write the tree graph in dot language to a file
        {
            std::ofstream output;
            output.open(fileName + ".tree", std::ios::out);
            output << "digraph G {" << std::endl;
            output << "size = \"5,5\";" << std::endl;
            output << "center = true;" << std::endl;
            std::set < PBSNode * > path_to_goal;
            auto curr = goal_node;
            while (curr != nullptr) {
                path_to_goal.insert(curr);
                curr = curr->parent;
            }
            for (const auto &node : allNodes_table) {
                output << node->time_generated << " [label=\"g=" << node->cost;
                if (node->time_expanded > 0) // the node has been expanded
                {
                    output << "\n #" << node->time_expanded;
                }
                output << "\"]" << std::endl;


                if (node == dummy_start)
                    continue;
                if (path_to_goal.find(node) == path_to_goal.end()) {
                    output << node->parent->time_generated << " -> " << node->time_generated << std::endl;
                } else {
                    output << node->parent->time_generated << " -> " << node->time_generated << " [color=red]" << std::endl;
                }
            }
            output << "}" << std::endl;
            output.close();
        }

        // Write the stats of the tree to a CSV file
        {
            std::ofstream output;
            output.open(fileName + "-tree.csv", std::ios::out);
            // header
            output << "time generated,g value,h value,h^ value,d value,depth,time expanded,chosen from,h computed,"
                   << "f of best in cleanup,f^ of best in cleanup,d of best in cleanup,"
                   << "f of best in open,f^ of best in open,d of best in open,"
                   << "f of best in focal,f^ of best in focal,d of best in focal,"
                   << "praent,goal node" << std::endl;
            for (auto &node : allNodes_table) {
                output << node->time_generated << ","
                       << node->cost << ","
                       << node->depth << ","
                       << node->time_expanded << ",";
                if (node->parent == nullptr)
                    output << "0,";
                else
                    output << node->parent->time_generated << ",";
                if (node == goal_node)
                    output << "1" << std::endl;
                else
                    output << "0" << std::endl;
            }
            output.close();
        }

    }

    void PBS::savePaths(const std::string &fileName) const {
        std::ofstream output;
        output.open(fileName, std::ios::out);
        for (int i = 0; i < num_of_agents; i++) {
            output << "Agent " << i << ": ";
            for (const auto &t : *paths[i])
                output << "(" << search_engines[0]->instance.getRowCoordinate(t.location)
                       << "," << search_engines[0]->instance.getColCoordinate(t.location) << ")->";
            output << std::endl;
        }
        output.close();
    }

    void PBS::savePaths() {
        fr_paths_.clear();
        if (!solution_found) { return; }
        for (int i = 0; i < num_of_agents; i++) {
            freeNav::Path<2> path;
            for (const auto &t : *paths[i]) {
                freeNav::Pointi<2> pt;
                pt[1] = search_engines[0]->instance.getRowCoordinate(t.location);
                pt[0] = search_engines[0]->instance.getColCoordinate(t.location);
                path.push_back(pt);
            }
            fr_paths_.push_back(path);
        }
    }

    void PBS::printConflicts(const PBSNode &curr) {
        for (const auto &conflict : curr.conflicts) {
            std::cout << *conflict << std::endl;
        }
    }


    std::string PBS::getSolverName() const {
        return "PBS with " + search_engines[0]->getName();
    }


    bool PBS::terminate(PBSNode *curr) {
        runtime = (double) (clock() - start) / CLOCKS_PER_SEC;
        if (curr->conflicts.empty()) //no conflicts
        {// found a solution
            solution_found = true;
            goal_node = curr;
            solution_cost = goal_node->cost;
            if (!validateSolution()) {
                std::cout << "Solution invalid!!!" << std::endl;
                printPaths();
                exit(-1);
            }
            if (screen > 0) // 1 or 2
                printResults();
            return true;
        }
        if (runtime > time_limit || num_HL_expanded > node_limit) {   // time/node out
            solution_cost = -1;
            solution_found = false;
            if (screen > 0) // 1 or 2
                printResults();
            return true;
        }
        return false;
    }


    bool PBS::generateRoot() {
        auto root = new PBSNode();
        root->cost = 0;
        paths.reserve(num_of_agents);

        std::set<int> higher_agents;
        for (auto i = 0; i < num_of_agents; i++) {
            //CAT cat(dummy_start->makespan + 1);  // initialized to false
            //updateReservationTable(cat, i, *dummy_start);
            auto new_path = search_engines[i]->findOptimalPath(higher_agents, paths, i, static_constraint);
            num_LL_expanded += search_engines[i]->num_expanded;
            num_LL_generated += search_engines[i]->num_generated;
            if (new_path.empty()) {
                std::cout << "No path exists for agent " << i << std::endl;
                return false;
            }
            root->paths.emplace_back(i, new_path);
            paths.emplace_back(&root->paths.back().second);
            root->makespan = std::max(root->makespan, new_path.size() - 1);
            root->cost += (int) new_path.size() - 1;
        }
        auto t = clock();
        root->depth = 0;
        // yz: detect conflicts
        for (int a1 = 0; a1 < num_of_agents; a1++) {
            for (int a2 = a1 + 1; a2 < num_of_agents; a2++) {
                if (hasConflicts(a1, a2)) {
                    root->conflicts.emplace_back(new Conflict(a1, a2));
                }
            }
        }
        runtime_detect_conflicts += (double) (clock() - t) / CLOCKS_PER_SEC;
        num_HL_generated++;
        root->time_generated = num_HL_generated;
        if (screen > 1)
            std::cout << "Generate " << *root << std::endl;
        pushNode(root);
        dummy_start = root;
        if (screen >= 2) // print start and goals
        {
            printPaths();
        }
        return true;
    }

    inline void PBS::releaseNodes() {
        // TODO:: clear open_list
        for (auto &node : allNodes_table)
            delete node;
        allNodes_table.clear();
    }


/*inline void PBS::releaseOpenListNodes()
{
	while (!open_list.empty())
	{
		PBSNode* curr = open_list.top();
		open_list.pop();
		delete curr;
	}
}*/

    PBS::~PBS() {
        releaseNodes();
    }

    void PBS::clearSearchEngines() {
        for (auto s : search_engines)
            delete s;
        search_engines.clear();
    }


    bool PBS::validateSolution() const {
        // check whether the paths are feasible
        size_t soc = 0;
        for (int a1 = 0; a1 < num_of_agents; a1++) {
            soc += paths[a1]->size() - 1;
            for (int a2 = a1 + 1; a2 < num_of_agents; a2++) {
                size_t min_path_length = paths[a1]->size() < paths[a2]->size() ? paths[a1]->size() : paths[a2]->size();
                for (size_t timestep = 0; timestep < min_path_length; timestep++) {
                    int loc1 = paths[a1]->at(timestep).location;
                    int loc2 = paths[a2]->at(timestep).location;
                    if (loc1 == loc2) {
                        std::cout << "Agents " << a1 << " and " << a2 << " collides at " << loc1 << " at timestep "
                             << timestep << std::endl;
                        return false;
                    } else if (timestep < min_path_length - 1
                               && loc1 == paths[a2]->at(timestep + 1).location
                               && loc2 == paths[a1]->at(timestep + 1).location) {
                        std::cout << "Agents " << a1 << " and " << a2 << " collides at (" <<
                             loc1 << "-->" << loc2 << ") at timestep " << timestep << std::endl;
                        return false;
                    }
                }
                if (paths[a1]->size() != paths[a2]->size()) {
                    int a1_ = paths[a1]->size() < paths[a2]->size() ? a1 : a2;
                    int a2_ = paths[a1]->size() < paths[a2]->size() ? a2 : a1;
                    int loc1 = paths[a1_]->back().location;
                    for (size_t timestep = min_path_length; timestep < paths[a2_]->size(); timestep++) {
                        int loc2 = paths[a2_]->at(timestep).location;
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

    inline int PBS::getAgentLocation(int agent_id, size_t timestep) const {
        size_t t = std::max(std::min(timestep, paths[agent_id]->size() - 1), (size_t) 0);
        return paths[agent_id]->at(t).location;
    }


// used for rapid random  restart
    void PBS::clear() {
        releaseNodes();
        paths.clear();
        dummy_start = nullptr;
        goal_node = nullptr;
        solution_found = false;
        solution_cost = -2;
    }


    void PBS::topologicalSort(std::list<int> &stack) {
        stack.clear();
        std::vector<bool> visited(num_of_agents, false);

        // Call the recursive helper function to store Topological
        // Sort starting from all vertices one by one
        for (int i = 0; i < num_of_agents; i++) {
            if (!visited[i])
                topologicalSortUtil(i, visited, stack);
        }
    }

    void PBS::topologicalSortUtil(int v, std::vector<bool> &visited, std::list<int> &stack) {
        // Mark the current node as visited.
        visited[v] = true;

        // Recur for all the vertices adjacent to this vertex
        assert(!priority_graph.empty());
        for (int i = 0; i < num_of_agents; i++) {
            if (priority_graph[v][i] and !visited[i])
                topologicalSortUtil(i, visited, stack);
        }
        // Push current vertex to stack which stores result
        stack.push_back(v);
    }

    void PBS::getHigherPriorityAgents(const std::list<int>::reverse_iterator &p1, std::set<int> &higher_agents) {
        for (auto p2 = std::next(p1); p2 != ordered_agents.rend(); ++p2) {
            if (priority_graph[*p1][*p2]) {
                auto ret = higher_agents.insert(*p2);
                if (ret.second) // insert successfully
                {
                    getHigherPriorityAgents(p2, higher_agents);
                }
            }
        }
    }

    void PBS::getLowerPriorityAgents(const std::list<int>::iterator &p1, std::set<int> &lower_subplans) {
        for (auto p2 = std::next(p1); p2 != ordered_agents.end(); ++p2) {
            if (priority_graph[*p2][*p1]) {
                auto ret = lower_subplans.insert(*p2);
                if (ret.second) // insert successfully
                {
                    getLowerPriorityAgents(p2, lower_subplans);
                }
            }
        }
    }

    // may fall into infinity loop
    bool PBS::hasHigherPriority(int low, int high) const // return true if agent low is lower than agent high
    {
        std::queue<int> Q;
        std::vector<bool> visited(num_of_agents, false);
        visited[low] = false;
        Q.push(low);
        while (!Q.empty()) {
            auto n = Q.front();
            Q.pop();
            if (n == high)
                return true;
            for (int i = 0; i < num_of_agents; i++) {
                if (priority_graph[n][i] and !visited[i])
                    Q.push(i);
            }
        }
        return false;
    }

}