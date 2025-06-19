//
// Created by yaozhuo on 6/12/25.
//

#ifndef LAYEREDMAPF_BREAK_LOOP_DECOMPOSITION_H
#define LAYEREDMAPF_BREAK_LOOP_DECOMPOSITION_H


#include <sys/time.h>
#include <boost/graph/subgraph.hpp>
#include <boost/graph/adjacency_list.hpp>

#include <boost/config.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/strong_components.hpp>

#include "../LA-MAPF/common.h"
#include "../LA-MAPF/large_agent_dependency_path_search.h"

//#include "../LA-MAPF/CBS/space_time_astar.h"

namespace freeNav::LayeredMAPF::LA_MAPF {

    // a general interfaces for both LA-MAPF and MAPF
    template<Dimension N, typename HyperNodeType>
    class MAPFInstanceDecompositionBreakLoop {

    public:

        typedef std::set<int> Level;
        typedef std::vector<Level> Levels;

        typedef std::vector<std::vector<bool> > LevelOrderGraph;

        MAPFInstanceDecompositionBreakLoop(DimensionLength* dim,
                                           const std::vector<ConnectivityGraph>& connectivity_graphs,
                                           const std::vector<SubGraphOfAgent<N> >& agent_sub_graphs,
                                           const std::vector<std::vector<int> >& heuristic_tables_sat, // distinguish_sat = true
                                           double time_limit = 10,
                                           int max_break_count = 1e3,
                                           int max_continue_failure = 50,
                                           int expected_min_level_size = 1):
                                     dim_(dim),
                                     agent_sub_graphs_(agent_sub_graphs),
                                     connect_graphs_(connectivity_graphs),
                                     heuristic_tables_sat_(heuristic_tables_sat),
                                     time_limit_(time_limit),
                                     max_break_count_(max_break_count),
                                     max_continue_failure_(max_continue_failure),
                                     expected_min_level_size_(expected_min_level_size) {

            start_t_ = clock();

            std::set<int> all_agent_id_set;

            // 1, calculate heuristic table for each connectivity graph
            for (int i = 0; i < connectivity_graphs.size(); i++) {
                all_agent_id_set_.insert(i);
            }

            all_dependency_paths_.clear();
            cluster_buffer_sat_ = AgentIdsToSATID(all_agent_id_set_);
            // 2, init all_dependency_paths_ with every agent's shortest path in connectivity_graphs
            for (int agent_id = 0; agent_id < connectivity_graphs.size(); agent_id++) {
                auto passing_start_and_targets = searchAgent(agent_id, {}, cluster_buffer_sat_, true); // pass test
                all_dependency_paths_[agent_id] = passing_start_and_targets;
            }
            all_levels_ = getLevelsFromDependencyPaths(all_dependency_paths_);

            auto now_t = clock();

            double time_cost =  ((double)now_t - start_t_)/CLOCKS_PER_SEC;;

            std::cout << "ns finish initial decomposition in " << time_cost << "s" << std::endl;

            // debug
            for(const auto& level : all_levels_) {
                for(const auto& agent_id : level) {
                    assert(agent_id < agent_sub_graphs_.size());
                }
            }
        }


        std::vector<bool> getAvoidAgentInCurrentLevel(int agent_id, const std::set<int>& level, bool pick_start) const {
            std::vector<bool> state_ids(2*connect_graphs_.size(), false);
            //std::cout << "break agent avoid: ";
            for(const auto& another_agent_id : level) {
                if(agent_id == another_agent_id) { continue; }
                int avoid_node_id;
                if(pick_start) {
                    avoid_node_id = 2*another_agent_id;
                } else {
                    avoid_node_id = 2*another_agent_id + 1;
                }
                state_ids[avoid_node_id] = true;
                //std::cout << avoid_node_id << " ";
            }
            //std::cout << std::endl;
            return state_ids;
        }

        std::vector<bool> getAvoidAgentInCurrentLevelOther(int agent_id, bool pick_start) const {
            std::vector<bool> state_ids(2*connect_graphs_.size(), false);
            if(pick_start) {
                state_ids[2*agent_id + 1] = true;
            } else {
                state_ids[2*agent_id] = true;
            }
            return state_ids;
        }

        void breakMaxLoopIteratively() {
            int count_of_break = 0;
            int count_of_continue_failure = 0;
            int i_th_largest_level = 0;
            while(true) {
                auto now_t = clock();
                double time_cost =  ((double)now_t - start_t_)/CLOCKS_PER_SEC;

                if(time_cost > time_limit_) { break; }
                if(count_of_break >= max_break_count_) { break; }
                //if(getMaxLevelSize(all_levels_) <= expected_min_level_size_) { break; }


                auto retv_pair = getMaxLevel(all_levels_, i_th_largest_level);
                if(retv_pair.second.size() <= expected_min_level_size_) { break; }

                bool success = breakMaxLoop(count_of_break, i_th_largest_level);
                if(success) {
                    count_of_continue_failure = 0;
                } else {
                    count_of_continue_failure ++;
                }

                if(count_of_continue_failure > max_continue_failure_) {
                    i_th_largest_level ++;
                    count_of_continue_failure = 0;
                }

                count_of_break ++;
            }

            auto now_t = clock();
            double time_cost =  ((double)now_t - start_t_)/CLOCKS_PER_SEC;
            std::cout << "finish break loops in " << time_cost << "s, " << count_of_break << " breaks" << std::endl;
        }

        std::vector<bool> getAvailNodesFromOtherLevels(
                int level_id,
                std::map<int, std::set<int> >& all_paths,
                const std::vector<std::set<int> >& all_levels) const {
            const LevelOrderGraph &g = getLevelOrderGraph(all_paths, all_levels);
            assert(g.size() == all_levels.size());

            // get all level that earlier than current level or later than current level
            std::vector<int> earlier_level_idv, later_level_idv;
            std::vector<int> buffer = {level_id}, next_buffer = {};
            std::vector<bool> with_order_flag(all_levels.size(), false);// whether the level have order limitation with current level
            //std::cout << "earlier_level_idv = ";
            while (!buffer.empty()) {
                next_buffer.clear();
                for (const int &aid : buffer) {
                    for (int neig = 0; neig < all_levels.size(); neig++) {
                        if (g[neig][aid]) {
                            next_buffer.push_back(neig);
                            earlier_level_idv.push_back(neig);
                            with_order_flag[neig] = true;
                            //std::cout << neig << " ";
                        }
                    }
                }
                std::swap(buffer, next_buffer);
            }
            //std::cout << std::endl;
            buffer = {level_id}, next_buffer = {};
            //std::cout << "later_level_idv = ";
            while (!buffer.empty()) {
                next_buffer.clear();
                for (const int &aid : buffer) {
                    for (int neig = 0; neig < all_levels.size(); neig++) {
                        if (g[aid][neig]) {
                            next_buffer.push_back(neig);
                            later_level_idv.push_back(neig);
                            with_order_flag[neig] = true;
                            //std::cout << neig << " ";
                        }
                    }
                }
                std::swap(buffer, next_buffer);
            }
            //std::cout << std::endl;
            std::vector<bool> avail_nodes(2 * connect_graphs_.size(), true);
            //std::cout << "not avail: ";
            // level earlier than another_lev_id
            for (const int &lid : earlier_level_idv) {
                for (const int &aid : all_levels[lid]) {
                    avail_nodes[2 * aid + 1] = false;
                    //std::cout << 2 * aid + 1 << " ";
                }
            }
            // level later than another_lev_id
            for(const int& lid : later_level_idv) {
                for (const int &aid : all_levels[lid]) {
                    avail_nodes[2 * aid] = false;
                    //std::cout << 2 * aid << " ";
                }
            }
            //std::cout << std::endl;

            with_order_flag[level_id] = true;
            //std::cout << "random not avail: ";
            // level that have no solving order with current level will set to random order
            // to avoid generate loop with agent in current level
            for(int i=0; i<all_levels.size(); i++) {
                if(!with_order_flag[i]) {
                    if(rand() % 2 ==0) {
                        for(const int& aid : all_levels[i]) {
                            avail_nodes[2 * aid] = false;
                            //std::cout << 2 * aid << " ";
                        }
                    } else {
                        for(const int& aid : all_levels[i]) {
                            avail_nodes[2 * aid + 1] = false;
                            //std::cout << 2 * aid + 1 << " ";
                        }
                    }
                }
            }
            //std::cout << std::endl;
            return avail_nodes;
        }

        bool breakMaxLoop(const int& iter_count, int th_largest_level = 0) {
            // 1, pick the largest loop
            std::pair<int, std::set<int> > max_level = getMaxLevel(all_levels_, th_largest_level);
            if(max_level.second.size() == 1) { return false; }
            // 2, random pick an agent from the loop
            auto start_iter = max_level.second.begin();
            int agent_id;
            int advance_step = rand() % max_level.second.size();
            for(int i=0; i<advance_step; i++) {
                start_iter ++;
            }
            agent_id = *start_iter;

//            std::cout << "all level = " << std::endl;
//            for(int i=0; i<all_levels_.size(); i++) {
//                const auto& level = all_levels_[i];
//                std::cout << i << "th: " << level << std::endl;
//            }

            // 3, update an agent's path in the path randomly, with heuristic to break loop
            // detect edge to agent in current loop
            // try avoid this link in search path

            const std::vector<bool>& avail_start_and_target = getAvailNodesFromOtherLevels(max_level.first,
                                                                                    all_dependency_paths_,
                                                                                    all_levels_);

            bool pick_start = (rand() % 2 == 0); // random choose whether earlier than other agents in the same level

            std::vector<bool> avoid_start_and_target = getAvoidAgentInCurrentLevel(agent_id, max_level.second, pick_start);

            std::set<int> new_dependency_path;

            //avail_start_and_target = std::vector<bool>(2*connect_graphs_.size(), true);

            //avoid_start_and_target = std::vector<bool>(2*connect_graphs_.size(), false);

            // try avoid edge of target id
            new_dependency_path = searchAgent(agent_id, avoid_start_and_target, avail_start_and_target, true); // pass test

            // if search path failed, exit
            if(new_dependency_path.empty()) {
                //std::cout << "break loop (" << max_level.second.size() << ")" << max_level.second << " failed at agent " << agent_id << std::endl;
                //std::cout << "flag 1" << std::endl;
                return false;
            } else {
                //std::cout << "break loop (" << max_level.second.size() << ")" << max_level.second << " success at agent " << agent_id << std::endl;
            }

            std::vector<bool> avoid_start_and_target_other = getAvoidAgentInCurrentLevelOther(agent_id, pick_start);

            //avoid_start_and_target_other = std::vector<bool>(2*connect_graphs_.size(), false);

            std::map<int, std::set<int> > new_level_paths;
            int avoid_node_id = pick_start ? 2*agent_id + 1 : 2* agent_id;
            for(const int& agent_id_other : max_level.second) {
                if(agent_id_other == agent_id) { continue; }
                // only update path that contain edge to agent
                if(all_dependency_paths_[agent_id_other].find(avoid_node_id) == all_dependency_paths_[agent_id_other].end()) {
                    continue;
                }
//                auto avoid_start_and_target_other_copy = avoid_start_and_target_other;
//                avoid_start_and_target_other_copy[2*agent_id_other] = false;
//                avoid_start_and_target_other_copy[2*agent_id_other + 1] = false;
//
//                auto avail_start_and_target_copy = avail_start_and_target;
//                avail_start_and_target_copy[2*agent_id_other] = true;
//                avail_start_and_target_copy[2*agent_id_other + 1] = true;

                std::set<int> new_dependency_path_other = searchAgent(agent_id_other,
                                                                      avoid_start_and_target_other,
                                                                      avail_start_and_target, true); // pass test
                if(new_dependency_path_other.empty()) {
                    //std::cout << "flag 2" << std::endl;
                    return false;
                }
                new_level_paths.insert({agent_id_other, new_dependency_path_other});
            }


            //std::cout << " new dependency path = " << new_dependency_path << std::endl;

            // 4, get new levels
            auto new_dependency_paths = all_dependency_paths_;
            new_dependency_paths[agent_id] = new_dependency_path;
            for(const auto& new_id_and_path : new_level_paths) {
                new_dependency_paths[new_id_and_path.first] = new_id_and_path.second;
            }

            auto new_levels = getLevelsFromDependencyPaths(new_dependency_paths);
            size_t old_max_level_size = max_level.second.size();
            auto new_max_level = getMaxLevel(new_levels, th_largest_level);

            //std::cout << "break loop at agent " << agent_id << " success " << std::endl;
            //std::cout << "old_max_level = " << max_level.second << std::endl;
            //std::cout << "new_max_level = " << new_max_level.second << std::endl;
            std::cout << th_largest_level << " th largest level, " << iter_count << " iter, update: new/old max_level_size = " << new_max_level.second.size() << " / " << old_max_level_size << std::endl;

            // adopt update only when generate smaller subproblems
            if(new_max_level.second.size() < old_max_level_size) {
                all_dependency_paths_ = new_dependency_paths;
                all_levels_ = new_levels;
                return true;
            }
            return false;
        }

        // ahead_sequence store agent > another agent
        // later_sequence agent < another agent
        std::pair<std::map<int, std::set<int> >, std::map<int, std::set<int> > >
        getAheadAndLaterSequence(const std::map<int, std::set<int> >& all_agents_path) const {
            // 1, construct the graph about the early and later relationship between agents
            // ahead_sequence: first: agent / second: which agent is later than this
            // later_sequence: first: agent / second: which agent is earlier than this
            std::map<int, std::set<int> > ahead_sequence, later_sequence;

            for(const auto& temp_pair : all_agents_path) {
                const int& agent_id = temp_pair.first;
                const std::set<int>& related_sat = temp_pair.second;
                for(const int& sat : related_sat) {
                    // filter edge that connect itself
                    //if(agent_id == sat/2) { continue; }
                    if(ahead_sequence.find(agent_id) == ahead_sequence.end()) { ahead_sequence.insert({agent_id, {}}); }
                    if(ahead_sequence.find(sat/2) == ahead_sequence.end()) { ahead_sequence.insert({sat/2, {}}); }
                    if(later_sequence.find(agent_id) == later_sequence.end()) { later_sequence.insert({agent_id, {}}); }
                    if(later_sequence.find(sat/2) == later_sequence.end()) { later_sequence.insert({sat/2, {}}); }

                    if(sat%2 == 0) { // sat is a start
                        ahead_sequence[sat/2].insert(agent_id);
                        later_sequence[agent_id].insert(sat/2);
                    } else { // sat is a target
                        ahead_sequence[agent_id].insert(sat/2);
                        later_sequence[sat/2].insert(agent_id);
                    }
                }
            }
            return {ahead_sequence, later_sequence};
        }


        std::pair<std::vector<std::set<int> >, std::map<int, int>>
        getStrongComponentFromAheadSequence(const std::map<int, std::set<int> >& ahead_sequence) const {

            typedef boost::subgraph< boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS,
                    boost::property< boost::vertex_color_t, int>, boost::property< boost::edge_index_t, int> > > Graph;

            typedef boost::graph_traits<Graph>::vertex_iterator vertex_iter;

            // 1, transform all nodes in ahead_sequence to continuous int sequence
            std::vector<int> id_to_node_table;
            std::map<int, int> node_to_id_table;
            for(const auto& temp_pair : ahead_sequence) {
                node_to_id_table.insert({temp_pair.first, id_to_node_table.size()});
                id_to_node_table.push_back(temp_pair.first);
            }
            // 2, get all strong component in the graph
            Graph g;
            int count_of_node = 0;
            for(const auto& temp_pair : ahead_sequence) {
                const int& node = temp_pair.first;
                count_of_node ++;
                if(temp_pair.second.empty()) {
                    //std::cerr << " empty node " << node_to_id_table[node] << std::endl;
                } else {
                    for (const int &next_node : temp_pair.second) {
                        boost::add_edge(node_to_id_table[node], node_to_id_table[next_node], g);
                    }
                }
            }
            //std::cout << " count_of_node : " << count_of_node << std::endl;
            std::vector<int> comp(num_vertices(g));

            int num = boost::strong_components(g, comp.data());

            std::vector<Graph *> comps(num);
            for (size_t i = 0; i < num; ++i) {
                comps[i] = &g.create_subgraph();
            }

            std::map<int, int> agent_and_sub_graph; // agent and it's sub-graph id
            for (size_t i = 0; i < num_vertices(g); ++i) {
                //cout << "add vertex " << i << " to sub graph " << comp[i] << endl;
                add_vertex(i, *comps[comp[i]]);
                agent_and_sub_graph.insert({id_to_node_table[i], comp[i]});
            }

            // 3, transform to multiple level, but unsorted
            std::vector<std::set<int> > retv;
            for (size_t i = 0; i < num; i++) {
                std::set<int> sub_graph;
                std::pair<vertex_iter, vertex_iter> lvip;
                lvip = vertices(*comps[i]);
                for (vertex_iter vi = lvip.first; vi != lvip.second; ++vi) {
                    sub_graph.insert(id_to_node_table[ (*comps[i]).local_to_global(*vi) ]);
                }
                retv.push_back(sub_graph);
            }
            return {retv, agent_and_sub_graph};
        }


        // pass test
        LevelOrderGraph getLevelOrderGraph(std::map<int, std::set<int> >& all_paths,
                                           const Levels& levels) const {
            // 1, construct agent id to level id map
            std::map<int, int> agent_to_level_map;
            for(int i=0; i<levels.size(); i++) {
                for(const int& agent_id : levels[i]) {
                    agent_to_level_map[agent_id] = i;
                }
            }
            // 2, construct level order graph from all_paths
            std::vector<bool> base(levels.size(), false);
            LevelOrderGraph g(levels.size(), base);
            //std::cout << "LevelOrderGraph: ";
            for(int agent_id=0; agent_id<connect_graphs_.size(); agent_id++) {
                int current_level_id = agent_to_level_map[agent_id];
                const auto& dependency_path = all_paths[agent_id];
                for(const auto& sat : dependency_path) {
                    int another_level_id = agent_to_level_map[sat/2];
                    if(current_level_id != another_level_id) {
//                        std::cout << "sat % 2 = " << sat % 2 << " |";
                        if(sat % 2 == 0) {
                            g[another_level_id][current_level_id] = true;
                            //std::cout << "(1)lv " << another_level_id << " > lv " << current_level_id << " / ";
                            assert(another_level_id < current_level_id);
//                            if(another_level_id >= current_level_id) {
//                                std::cout << "error = " << another_level_id << ">=" << current_level_id << std::endl;
//                            }
                        } else {
                            g[current_level_id][another_level_id] = true;
                            //std::cout << "(2)lv " << current_level_id << " > lv " << another_level_id << " / ";
                            assert(current_level_id < another_level_id);
//                            if(current_level_id >= another_level_id) {
//                                std::cout << "error = " << current_level_id << ">=" << another_level_id << std::endl;
//                            }
                        }
                    }
                }
            }
            //std::cout << std::endl;
            return  g;
        }


         // return level order graph and all levels
        Levels getSortedLevelFromStrongComponent(const std::vector<std::set<int> >& all_strong_components,
                                                 const std::map<int, std::set<int> >& ahead_sequence,
                                                 const std::map<int, std::set<int> >& later_sequence) const {

            // the order get by by depth first traversal of all sub-graphs
            // store the sub-graph id of each node
            std::map<int, int> node_sub_graph_id;
            for(int i=0; i<all_strong_components.size(); i++) {
                for(const int& agent : all_strong_components[i]) {
                    node_sub_graph_id.insert({agent, i});
                }
            }
            // pick the root sub-graph from all sub-graph, which is not later than any external sub-graph

            std::vector<bool> sub_graph_root_check(all_strong_components.size(), true);
            for(int i=0; i<all_strong_components.size(); i++) {
                for(const int& agent : all_strong_components[i]) {
                    // if a level have a agent that not later than any other agent, it is a root level
                    for(const int& later_agent : later_sequence.at(agent)) {
                        if(all_strong_components[i].find(later_agent) == all_strong_components[i].end()) {
                            sub_graph_root_check[i] = false;
                            break;
                        }
                    }
                    // if have proof current level is a root level, no need to check further
                    if(sub_graph_root_check[i] == false) { break; }
                }
            }
            // set all root level as seeds, to traversal all level in a BFS mode
            std::set<int> root_sub_graphs;
            for(int i=0; i<sub_graph_root_check.size(); i++) {
                if(sub_graph_root_check[i]) { root_sub_graphs.insert(i); }
            }
            // storage each level's index
            std::vector<int> sub_graphs_level(all_strong_components.size(), 0);
            // start from root sub graph, get the order to traversal all sub-graph
            std::set<int> buffer_sub_graphs = root_sub_graphs;
            int max_level = 0;
            while(!buffer_sub_graphs.empty()) {
                std::set<int> next_sub_graphs;
                for(const int& current_sub_graph : buffer_sub_graphs) {
                    //std::cout << " sub_graph " << current_sub_graph << std::endl;
                    const std::set<int> current_sub_graph_agents = all_strong_components[current_sub_graph];
                    // traversal all agent current sub-graph
                    for(const int& agent : current_sub_graph_agents) {
                        // traversal all agent that later than this agent
                        for(const int& next_agent : ahead_sequence.at(agent)) {
                            // if belong to the same sub-graph, no need to update

                            if(node_sub_graph_id[agent] == node_sub_graph_id[next_agent]) { continue; }
                            //std::cout << "new sub_graph " << node_sub_graph_id[next_agent] << std::endl;

                            if(sub_graphs_level[current_sub_graph] >= sub_graphs_level[node_sub_graph_id[next_agent]]) {
                                // if current sub-graph find un-visited sub-graph
                                next_sub_graphs.insert(node_sub_graph_id[next_agent]);

                                sub_graphs_level[node_sub_graph_id[next_agent]] = sub_graphs_level[current_sub_graph] + 1;

                                max_level = std::max(max_level, sub_graphs_level[current_sub_graph] + 1);
                            }
                        }
                    }
                }
                if(!next_sub_graphs.empty()) {
                    //sorted_sub_graphs.push_back(next_sub_graphs);
                    std::swap(next_sub_graphs, buffer_sub_graphs);
                } else { break; }
            }

            std::vector<std::vector<std::set<int> > > sorted_sub_graphs(max_level + 1);
            for(int i=0; i<sub_graphs_level.size(); i++) {
                sorted_sub_graphs[ sub_graphs_level[i] ].push_back(all_strong_components[i]);
            }
            std::vector<std::set<int> > sorted_levels;
            sorted_levels.reserve(all_strong_components.size());
            for(const auto& levels : sorted_sub_graphs) {
                for(const auto& level : levels) {
                    sorted_levels.push_back(level);
                }
            }
            assert(all_strong_components.size() == sorted_levels.size());

            return sorted_levels; // sorted levels
        }

        // transform agents' id to 2*id and 2*id + 1
        std::vector<bool> AgentIdsToSATID(const std::set<int>& agent_ids) const {
            std::vector<bool> retv(2*connect_graphs_.size(), false);
            for(const int& agent_id : agent_ids) {
                retv[2*agent_id] = true;
                retv[2*agent_id + 1] = true;
            }
            return retv;
        }


        // return path that consists of agents
        // distinguish_sat whether consider start and target as one in calculate cost
        std::set<int> searchAgent(int agent_id,
                                  const std::vector<bool>& avoid_agents,
                                  const std::vector<bool>& passing_agents,
                                  bool distinguish_sat = false,
                                  const std::vector<bool>& ignore_cost_set = {}) const {
            assert(!heuristic_tables_sat_.empty());
            DependencyPathSearch<N, HyperNodeType> search_machine;
            /*
             * DependencyPathSearch::search(int agent_id,
                                            int start_hyper_node_id,
                                            const SubGraphOfAgent<N>& sub_graph,
                                            const ConnectivityGraph& con_graph,
                                            const std::vector<bool> &avoid_agents,
                                            const std::vector<bool> &passing_agents,
                                            const std::vector<int> heuristic_table,
                                            bool distinguish_sat = false,
                                            const std::vector<int> & ignore_cost_set = {}
                                             )
             * */
            assert(connect_graphs_[agent_id].start_hyper_node_ != MAX<size_t>);
            assert(connect_graphs_[agent_id].target_hyper_node_ != MAX<size_t>);

            return search_machine.search(agent_id,
                                         this->agent_sub_graphs_[agent_id].start_node_id_,
                                         this->agent_sub_graphs_[agent_id].target_node_id_,
                                         this->connect_graphs_[agent_id],
                                         avoid_agents, passing_agents,
                                         heuristic_tables_sat_[agent_id],
                                         true, ignore_cost_set);
        }

        // get all levels from all agent's dependency paths
        Levels getLevelsFromDependencyPaths(const std::map<int, std::set<int> >& all_dependency_paths)const {

            // 1, get relation between agents
            auto ahead_and_later_sequence = getAheadAndLaterSequence(all_dependency_paths);

            const auto& ahead_sequence  = ahead_and_later_sequence.first;
            const auto& later_sequence = ahead_and_later_sequence.second;

            // 2, get strong connected components in graph of agent's relavance
            const auto& retv = getStrongComponentFromAheadSequence(ahead_sequence);
            std::vector<std::set<int> > all_strong_components = retv.first;
            std::map<int, int> agent_and_sub_graph = retv.second;

            // 3, get sorted level from all strong components, after all strong connected components are determined
            const auto& sorted_level = getSortedLevelFromStrongComponent(all_strong_components, ahead_sequence, later_sequence);
            return sorted_level;
        };


        /* constant value during break loops */

        DimensionLength* dim_;

        std::vector<ConnectivityGraph> connect_graphs_;

        std::vector<SubGraphOfAgent<N> > agent_sub_graphs_;

        std::vector<std::vector<int> > heuristic_tables_sat_; // distinguish_sat = true

        std::set<int> all_agent_id_set_;

        std::vector<bool> cluster_buffer_sat_;

        double time_limit_; // when run out of time exit

        int max_break_count_; // when break reach max times, exit

        int max_continue_failure_; // when break keep failure many times, exit

        int expected_min_level_size_; // when max level size reach this value, do not decompose further

        clock_t start_t_;

        std::vector<AgentPtr<N> > agents_;

        /* variables during break loops */

        std::map<int, std::set<int> > all_dependency_paths_; // result of decomposition is determined by all_dependency_paths_

        std::vector<std::set<int> > all_levels_;

    };

}

#endif //LAYEREDMAPF_BREAK_LOOP_DECOMPOSITION_H
