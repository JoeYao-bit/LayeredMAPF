//
// Created by yaozhuo on 6/12/25.
//

#ifndef LAYEREDMAPF_NEIGHBORHOOD_SEARCH_DECOMPOSITION_H
#define LAYEREDMAPF_NEIGHBORHOOD_SEARCH_DECOMPOSITION_H


#include <sys/time.h>
#include <boost/graph/subgraph.hpp>
#include <boost/graph/adjacency_list.hpp>

#include <boost/config.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/strong_components.hpp>

#include "../LA-MAPF/common.h"
#include "../LA-MAPF/large_agent_dependency_path_search.h"

#include "../LA-MAPF/CBS/space_time_astar.h"

namespace freeNav::LayeredMAPF::LA_MAPF {

    // a general interfaces for both LA-MAPF and MAPF
    template<Dimension N>
    class MAPFInstanceDecompositionLNS {
    public:
        MAPFInstanceDecompositionLNS(DimensionLength* dim,
                                     const std::vector<ConnectivityGraph>& connectivity_graphs,
                                     const std::vector<SubGraphOfAgent<N> >& agent_sub_graphs,
                                     const std::vector<std::vector<int> >& heuristic_tables_sat, // distinguish_sat = true
                                     const std::vector<std::vector<int> >& heuristic_tables) :
                                     dim_(dim),
                                     agent_sub_graphs_(agent_sub_graphs),
                                     connect_graphs_(connectivity_graphs),
                                     heuristic_tables_sat_(heuristic_tables_sat),
                                     heuristic_tables_(heuristic_tables) {

            auto start_t = clock();

            std::set<int> all_agent_id_set;

            // 1, calculate heuristic table for each connectivity graph
            for (int i = 0; i < connectivity_graphs.size(); i++) {
//                heuristic_tables_.push_back(
//                        calculateLargeAgentHyperGraphStaticHeuristic<N>(i, dim_, connect_graphs_[i], false));
//                heuristic_tables_sat_.push_back(
//                        calculateLargeAgentHyperGraphStaticHeuristic<N>(i, dim_, connect_graphs_[i], true));
                all_agent_id_set.insert(i);
            }

            all_dependency_paths_.clear();
            std::vector<bool> cluster_buffer_sat = AgentIdsToSATID(all_agent_id_set);
            // 2, init all_dependency_paths_ with every agent's shortest path in connectivity_graphs
            for (int agent_id = 0; agent_id < connectivity_graphs.size(); agent_id++) {
                auto passing_start_and_targets = searchAgent(agent_id, {}, cluster_buffer_sat, true); // pass test
                all_dependency_paths_[agent_id] = passing_start_and_targets;
            }
            all_levels_ = getLevelsFromDependencyPaths(all_dependency_paths_);

            auto now_t = clock();

            double time_cost =  ((double)now_t - start_t)/CLOCKS_PER_SEC;;

            std::cout << "ns decomposition finish in " << time_cost << std::endl;
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


        std::vector<std::set<int> > getSortedLevelFromStrongComponent(const std::vector<std::set<int> >& all_strong_components,
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
            assert(!heuristic_tables_.empty() && !heuristic_tables_sat_.empty());
            DependencyPathSearch<N> search_machine;
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

            return search_machine.search(agent_id, connect_graphs_[agent_id].start_hyper_node_,
                                         this->agent_sub_graphs_[agent_id],
                                         this->connect_graphs_[agent_id],
                                         avoid_agents, passing_agents,
                                         distinguish_sat ? heuristic_tables_sat_[agent_id] : heuristic_tables_[agent_id],
                                         distinguish_sat, ignore_cost_set);
        }

        // get all levels from all agent's dependency paths
        std::vector<std::set<int> > getLevelsFromDependencyPaths(const std::map<int, std::set<int> >& all_dependency_paths) const {

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



        DimensionLength* dim_;

        std::vector<ConnectivityGraph> connect_graphs_;

        std::vector<SubGraphOfAgent<N> > agent_sub_graphs_;

        std::vector<std::vector<int> > heuristic_tables_sat_; // distinguish_sat = true

        std::vector<std::vector<int> > heuristic_tables_; // distinguish_sat = false

        std::map<int, std::set<int> > all_dependency_paths_; // result of decomposition is determined by all_dependency_paths_

        std::vector<std::set<int> > all_levels_;

    };

}

#endif //LAYEREDMAPF_NEIGHBORHOOD_SEARCH_DECOMPOSITION_H
