//
// Created by yaozhuo on 2023/10/12.
//

#ifndef FREENAV_LAYERED_MAPF_INSTANCE_DECOMPOSITION_H
#define FREENAV_LAYERED_MAPF_INSTANCE_DECOMPOSITION_H

#include "basic.h"
#include "dependency_path_search.h"
#include <fstream>
#include <sys/time.h>
#include <boost/graph/subgraph.hpp>
#include <boost/graph/adjacency_list.hpp>

#include <boost/config.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/strong_components.hpp>



namespace freeNav::LayeredMAPF {

    template<Dimension N>
    struct HyperGraphNode;

    template <Dimension N>
    struct GridMAPF {
        // -1 for no agent's start and target in the target
        // the agent that in that grid, if %=2, is start of agent agent_id_/2, otherwise target of agent agent_id_/2
        // in a way 1s1t, 2s2t, 3s3t,...
        // assume there is no overlap start and target
        int agent_id_ = -1;

        // belong to free grid group or agent's start and target
        // if is occupied, always be nullptr
        //HyperGraphNodePtr hyper_node_ptr_ = nullptr;

        int hyper_node_id_ = -1;

        // id of current grid, PointiToId
        int id_;

        Pointi<N> pt_;

        bool is_occupied_;
    };

    template<Dimension N>
    using GridMAPFPtr = GridMAPF<N>*;

    template<Dimension N>
    struct GridMAPFGroup {
        std::vector<GridMAPFPtr<N> > free_grids_;

    };

    template<Dimension N>
    struct HyperGraphNode {

        // free_grid_group_ and agent_grid_ptr_ has one and only one is nullptr

        GridMAPFGroup<N> free_grid_group_; //if not empty, this is a free grid group, rather than a agent's start or target

        GridMAPFPtr<N> agent_grid_ptr_; // if nullptr, this is of agent's start and target

        std::set<int> connecting_nodes_set_;// other nodes that connect to current node

        std::vector<int> connecting_nodes_;// other nodes that connect to current node

        int hyper_node_id_ = -1;

    };

    std::ostream & operator<<(std::ostream& os, const std::set<int>& agent_ids);

    std::string toString(const std::set<int>& agent_ids);

    //
    //       1，define a general interfaces for multiple MAPF problem
    //       2，test how can decomposition accelerate various of MAPF, under different config
    // TODO
    //       3，then consider how set the order of planning within cluster
    //       4, test how can decomposition and ordering can accelerate various of MAPF, under different config
    template<Dimension N>
    class MAPFInstanceDecomposition {
    public:
        // decompose_level = 0: no decomposition
        //                 = 1: decompose instance to initial cluster
        //                 = 2: decompose initial cluster to smaller cluster
        //                 = 3: decompose cluster into smaller sort level
        MAPFInstanceDecomposition(const Instances<N>& instance, DimensionLength* dim, const IS_OCCUPIED_FUNC<N>& is_occupied, int decompose_level=3) {
            assert(decompose_level >= 0 && decompose_level <= 3);
            struct timezone tz;
            struct timeval  tv_pre;
            struct timeval  tv_after;
            gettimeofday(&tv_pre, &tz);

            dimen_ = dim;
            instance_ = instance;
            for(int i=0; i<instance_.size(); i++) {
                instance_id_set_.insert(i);
            }
            isoc_ = is_occupied;
            all_clusters_= { instance_id_set_ }; // initialize of cluster (equal to the raw cluster)

            if(decompose_level >= 1) {
                // initialize grid map
                initializeGridMap();
                detectFreeGroup();
                // establish connection between hyper nodes (between free group and agent instance (start and target), and between agent instance)
                establishConnectionOfHyperNode();
                establishHeuristicTable();
                instanceDecomposition();
                gettimeofday(&tv_after, &tz);
                instance_decomposition_time_cost_ =
                        (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3;
            }

            if(decompose_level >= 2) {
                gettimeofday(&tv_pre, &tz);
                clusterDecomposition();
                gettimeofday(&tv_after, &tz);
                cluster_decomposition_time_cost_ =
                        (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3;
            }

            if(decompose_level >= 3) {
                gettimeofday(&tv_pre, &tz);
                levelSorting();
                gettimeofday(&tv_after, &tz);
                sort_level_time_cost_ =
                        (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3;
            }
            std::cout << "-- instance_decomposition_time_cost_ (ms) = " << instance_decomposition_time_cost_ << std::endl;
            std::cout << "-- cluster_decomposition_time_cost_  (ms) = " << cluster_decomposition_time_cost_ << std::endl;
            std::cout << "-- sort_level_time_cost_             (ms) = " << sort_level_time_cost_ << std::endl;


            /* print details of decomposition */
            int total_count = 0;
            int max_cluster_size = 0;
            for(int i=0; i<all_clusters_.size(); i++) {
                total_count += all_clusters_[i].size();
                if(all_clusters_[i].size() > max_cluster_size) { max_cluster_size = all_clusters_[i].size(); }
            }
            assert(total_count == instance.size());
            std::cout << "-- Decomposition completeness ? " << decompositionValidCheck(all_clusters_) << std::endl;
            std::cout << " max/total size " << max_cluster_size << " / " << instance.size() << std::endl;
        }


        size_t getMaximalSubProblem() const {
            size_t vec_size = 0;
            for(const auto& vec : all_clusters_) {
                vec_size = std::max(vec.size(), vec_size);
            }
            return vec_size;
        }

        size_t getNumberOfSubProblem() const {
            return all_clusters_.size();
        }

        bool decompositionValidCheck(const std::vector<std::set<int> >& all_levels) const {
            for(int i=0; i<all_levels.size(); i++) {
                std::vector<bool> avoid_sats(2*instance_.size(), false);
                for(int j=0; j<all_levels.size(); j++) {
                    if(i == j) { continue; }
                    if(i > j) {
                        for(const int& pre_agent : all_levels[j]) { avoid_sats[2*pre_agent + 1] = true; }
                    } else {
                        for(const int& after_agent : all_levels[j]) { avoid_sats[2*after_agent] = true; }
                    }
                }
                for(const int& agent : all_levels[i]) {
                    auto passing_agents = searchAgent(agent, avoid_sats, {}, true);
                    if(passing_agents.empty()) {
                        std::cout << "ERROR: cluster " << i << ", agent " << agent << " is im-complete" << std::endl;
                        return false;
                    }
                }
            }
            return true;
        }

        HyperGraphNodePtrs<N> all_hyper_nodes_;

        std::vector<GridMAPFPtr<N> > grid_map_;

        std::vector<std::set<int> > all_clusters_;

        // store which agents current agent passing, may change after method
        // NOTICE: the goal of the method is to partition this matrix into lots of small block, thus MAPF is more efficient
        std::map<int, std::set<int> > all_passing_agent_;

        double instance_decomposition_time_cost_ = 0.,
               cluster_decomposition_time_cost_ = 0.,
               sort_level_time_cost_ = 0.;

        ~ MAPFInstanceDecomposition() {
            releaseData();
        }

        std::vector<std::vector<int> > all_heuristic_table_sat_;

    private:



        void releaseData() {
            for(auto& grid_ptr : grid_map_) {
                delete grid_ptr;
                grid_ptr = nullptr;
            }
            for(auto& hyper_node_ptr : all_hyper_nodes_) {
                delete hyper_node_ptr;
                hyper_node_ptr = nullptr;
            }
        }

        // transform agents' id to 2*id and 2*id + 1
        std::vector<bool> AgentIdsToSATID(const std::set<int>& agent_ids) const {
            std::vector<bool> retv(2*instance_.size(), false);
            for(const int& agent_id : agent_ids) {
                retv[2*agent_id] = true;
                retv[2*agent_id + 1] = true;
            }
            return retv;
        }

        bool isClusterIndependent(const std::set<int>& instance_id_set, const std::set<int>& specific_agents = {}) {
            auto set_need_check = specific_agents.empty() ? instance_id_set : specific_agents;
            std::vector<bool> buffer_sat = AgentIdsToSATID(instance_id_set);
            for (const auto &unavoid_agent : set_need_check) {
                auto passing_agents = searchAgent(unavoid_agent, {}, buffer_sat); // pass test
                if (passing_agents.empty()) {
                    // failed always caused by cluster in remaining set that block the whole way
                    return false;
                }
            }
            return true;
        }

        // check whether current set is self-relevant, the first is self-relevant, the second is which agent depend external agent
        std::pair<std::set<int>, std::set<int>> clusterIndependentCheck(const std::set<int>& instance_id_set, const std::set<int>& specific_agents = {}) {
            std::set<int> success_in_set, failed_in_set;
            auto set_need_check = specific_agents.empty() ? instance_id_set : specific_agents;
            std::vector<bool> buffer_sat = AgentIdsToSATID(instance_id_set);
            for (const auto &unavoid_agent : set_need_check) {
                auto passing_agents = searchAgent(unavoid_agent, {}, buffer_sat); // pass test
                if (passing_agents.empty()) {
                    // failed always caused by cluster in remaining set that block the whole way
                    failed_in_set.insert(unavoid_agent);
                } else {
                    success_in_set.insert(unavoid_agent);
                }
            }
            return {success_in_set, failed_in_set};
        }

        // search what agents is unavoidable for each agent
        // any legal pass agents graph contain all unavoidable agent
        // input: instance_id_set: all agent in current cluster
        //        all_passing_agent: all path of current cluster
        std::map<int, std::set<int> > searchUnAvoidAgentForEachAgent(
                const std::map<int, std::set<int> >& all_passing_agent, const std::set<int>& instance_id_set, bool distinguish_sat = false) const {
            std::map<int, std::set<int> > all_unavoidable_agent;
            std::vector<bool> within_set = AgentIdsToSATID(instance_id_set);
            for(const auto& id_agent_pair : all_passing_agent) {
                const int& agent_id = id_agent_pair.first;
                const std::set<int>& agent_path = id_agent_pair.second;
                for(const int& other_agent : agent_path) {

                    if(all_unavoidable_agent.find(agent_id) == all_unavoidable_agent.end()) {
                        all_unavoidable_agent.insert(std::pair<int, std::set<int> >(agent_id, {}));
                    }
                    std::vector<bool> avoid_list(2*instance_.size(), false);
                    avoid_list[other_agent*2] = true, avoid_list[other_agent*2 + 1] = true;
                    if((searchAgent(agent_id, avoid_list, within_set, distinguish_sat)).empty()) {
                        all_unavoidable_agent.at(agent_id).insert(other_agent);
                    }
                }
            }
            return all_unavoidable_agent;
        }

        // 1, the input agent set must be solvable
        // 2, split current instance set into to a set (based on largest unavoidable agent's cluster), and remaining agents as the other set

        // 1, divide current instance into unavoidable cluster, in other words, what agents must be MAPF together
        // 2, pick the largest unavoidable cluster as base, check it and remaining cluster whether belong to two separating
        // 3, if can, return to step 1 and input remaining agents
        // 4, if can't, move agents from remaining to the largest unavoidable cluster, until it can
        std::pair<std::set<int>, std::set<int> > biPartitionCluster(const std::set<int>& agents) {
            // upper bound of cluster size
            std::map<int, std::set<int> > all_agents_path;
            std::vector<bool> buffer_sat = AgentIdsToSATID(agents);
            for(const int& agent_id : agents) {
                auto passing_agents = searchAgent(agent_id, {}, buffer_sat); // pass test
                all_agents_path.insert({agent_id, passing_agents});
            }
            // determine agent's unavoidable agent
            std::map<int, std::set<int> > all_unavoidable_agent = searchUnAvoidAgentForEachAgent(all_agents_path, agents);

            // lower bound of cluster size
            // determine each agent's related agent (unavoidable)
            std::map<int, std::set<int> > all_unavoidable_related_agent = updateRelatedGraphFromPassingGraph(all_unavoidable_agent);
            //
            std::map<int, std::set<int> > cluster_of_unavoidable_agents = clusterAgents(all_unavoidable_related_agent);

            // get the largest cluster and merge the remaining into a secondary set
            auto cluster_pair = splitPartitionCluster(cluster_of_unavoidable_agents);

            int count_of_phase = 0;
            while(1) {
                while (1) {
                    auto retv = clusterIndependentCheck(cluster_pair.second);
                    std::set<int> keep_in_remaining = retv.first, move_to_unavoid = retv.second;
                    // move agent that cannot stay in remaining set to unavoidable set
                    for (const int &moved_agent_id : move_to_unavoid) {
                        cluster_pair.first.insert(moved_agent_id);
                        cluster_pair.second.erase(moved_agent_id);
                    }
                    // move agent to remaining set to unavoidable cluster
                    if (move_to_unavoid.empty()) { break; }
                }
                // till here, the remaining set is independent, it related no external agent to keep completeness
                std::set<int> specific_set_unavoidable = {};
                while (1) {
                    // when add new agent to unavoidable set, only check new added agent, to save time cost
                    auto retv = clusterIndependentCheck(cluster_pair.first, specific_set_unavoidable);
                    specific_set_unavoidable.clear();
                    std::set<int> success_in_unavoid = retv.first, failed_in_unavoid = retv.second;
                    if (failed_in_unavoid.empty()) {
                        break;
                    }
                    // pick the shortest failed path's agent
                    int failed_shortest_agent_id;
                    int shortest_path_size = MAX<int>;
                    for (const int &failed_agent : failed_in_unavoid) {
                        if (shortest_path_size > all_agents_path.at(failed_agent).size()) {
                            shortest_path_size = all_agents_path.at(failed_agent).size();
                            failed_shortest_agent_id = failed_agent;
                        }
                    }
                    // add all related agent of the shortest (containing fewer agents) path into unavoidable set
                    // there are multiple solution path, pick one with least modification to unavoid set, i.e., involve less new agent
                    //const auto& alternative_path = searchAgent(failed_shortest_agent_id, {}, AgentIdsToSATID(agents), false, AgentIdsToSATID(cluster_pair.first));
                    const auto& alternative_path = all_agents_path.at(failed_shortest_agent_id);
                    for (const int &new_agent_to_unavoid : alternative_path) {
                        if(cluster_pair.first.find(new_agent_to_unavoid) == cluster_pair.first.end()) {
                            cluster_pair.first.insert(new_agent_to_unavoid);
                            cluster_pair.second.erase(new_agent_to_unavoid);
                            specific_set_unavoidable.insert(new_agent_to_unavoid);
                        }
                    }
                }

                bool unavoid_independent = isClusterIndependent(cluster_pair.first, specific_set_unavoidable),
                     remaining_independent = isClusterIndependent(cluster_pair.second);
                count_of_phase ++;
                // if both remaining set and unavoid set is independent, we find a legal bi-partition
                if(unavoid_independent && remaining_independent) {
                    break;
                }

            }
            return cluster_pair;
        }

        // an instance is split into multiple un-realted cluster
        void instanceDecomposition() {

            // decompose the whole instance to multiple unrelated cluster

            std::set<int> buffer_agents = instance_id_set_;
            std::vector<std::set<int> > all_clusters;

            // get top level isolated clusters
            std::map<int, std::set<int> > all_agents_path;
            std::vector<bool> buffer_sat = AgentIdsToSATID(buffer_agents);
            for(const int& agent_id : buffer_agents) {
                auto passing_agents = searchAgent(agent_id, {}, buffer_sat); // pass test
                all_agents_path.insert({agent_id, passing_agents});
            }
            all_passing_agent_ = all_agents_path;
            // get each agent's dependence agents
            std::map<int, std::set<int> > all_related_agent = updateRelatedGraphFromPassingGraph(all_agents_path);
            //
            std::map<int, std::set<int> > cluster_of_agents = clusterAgents(all_related_agent);
            all_clusters_.clear();
            for(const auto& iter : cluster_of_agents) {
                all_clusters_.push_back(iter.second);
            }
        }

        void clusterDecomposition() {
            std::vector<std::set<int> > all_clusters;
            auto cluster_of_agents = all_clusters_;
            int count_top_cluster = 0;
            std::set<int> buffer_agents;
            for(const auto& top_cluster : cluster_of_agents) {
                if(top_cluster.size() < 2) {
                    // add small clusters at this stage to all_clusters, no need to join further bi-partition
                    all_clusters.push_back(top_cluster);
                } else {
                    count_top_cluster ++;
                    int count = 0;
                    buffer_agents = top_cluster;
                    // bi-partition until can not bi-partition
                    while (buffer_agents.size() > 1) {
                        auto agents_pair = biPartitionCluster(buffer_agents);
                        std::swap(buffer_agents, agents_pair.second);
                        all_clusters.push_back(agents_pair.first);
                        count++;
                    }
                    if (!buffer_agents.empty()) {
                        all_clusters.push_back(buffer_agents);
                    }
                }
            }
            all_clusters_ = all_clusters;
        }

        void levelSorting() {
            // decompose each cluster into multiple time indexed sequence
            // cluster decomposition into level
            std::vector<std::set<int> > all_levels_;
            for(const auto& cluster : all_clusters_) {
                if(cluster.size() > 1) {
                    auto current_levels = clusterDecomposeToLevel(cluster);
                    all_levels_.insert(all_levels_.end(), current_levels.begin(), current_levels.end());
                } else {
                    all_levels_.push_back(cluster);
                }
            }
            all_clusters_ = all_levels_;
        }

        std::set<int> getCurrentAgentLoopInPaths(const std::map<int, std::set<int> >& all_agents_path, const int& agent_id) const {
            auto ahead_and_later_sequence = getAheadAndLaterSequence(all_agents_path);

            const auto& ahead_sequence  = ahead_and_later_sequence.first;
            const auto& later_sequence = ahead_and_later_sequence.second;
            const auto& retv = getStrongComponentFromAheadSequence(ahead_sequence);
            std::vector<std::set<int> > all_strong_components = retv.first;
            std::map<int, int> agent_and_sub_graph = retv.second;
            return all_strong_components[agent_and_sub_graph[agent_id]];
        }

        // get the sat in sat_path that caused the loop
        std::set<int> getIntersectionBetweenLoopAndPath(const int& agent_id, const std::set<int>& sat_path, const std::set<int>& loop) const {
            std::set<int> common_sats;
            for(const int& sat : sat_path) {
                // no need to avoid itself
                if(sat/2 == agent_id) { continue; }
                if(loop.find(sat/2) != loop.end()) {
                    common_sats.insert(sat);
                }
            }
            return common_sats;
        }

        // a cluster is split into multiple time indexed level
        // each level may have one or multiple agent
        // by update sat path of agents
        std::vector<std::set<int> >  clusterDecomposeToLevel(const std::set<int>& cluster, bool active_loop_avoidance = false) const {
            // 1, get each agent's sat path
            std::map<int, std::set<int> > all_agents_path;
            // search path which length is in an increasing order
            // provide more room to avoid large loops
            std::vector<std::pair<int, int>> sat_path_length_and_agent;

            std::vector<bool> cluster_sat = AgentIdsToSATID(cluster);
            for(const int& agent_id : cluster) {
                auto passing_sats = searchAgent(agent_id, {}, cluster_sat, true); // pass test
                assert(!passing_sats.empty());
                //std::cout << agent_id << " agent passing_sats " << passing_sats << std::endl;
                sat_path_length_and_agent.push_back({agent_id, passing_sats.size()});
            }

            // if >, path is in decrease order, if <, path length in increase order
            // TODO: it's still un clear that which way is better, massive test is need ...
            std::sort(sat_path_length_and_agent.begin(), sat_path_length_and_agent.end(),
                      [=](std::pair<int, int>& a, std::pair<int, int>& b) { return a.second > b.second; });

#define SORTED 1
#if SORTED
            for(const auto& temp_pair : sat_path_length_and_agent) {
                const int& agent_id = temp_pair.first;
#else
            for(const auto& agent_id : cluster) {
#endif
                auto passing_sats = searchAgent(agent_id, {}, cluster_sat, true); // pass test
                assert(!passing_sats.empty());

                // add agent's sat path in an incremental way
                all_agents_path.insert({agent_id, passing_sats});

                // if current agent is in loop, try to find alternative path that can avoid loop
#define TRY_AVOID_LOOP 0
#if TRY_AVOID_LOOP
                std::vector<bool> avoid_sats(2*instance_.size(), false);
                //int count=0;
                while(1) {
                    std::set<int> current_loop = getCurrentAgentLoopInPaths(all_agents_path, agent_id);
//                    std::cout << " current loop " << current_loop << std::endl;
                    if(current_loop.size() == 1) { break; }
                    // get the intersection between current agent's sat path and
                    std::set<int> intersected_sat = getIntersectionBetweenLoopAndPath(agent_id, all_agents_path[agent_id], current_loop);
                    for(const auto& sat : intersected_sat) {
                        avoid_sats[sat] = true;
                    }
//                    avoid_sats.insert(intersected_sat.begin(), intersected_sat.end());
//                    std::cout << " agent " << agent_id << " intersect with sat " << intersected_sat << std::endl;
//                    std::cout << " agent " << agent_id << " avoid sat " << avoid_sats << std::endl;
                    if(intersected_sat.empty()) {
//                        std::cout << " agent " << agent_id << " path = " << all_agents_path[agent_id] << std::endl;
//                        std::cout << " agent " << agent_id << " intersect WITHOUT sat " << std::endl;
                        break;
                    }
                    //break;
                    auto alternative_passing_sats = searchAgent(agent_id, avoid_sats, cluster_sat, true); // pass test
                    // if find no legal path, exit, mean this agent will form a new loop
                    if(alternative_passing_sats.empty()) { break; }
                    all_agents_path[agent_id] = alternative_passing_sats;
                    //count ++;
                }
                //if(count >= 1) { std::cout << " loop count " << count << std::endl; } // count may > 1
#endif
            }

            // 2, get all strong components
            auto ahead_and_later_sequence = getAheadAndLaterSequence(all_agents_path);

            const auto& ahead_sequence  = ahead_and_later_sequence.first;
            const auto& later_sequence = ahead_and_later_sequence.second;
            const auto& retv = getStrongComponentFromAheadSequence(ahead_sequence);
            std::vector<std::set<int> > all_strong_components = retv.first;
            std::map<int, int> agent_and_sub_graph = retv.second;

            // 3, get sorted level from all strong components, after all strong components are determined
            const auto& sorted_level = getSortedLevelFromStrongComponent(all_strong_components, ahead_sequence, later_sequence);

            return sorted_level;
        }

        /* unfinished algorithm */
        std::vector<std::vector<int> > levelDecomposition(const std::vector<std::set<int> >& levels) {
            for(int i=0; i<levels.size(); i++) {
                // construct available set
                std::vector<bool> passing_sat(2*instance_.size(), false);
                for(int j=0; j<levels.size(); j++) {
                    if(j == i) { continue; }
                    if(j > i) {
                        for(const auto& pre_agent : levels[j]) {
                            // previous sub-problems start is passable
                            passing_sat[2*pre_agent] = true;
                        }
                    } else {
                        for(const auto& later_agent : levels[j]) {
                            // later sub-problems target is passable
                            passing_sat[2*later_agent + 1] = true;
                        }
                    }
                }
                //
            }
            return {};
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

        typedef boost::subgraph< boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS,
                boost::property< boost::vertex_color_t, int>, boost::property< boost::edge_index_t, int> > > Graph;

        typedef boost::graph_traits<Graph>::vertex_iterator vertex_iter;

        std::pair<std::vector<std::set<int> >, std::map<int, int>>
            getStrongComponentFromAheadSequence(const std::map<int, std::set<int> >& ahead_sequence) const {
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


        // when try to split one cluster into multiple sub-clusters,
        // considering all other cluster's agent as avoidance, avoid increases of queue size involve other agent
        // pick out the largest cluster to decompose (decrease agent in the cluster), if success, repeat
        // otherwise pick the second largest cluster to decompose, until there is no cluster can be decompose
        // # the size of clusters is in a power law (幂律分布), the biggest one have lots of agents, while the end have a few agents
        // # select which edge (relation of two agents) to break (candidate), is generated by graph partitioning (edge partitioning)
        // # graph partitioning try to split a graph into multiple same size sub-graph, meets the requirement of efficient MAPF
        // find multiple sub-graph that connect only by one edge and try to break it

        // get each agent's related agents
        std::map<int, std::set<int> > updateRelatedGraphFromPassingGraph(const std::map<int, std::set<int> >& ref) const {
            std::map<int, std::set<int> > red_related;
            red_related = ref;
            for(const auto& agent : ref) {
                for(const auto& other_agent : ref) {
                    if(agent.first == other_agent.first) {
                        continue;
                    }
                    // if agent j cross agent i
                    if(ref.at(other_agent.first).find(agent.first) != ref.at(other_agent.first).end()) {
                        red_related.at(agent.first).insert(other_agent.first);
                    }
                }
            }
            return red_related;
        }

        std::map<int, std::set<int> > clusterAgents(const std::map<int, std::set<int> >& ref_related) const {
            std::map<int, std::set<int> > cluster_of_agents;// multiple cluster
            std::map<int, int> cluster_id_of_agents; // cluster id of each agent
            for(const auto& pair: ref_related) {
                cluster_id_of_agents.insert({pair.first, MAX<int>});
            }
            for(const auto& pair : ref_related) {
                if(cluster_id_of_agents[pair.first] != MAX<int>) {
                    continue;
                }
                // while loop exit when current cluster have no new member
                std::set<int> current_set = {pair.first};
                std::set<int> buffer_set  = {pair.first}, next_buffer_set;
                cluster_id_of_agents.at(pair.first)   = cluster_of_agents.size();
                while(!buffer_set.empty()) {
                    next_buffer_set.clear();
                    for(const int& new_agent_id : buffer_set) {
                        std::set<int> relative_agents = ref_related.at(new_agent_id);
                        for(const int& depended_agent_id : relative_agents) {
                            if(cluster_id_of_agents.at(depended_agent_id) != MAX<int>) {
                                if(cluster_id_of_agents.at(depended_agent_id) != cluster_of_agents.size()) {
                                    // what belong is an error condition
                                    std::cerr << " different cluster intersect " <<
                                    cluster_id_of_agents.at(depended_agent_id) << " and " << cluster_of_agents.size()
                                            << std::endl;
                                }
                            } else{
                                cluster_id_of_agents.at(depended_agent_id) = cluster_of_agents.size();
                                current_set.insert(depended_agent_id);
                                next_buffer_set.insert(depended_agent_id);
                            }
                        }
                    }
                    std::swap(buffer_set, next_buffer_set);
                }
                cluster_of_agents.insert({cluster_of_agents.size(), current_set});
            }
            return cluster_of_agents;
        }

        std::set<int> selectLargestCluster(const std::map<int, std::set<int> >& clusters) const {
            std::set<int> max_cluster;
            for(const auto& cluster : clusters) {
                if(max_cluster.size() < cluster.second.size()) {
                    max_cluster = cluster.second;
                }
            }
            return max_cluster;
        }

        // select the largest cluster and merge remaining agent into one cluster and return
        std::pair<std::set<int>, std::set<int> > splitPartitionCluster(const std::map<int, std::set<int> >& clusters) const {
            std::set<int> max_cluster;
            std::set<int> remaining_clusters;
            for(const auto& cluster : clusters) {
                if(max_cluster.size() < cluster.second.size()) {
                    remaining_clusters.insert(max_cluster.begin(), max_cluster.end());
                    max_cluster = cluster.second;
                } else {
                    remaining_clusters.insert(cluster.second.begin(), cluster.second.end());
                }
            }
            return {max_cluster, remaining_clusters};
        }

        // return path that consists of agents
        // distinguish_sat whether consider start and target as one in calculate cost
        std::set<int> searchAgent(int agent_id,
                                  const std::vector<bool>& avoid_agents,
                                  const std::vector<bool>& passing_agents,
                                  bool distinguish_sat = false,
                                  const std::vector<bool>& ignore_cost_set = {}) const {
            // test search pass agent search
            Pointi<N> start_pt = instance_[agent_id].first;
            int start_hyper_node = grid_map_[PointiToId(start_pt, dimen_)]->hyper_node_id_;
            assert(!all_heuristic_table_sat_.empty());
            HyperGraphNodePathSearch<N> search_machine;
            return search_machine.search(start_hyper_node, all_hyper_nodes_, avoid_agents, passing_agents, all_heuristic_table_sat_[agent_id], distinguish_sat, ignore_cost_set);
        }

        /* function about initialize of Layered MAPF */
        void initializeGridMap() {
            Id total_index = getTotalIndexOfSpace<N>(dimen_);
            grid_map_.resize(total_index, nullptr);
            for(Id id=0; id<total_index; id++) {
                Pointi<N> pt = IdToPointi<N>(id, dimen_);
                grid_map_[id] = new GridMAPF<N>();
                grid_map_[id]->agent_id_ = -1;
                grid_map_[id]->id_ = id;
                grid_map_[id]->is_occupied_ = isoc_(pt);
            }
            // set agent's start and target
            for(int i=0; i<instance_.size(); i++) {
                const auto& ist = instance_[i];
                Id start_id = PointiToId(ist.first, dimen_), target_id = PointiToId(ist.second, dimen_);
                if(grid_map_[start_id]->agent_id_ != -1) {
                    std::cout << " overlap start/target " << ist.first << std::endl;
                    exit(0);
                }
                if(grid_map_[target_id]->agent_id_ != -1) {
                    std::cout << " overlap start/target " << ist.second << std::endl;
                    exit(0);
                }
                grid_map_[start_id]->agent_id_  = i*2;
                grid_map_[target_id]->agent_id_ = i*2 + 1;
            }
        }

        void detectFreeGroup() {
            // 1, add agent's start and target as group
            for(int i=0; i<instance_.size(); i++) {
                const auto& ist = instance_[i];
                Id start_id = PointiToId(ist.first, dimen_), target_id = PointiToId(ist.second, dimen_);
                auto new_group_of_start = new HyperGraphNode<N>();
                new_group_of_start->agent_grid_ptr_  = grid_map_[start_id];
                auto new_group_of_target = new HyperGraphNode<N>();
                new_group_of_target->agent_grid_ptr_ = grid_map_[target_id];

                grid_map_[start_id]->hyper_node_id_  = all_hyper_nodes_.size();
                new_group_of_start->hyper_node_id_  = all_hyper_nodes_.size();
                all_hyper_nodes_.push_back(new_group_of_start);

                grid_map_[target_id]->hyper_node_id_ = all_hyper_nodes_.size();
                new_group_of_target->hyper_node_id_  = all_hyper_nodes_.size();
                all_hyper_nodes_.push_back(new_group_of_target);

            }
            //std::cout << " add " << all_hyper_nodes_.size() << " hyper nodes of instance " << std::endl;
            // 2, allocate free grids into group and
            int free_groups = 0;
            auto neighbor_offsets = GetNearestOffsetGrids<N>();
            for(int id=0; id<grid_map_.size(); id++) {
                Pointi<N> pt = IdToPointi<N>(id, dimen_);
                if(!isoc_(pt) && grid_map_[id]->agent_id_ == -1 && grid_map_[id]->hyper_node_id_ < 0) {
                    // if found an isolate free grid, start a new free group
                    auto new_group = new HyperGraphNode<N>();
                    grid_map_[id]->hyper_node_id_ = all_hyper_nodes_.size();
                    new_group->free_grid_group_.free_grids_.push_back(grid_map_[id]);

                    std::vector<Pointi<N> > buffer = {pt};
                    // traversal all connecting free grid into current group
                    while(!buffer.empty()) {
                        std::vector<Pointi<N> > next_buffer;
                        for(const auto& buff_pt : buffer) {
                            for(const auto& offset : neighbor_offsets) {
                                Pointi<N> new_pt = buff_pt + offset;
                                // if is occupied
                                if(isoc_(new_pt)) { continue; }
                                Id new_id = PointiToId(new_pt, dimen_);
                                // if is on agent's start or target or other free group
                                if(grid_map_[new_id]->hyper_node_id_ >= 0) { continue; }
                                next_buffer.push_back(new_pt);
                                grid_map_[new_id]->hyper_node_id_ = all_hyper_nodes_.size();
                                new_group->free_grid_group_.free_grids_.push_back(grid_map_[new_id]);
                            }
                        }
                        std::swap(buffer, next_buffer);
                    }
                    free_groups ++;
                    new_group->hyper_node_id_ = all_hyper_nodes_.size();
                    all_hyper_nodes_.push_back(new_group);
                }
            }
        }

        void updateConnectionToNearbyHyperNode(const Pointi<N>& pt) {
            Id id = PointiToId(pt, dimen_);
            if(grid_map_[id]->agent_id_ < 0) {
                std::cout << " this is not a hyper node of agent " << std::endl;
                return;
            }
            auto& current_hyper_node = all_hyper_nodes_[grid_map_[id]->hyper_node_id_];
            auto neighbor_offsets = GetNearestOffsetGrids<N>();
            for(const auto& offset : neighbor_offsets) {
                Pointi<N> new_pt = pt + offset;
                if(isoc_(new_pt)) { continue; }
                Id new_id = PointiToId(new_pt, dimen_);
                if(grid_map_[new_id]->hyper_node_id_ < 0) { continue; }
                // if is near to another hyper node
                auto retv_pair1 = current_hyper_node->connecting_nodes_set_.insert(grid_map_[new_id]->hyper_node_id_);
                if(retv_pair1.second) {
                    current_hyper_node->connecting_nodes_.push_back(grid_map_[new_id]->hyper_node_id_);
                }
                if(grid_map_[new_id]->agent_id_ < 0) {
                    // if this is a free group
                    auto retv_pair2 = all_hyper_nodes_[grid_map_[new_id]->hyper_node_id_]->connecting_nodes_set_.insert(grid_map_[id]->hyper_node_id_);
                    if(retv_pair2.second) {
                        all_hyper_nodes_[grid_map_[new_id]->hyper_node_id_]->connecting_nodes_.push_back(grid_map_[id]->hyper_node_id_);
                    }
                }
            }
        }

        void establishConnectionOfHyperNode() {
            if(instance_.empty()) { return; }
            if(all_hyper_nodes_.empty()) { return; }
            if(all_hyper_nodes_.size() < 2*instance_.size()) {
                std::cout << " hyper node's size < 2*instance.size() " << std::endl;
            }
            // NOTICE: the first 2*instance_.size() node is hyper node of instance
            for(int i=0; i < instance_.size(); i++) {
                updateConnectionToNearbyHyperNode(instance_[i].first);
                updateConnectionToNearbyHyperNode(instance_[i].second);
            }
            // clear std::set in hyper node
            for(const auto& node_ref : all_hyper_nodes_) {
                node_ref->connecting_nodes_set_.clear();
            }
        }

        void establishHeuristicTable() {
            auto& all_heuristic_table = all_heuristic_table_sat_;
            all_heuristic_table.clear();
            for(const auto& start_and_target : instance_) {
                Pointi<N> target = start_and_target.second;
                auto target_id = PointiToId(target, dimen_);
                auto hyper_graph_node_of_target = all_hyper_nodes_[grid_map_[target_id]->hyper_node_id_];
                const std::vector<int> heuristic_table = calculateHyperGraphStaticHeuristic(hyper_graph_node_of_target, all_hyper_nodes_);
                all_heuristic_table.push_back(heuristic_table);
            }
            //std::cout << " finish " << __FUNCTION__ << std::endl;
        }

        void serializeAllPassingAgent() {
            std::ofstream osf;
            osf.open("/home/yaozhuo/code/free-nav/scripts/passing_graph.txt", std::ios_base::out|std::ios::trunc);
            for(int i=0; i<all_passing_agent_.size(); i++) {
                osf << i << " ";
                for(const auto& id : all_passing_agent_[i]) {
                    if(i == id) { continue; }
                    osf << id << " ";
                }
                osf << std::endl;
            }
            osf.close();
        }

        DimensionLength* dimen_;

        IS_OCCUPIED_FUNC<N> isoc_;

        Instances<N> instance_;

        std::set<int> instance_id_set_;

    };


    template<Dimension N>
    using MAPFInstanceDecompositionPtr = std::shared_ptr<MAPFInstanceDecomposition<N> >;

    int runPython(const std::vector<std::set<int> >& agent_dependency_graph);

}
#endif //FREENAV_INSTANCE_DECOMPOSITION_H
