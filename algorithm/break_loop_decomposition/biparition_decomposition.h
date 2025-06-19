//
// Created by yaozhuo on 6/16/25.
//

#ifndef LAYEREDMAPF_BIPARITION_DECOMPOSITION_H
#define LAYEREDMAPF_BIPARITION_DECOMPOSITION_H

#include "../LA-MAPF/common.h"
#include "../LA-MAPF/large_agent_dependency_path_search.h"

#include "../algorithm/LA-MAPF/CBS/space_time_astar.h"
#include "../third_party/EECBS/inc/SpaceTimeAStar.h"

namespace freeNav::LayeredMAPF {

    // a general interfaces for both LA-MAPF and MAPF
    template<Dimension N, typename HyperNodeType>
    class MAPFInstanceDecompositionBipartition {
    public:

        MAPFInstanceDecompositionBipartition(DimensionLength* dim,
                                             const std::vector<LA_MAPF::ConnectivityGraph>& connectivity_graphs,
                                             const std::vector<LA_MAPF::SubGraphOfAgent<N> >& agent_sub_graphs,
                                             const std::vector<std::vector<int> >& heuristic_tables_sat,
                                             const std::vector<std::vector<int> >& heuristic_tables,
                                             double time_limit = 10) :
                                             agent_sub_graphs_(agent_sub_graphs),
                                             connect_graphs_(connectivity_graphs),
                                             heuristic_tables_sat_(heuristic_tables_sat),
                                             heuristic_tables_(heuristic_tables) {

            auto start_t = clock();

            for(int i=0; i<connectivity_graphs.size(); i++) {
                instance_id_set_.insert(i);
            }
            instanceDecomposition();

            auto now_t = clock();
            instance_decomposition_time_cost_ = 1e3*((double)now_t - start_t)/CLOCKS_PER_SEC;

            // debug
            for(const auto& level : all_clusters_) {
                for(const auto& agent_id : level) {
                    assert(agent_id < agent_sub_graphs_.size());
                }
            }

            start_t = clock();
            clusterDecomposition();
            now_t = clock();
            cluster_bipartition_time_cost_ = 1e3*((double)now_t - start_t)/CLOCKS_PER_SEC;

            // debug
            for(const auto& level : all_clusters_) {
                for(const auto& agent_id : level) {
                    assert(agent_id < agent_sub_graphs_.size());
                }
            }

            start_t = clock();
            levelSorting();
            now_t = clock();
            level_sorting_time_cost_ = 1e3*((double)now_t - start_t)/CLOCKS_PER_SEC;

            // debug
            for(const auto& level : all_clusters_) {
                for(const auto& agent_id : level) {
                    assert(agent_id < agent_sub_graphs_.size());
                }
            }

            start_t = clock();
            levelDecomposition();
            now_t = clock();
            level_bipartition_time_cost_ = 1e3*((double)now_t - start_t)/CLOCKS_PER_SEC;

            std::cout << "-- instance_decomposition_time_cost_ (ms) = " << instance_decomposition_time_cost_ << std::endl;
            std::cout << "-- cluster_bipartition_time_cost_    (ms) = " << cluster_bipartition_time_cost_ << std::endl;
            std::cout << "-- level_sorting_time_cost_          (ms) = " << level_sorting_time_cost_ << std::endl;
            std::cout << "-- level_bipartition_time_cost_      (ms) = " << level_bipartition_time_cost_ << std::endl;

            // debug
            for(const auto& level : all_clusters_) {
                for(const auto& agent_id : level) {
                    assert(agent_id < agent_sub_graphs_.size());
                }
            }

        }


        std::map<int, std::set<int> > searchUnAvoidSATForEachAgent(const std::map<int, std::set<int> >& all_passing_agent,
                                                                   const std::set<int>& instance_id_set,
                                                                   const std::set<int>& external_pre,
                                                                   const std::set<int>& external_next) const {
            std::map<int, std::set<int> > all_unavoidable_agent;
            std::vector<bool> within_set = AgentIdsToSATID(instance_id_set);
            // set external_pre's start to passable
            for(const auto& agent : external_pre) {
                within_set[2*agent] = true;
            }
            // set external_next's target to passable
            for(const auto& agent : external_next) {
                within_set[2*agent + 1] = true;
            }

            for(const auto& id_agent_pair : all_passing_agent) {
                const int& agent_id = id_agent_pair.first;
//                std::cout << __FUNCTION__ << " agent_id = " << agent_id << std::endl;
                const std::set<int>& agent_path = id_agent_pair.second;

                for(const int& other_sat : agent_path) {
                    if(all_unavoidable_agent.find(agent_id) == all_unavoidable_agent.end()) {
                        all_unavoidable_agent.insert(std::pair<int, std::set<int> >(agent_id, {2*agent_id, 2*agent_id + 1}));
                    }

                    if(other_sat == 2*agent_id || other_sat == 2*agent_id + 1) { continue; }

                    std::vector<bool> avoid_list(2*this->connect_graphs_.size(), false);
                    avoid_list[other_sat] = true;
//                    std::cout << __FUNCTION__ << " try avoid  " << other_agent << std::endl;

                    if((searchAgent(agent_id, avoid_list, within_set)).empty()) {
                        all_unavoidable_agent.at(agent_id).insert(other_sat);
                        //std::cout << " failed " << std::endl;
                    } else {
                        //std::cout << " success " << std::endl;
                    }
                }
            }
            return all_unavoidable_agent;
        }


        void setTargetSATToUnpassable(std::vector<bool>& avoid_table, const std::set<int>& avoid_set) {
            assert(avoid_table.size() == 2*this->connect_graphs_.size());
            for(const auto& agent : avoid_set) {
                avoid_table[2*agent + 1] = true;
            }
        }

        void setStartSATToUnpassable(std::vector<bool>& avoid_table, const std::set<int>& avoid_set) {
            assert(avoid_table.size() == 2*this->connect_graphs_.size());
            for(const auto& agent : avoid_set) {
                avoid_table[2*agent] = true;
            }
        }

        // try decompose a level into two level, and the first level should be as small as possible
        // the combination of the three input is a cluster
        std::pair<std::set<int>, std::set<int> > biPartitionLevel(const std::set<int>& agents,
                                                                  const std::set<int>& external_pre,
                                                                  const std::set<int>& external_next) {
            // get initial paths
            std::vector<bool> cluster_buffer_sat = AgentIdsToSATID(agents);
            // set external_pre's start to passable
            for(const auto& agent : external_pre) {
                cluster_buffer_sat[2*agent] = true;
            }
            // set external_next's target to passable
            for(const auto& agent : external_next) {
                cluster_buffer_sat[2*agent + 1] = true;
            }
            std::map<int, std::set<int> > all_agents_path;
            for(const int& agent_id : agents) {
                auto passing_agents = searchAgent(agent_id, {}, cluster_buffer_sat, true); // pass test
                assert(!passing_agents.empty());
                all_agents_path.insert({agent_id, passing_agents});
            }
            // get each set's unavoidable start and target (of other agent)
            std::map<int, std::set<int> > all_agent_and_unavoid_sat = searchUnAvoidSATForEachAgent(all_agents_path,
                                                                                                   agents,
                                                                                                   external_pre,
                                                                                                   external_next);
            // find the agent that have the fewest start dependency and related agent as seed of previous level
            int global_max_target_count_ = 0, local_max_target_count_;
            int global_max_target_count_agent_ = all_agent_and_unavoid_sat.begin()->first;
            for(const auto& temp_pair : all_agent_and_unavoid_sat) {
                local_max_target_count_ = 0;
                // count number of start
                for(const auto& sat : temp_pair.second) {
                    if(sat%2 == 1) {
                        local_max_target_count_ ++;
                    }
                }
                if(local_max_target_count_ > global_max_target_count_) {
                    global_max_target_count_ = local_max_target_count_;
                    global_max_target_count_agent_ = temp_pair.first;
                }
            }
            // start from the global_min_start_count_agent_, get pre_level
            // get what not in pre_level as next_level
            std::set<int> pre_level = {global_max_target_count_agent_}, next_level = agents;
            next_level.erase(global_max_target_count_agent_);
            std::vector<int> buffer = {global_max_target_count_agent_}, next_buffer;
//            std::cout << "global_min_target_count_agent_ " << global_min_target_count_agent_ << std::endl;
            while(!buffer.empty()) {
                next_buffer.clear();
                for(const auto& agent : buffer) {
                    // as pre_level, it can't pass next_level's start,
                    // so move all it need to pass start as pre_level
                    for(const auto& sat : all_agent_and_unavoid_sat.at(agent)) {
                        // do not add sat from agent not in current cluster
                        if(agents.find(sat/2) == agents.end()) { continue; }
                        if(sat % 2 == 0 && pre_level.find(sat/2) == pre_level.end()) {
                            pre_level.insert(sat/2);
                            next_level.erase(sat/2);

                            next_buffer.push_back(sat/2);

//                            std::cout << "pre_level = " << pre_level << std::endl;
//                            std::cout << "cluster_level = " << next_level << std::endl;

                        }
                    }
                }
                std::swap(next_buffer, buffer);
            }
            assert(agents.size() == pre_level.size() +  next_level.size());
            // if have no room for next level, return
            if(next_level.empty()) { return {agents, {}}; }

            // move agent from next_level to pre_level, util both of them are independent or no room for next_level
            std::pair<std::set<int>, std::set<int> > level_pair = {pre_level, next_level};
            int count_of_phase = 0;
//            std::cout << "flag 1" << std::endl;
            while(1) {
                assert(agents.size() == level_pair.first.size() + level_pair.second.size());
                int count_of_first_step = 0;
                while (1) {
//                    std::cout << "flag 2" << std::endl;
                    count_of_first_step ++;
                    std::vector<bool> local_avoid_set(2*this->connect_graphs_.size(), false);
                    setTargetSATToUnpassable(local_avoid_set, level_pair.first);
                    auto retv = levelIndependentCheck(level_pair.second, cluster_buffer_sat, local_avoid_set);
                    std::set<int> keep_in_next_level = retv.first, move_to_pre_level = retv.second;
                    // move agent that cannot stay in remaining set to unavoidable set
                    for (const int &moved_agent_id : move_to_pre_level) {
                        level_pair.first.insert(moved_agent_id);
                        level_pair.second.erase(moved_agent_id);
                    }
                    assert(agents.size() == level_pair.first.size() + level_pair.second.size());
                    // move agent to remaining set to unavoidable cluster
                    if (move_to_pre_level.empty()) {
                        break;
                    }
                }
//                if(count_of_first_step > 1) {
//                    std::cout << "biPartitionLevel step 1 exit after " << count_of_first_step << std::endl;
//                }
                int count_of_second_step = 0;
                // till here, the remaining set is independent, it related no external agent to keep completeness
//                std::cout << "flag 3" << std::endl;

                while (1) {
//                    std::cout << "flag 4" << std::endl;
//                    std::cout << "level.first = " << level_pair.first << " / level.second = " << level_pair.second << std::endl;
                    count_of_second_step ++;
                    // when add new agent to unavoidable set, only check new added agent, to save time
                    std::vector<bool> local_avoid_set(2*this->connect_graphs_.size(), false);
                    setStartSATToUnpassable(local_avoid_set, level_pair.second);

                    auto retv = levelIndependentCheck(level_pair.first, cluster_buffer_sat, local_avoid_set);

                    std::set<int> success_in_pre = retv.first, failed_in_pre = retv.second;
//                    std::cout << "failed_in_pre = " << failed_in_pre << std::endl;
                    if (failed_in_pre.empty()) {
                        break;
                    }
                    // pick the shortest failed path's agent
                    int failed_shortest_agent_id;
                    int shortest_path_size = MAX<int>;
                    for (const int &failed_agent : failed_in_pre) {
                        if (shortest_path_size > all_agents_path.at(failed_agent).size()) {
                            shortest_path_size = all_agents_path.at(failed_agent).size();
                            failed_shortest_agent_id = failed_agent;
                        }
                    }
                    // add all related agent of the shortest (containing fewer agents) path into unavoidable set
                    // there are multiple solution path, pick one with least modification to unavoid set, i.e., involve less new agent
                    //const auto& alternative_path = searchAgent(failed_shortest_agent_id, {}, AgentIdsToSATID(agents), false, AgentIdsToSATID(cluster_pair.first));
                    const auto& alternative_path = all_agents_path.at(failed_shortest_agent_id);
//                    std::cout << "alternative_path = " << alternative_path << std::endl;
                    assert(!alternative_path.empty());
                    for (const int &new_sat_to_unavoid : alternative_path) {
                        // do not add sat from agent not in current cluster
                        if(agents.find(new_sat_to_unavoid/2) == agents.end()) { continue; }
                        if(level_pair.first.find(new_sat_to_unavoid/2) == level_pair.first.end()) {
                            level_pair.first.insert(new_sat_to_unavoid/2);
                            level_pair.second.erase(new_sat_to_unavoid/2);
                        }
                    }
                    assert(agents.size() == level_pair.first.size() + level_pair.second.size());
                    count_of_second_step ++;

                }
//                if(count_of_second_step > 1) {
//                    std::cout << "biPartitionLevel step 2 exit after " << count_of_second_step << std::endl;
//                }
                std::vector<bool> local_avoid_set_pre(2*this->connect_graphs_.size(), false);
                setStartSATToUnpassable(local_avoid_set_pre, level_pair.second);

                std::vector<bool> local_avoid_set_next(2*this->connect_graphs_.size(), false);
                setTargetSATToUnpassable(local_avoid_set_next, level_pair.first);

                bool pre_level_independent = isLevelIndependentCheck(level_pair.first,  cluster_buffer_sat, local_avoid_set_pre),
                        nex_level_independent = isLevelIndependentCheck(level_pair.second, cluster_buffer_sat, local_avoid_set_next);

                count_of_phase ++;
                // if both remaining set and unavoid set is independent, we find a legal bi-partition
                if(pre_level_independent && nex_level_independent) {
                    break;
                }

            }
//            if(count_of_phase > 1) {
//                std::cout << "biPartitionLevel exit after " << count_of_phase << std::endl;
//            }
            assert(agents.size() == level_pair.first.size() + level_pair.second.size());
            return level_pair;
        }


        // check whether current set is self-relevant, the first is self-relevant, the second is which agent depend external agent
        std::pair<std::set<int>, std::set<int>> levelIndependentCheck(const std::set<int>& level_agent_set,
                                                                      const std::vector<bool>& cluster_sat_sat,
                                                                      const std::vector<bool>& avoid_sat_set,
                                                                      const std::set<int>& specific_agents = {}
        ) {
            std::set<int> success_in_set, failed_in_set;

            auto cur_level_agent_set = specific_agents.empty()? level_agent_set : specific_agents;

            for (const auto &agent : cur_level_agent_set) {
                auto passing_agents = searchAgent(agent, avoid_sat_set, cluster_sat_sat); // pass test
                if (passing_agents.empty()) {
                    // failed always caused by cluster in remaining set that block the whole way
                    failed_in_set.insert(agent);
                } else {
                    success_in_set.insert(agent);
                }
            }
            return {success_in_set, failed_in_set};
        }

        // (decompose a level (old level) into two) and check whether the two level are complete
        bool isLevelIndependentCheck(const std::set<int>& level,
                                     const std::vector<bool>& cluster_sat_sat,
                                     const std::vector<bool>& avoid_sat_set) {
            // check pre level's independence
            for(const auto& agent : level) {
                auto passing_agents = searchAgent(agent, avoid_sat_set, cluster_sat_sat, true);
                if(passing_agents.empty()) {
                    return false;
                }
            }
            return true;
        }

        void levelDecomposition() {
            std::vector<std::set<int> > all_clusters;
            auto cluster_of_agents = all_clusters_;
            int count_top_cluster = 0;
            std::set<int> buffer_agents;
            for(int i=0; i<cluster_of_agents.size(); i++) {
                const auto& top_cluster = cluster_of_agents[i];
                assert(top_cluster.size() >= 1);
                if(top_cluster.size() == 1) {
                    // add small clusters at this stage to all_clusters, no need to join further bi-partition
                    all_clusters.push_back(top_cluster);
                } else {
                    count_top_cluster ++;
                    int count = 0;
                    buffer_agents = top_cluster;
                    std::set<int> external_pre = {}, external_next = {};
                    if(!all_level_pre_and_next_.empty()) {
                        assert(all_level_pre_and_next_.size() == all_clusters_.size());
                        external_pre.insert(all_level_pre_and_next_[i].first.begin(), all_level_pre_and_next_[i].first.end());
                        external_next.insert(all_level_pre_and_next_[i].second.begin(), all_level_pre_and_next_[i].second.end());
                    }
                    // bi-partition until can not bi-partition
                    //if(count_top_cluster == 1)
                    {
                        while (buffer_agents.size() > 1) {
//                            std::cout << "start biPartition of level: " << buffer_agents << std::endl;
                            auto agents_pair = biPartitionLevel(buffer_agents, external_pre, external_next);
                            std::swap(buffer_agents, agents_pair.second);
                            all_clusters.push_back(agents_pair.first);
                            external_pre.insert(agents_pair.first.begin(), agents_pair.first.end());
                            count++;
                        }
                    }
                    if (!buffer_agents.empty()) {
                        all_clusters.push_back(buffer_agents);
                    }
                }
            }
            // no sorting in increase size, as order of agent matters
            all_clusters_ = all_clusters;

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
                    //if(count_top_cluster == 1)
                    {
                        while (buffer_agents.size() > 1) {
                            auto agents_pair = biPartitionCluster(buffer_agents);
                            std::swap(buffer_agents, agents_pair.second);
                            all_clusters.push_back(agents_pair.first);
                            count++;
                        }
                    }
                    if (!buffer_agents.empty()) {
                        all_clusters.push_back(buffer_agents);
                    }
                }
            }
            // sorting in increase size, to enable large cluster have fewer external path constraint
            std::sort(all_clusters.begin(), all_clusters.end(), [](std::set<int> x,std::set<int> y){return x.size()>y.size();});
            all_clusters_ = all_clusters;

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
                assert(!passing_agents.empty());
                all_agents_path.insert({agent_id, passing_agents});
            }
//            std::cout << "all_agents_path 0: " << std::endl;
//            for(const auto& pair : all_agents_path) {
//                std::cout << "agent " << pair.first << ": ";
//                for(const auto& id : pair.second) {
//                    std::cout << id << " ";
//                }
//                std::cout << std::endl;
//            }

            // determine agent's unavoidable agent
            std::map<int, std::set<int> > all_unavoidable_agent = searchUnAvoidAgentForEachAgent(all_agents_path, agents);
//            std::cout << "all_unavoidable_agent: " << std::endl;
//            for(const auto& pair : all_unavoidable_agent) {
//                std::cout << "agent " << pair.first << ": ";
//                for(const auto& id : pair.second) {
//                    std::cout << id << " ";
//                }
//                std::cout << std::endl;
//            }

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
                    // when add new agent to unavoidable set, only check new added agent, to save time
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
                    assert(!alternative_path.empty());
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

        // check whether current set is self-relevant, the first is self-relevant, the second is which agent depend external agent
        std::pair<std::set<int>, std::set<int>> clusterIndependentCheck(const std::set<int>& instance_id_set,
                                                                        const std::set<int>& specific_agents = {}) {
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
//                std::cout << __FUNCTION__ << " agent_id = " << agent_id << std::endl;
                const std::set<int>& agent_path = id_agent_pair.second;

                for(const int& other_agent : agent_path) {
                    if(all_unavoidable_agent.find(agent_id) == all_unavoidable_agent.end()) {
                        all_unavoidable_agent.insert(std::pair<int, std::set<int> >(agent_id, {agent_id}));
                    }

                    if(other_agent == agent_id) { continue; }

                    std::vector<bool> avoid_list(2*this->connect_graphs_.size(), false);
                    avoid_list[other_agent*2] = true, avoid_list[other_agent*2 + 1] = true;
//                    std::cout << __FUNCTION__ << " try avoid  " << other_agent << std::endl;

                    if((searchAgent(agent_id, avoid_list, within_set, distinguish_sat)).empty()) {
                        all_unavoidable_agent.at(agent_id).insert(other_agent);
                        //std::cout << " failed " << std::endl;
                    } else {
                        //std::cout << " success " << std::endl;
                    }
                }
            }
            return all_unavoidable_agent;
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
            }

//            std::cout << "all_agents_path: " << std::endl;
//            for(const auto& pair : all_agents_path) {
//                std::cout << "agent " << pair.first << ": ";
//                for(const auto& id : pair.second) {
//                    std::cout << id << " ";
//                }
//                std::cout << std::endl;
//            }

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

        void levelSorting() {
            // decompose each cluster into multiple time indexed sequence
            // cluster decomposition into level
            all_level_pre_and_next_.clear();
            all_level_pre_and_next_.reserve(connect_graphs_.size()); // maximum number of levels
            std::vector<std::set<int> > all_levels_;
            for(const auto& cluster : all_clusters_) {
                if(cluster.size() > 1) {
                    auto current_levels = clusterDecomposeToLevel(cluster);
                    all_levels_.insert(all_levels_.end(), current_levels.begin(), current_levels.end());
                    for(int i=0; i<current_levels.size(); i++) {
                        std::pair<std::set<int>, std::set<int> > pre_and_next;
                        for(int j=0; j<current_levels.size(); j++) {
                            if(i == j) { continue; }
                            else if(j < i) {
                                // insert level in same cluster and early than current level as pre_level
                                pre_and_next.first.insert(current_levels[j].begin(), current_levels[j].end());
                            } else {
                                // insert level in same cluster and later than current level as next_level
                                pre_and_next.second.insert(current_levels[j].begin(), current_levels[j].end());
                            }
                        }
                        all_level_pre_and_next_.push_back(pre_and_next);
                    }
                } else {
                    all_levels_.push_back(cluster);
                    std::pair<std::set<int>, std::set<int> > pre_and_next;
                    all_level_pre_and_next_.push_back(pre_and_next);
                }
            }
            all_clusters_ = all_levels_;
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

        // transform agents' id to 2*id and 2*id + 1
        std::vector<bool> AgentIdsToSATID(const std::set<int>& agent_ids) const {
            std::vector<bool> retv(2*this->connect_graphs_.size(), false);
            for(const int& agent_id : agent_ids) {
                retv[2*agent_id] = true;
                retv[2*agent_id + 1] = true;
            }
            return retv;
        }

        // an instance is split into multiple un-realted cluster
        void instanceDecomposition() {

            // decompose the whole instance to multiple unrelated cluster
            std::set<int> buffer_agents = instance_id_set_; // all agents
            std::vector<std::set<int> > all_clusters;

            // get top level isolated clusters
            std::map<int, std::set<int> > all_agents_path;
            std::vector<bool> buffer_sat = AgentIdsToSATID(buffer_agents);
            for(const int& agent_id : buffer_agents) {
                auto passing_agents = searchAgent(agent_id, {}, buffer_sat); // pass test
                assert(!passing_agents.empty());
                all_agents_path.insert({agent_id, passing_agents});
            }
            all_passing_agent_ = all_agents_path;
            // get each agent's dependence agents
            std::map<int, std::set<int> > all_related_agent = updateRelatedGraphFromPassingGraph(all_agents_path);
            std::map<int, std::set<int> > cluster_of_agents = clusterAgents(all_related_agent);
            all_clusters_.clear();
            for(const auto& iter : cluster_of_agents) {
                all_clusters_.push_back(iter.second);
            }
            std::sort(all_clusters_.begin(), all_clusters_.end(),[](std::set<int> x,std::set<int> y){return x.size()>y.size();});
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
                                    assert(0);
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

        // return path that consists of agents
        // distinguish_sat whether consider start and target as one in calculate cost
        std::set<int> searchAgent(int agent_id,
                                  const std::vector<bool>& avoid_agents,
                                  const std::vector<bool>& passing_agents,
                                  bool distinguish_sat = false,
                                  const std::vector<bool>& ignore_cost_set = {}) const {
            assert(!heuristic_tables_.empty() && !heuristic_tables_sat_.empty());
            LA_MAPF::DependencyPathSearch<N, HyperNodeType> search_machine;
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
                                         distinguish_sat ? heuristic_tables_sat_[agent_id] : heuristic_tables_[agent_id],
                                         distinguish_sat, ignore_cost_set);
        }

        // store which agents current agent passing, may change after method
        // NOTICE: the goal of the method is to partition this matrix into lots of small block, thus MAPF is more efficient
        std::map<int, std::set<int> > all_passing_agent_;

        std::vector<std::set<int> > all_clusters_;

        std::set<int> instance_id_set_; // set of all agent's id

        std::vector<LA_MAPF::ConnectivityGraph> connect_graphs_;

        std::vector<std::vector<int> > heuristic_tables_sat_; // distinguish_sat = true

        std::vector<std::vector<int> > heuristic_tables_; // distinguish_sat = false

        std::vector<LA_MAPF::SubGraphOfAgent<N> > agent_sub_graphs_;

        // save each level's pre level and next level (belong to the same cluster) after clusterDecomposeToLevel(...)
        std::vector<std::pair<std::set<int>, std::set<int> > > all_level_pre_and_next_;


        double time_limit_; // TODO: when run out of time, stop and return current decomposition result

        float instance_decomposition_time_cost_ = 0;
        float cluster_bipartition_time_cost_    = 0;
        float level_sorting_time_cost_          = 0;
        float level_bipartition_time_cost_      = 0;


    };





}

#endif //LAYEREDMAPF_BIPARITION_DECOMPOSITION_H
