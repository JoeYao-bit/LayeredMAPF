//
// Created by yaozhuo on 2024/8/14.
//

#ifndef LAYEREDMAPF_LARYERED_LARGE_AGENT_MAPF_H
#define LAYEREDMAPF_LARYERED_LARGE_AGENT_MAPF_H

#include "common.h"

#include "large_agent_instance_decomposition.h"
#include "../../algorithm/LA-MAPF/CBS/constraint.h"

namespace freeNav::LayeredMAPF::LA_MAPF {

    // current only considering methods that take external path as constraint, like LA-CBS
    template<Dimension N, typename AgentType>
    PosePaths<N> layeredLargeAgentMAPF(const InstanceOrients<N> & instances,
                                       const std::vector<AgentType>& agents,
                                       DimensionLength* dim,
                                       const IS_OCCUPIED_FUNC<N> & isoc,
                                       const LA_MAPF_FUNC<N, AgentType> & mapf_func,
                                       int cutoff_time = 60
                                       ) {

        struct timezone tz;
        struct timeval  tv_pre;
        struct timeval  tv_after;
        gettimeofday(&tv_pre, &tz);

        LargeAgentMAPFInstanceDecomposition<2, AgentType> decomposer(instances,
                                                                     agents,
                                                                     dim, isoc);
        gettimeofday(&tv_after, &tz);
        double decomposition_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
        std::cout << "-- decomposition take " << decomposition_cost << " ms to get "
                  << decomposer.all_clusters_.size() << " clusters " << std::endl;

        assert(decomposer.all_clusters_.size() >= 1);

        LargeAgentPathConstraintTablePtr<N, AgentType> layered_cts =
                std::make_shared<LargeAgentPathConstraintTable<N, AgentType> >(decomposer.all_poses_, agents, dim);

        std::vector<std::vector<std::pair<int, LAMAPF_Path> > > pathss;
        for(int i=0; i<decomposer.all_clusters_.size(); i++) {
            // instance_decompose->all_clusters_[i] to instances
            std::set<int> current_id_set = decomposer.all_clusters_[i];
            InstanceOrients<2> ists;
            for(const int& id : current_id_set) {
                ists.push_back({instances[id].first, instances[id].second});
            }
            // insert previous path as static constraint
            if (!pathss.empty()) {
                layered_cts->insertPaths(pathss.back());
            }
            // insert future agents' start as static constraint

            for(int j = i+1; j<decomposer.all_clusters_.size(); j++)
            {
                if(j == i) continue;
                const auto& current_cluster = decomposer.all_clusters_[j];
                for(const int& agent_id : current_cluster) {
//                    avoid_locs[instances[agent_id].first[1]][instances[agent_id].first[0]] = true;
                    layered_cts->insertPath({agent_id, {decomposer.instance_node_ids_[agent_id].first}});
                }
            }

            double remaining_time = cutoff_time - (tv_after.tv_sec - tv_pre.tv_sec) + (tv_after.tv_usec - tv_pre.tv_usec)/1e6;
            if(remaining_time < 0) {
                return {};
            }

            InstanceOrients<N> cluster_instances;
            std::vector<AgentType> cluster_agents;

            std::vector<int> current_id_vec;
            for(const auto& current_id : current_id_set) {
                current_id_vec.push_back(current_id);
                cluster_instances.push_back(instances[current_id]);
                cluster_agents.push_back(agents[current_id]);
            }

            gettimeofday(&tv_after, &tz);
            std::vector<LAMAPF_Path> next_paths = mapf_func(cluster_instances, cluster_agents, dim, isoc, layered_cts, remaining_time);
            if(next_paths.empty()) {
                std::cout << " layered MAPF failed " << i << " th cluster: ";
                for(const auto& id : current_id_set) {
                    std::cout << id << " ";
                }
                std::cout << std::endl;
                return {};
            }
            assert(next_paths.size() == current_id_set.size());
            std::vector<std::pair<int, LAMAPF_Path> > next_paths_with_id;
            for(int k=0; k<current_id_vec.size(); k++) {
                next_paths_with_id.push_back({current_id_vec[k], next_paths[k]});
            }
            pathss.push_back(next_paths_with_id);
        }

        PosePaths<N> retv(instances.size());
        for(const auto& cluster_paths : pathss) {
            for(const auto& path_with_id : cluster_paths) {
                for(const auto& pose_id : path_with_id.second) {
                    retv[path_with_id.first].push_back(*decomposer.all_poses_[pose_id]);
                }
            }
        }
        assert(instances.size() == retv.size());
//        if(!use_path_constraint) { retv = multiLayerCompress(dim, pathss); }
        std::cout << " layered large agent mapf success " << !retv.empty() << std::endl;
        return retv;
    }

}

#endif //LAYEREDMAPF_LARYERED_LARGE_AGENT_MAPF_H