//
// Created by yaozhuo on 2023/12/6.
//

#ifndef FREENAV_LAYERED_MAPF_LAYERED_MAPF_H
#define FREENAV_LAYERED_MAPF_LAYERED_MAPF_H

#include "basic.h"
#include "instance_decomposition.h"
//#include "../third_party/EECBS/inc/ConstraintTable.h"
#include "constraint_table_CBS/ConstraintTable.h"
#include "freeNav-base/basic_elements/misc.h"

//#include "../third_party/EECBS/inc/Instance.h"
//#include "EECBS/inc/driver.h"

// EECBS: run success and transferred
// PBS: run success (not complete to find all path)
// Rolling-Horizon Collision Resolution (RHCR): run success

// IDCBS Disjoint/non-disjoint (E)CBSH and ID(E)CBS: integrated with LPA* and other incremental techniques

// considering a general interfaces for multi-agent path finding, ought to be complete to find path

// download Push-and-Rotate--CBS--PrioritizedPlanning from GitHub, look like EECBS
// download LaCAM from GitHub, and run successful, seems faster than EECBS,

// The increasing cost tree search for optimal multi-agent pathfinding, ICTS, rocketman1243/IncreasingCostTreeSearch


// LaCAM and LaCAM-2 come from https://github.com/Kei18/lacam

// MAPF-LNS2, from https://github.com/Jiaoyang-Li/MAPF-LNS2, MAPF-LNS2: Fast Repairing for Multi-Agent Path Finding
//via Large Neighborhood Search

// PIBT_2: https://github.com/Kei18/pibt2/

// layered method list:
// EECBS,     PBS,         CBSH2-RTC,   LaCAM,
// MAPF-LNS2, LaCAM2,      PIBT,        PIBT_2,
// HCA,       PushAndSwap, AnytimeBCBS, AnytimeEECBS

namespace freeNav::LayeredMAPF {

    // return the path have been modified to avoid conflict with lasted_occupied_time_table
    // synchronous is easy but too long, current way
    // asynchronous is not easy to implement, but too
    template <Dimension N>
    Paths<N> SingleLayerCompress(DimensionLength* dim, std::vector<int>& lasted_occupied_time_table, const Paths<N>& paths) {
        int t=1;
        std::vector<bool> reach_states(paths.size(), false);
        Paths<N> modified_paths(paths);
        while(1) {
            // check whether current proceed
            bool proceed = true, all_finished = true;
            for(auto& path : modified_paths) {
                // check whether finished
                if(t > path.size() - 1) { continue; }
                else { all_finished = false; }
                // if not finish, check whether proceed
                Id temp_id = PointiToId(path[t], dim);
                if(lasted_occupied_time_table[temp_id] >= t) {
                    proceed = false;
                    break;
                }
            }
            if(all_finished) { break; }
            if(proceed) {
                // if proceed, update lasted_occupied_time_table
                for(auto& path : modified_paths) {
                    // check whether finished
                    if(t > path.size() - 1) { continue; }
                    // if not finish, check whether proceed
                    Id temp_id = PointiToId(path[t], dim);
                    lasted_occupied_time_table[temp_id] = t;
                }
            } else {
                // traversal all path, wait at previous frame
                for(auto& path : modified_paths) {
                    if(t-1 > path.size()-1) { continue; }
                    path.insert(path.begin() + t-1, path[t-1]);
                    Id temp_id = PointiToId(path[t-1], dim);
                    lasted_occupied_time_table[temp_id] = t;
                }
            }
            t++;
        }
        return modified_paths;
    }

    template <Dimension N>
    Paths<N> multiLayerCompress(DimensionLength* dim, const std::vector<Paths<N> >& pathss) {
        if(pathss.size() == 1) { return pathss[0]; }
        else if(pathss.empty()) { return {}; }
        Id total_index = getTotalIndexOfSpace<N>(dim);
        // store the last occupied time index, update dynamically

        std::vector<int> lasted_occupied_time_table(total_index, -1);
        // initialize the lasted_occupied_time_table
        for(const auto& path : pathss[0]) {
            for(int i=0; i<path.size(); i++) {
                Id temp_id = PointiToId(path[i], dim);
                lasted_occupied_time_table[temp_id] = std::max(lasted_occupied_time_table[temp_id], i);
            }
        }
        // delay other path to avoid conflict with previous
        Paths<N> mergedPaths = pathss[0];
        for(int i=1; i<pathss.size(); i++) {
            Paths<N> delayed_paths = SingleLayerCompress(dim, lasted_occupied_time_table, pathss[i]);
            mergedPaths.insert(mergedPaths.end(), delayed_paths.begin(), delayed_paths.end());
        }
        return mergedPaths;
    }

//    struct ConflictInfo {
//        int wait_t_; // time index when wait can avoid conflict
//        int delay_count_; // how many step it needs wait to avoid conflict
//    };

//    template<Dimension N>
//    int getDiffPrePointIndex(const Path<N>& path, int start) {
//        assert(start <= path.size());
//        for(int i=start-1; i>=0; i--) {
//            if(path[i] != path[start])
//                return i;
//        }
//        //std::cout << "ERROR: find no diff point" << std::endl;
//        return MAX<int>;
//    }


    // two paths, the first are planed earlier than the second
//    template <Dimension N>
//    ConflictInfo getMinConflictTimeBetweenTwoPath(const Path<N>& p1, const Path<N>& p2, int begin_time_step) {
//        // yz: check conflicts in common part
//        const auto& longer_path  = p1.size() > p2.size() ? p1 : p2;
//        const auto& shorter_path = p1.size() > p2.size() ? p2 : p1;
//
//        int min_path_length = shorter_path.size();
//        // yz: determine constraint of current hyper node in increasing time order
//
//        for (int timestep = begin_time_step; timestep < min_path_length; timestep++) {
//            const auto& loc1 = p1[timestep], loc2 = p2[timestep];
//            // yz: check whether existing node conflict
//            if (loc1 == loc2) {
//                int diff_index_2 = getDiffPrePointIndex(p2, timestep);
//                ConflictInfo retv{diff_index_2, timestep - diff_index_2};
//                return retv;
//            }
//            // yz: check whether existing edge conflict
//            else if (timestep < min_path_length - 1
//                     && loc1 == p2[timestep + 1]
//                     && loc2 == p1[timestep + 1]) {
//
//                int diff_index_2 = getDiffPrePointIndex(p2, timestep);
//                ConflictInfo retv{diff_index_2, timestep - diff_index_2 + 1};
//                return retv;
//            }
//        }
//        // yz: check conflict that not in common part
//        if (longer_path.size() != shorter_path.size()) {
//            const auto& loc1 = shorter_path.back();
//            for (int timestep = std::max(min_path_length, begin_time_step);
//                 timestep < (int) longer_path.size();
//                 timestep++) {
//                Pointi<N> loc2 = longer_path[timestep];
//                if (loc1 == loc2) {
//                    assert(p1.size() > p2.size()); // ensured by decomposition of mapf instance
//                    int diff_index_2 = getDiffPrePointIndex(p2, p2.size()-1);
//                    ConflictInfo retv{diff_index_2, timestep - diff_index_2};
//                    return retv;
//                }
//            }
//        }
//        return ConflictInfo{MAX<int>, MAX<int>};
//    }

    // two group of paths, the first are planed earlier than the second
//    template <Dimension N>
//    ConflictInfo getMinConflictTimeBetweenTwoGroupOfPaths(const Paths<N>& paths_1, const Paths<N>& paths_2, int begin_time_step) {
//        ConflictInfo min_conflict_time_step = {MAX<int>, MAX<int>};
//        for(int i=0; i<paths_1.size(); i++) {
//            for(int j=0; j<paths_2.size(); j++) {
//                ConflictInfo local_min_conf_time_step = getMinConflictTimeBetweenTwoPath(paths_1[i], paths_2[j], begin_time_step);
//                if(local_min_conf_time_step.wait_t_ < min_conflict_time_step.wait_t_) {
//                    min_conflict_time_step = local_min_conf_time_step;
//                } else if(local_min_conf_time_step.wait_t_ == min_conflict_time_step.wait_t_) {
//                    if(local_min_conf_time_step.delay_count_ > min_conflict_time_step.delay_count_) {
//                        min_conflict_time_step = local_min_conf_time_step;
//                    }
//                }
//            }
//        }
////        std::cout << "local min_conflict_time_step = " << min_conflict_time_step.t1_ << ", " << min_conflict_time_step.t2_ << std::endl;
//        return min_conflict_time_step;
//    }

    // delay a set of paths one step simultaneously at time step t
    template <Dimension N>
    void delayPath(Paths<N>& paths, int t, int count = 1) {
//        std::cout << "delay t = " << t << ", delay count = " << count << std::endl;
        // traversal all path, wait at previous frame
        for(auto& path : paths) {
            // do not delay which have arrived target
            if(t > path.size()-1) { continue; }
            for(int i=0; i<count; i++) {
                path.insert(path.begin() + t, path[t]);
            }
        }
    }

    // merge two group of paths and the first are planed earlier than the second
//    template <Dimension N>
//    Paths<N> mergeTwoGroupOfPath(const Paths<N>& paths_1, const Paths<N>& paths_2) {
//        Paths<N> copy_1 = paths_1, copy_2 = paths_2;
//        int count = 0;
//        while(1) {
//            count ++;
//            ConflictInfo info = getMinConflictTimeBetweenTwoGroupOfPaths<N>(copy_1, copy_2, 0);
//            if(info.wait_t_ != MAX<int> || info.wait_t_ != MAX<int>) {
//                // delay the second group to fix conflict
//                delayPath<N>(copy_2, info.wait_t_, info.delay_count_);
//            } else {
//                break;
//            }
//        }
//        std::cout << __FUNCTION__ << " take " << count << " step " << std::endl;
//        copy_1.insert(copy_1.end(), copy_2.begin(), copy_2.end());
//        return copy_1;
//    }

    template<Dimension N>
    std::ostream & operator<<(std::ostream& os, const Paths<N>& paths) {
        for(const auto& path : paths) {
            os << path << "\n";
        }
        return os;
    }

    // slow when there are lots of clusters
//    template <Dimension N>
//    Paths<N> multiLayerCompressAnother(const std::vector<Paths<N> >& pathss) {
//        if(pathss.size() == 1) { return pathss.front(); }
//        else if(pathss.empty()) { return {}; }
//        Paths<N> mergedPaths = pathss.front();
////        std::cout << "--level 0: " << std::endl;
//        std::cout << mergedPaths;
//        for(int i=1; i<pathss.size(); i++) {
//            mergedPaths = mergeTwoGroupOfPath(mergedPaths, pathss[i]);
////            std::cout << "--level " << i << ": " << std::endl;
////            std::cout << mergedPaths;
//        }
//        return mergedPaths;
//    }


    // input: static occupancy map / current solving problem / previous path as constraints
    // output: what path was found, or empty is failed
    template<Dimension N>
    using MAPF_FUNC = std::function<Paths<N>(DimensionLength*, const IS_OCCUPIED_FUNC<N> &, const Instances<N> &, CBS_Li::ConstraintTable*, int)>;

    template<Dimension N>
    std::vector<std::set<int> > layeredMAPFDecomposition(const Instances<N> & instances,
                                  DimensionLength* dim,
                                  const IS_OCCUPIED_FUNC<N> & isoc) {

        MAPFInstanceDecompositionPtr<N> instance_decompose = std::make_shared<MAPFInstanceDecomposition<N> >(instances, dim, isoc);

        return instance_decompose->all_levels_;
    }

    template<Dimension N>
    Paths<N> layeredMAPF(const Instances<N> & instances,
                         DimensionLength* dim,
                         const IS_OCCUPIED_FUNC<N> & isoc,
                         const MAPF_FUNC<N> & mapf_func,
                         const MAPF_FUNC<N> & mapf_func_verified,
                         const std::vector<std::set<int>>& all_levels,
                         bool use_path_constraint = true,
                         double cutoff_time = 60,
                         bool completeness_verified = false,
                         bool new_path_legal_check = false) {
        MSTimer mst;
        assert(all_levels.size() >= 1);
        CBS_Li::ConstraintTable *layered_ct = new CBS_Li::ConstraintTable(dim[0], dim[0]*dim[1]);

        Paths<N> retv;// = paths;
        std::vector<Paths<N> > pathss;

        max_size_of_stack_layered = 0; // yz: add for statistics
        max_size_of_stack = 0;
        for(int i=0; i<all_levels.size(); i++) {
            // instance_decompose->all_clusters_[i] to instances
            std::set<int> current_id_set = all_levels[i];
            Instances<2> ists;
            for(const int& id : current_id_set) {
                ists.push_back({instances[id].first, instances[id].second});
            }
            // insert previous path as static constraint
            if (use_path_constraint && !pathss.empty()) {
                for (const auto &previous_path : pathss.back()) {
                    CBS_Li::MAPFPath path_eecbs;
                    for (int i = 0; i < previous_path.size(); i++) {
                        path_eecbs.push_back(
                                CBS_Li::PathEntry(dim[0] * previous_path[i][1] + previous_path[i][0]));
                    }
                    layered_ct->insert2CT(path_eecbs);
                }
            }
            // insert future agents' start as static constraint
            std::vector<std::vector<bool> > avoid_locs(dim[1], std::vector<bool>(dim[0], false));

            for(int j = (use_path_constraint ? i+1 : 0); j<all_levels.size(); j++)
            {
                if(j == i) continue;
                const auto& current_cluster = all_levels[j];
                for(const int& agent_id : current_cluster) {
                    if(j < i) {
                        avoid_locs[instances[agent_id].second[1]][instances[agent_id].second[0]] = true;
                    } else {
                        avoid_locs[instances[agent_id].first[1]][instances[agent_id].first[0]] = true;
                    }
                }
            }
            double remaining_time = cutoff_time - mst.elapsed()/1e3;
            if(remaining_time < 0) {
                if(layered_ct != nullptr) {
                    delete layered_ct;
                    layered_ct = nullptr;
                }
                return {};
            }
            auto new_isoc = [&](const Pointi<2> & pt) -> bool {
                if(pt[0] < 0 || pt[0] >= dim[0] || pt[1] < 0 || pt[1] >= dim[1]) { return true; }
                return isoc(pt) || avoid_locs[pt[1]][pt[0]];
            };

            // check whether current problem is solvable
            if(completeness_verified) {
                Paths<N> EECBS_paths = mapf_func_verified(dim, new_isoc, ists, layered_ct, cutoff_time);
                if(EECBS_paths.empty()) {
                    std::cout << " layered MAPF verify failed, " << i << " th cluster: " << current_id_set << std::endl;
                    if(layered_ct != nullptr) {
                        delete layered_ct;
                        layered_ct = nullptr;
                    }
                    return {};
                }
            }
            Paths<N> next_paths = mapf_func(dim, new_isoc, ists, layered_ct, remaining_time);
            max_size_of_stack_layered = std::max(max_size_of_stack_layered, max_size_of_stack);
            if(next_paths.empty()) {
                std::cout << " layered MAPF failed " << i << " th cluster: " << current_id_set << std::endl;
                if(layered_ct != nullptr) {
                    delete layered_ct;
                    layered_ct = nullptr;
                }
                return {};
            }
            // check whether new path meet static constraints
            if(new_path_legal_check) {
                for(int new_path_id=0; new_path_id < next_paths.size(); new_path_id++) {
                    const auto& current_path = next_paths[new_path_id];
                    for(int t=1; t<current_path.size(); t++) {
                        int next_location = dim[0] * current_path[t][1] + current_path[t][0],
                                curr_location = dim[0] * current_path[t-1][1] + current_path[t-1][0];
                        int next_timestep = t;
                        if (layered_ct->constrained(next_location, next_timestep)) {
                            std::cout << "new path Agent " << i << " have vertex conflict at " << next_location << " " << freeNav::IdToPointi<2>(next_location, dim) << " at timestep " << next_timestep << std::endl;
                            if(layered_ct != nullptr) {
                                delete layered_ct;
                                layered_ct = nullptr;
                            }
                            return {};
                        }
                        if(layered_ct->constrained(curr_location, next_location, next_timestep)) {
                            std::cout << "new path Agent " << i << " have edge conflict from " << freeNav::IdToPointi<2>(curr_location, dim)  << " to " << freeNav::IdToPointi<2>(next_location, dim)  << " at timestep " << next_timestep << std::endl;
                            if(layered_ct != nullptr) {
                                delete layered_ct;
                                layered_ct = nullptr;
                            }
                            return {};
                        }
                    }
                }
            }
            retv.insert(retv.end(), next_paths.begin(), next_paths.end());
            pathss.push_back(next_paths);

        }
        assert(instances.size() == retv.size());
        if(!use_path_constraint) { retv = multiLayerCompress(dim, pathss); }
        //if(!use_path_constraint) { retv = multiLayerCompressAnother(pathss); }
        std::cout << " layered mapf success " << !retv.empty() << std::endl;
        if(layered_ct != nullptr) {
            delete layered_ct;
            layered_ct = nullptr;
        }
        return retv;
    }


}

#endif //FREENAV_LAYERED_MAPF_H
