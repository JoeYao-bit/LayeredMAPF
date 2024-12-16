//
// Created by yaozhuo on 2024/12/15.
//

#ifndef LAYEREDMAPF_INDEPENDENCE_DETECTION_H
#define LAYEREDMAPF_INDEPENDENCE_DETECTION_H

#include "layered_mapf.h"
#include "../third_party/EECBS/inc/SIPP.h"
#include "../third_party/EECBS/inc/SpaceTimeAStar.h"

/* Independence Detection:
 * Standley, T. (2010). Finding Optimal Solutions to Cooperative Pathfinding Problems.
 * Proceedings of the AAAI Conference on Artificial Intelligence, 24(1), 173-178. http
 * s://doi.org/10.1609/aaai.v24i1.7564
 * These independence detection algorithms do not solve
   pathfinding problems on their own. They simply call a
   search algorithm many times on subproblems allowing that
   algorithm to solve many smaller problems rather than the
   full problem. The independence detection algorithms can
   use the standard algorithm or operator decomposition as the
   needed search algorithm. ID will merge two groups and run
   a search algorithm on the combined group if it cannot find
   nonconflicting paths for both groups. Therefore ID is com-
   plete when coupled with a complete search algorithm.
 * */

namespace freeNav::LayeredMAPF {

    // if there are few agents, it would be memory efficient to check every two paths
    // if there are lots of agents, it would be time efficient to check them
    // there should be no conflict inside the same group
    template<Dimension N>
    struct MultiPathConflictDetector {
        MultiPathConflictDetector(const Paths<N>& paths, const std::vector<size_t>& group_id_map, DimensionLength* dim)
        : paths_(paths), group_id_map_(group_id_map),dim_(dim) {
        }

        // detect whether path belong to different group have conflict,
        // if have, return true, set group_id1 and group_id2 to the first two group
        bool detectConflict(size_t& group_id1, size_t& group_id2) const {
            size_t makespan = 0;
            for(int i=0; i<paths_.size(); i++) {
                makespan = std::max(makespan, paths_.size());
            }
            int map_size = dim_[0]*dim_[1];
            Pointi<N> current_pt;
            Id id; // grid id
            int agent_id2;
            std::vector<Id> agent_occ_ids_cur(paths_.size(), -1); // id of grid occupied by agent
            std::vector<Id> agent_occ_ids_next(paths_.size(), -1);
            for(int t=0; t<makespan; t++) {
                std::vector<int> current_floor(map_size, -1);
                // set current floor
                for(int agent_id=0; agent_id<paths_.size(); agent_id++) {
                    if(t <= paths_[agent_id].size()-1) {
                        current_pt = paths_[agent_id][t];
                    } else {
                        current_pt = paths_[agent_id].back();
                    }
                    id = PointiToId(current_pt, dim_);
                    if(current_floor[id] != -1) {
                        // vertex conflict happen
                        agent_id = current_floor[id];
                        // check whether belong to the same group, there shouldn't be conflict inside a group
                        assert(group_id_map_[agent_id] != group_id_map_[agent_id2]);
                        group_id1 = group_id_map_[agent_id];
                        group_id2 = group_id_map_[agent_id2];
                        return true;
                    } else {
                        // set current floor occupied by this agent
                        current_floor[id] = agent_id;
                    }
                    agent_occ_ids_cur[agent_id] = id;
                }
                // set next floor
                t = t+1;
                std::vector<int> next_floor(map_size, -1);
                for(int agent_id=0; agent_id<paths_.size(); agent_id++) {
                    if(t <= paths_[agent_id].size()-1) {
                        current_pt = paths_[agent_id][t];
                    } else {
                        current_pt = paths_[agent_id].back();
                    }
                    id = PointiToId(current_pt, dim_);
                    if(next_floor[id] != -1) {
                        // vertex conflict happen
                        agent_id2 = next_floor[id];
                        // check whether belong to the same group, there shouldn't be conflict inside a group
                        assert(group_id_map_[agent_id] != group_id_map_[agent_id2]);
                        group_id1 = group_id_map_[id];
                        group_id2 = group_id_map_[agent_id2];
                        return true;
                    } else {
                        // set current floor occupied by this agent
                        next_floor[id] = agent_id;
                    }
                    agent_occ_ids_next[agent_id] = id;
                }
                // detect edge conflict
                for(int agent_id=0; agent_id<paths_.size(); agent_id++) {
                    // if agent's next grid was occupied by another agent's current grid
                    Id next_id = agent_occ_ids_next[agent_id]; // grid id
                    Id cur_id  = agent_occ_ids_cur [agent_id]; // grid id
                    if(next_id == cur_id) { continue; } // only move between different grid cause edge conflict
                    if(current_floor[next_id] != -1) {
                        agent_id2 = current_floor[next_id];
                        // if another agent's next occ grid is agent's current occ grid
                        if(agent_occ_ids_next[agent_id2] == cur_id) {
                            assert(group_id_map_[agent_id] != group_id_map_[agent_id2]);
                            group_id1 = group_id_map_[id];
                            group_id2 = group_id_map_[agent_id2];
                            return true;
                        }
                    }
                }
            }
            return false;
        }

        const Paths<N>& paths_;
        const std::vector<size_t>& group_id_map_;
        DimensionLength* dim_;

    };

    template<Dimension N>
    Paths<N> IndependenceDetectionMAPF(const Instances<N> & instances,
                                      DimensionLength* dim,
                                      const IS_OCCUPIED_FUNC<N> & isoc,
                                      const MAPF_FUNC<N> & mapf_func,
                                      const MAPF_FUNC<N> & mapf_func_verified,
                                      bool use_path_constraint = true,
                                      int cutoff_time = 60,
                                      bool completeness_verified = false,
                                      bool new_path_legal_check = false) {
        assert(N == 2); // current code only support 2D instance
        struct timezone tz;
        struct timeval  tv_pre;
        struct timeval  tv_after;
        gettimeofday(&tv_pre, &tz);
        double remaining_time = cutoff_time;
        // 1，assign each agent to its own group, plan initial paths
        // 2, fill conflict avoidance table with every path
        Paths<N> paths, retv; // every agent's path
        std::vector<size_t> group_id_map(instances.size()); // when merge groups, set them id to the minimal group id
        std::map<size_t, std::vector<int>> group_id_set; // group id and its agent ids
        // init group id
        for(int i=0; i<instances.size(); i++) {
            group_id_map[i] = i;
            if(group_id_set.find(i) == group_id_set.end()) {
                group_id_set[i] = {i};
            } else {
                // there shouldn't be reach, but add for continuous
                assert(0);
                group_id_set[i].push_back(i);
            }
        }
        // get initial paths
        for(int i=0; i<instances.size(); i++) {
            // initialize instance in an freeNav way
            CBS_Li::Instance single_ins(dim, isoc, {instances[i]});
            CBS_Li::SingleAgentSolver* solver = nullptr;
            if(1) {
                solver = new CBS_Li::SpaceTimeAStar(single_ins, i);
            } else {
                solver = new CBS_Li::SIPP(single_ins, 0);
            }
            auto root = new CBS_Li::CBSNode();
            CBS_Li::ConstraintTable empty_ct = CBS_Li::ConstraintTable(dim[0], dim[0]*dim[1]);
            std::vector<CBS_Li::PathEntry> solution =
                    solver->findOptimalPath(*root, empty_ct, {}, 0, 0);
            delete solver;
            delete root;
            if(solution.empty()) {
                std::cout << "search solution for agent " << i << ", " << instances[i].first << " to "
                          << instances[i].second << " failed " << std::endl;
                assert(0);
            } else {
                Path<N> init_path;
                for(int t=0; t<solution.size(); t++) {
                    init_path.push_back(IdToPointi<N>(solution[t].location, dim));
                }
                paths[i] = init_path;
            }
        }
        std::cout << "finish search init paths" << std::endl;
        while(true) { // until no conflicts occur
            // simulate execution of all paths until a conflict between two groups G 1 and G 2 occurs
            // if these two groups have not conflicted before then
            // execute all path simultaneously, util found conflict between paths belong to different group
            MultiPathConflictDetector<N> detector(paths, group_id_map, dim);
            size_t gid1, gid2;
            if(!detector.detectConflict(gid1, gid2)) {
                // there is no conflict between paths
                retv = paths;
                break;
            }
            std::cout << "detect conflict between group " << gid1 << " and " << gid2 << std::endl;
            assert(gid1 != gid2);
            // fill illegal move table with the current paths for G 2
            // find another set of paths with the same cost for G 1
            // if failed to find such a set then
            // fill illegal move table with the current paths for G 1
            //  find another set of paths with the same cost for G 2
            // end if

            // if failed to find an alternate set of paths for G 1 and G 2 then
            // merge G 1 and G 2 into a single group
            // cooperatively plan new group

            // insert G 2's path as hard constraintm while ramainning paths as soft constraint
            // insert soft constraint to ct

            // insert hard constraint to ct

            // plan new path, avoid other group's paths (hard constraint), try to avoid remaining group's path

            gettimeofday(&tv_after, &tz);
            remaining_time = cutoff_time - (tv_after.tv_sec - tv_pre.tv_sec) + (tv_after.tv_usec - tv_pre.tv_usec)/1e6;
            if(remaining_time <= 0) {
                std::cout << "run out of time 1" << std::endl;
                break;
            }
            std::cout << "start try make group " << gid2 << "'s path avoid group " << gid1 << std::endl;
            CBS_Li::ConstraintTable *layered_ct = new CBS_Li::ConstraintTable(dim[0], dim[0]*dim[1]);

            for (int i=0; i<paths.size(); i++) {
                // avoid conflict with group2's paths
                if(group_id_map[i] == gid2) { continue; }
                CBS_Li::MAPFPath path_eecbs;
                for (int t = 0; t < paths[i].size(); t++) {
                    path_eecbs.push_back(CBS_Li::PathEntry(PointiToId(paths[i][t], dim)));
                }
                layered_ct->insert2CT(path_eecbs);
            }
            // set soft constraint
            for (int i=0; i<paths.size(); i++) {
                // avoid conflict with group2's paths
                if(group_id_map[i] != gid1 && group_id_map[i] != gid2) { continue; }
                CBS_Li::MAPFPath path_eecbs;
                for (int t = 0; t < paths[i].size(); t++) {
                    path_eecbs.push_back(CBS_Li::PathEntry(PointiToId(paths[i][t], dim)));
                }
                layered_ct->insert2CAT(path_eecbs);
            }
            // TODO: set upperbound of cost

            // construct local instance
            Instances<N> ists;
            for(auto iter = group_id_set[gid1].begin(); iter != group_id_set[gid1].end(); iter++) {
                ists.push_back(instances[*iter]);
            }
            Paths<N> next_paths = mapf_func(dim, isoc, ists, layered_ct, remaining_time);
            delete layered_ct;
            std::cout << "finish try make group " << gid2 << "'s path avoid group " << gid1
                                      << ", success ?" << !next_paths.empty() << std::endl;
            if(!next_paths.empty()) {
                int local_path_id = 0;
                for(auto iter = group_id_set[gid1].begin(); iter != group_id_set[gid1].end(); iter++) {
                    paths[*iter] = next_paths[local_path_id];
                    local_path_id ++;
                }
                continue;
            }
            // if play failed, try search gid2 while set gid1 as constraint
            gettimeofday(&tv_after, &tz);
            remaining_time = cutoff_time - (tv_after.tv_sec - tv_pre.tv_sec) + (tv_after.tv_usec - tv_pre.tv_usec)/1e6;
            if(remaining_time <= 0) {
                std::cout << "run out of time 2" << std::endl;
                break;
            }
            std::cout << "start try make group " << gid1 << "'s path avoid group " << gid2 << std::endl;

            layered_ct = new CBS_Li::ConstraintTable(dim[0], dim[0]*dim[1]);

            for (int i=0; i<paths.size(); i++) {
                // avoid conflict with group 1's paths
                if(group_id_map[i] == gid1) { continue; }
                CBS_Li::MAPFPath path_eecbs;
                for (int t = 0; t < paths[i].size(); t++) {
                    path_eecbs.push_back(CBS_Li::PathEntry(PointiToId(paths[i][t], dim)));
                }
                layered_ct->insert2CT(path_eecbs);
            }
            // set soft constraint
            for (int i=0; i<paths.size(); i++) {
                // avoid conflict with group2's paths
                if(group_id_map[i] != gid1 && group_id_map[i] != gid2) { continue; }
                CBS_Li::MAPFPath path_eecbs;
                for (int t = 0; t < paths[i].size(); t++) {
                    path_eecbs.push_back(CBS_Li::PathEntry(PointiToId(paths[i][t], dim)));
                }
                layered_ct->insert2CAT(path_eecbs);
            }
            // TODO: set upperbound of cost，SOC (according to article)

            // construct local instance
            ists.clear();
            for(auto iter = group_id_set[gid2].begin(); iter != group_id_set[gid2].end(); iter++) {
                ists.push_back(instances[*iter]);
            }
            next_paths = mapf_func(dim, isoc, ists, layered_ct, remaining_time);
            delete layered_ct;
            std::cout << "finish try make group " << gid1 << "'s path avoid group " << gid2
                      << ", success ?" << !next_paths.empty() << std::endl;
            if(!next_paths.empty()) {
                int local_path_id = 0;
                for(auto iter = group_id_set[gid2].begin(); iter != group_id_set[gid2].end(); iter++) {
                    paths[*iter] = next_paths[local_path_id];
                    local_path_id ++;
                }
                continue;
            }
            // if both failed, merge gid1 and gid2, and search path again
            // end if
            // update conflict avoidance table with changes made to paths
            gettimeofday(&tv_after, &tz);
            remaining_time = cutoff_time - (tv_after.tv_sec - tv_pre.tv_sec) + (tv_after.tv_usec - tv_pre.tv_usec)/1e6;
            if(remaining_time <= 0) {
                std::cout << "run out of time 3" << std::endl;
                break;
            }

            layered_ct = new CBS_Li::ConstraintTable(dim[0], dim[0]*dim[1]);
            // set soft constraint
            for (int i=0; i<paths.size(); i++) {
                // avoid conflict with group2's paths
                if(group_id_map[i] != gid1 && group_id_map[i] != gid2) { continue; }
                CBS_Li::MAPFPath path_eecbs;
                for (int t = 0; t < paths[i].size(); t++) {
                    path_eecbs.push_back(CBS_Li::PathEntry(PointiToId(paths[i][t], dim)));
                }
                layered_ct->insert2CAT(path_eecbs);
            }
            std::cout << " merge group " << gid2 << " into " << gid1 << std::endl;
            // merge group gid1 and gid2, merge gid2 into gid1
            for(const auto& temp_id : group_id_set[gid2]) {
                group_id_map[temp_id] = gid1;
                group_id_set[gid1].push_back(temp_id);
            }
            group_id_set.erase(gid2);
            // construct local instance
            ists.clear();
            for(auto iter = group_id_set[gid1].begin(); iter != group_id_set[gid1].end(); iter++) {
                ists.push_back(instances[*iter]);
            }
            std::cout << "start search path of merge group " << gid1 << " and " << gid2 << std::endl;

            next_paths = mapf_func(dim, isoc, ists, layered_ct, remaining_time);
            delete layered_ct;
            if(!next_paths.empty()) {
                int local_path_id = 0;
                for(auto iter = group_id_set[gid2].begin(); iter != group_id_set[gid2].end(); iter++) {
                    paths[*iter] = next_paths[local_path_id];
                    local_path_id ++;
                }
                continue;
            } else {
                // shouldn't reach there, as merge of two group should always be solvable
                std::cout << "merge group " << gid1 << " and " << gid2 << " search path failed " << std::endl;
                assert(0);
            }

        }
        return retv;
    }

}


#endif //LAYEREDMAPF_INDEPENDENCE_DETECTION_H
