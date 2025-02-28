//
// Created by yaozhuo on 2024/12/15.
//

#ifndef LAYEREDMAPF_INDEPENDENCE_DETECTION_H
#define LAYEREDMAPF_INDEPENDENCE_DETECTION_H

#include "layered_mapf.h"
#include "../third_party/EECBS/inc/SIPP.h"
#include "../third_party/EECBS/inc/SpaceTimeAStar.h"
#include "../algorithm/constraint_table_CBS/common.h"
#include "../third_party/EECBS/inc/CBSNode.h"

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
    struct PathConflictDetector {
        PathConflictDetector(const Paths<N>& paths, const std::vector<size_t>& group_id_map, DimensionLength* dim)
        : paths_(paths), group_id_map_(group_id_map),dim_(dim) {
        }

        // detect whether path belong to different group have conflict,
        // if have, return true, set group_id1 and group_id2 to the first two group
        bool detectConflict(size_t& group_id1, size_t& group_id2) const {
            size_t makespan = 0;
            for(int i=0; i<paths_.size(); i++) {
                makespan = std::max(makespan, paths_[i].size());
//                std::cout << "path " << i << ": " << paths_[i] << std::endl;
            }
            int map_size = dim_[0]*dim_[1];
            Pointi<N> current_pt;
            Id id; // grid id
            int agent_id2;
            std::vector<Id> agent_occ_ids_cur(paths_.size(), -1); // id of grid occupied by agent
            std::vector<Id> agent_occ_ids_next(paths_.size(), -1);
            for(int t=0; t<makespan; t++) {
                //std::cout << "t = " << t << std::endl;
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
                        std::cout << "detect vertex conflict 1 " << std::endl;
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
                std::vector<int> next_floor(map_size, -1);
                for(int agent_id=0; agent_id<paths_.size(); agent_id++) {
                    if(t+1 <= paths_[agent_id].size()-1) {
                        current_pt = paths_[agent_id][t+1];
                    } else {
                        current_pt = paths_[agent_id].back();
                    }
                    id = PointiToId(current_pt, dim_);
                    if(next_floor[id] != -1) {
                        // vertex conflict happen
                        agent_id2 = next_floor[id];
//                        std::cout << "detect vertex conflict 2 at " << current_pt << std::endl;
//                        std::cout << "agent id 1/2 = " << agent_id << " / " << agent_id2 << std::endl;
//                        std::cout << "group_id_map_[agent_id] / group_id_map_[agent_id2] = "
//                                  << group_id_map_[agent_id] << " / " << group_id_map_[agent_id2] << std::endl;
                        // check whether belong to the same group, there shouldn't be conflict inside a group
                        assert(group_id_map_[agent_id] != group_id_map_[agent_id2]);
                        group_id1 = group_id_map_[agent_id];
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
                            std::cout << "detect edge conflict 1 " << std::endl;
                            assert(group_id_map_[agent_id] != group_id_map_[agent_id2]);
                            group_id1 = group_id_map_[agent_id];
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
    struct IndependenceDetection {

        IndependenceDetection(const Instances<N> & instances, DimensionLength* dim,
                              const IS_OCCUPIED_FUNC<N> & isoc, const MAPF_FUNC<N> & mapf_func, float cutoff_time = 60)
                              : instances_(instances), isoc_(isoc), dim_(dim),
                              mapf_func_(mapf_func), cutoff_time_(cutoff_time) {
            gettimeofday(&tv_pre, &tz);
            getInitGroupId();
            createInitialPaths();
            run();
            if(solved_) {
                std::cout << "IndependenceDetection success, max/total = "
                          << getMaximalSubProblem() << "/" << instances_.size() << std::endl;
            } else {
                std::cout << "IndependenceDetection failed, max/total = "
                          << getMaximalSubProblem() << "/" << instances_.size() << std::endl;
            }
        }

        size_t getMaximalSubProblem() const {
            size_t vec_size = 0;
            for(const auto& vec : group_id_set_) {
                vec_size = std::max(vec.second.size(), vec_size);
            }
            return vec_size;
        }

        size_t getNumberOfSubProblem() const {
            return group_id_set_.size();
        }

        void getInitGroupId() {
            group_id_map_ = std::vector<size_t>(instances_.size()); // when merge groups, set them id to the minimal group id
            group_id_set_ = std::map<size_t, std::vector<int> >(); // group id and its agent ids
            maximal_group_id_ = instances_.size() - 1;

            for(int i=0; i<instances_.size(); i++) {
                group_id_map_[i] = i;
                if(group_id_set_.find(i) == group_id_set_.end()) {
                    group_id_set_[i] = {i};
                } else {
                    // there shouldn't be reach, but add for continuous
                    assert(0);
                    group_id_set_[i].push_back(i);
                }
            }
        }

        void createInitialPaths() {
            paths_ = Paths<N>(instances_.size()   );
            maximum_length_of_paths_ = std::vector<size_t>(instances_.size());
            for(int i=0; i<instances_.size(); i++) {
//            std::cout << "flag 1.1" << std::endl;

                // initialize instance in an freeNav way
                CBS_Li::Instance single_ins(dim_, isoc_, {instances_[i]});
                CBS_Li::SingleAgentSolver* solver = nullptr;
//            std::cout << "flag 1.2" << std::endl;

                if(1) {
                    solver = new CBS_Li::SpaceTimeAStar(single_ins, 0);
//                std::cout << "flag 1.3" << std::endl;

                } else {
                    solver = new CBS_Li::SIPP(single_ins, 0);
//                std::cout << "flag 1.4" << std::endl;
                }
                //std::cout << "flag 2" << std::endl;

                auto root = new CBS_Li::CBSNode();
                CBS_Li::ConstraintTable empty_ct = CBS_Li::ConstraintTable(dim_[0], dim_[0]*dim_[1]);
                std::vector<CBS_Li::PathEntry> solution =
                        solver->findOptimalPath(*root, empty_ct, {}, 0, 0);
                delete solver;
                delete root;
                //std::cout << "flag 3" << std::endl;

                if(solution.empty()) {
                    std::cout << "search solution for agent " << i << ", " << instances_[i].first << " to "
                              << instances_[i].second << " failed " << std::endl;
                    assert(0);
                } else {
                    Path<N> init_path;
                    for(int t=0; t<solution.size(); t++) {
                        init_path.push_back(IdToPointi<N>(solution[t].location, dim_));
                    }
                    paths_[i] = init_path;
                    maximum_length_of_paths_[i] = init_path.size();
                }
                //std::cout << "flag 4" << std::endl;
            }
            std::cout << "finish search init paths" << std::endl;
        }

        // true: avoid success; false: failed to merge
        bool tryMakeGroupAvoidAnother(const size_t& gid1, const size_t& gid2) {
            gettimeofday(&tv_after, &tz);
            remaining_time_ = cutoff_time_ - (tv_after.tv_sec - tv_pre.tv_sec) + (tv_after.tv_usec - tv_pre.tv_usec)/1e6;
            if(remaining_time_ <= 0) {
                std::cout << "run out of time 1" << std::endl;
                return false;
            }
//            std::cout << "1, start try make group " << gid1 << "'s path avoid group " << gid2 << std::endl;
            CBS_Li::ConstraintTable *layered_ct = new CBS_Li::ConstraintTable(dim_[0], dim_[0]*dim_[1]);
//            return paths;

            Paths<N> other_group_paths;
//            for (int i=0; i<paths_.size(); i++) {
//                // avoid conflict with group2's paths
//                if(group_id_map_[i] != gid1) { continue; }
//                CBS_Li::MAPFPath path_eecbs;
//                for (int t = 0; t < paths_[i].size(); t++) {
//                    path_eecbs.push_back(CBS_Li::PathEntry(PointiToId(paths_[i][t], dim_)));
//                }
//                other_group_paths.push_back(paths_[i]);
//                layered_ct->insert2CT(path_eecbs);
//            }
            // avoid previous separated group's path as hard constraint
            for(int i=0; i<paths_.size(); i++) {
//                if(group_id_map_[i] == gid1) { continue; } // try avoid all path exit from group 1
                if(group_id_map_[i] != gid2) { continue; } // try avoid all path exit from group 1
                CBS_Li::MAPFPath path_eecbs;
                for (int t = 0; t < paths_[i].size(); t++) {
                    path_eecbs.push_back(CBS_Li::PathEntry(PointiToId(paths_[i][t], dim_)));
                }
//                other_group_paths.push_back(paths_[i]);
                layered_ct->insert2CT(path_eecbs);
            }
            // set soft constraint
            for (int i=0; i<paths_.size(); i++) {
                // avoid conflict with group2's paths
                if(group_id_map_[i] == gid1 || group_id_map_[i] == gid2) { continue; }
                CBS_Li::MAPFPath path_eecbs;
                for (int t = 0; t < paths_[i].size(); t++) {
                    path_eecbs.push_back(CBS_Li::PathEntry(PointiToId(paths_[i][t], dim_)));
                }
                layered_ct->insert2CAT(path_eecbs);
            }
            // construct local instance
            Instances<N> ists;
            layered_ct->maximum_length_of_paths_.clear(); // set upperbound of path cost
            for(auto iter = group_id_set_[gid1].begin(); iter != group_id_set_[gid1].end(); iter++) {
                ists.push_back(instances_[*iter]);
                layered_ct->maximum_length_of_paths_.push_back(maximum_length_of_paths_[*iter]);
//                std::cout << "set agent " << *iter << " max length to " << maximum_length_of_paths_[*iter] << std::endl;
            }
            Paths<N> next_paths = mapf_func_(dim_, isoc_, ists, layered_ct, remaining_time_);
            delete layered_ct;
            if(!next_paths.empty()) {
                int local_path_id = 0;
                for(auto iter = group_id_set_[gid1].begin(); iter != group_id_set_[gid1].end(); iter++) {
                    paths_[*iter] = next_paths[local_path_id];
//                    std::cout << "update path of " << *iter << " to " << next_paths[local_path_id] << std::endl;
                    local_path_id ++;
                }
                // debug
//                auto multiple_paths = next_paths;
//                multiple_paths.insert(multiple_paths.end(), other_group_paths.begin(), other_group_paths.end());
//                assert(validateSolution<2>(multiple_paths));
                //layered_ct->constrained()
                return true;
            }
            return false;
        }

        bool mergeGroupAndAnother(const size_t& gid1, const size_t& gid2) {
            gettimeofday(&tv_after, &tz);
            remaining_time_ = cutoff_time_ - (tv_after.tv_sec - tv_pre.tv_sec) + (tv_after.tv_usec - tv_pre.tv_usec)/1e6;
            if(remaining_time_ <= 0) {
                std::cout << "run out of time 3" << std::endl;
                return false;
            }
            CBS_Li::ConstraintTable *layered_ct = new CBS_Li::ConstraintTable(dim_[0], dim_[0]*dim_[1]);
            layered_ct = new CBS_Li::ConstraintTable(dim_[0], dim_[0]*dim_[1]);
            // set soft constraint
            for (int i=0; i<paths_.size(); i++) {
                // avoid conflict with group2's paths
                if(group_id_map_[i] == gid1 || group_id_map_[i] == gid2) { continue; }
                CBS_Li::MAPFPath path_eecbs;
                for (int t = 0; t < paths_[i].size(); t++) {
                    path_eecbs.push_back(CBS_Li::PathEntry(PointiToId(paths_[i][t], dim_)));
                }
                layered_ct->insert2CAT(path_eecbs);
            }
            maximal_group_id_ ++;
            size_t new_group_id = maximal_group_id_;
            for(const auto& temp_id : group_id_set_[gid1]) {
                group_id_map_[temp_id] = new_group_id;
                group_id_set_[new_group_id].push_back(temp_id);
            }
            group_id_set_.erase(gid1);
            for(const auto& temp_id : group_id_set_[gid2]) {
                group_id_map_[temp_id] = new_group_id;
                group_id_set_[new_group_id].push_back(temp_id);
            }
            group_id_set_.erase(gid2);
//            std::cout << "group_id_set[gid1] = ";
            // construct local instance
            Instances<N> ists;
            layered_ct->maximum_length_of_paths_.clear(); // no cost limitation when merge groups
            for(auto iter = group_id_set_[new_group_id].begin(); iter != group_id_set_[new_group_id].end(); iter++) {
                ists.push_back(instances_[*iter]);
                layered_ct->maximum_length_of_paths_.push_back(maximum_length_of_paths_[*iter]);
//                std::cout << *iter << " ";
            }
//            std::cout << std::endl;
//            std::cout << "3, start search path of merged group " << new_group_id << std::endl;

            Paths<N> next_paths = mapf_func_(dim_, isoc_, ists, layered_ct, remaining_time_);
            delete layered_ct;

//            std::cout << "next_paths = " << std::endl;
//            for(int i=0; i<next_paths.size(); i++) {
//                std::cout << next_paths[i] << std::endl;
//            }
//            std::cout << "next_paths end" << std::endl;
            if(!next_paths.empty()) {
                int local_path_id = 0;
                for(auto iter = group_id_set_[new_group_id].begin(); iter != group_id_set_[new_group_id].end(); iter++) {
                    paths_[*iter] = next_paths[local_path_id];
//                    std::cout << "update path of " << *iter << " to " << next_paths[local_path_id] << std::endl;
                    local_path_id ++;
                }
                return true;
            } else {
                // shouldn't reach there (except run out of time), as merge of two group should always be solvable
                std::cout << "merge group failed: " << gid1 << " and " << gid2 << std::endl;
                return false;
                //assert(0);
            }
            return false;
        }

        void run() {
            remaining_time_ = cutoff_time_;
            int count = 0;
            while(true) { // until no conflicts occur
//                std::cout << "-- step " << count << std::endl;
                count ++;
//                if(count >= 2000) {
//                    return;
//                }
                // simulate execution of all paths until a conflict between two groups G 1 and G 2 occurs
                // if these two groups have not conflicted before then
                // execute all path simultaneously, util found conflict between paths belong to different group
                PathConflictDetector<N> detector(paths_, group_id_map_, dim_);
                size_t gid1, gid2;
                if(!detector.detectConflict(gid1, gid2)) {
                    // there is no conflict between paths
//                    retv = paths_;
                    solved_ = true;
                    break;
                }
//                std::cout << "detect conflict between group " << gid1 << " and " << gid2 << std::endl;
                assert(gid1 != gid2);

                // plan new path, avoid other group's paths (hard constraint), try to avoid remaining group's path
//                std::cout << "try make group " << gid1 << " avoid " << gid2 << ", ";
                if(tryMakeGroupAvoidAnother(gid1, gid2)) {
//                    std::cout << "success " << std::endl;
                    continue;
                } else {
//                    std::cout << "failed " << std::endl;
                }
                // if play failed, try search gid2 while set gid1 as constraint
//                std::cout << "try make group " << gid2 << " avoid " << gid1 << ", ";
                if(tryMakeGroupAvoidAnother(gid2, gid1)) {
//                    std::cout << "success " << std::endl;
                    continue;
                } else {
//                    std::cout << "failed " << std::endl;
                }

                // if both failed, merge gid1 and gid2, and search path again
                // end if
                // update conflict avoidance table with changes made to paths
//                std::cout << "try make group " << gid1 << " merge with " << gid2 << ", ";
                if(!mergeGroupAndAnother(gid1, gid2)) {
//                    std::cout << "failed " << std::endl;
                    return;
                } else {
//                    std::cout << "success, get new group " << maximal_group_id_ << std::endl;
                }
            }
        }

        // group tree is a reversed version of tree node
        struct GroupTreeNode;
        typedef std::shared_ptr<GroupTreeNode> GroupTreeNodePtr;
        
        struct GroupTreeNode {
            
            GroupTreeNode(size_t group_id, GroupTreeNodePtr pa1 = nullptr, GroupTreeNodePtr pa2 = nullptr) {
                assert((pa1 == nullptr && pa2 == nullptr) || (pa1 != nullptr && pa2 != nullptr));
                group_id_ = group_id;
                pa1_ = pa1;
                pa2_ = pa2;
            }

            GroupTreeNodePtr pa1_   = nullptr;

            GroupTreeNodePtr pa2_   = nullptr;

            GroupTreeNodePtr child_ = nullptr;

            size_t group_id_ = MAX<int>;

        };

        const Instances<N> & instances_;
        DimensionLength* dim_;
        const IS_OCCUPIED_FUNC<N> isoc_;
        const MAPF_FUNC<N> & mapf_func_;
        float cutoff_time_ = 60;

        Paths<N> paths_; // every agent's path
        std::vector<size_t> group_id_map_; // when merge groups, set their id to the minimal group id
        std::map<size_t, std::vector<int> > group_id_set_; // group id and its agent ids
        size_t maximal_group_id_ = 0;// avoid repeat of group id, so singleton increases from 0
        std::vector<size_t> maximum_length_of_paths_;
        bool solved_ = false;

        std::vector<std::set<size_t> > group_avoid_ids_; // store which pair of group are prove could be separated

        struct timezone tz;
        struct timeval  tv_pre;
        struct timeval  tv_after;
        float remaining_time_;


    };


}


#endif //LAYEREDMAPF_INDEPENDENCE_DETECTION_H
