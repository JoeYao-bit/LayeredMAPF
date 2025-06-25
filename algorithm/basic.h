//
// Created by yaozhuo on 2023/6/18.
//

#ifndef FREENAV_LAYERED_MAPF_TCBS_BASIC_H
#define FREENAV_LAYERED_MAPF_TCBS_BASIC_H

#include <memory>
#include <boost/unordered_map.hpp>
#include <boost/heap/pairing_heap.hpp>
#include <boost/unordered_set.hpp>
#include <limits>
#include "../freeNav-base/basic_elements/point.h"
#include <chrono>

namespace freeNav::LayeredMAPF {

#define MAX_TIMESTEP INT_MAX / 2
#define MAX_COST INT_MAX / 2
#define MAX_NODES std::numeric_limits<size_t>::max() / 2

    template <Dimension N, typename NODE>
    struct TreeNode {

        TreeNode(const NODE& pa = nullptr) : pa_(pa) {}

        NODE pa_ = nullptr; // parent node

        std::vector<NODE> ch_ = {}; // children node, optional

    };

    template<Dimension N>
    bool WhetherTwoPathConflict(const Path<N>& p1, const Path<N>& p2) {
        assert(!p1.empty() && !p2.empty());
        size_t min_path_length = p1.size() < p2.size() ? p1.size() : p2.size();
        // yz: check conflict in common time range
        for (size_t timestep = 0; timestep < min_path_length; timestep++)
        {
            Pointi<N> loc1 = p1[timestep];
            Pointi<N> loc2 = p2[timestep];
            // yz: vertex conflict error
            if (loc1 == loc2)
            {
                std::cout << "Agents " << 1 << " and " << 2 << " have vertex conflict 1 at " << loc1 << " at timestep " << timestep << std::endl;
                return true;
            }
            else if (timestep < min_path_length - 1
                     && loc1 == p2[timestep + 1]
                     && loc2 == p1[timestep + 1])
            {
                std::cout << "Agents " << 1 << " and " << 2 << " have edge conflict at (" <<
                          loc1 << "-->" << loc2 << ") at timestep " << timestep << std::endl;
                return true;
            }
        }
        // yz: check conflict when one of them is stop
        if (p1.size() != p2.size())
        {
            const auto& shorter_path = p1.size() < p2.size() ? p1 : p2;
            const auto& longer_path  = p1.size() < p2.size() ? p2 : p1;
            Pointi<N> loc1 = shorter_path.back();
            int a1_ = p1.size() < p2.size() ? 1 : 2;
            int a2_ = p1.size() < p2.size() ? 2 : 1;
            for (size_t timestep = min_path_length; timestep < longer_path.size(); timestep++)
            {
                Pointi<N> loc2 = longer_path[timestep];
                if (loc1 == loc2)
                {
                    std::cout << "Agents " << a1_ << " and " << a2_ << " have vertex conflict 2 at " << loc1 << " at timestep " << timestep << std::endl;
                    return true; // It's at least a semi conflict
                }
            }
        }
        return false;
    }

    template<Dimension N>
    bool validateSolution(const Paths<N>& paths)
    {
        int num_of_agents = paths.size();
        // check whether the paths are feasible
        size_t soc = 0;
        for (int a1 = 0; a1 < num_of_agents; a1++)
        {
            soc += paths[a1].size() - 1; // yz: soc: sum of cost
            for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
            {
//                size_t min_path_length = paths[a1].size() < paths[a2].size() ? paths[a1].size() : paths[a2].size();
//                // yz: check conflict in common time range
//                for (size_t timestep = 0; timestep < min_path_length; timestep++)
//                {
//                    Pointi<N> loc1 = paths[a1][timestep];
//                    Pointi<N> loc2 = paths[a2][timestep];
//                    // yz: vertex conflict error
//                    if (loc1 == loc2)
//                    {
//                        std::cout << "Agents " << a1 << " and " << a2 << " have vertex conflict 1 at " << loc1 << " at timestep " << timestep << std::endl;
//                        return false;
//                    }
//                    else if (timestep < min_path_length - 1
//                             && loc1 == paths[a2][timestep + 1]
//                             && loc2 == paths[a1][timestep + 1])
//                    {
//                        std::cout << "Agents " << a1 << " and " << a2 << " have edge conflict at (" <<
//                             loc1 << "-->" << loc2 << ") at timestep " << timestep << std::endl;
//                        return false;
//                    }
//                }
//                // yz: check conflict when one of them is stop
//                if (paths[a1].size() != paths[a2].size())
//                {
//                    int a1_ = paths[a1].size() < paths[a2].size() ? a1 : a2;
//                    int a2_ = paths[a1].size() < paths[a2].size() ? a2 : a1;
//                    Pointi<N> loc1 = paths[a1_].back();
//                    for (size_t timestep = min_path_length; timestep < paths[a2_].size(); timestep++)
//                    {
//                        Pointi<N> loc2 = paths[a2_][timestep];
//                        if (loc1 == loc2)
//                        {
//                            std::cout << "Agents " << a1_ << " and " << a2_ << " have vertex conflict 2 at " << loc1 << " at timestep " << timestep << std::endl;
//                            return false; // It's at least a semi conflict
//                        }
//                    }
//                }
                if(WhetherTwoPathConflict(paths[a1], paths[a2])) {
                    return false;
                }
            }
        }
        //std::cout << " TCBS get valid solution " << std::endl;
        return true;
    }

    template<Dimension N>
    std::vector<std::set<int> > pickCasesFromScene(int total_case_count,
                                                   const std::vector<int>& required_counts,
                                                   int instance_count) {
        std::vector<std::set<int> > retv;
        for(int i=0; i<instance_count; i++) {
            for(const int& required_count : required_counts) {
                std::set<int> instance; // current instance, id set of instance
                if(required_count >= total_case_count) {
                    for(int j=0; j<total_case_count; j++) {
                        instance.insert(j);
                    }
                    retv.push_back(instance);
                } else {
                    while(1) {
                        int current_pick = rand() % total_case_count;
                        if(instance.find(current_pick) == instance.end()) {
                            instance.insert(current_pick);
                            if(instance.size() == required_count) {
                                retv.push_back(instance);
                                break;
                            }
                        }
                    }
                }
            }
        }
        return retv;
    }

    extern size_t max_size_of_stack; // yz: add for statistics in every mapf method's key memory occupation factor
    extern size_t max_size_of_stack_layered; // yz: add for statistics in every mapf method's key memory occupation factor


    struct MSTimer {
        MSTimer() {
            start_time_ = std::chrono::steady_clock::now();
        }

        int elapsed() {
            end_time_ = std::chrono::steady_clock::now();
            return std::chrono::duration_cast<std::chrono::milliseconds>(end_time_ - start_time_).count();
        }

        void reset() {
            start_time_ = std::chrono::steady_clock::now();
        }

        std::chrono::steady_clock::time_point start_time_;
        std::chrono::steady_clock::time_point end_time_;
    };

}

#endif //FREENAV_CONFLICT_H
