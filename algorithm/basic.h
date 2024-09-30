//
// Created by yaozhuo on 2023/6/18.
//

#ifndef FREENAV_LAYERED_MAPF_TCBS_BASIC_H
#define FREENAV_LAYERED_MAPF_TCBS_BASIC_H

#include <memory>
#include <boost/unordered_map.hpp>
#include <boost/heap/pairing_heap.hpp>
#include <boost/unordered_set.hpp>

#include "../freeNav-base/basic_elements/point.h"

namespace freeNav::LayeredMAPF {

#define MAX_TIMESTEP INT_MAX / 2
#define MAX_COST INT_MAX / 2
#define MAX_NODES MAX<size_t> / 2

    template <Dimension N, typename NODE>
    struct TreeNode {

        TreeNode(const NODE& pa = nullptr) : pa_(pa) {}

        NODE pa_ = nullptr; // parent node

        std::vector<NODE> ch_ = {}; // children node, optional

    };

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
                size_t min_path_length = paths[a1].size() < paths[a2].size() ? paths[a1].size() : paths[a2].size();
                // yz: check conflict in common time range
                for (size_t timestep = 0; timestep < min_path_length; timestep++)
                {
                    Pointi<N> loc1 = paths[a1][timestep];
                    Pointi<N> loc2 = paths[a2][timestep];
                    // yz: vertex conflict error
                    if (loc1 == loc2)
                    {
                        std::cout << "Agents " << a1 << " and " << a2 << " have vertex conflict 1 at " << loc1 << " at timestep " << timestep << std::endl;
                        return false;
                    }
                    else if (timestep < min_path_length - 1
                             && loc1 == paths[a2][timestep + 1]
                             && loc2 == paths[a1][timestep + 1])
                    {
                        std::cout << "Agents " << a1 << " and " << a2 << " have edge conflict at (" <<
                             loc1 << "-->" << loc2 << ") at timestep " << timestep << std::endl;
                        return false;
                    }
                }
                // yz: check conflict when one of them is stop
                if (paths[a1].size() != paths[a2].size())
                {
                    int a1_ = paths[a1].size() < paths[a2].size() ? a1 : a2;
                    int a2_ = paths[a1].size() < paths[a2].size() ? a2 : a1;
                    Pointi<N> loc1 = paths[a1_].back();
                    for (size_t timestep = min_path_length; timestep < paths[a2_].size(); timestep++)
                    {
                        Pointi<N> loc2 = paths[a2_][timestep];
                        if (loc1 == loc2)
                        {
                            std::cout << "Agents " << a1_ << " and " << a2_ << " have vertex conflict 2 at " << loc1 << " at timestep " << timestep << std::endl;
                            return false; // It's at least a semi conflict
                        }
                    }
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
        return retv;
    }

}

#endif //FREENAV_CONFLICT_H
