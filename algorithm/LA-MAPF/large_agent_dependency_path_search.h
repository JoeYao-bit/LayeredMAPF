//
// Created by yaozhuo on 2024/8/12.
//

#ifndef LAYEREDMAPF_LARGE_AGENT_DEPENDENCY_PATH_SEARCH_H
#define LAYEREDMAPF_LARGE_AGENT_DEPENDENCY_PATH_SEARCH_H

#include "common.h"
#include <boost/heap/pairing_heap.hpp>
#include <unordered_set>

namespace freeNav::LayeredMAPF::LA_MAPF {

    template<Dimension N>
    struct HyperGraphNodeData;

    template<Dimension N>
    using HyperGraphNodeDataPtr = HyperGraphNodeData<N>*;

    template<Dimension N>
    struct HyperGraphNodeData : public TreeNode<N, HyperGraphNodeDataPtr<N> > {

    explicit HyperGraphNodeData(const size_t & current_node, const HyperGraphNodeDataPtr<N>& parent,
                                const ConnectivityGraph& graph, bool ignore_cost = false) :
            current_node_(current_node), graph_(graph), TreeNode<N, HyperGraphNodeDataPtr<N>>(parent) {
            if(parent != nullptr) {
                visited_agent_ = parent->visited_agent_;
                // if is a agent node, rather than a free group node
                if(!ignore_cost) {
                    for(const int& agent_id : graph_.hyper_node_with_agents_[current_node_]) {
                        visited_agent_.insert(agent_id);
                    }
                }
            }
        }

        void copy(const HyperGraphNodeData<N>& other_node) {
            visited_agent_    = other_node.visited_agent_;
            h_val_            = other_node.h_val_;
            current_node_     = other_node.current_node_;
            this->pa_         = other_node.pa_;
            this->ch_         = other_node.ch_;
        }

        size_t current_node_;

        // only considering agent grid it path, ignore free grid group it pass
        // how many agent it need to cross till reach target, if both start and target of an agent occur in the path, dist plus only one
        // int g_val_ = MAX<int>; // dist from start to here
        // g_val = visited_agent_.size()

        std::set<int> visited_agent_;

        int h_val_ = 0; // estimation dist from here to target

        const ConnectivityGraph& graph_;

        int getFVal() {
            return visited_agent_.size() + h_val_;
        }

        struct compare_node {
            // returns true if n1 > n2 (note -- this gives us *min*-heap).
            bool operator()(const HyperGraphNodeDataPtr<N> &n1, const HyperGraphNodeDataPtr<N> &n2) const {
                return n1->g_val_ + n1->h_val_ >= n2->g_val_ + n2->h_val_;
            }
        };

        struct equal_node {
            // returns true if n1 > n2 (note -- this gives us *min*-heap).
            bool operator()(const HyperGraphNodeDataPtr<N> &n1, const HyperGraphNodeDataPtr<N> &n2) const {
                return n1->current_node_ptr_->hyper_node_id_ == n2->current_node_ptr_->hyper_node_id_;
            }
        };

        struct NodeHasher
        {
            size_t operator() (const HyperGraphNodeDataPtr<N>& n) const
            {
                return std::hash<int>()(n->current_node_ptr_->hyper_node_id_); // yz: 按位异或
            }
        };

        bool in_openlist_ = false;

        typedef typename boost::heap::pairing_heap< HyperGraphNodeDataPtr<N>, boost::heap::compare<typename HyperGraphNodeData<N>::compare_node> >::handle_type open_handle_t;

        open_handle_t open_handle_;

    };


    // calculate static heuristic table, using BFS, dist to target = 0
    // dist is defined like g_val and h_val, how many agent need to cross to reach target
    template <Dimension N>
    std::vector<int> calculateLargeAgentHyperGraphStaticHeuristic(DimensionLength* dim, const ConnectivityGraph& graph) {
//        std::cout << "call " << __FUNCTION__ << std::endl;
        // the two table update simultaneously
        int total_index = getTotalIndexOfSpace<N>(dim);

        std::vector<int> heuristic_table(graph.all_edges_vec_.size(), MAX<int>);

        std::vector<HyperGraphNodeDataPtr<N> > current_set, all_ptr_set;

        HyperGraphNodeDataPtr<N> init_node_ptr = new HyperGraphNodeData<N>(graph.target_hyper_node_, nullptr, graph);

        current_set.push_back(init_node_ptr);

        all_ptr_set.push_back(init_node_ptr);

        heuristic_table[init_node_ptr->current_node_] = 0; // in target, dist to target = 0

        int current_h, next_h;

        int count = 0;
        while(!current_set.empty()) {
//            std::cout << "current_set.size " << current_set.size() << std::endl;
            std::vector<HyperGraphNodeDataPtr<N> > next_set;
            for(const auto& node_ptr : current_set) {

                current_h = heuristic_table[node_ptr->current_node_];

                for(const auto& neighbor_node_id : graph.all_edges_vec_[node_ptr->current_node_]) {

                    HyperGraphNodeDataPtr<N> next_node_data_ptr = new HyperGraphNodeData<N>(neighbor_node_id, node_ptr, graph);
                    all_ptr_set.push_back(next_node_data_ptr);

                    next_h = next_node_data_ptr->visited_agent_.size();

                    if(heuristic_table[neighbor_node_id] > next_h) {
                        heuristic_table[neighbor_node_id] = next_h;
//                        std::cout << "set hyper node " << neighbor_node_id << " heuristic value to " << next_h << std::endl;
                        next_set.push_back(next_node_data_ptr);
                    }
                }
            }
            current_set.clear();
            std::swap(current_set, next_set);
            count ++;
        }
        // release data
        for(auto& a_ptr : all_ptr_set) {
            delete a_ptr;
            a_ptr = nullptr;
        }
        // use BFS to calculate heuristic value for all free grid, and obstacle heuristic
        return heuristic_table;
    }

}

#endif //LAYEREDMAPF_LARGE_AGENT_DEPENDENCY_PATH_SEARCH_H
