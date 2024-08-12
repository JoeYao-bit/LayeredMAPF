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

    explicit HyperGraphNodeData(const size_t & current_node,
                                const HyperGraphNodeDataPtr<N>& parent,
                                const ConnectivityGraph& graph,
                                bool distinguish_sat = false,
                                const std::vector<bool>& ignore_cost_agent_ids = {}) :
            current_node_(current_node), graph_(graph), TreeNode<N, HyperGraphNodeDataPtr<N>>(parent) {
            if(parent != nullptr) {
                visited_agent_ = parent->visited_agent_;
                // if is a agent node, rather than a free group node
                for(const int& agent_id : graph_.hyper_node_with_agents_[current_node_]) {
                    if(!ignore_cost_agent_ids.empty() && ignore_cost_agent_ids[agent_id]) { continue; }
                    visited_agent_.insert(distinguish_sat ? agent_id : agent_id/2);
                }
//                std::cout << "visited_agent_ size = " << visited_agent_.size() << std::endl;
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
                return n1->visited_agent_.size() + n1->h_val_ >= n2->visited_agent_.size() + n2->h_val_;
            }
        };

        struct equal_node {
            // returns true if n1 > n2 (note -- this gives us *min*-heap).
            bool operator()(const HyperGraphNodeDataPtr<N> &n1, const HyperGraphNodeDataPtr<N> &n2) const {
                return n1->current_node_ == n2->current_node_;
            }
        };

        struct NodeHasher
        {
            size_t operator() (const HyperGraphNodeDataPtr<N>& n) const
            {
                return std::hash<int>()(n->current_node_); // yz: 按位异或
            }
        };

        bool in_openlist_ = false;

        typedef typename boost::heap::pairing_heap< HyperGraphNodeDataPtr<N>, boost::heap::compare<typename HyperGraphNodeData<N>::compare_node> >::handle_type open_handle_t;

        open_handle_t open_handle_;

    };


    // calculate static heuristic table, using BFS, dist to target = 0
    // dist is defined like g_val and h_val, how many agent need to cross to reach target
    template <Dimension N>
    std::vector<int> calculateLargeAgentHyperGraphStaticHeuristic(int agent_id, DimensionLength* dim, const ConnectivityGraph& graph, bool distinguish_sat = false) {
//        std::cout << "call " << __FUNCTION__ << std::endl;
        // the two table update simultaneously
        int total_index = getTotalIndexOfSpace<N>(dim);

        std::vector<int> heuristic_table(graph.all_edges_vec_.size(), MAX<int>);

        std::vector<HyperGraphNodeDataPtr<N> > current_set, all_ptr_set;

        HyperGraphNodeDataPtr<N> init_node_ptr = new HyperGraphNodeData<N>(graph.target_hyper_node_, nullptr, graph);
        init_node_ptr->visited_agent_ = { distinguish_sat ? 2*agent_id + 1 : agent_id };

        current_set.push_back(init_node_ptr);

        all_ptr_set.push_back(init_node_ptr);

        heuristic_table[init_node_ptr->current_node_] = init_node_ptr->visited_agent_.size(); // in target, related agent is itself, so heuristic_table = 1

        int current_h, next_h;

        int count = 0;
        while(!current_set.empty()) {
//            std::cout << "current_set.size " << current_set.size() << std::endl;
            std::vector<HyperGraphNodeDataPtr<N> > next_set;
            for(const auto& node_ptr : current_set) {

                current_h = heuristic_table[node_ptr->current_node_];

                for(const auto& neighbor_node_id : graph.all_edges_vec_[node_ptr->current_node_]) {

                    HyperGraphNodeDataPtr<N> next_node_data_ptr = new HyperGraphNodeData<N>(neighbor_node_id, node_ptr, graph, distinguish_sat);
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

    template<Dimension N>
    struct DependencyPathSearch {

        explicit DependencyPathSearch() {}

        // unordered_set
        std::set<int> getPassingAgents(const HyperGraphNodeDataPtr<N>& node_ptr) {
//            std::set<int> retv;
//            HyperGraphNodeDataPtr<N> buffer = node_ptr;
//            while(buffer != nullptr) {
//
//                std::set<int> raw_agent_ids = buffer->visited_agent_;
//                for(const auto& raw_agent_id : raw_agent_ids) {
//                    if(distinguish_sat) {
//                        retv.insert(raw_agent_id);
//                    } else {
//                        retv.insert(raw_agent_id / 2);
//                    }
//                }
//                buffer = buffer->pa_;
//            }
//            return retv;
            return node_ptr->visited_agent_;
        }

        /* determine the path for a agent in the hyper graph, considering avoidance for other agents */
        // search in a Best-First way or Breadth-First-way ? I prefer Best-First way
        // how to set heuristic value ? only considering the last agent node rather than free group node
        // agent in ignore_cost_set is not considered in cost accumulation
        // assume all start node is id of agent, as well as id in avoid_agents
        // return all agent involve in the path, if return empty set, considering as find no result
        // distinguish_sat means whether considering
        std::set<int> search(int agent_id,
                             int start_hyper_node_id,
                             const SubGraphOfAgent<N>& sub_graph,
                             const ConnectivityGraph& con_graph,
                             const std::vector<bool> &avoid_agents,
                             const std::vector<bool> &passing_agents,
                             const std::vector<int> heuristic_table,
                             bool distinguish_sat = false,
                             const std::vector<bool> & ignore_cost_set = {}) {
            const auto &start_node_id = con_graph.start_hyper_node_;
            assert(start_node_id != MAX<size_t>);

            // check whether avoid specific agents
            if(!avoid_agents.empty() && (avoid_agents[2*agent_id] || avoid_agents[2*agent_id+1]) ) { return {}; }
            // check whether passing specific agents, if passing_agents_ == empty, ignore this constraint
            if(!passing_agents.empty() && (!passing_agents[2*agent_id] || !passing_agents[2*agent_id+1])) { return {}; }

            HyperGraphNodeDataPtr<N> start_node = new HyperGraphNodeData<N>(start_hyper_node_id, nullptr, con_graph);

//            std::cout << "start_node cur and pre " << start_node << " / " << start_node->pa_ << std::endl;
            start_node->h_val_ = heuristic_table[start_hyper_node_id];
            start_node->in_openlist_ = true;
            start_node->visited_agent_ = { distinguish_sat ? 2*agent_id : agent_id };

            start_node->open_handle_ = open_list_.push(start_node);
            allNodes_table_.insert(start_node); // yz: visited vertex

            int count = 0;
            while (!open_list_.empty()) // yz: focal may be empty and this is legal !
            {
//                std::cout << "count = " << count << std::endl;
                count ++;
                auto curr_node = popNode();
                // check if the popped node is a goal
                if(curr_node->current_node_ == con_graph.target_hyper_node_) // if current node is belong to an agent
                {
                    auto passed_agents = getPassingAgents(curr_node);//curr_node->passed_agents_;
                    releaseNodes();
                    return passed_agents;
                }

                for(const auto& neighbor_node_id : con_graph.all_edges_vec_[curr_node->current_node_])
                {

                    bool avoid_failed = false, passing_failed = false;
                    for(const int& agent_id : con_graph.hyper_node_with_agents_[neighbor_node_id]) {
                        // check whether avoid specific agents
                        if(!avoid_agents.empty() && avoid_agents[agent_id]) { avoid_failed = true; }
                        // check whether passing specific agents, if passing_agents_ == empty, ignore this constraint
                        if(!passing_agents.empty() && passing_agents[agent_id] == false) { passing_failed = true; }
                    }

                    if(avoid_failed || passing_failed) { continue; }

                    auto next_node = new HyperGraphNodeData<N>(neighbor_node_id, curr_node, con_graph, distinguish_sat, ignore_cost_set);
                    next_node->h_val_ = heuristic_table[neighbor_node_id];
                    bool is_new_node = true;
                    // try to retrieve it from the hash table
                    auto it = allNodes_table_.find(next_node);
                    if (it == allNodes_table_.end())
                    {
                        pushNode(next_node);
                        allNodes_table_.insert(next_node); // yz: add to hash table
                        continue;
                    } else {
                        is_new_node = false;
                    }
                    // update existing node's if needed (only in the open_list)
                    auto existing_next = *it;

                    if (existing_next->getFVal() > next_node->getFVal())// if f-val decreased through this new path
                        // or it remains the same but there's fewer conflicts
                    {
                        if (!existing_next->in_openlist_) // if it is in the closed list (reopen)
                        {
                            existing_next->copy(*next_node);
                            pushNode(existing_next);
                        } else {
                            bool update_open = false;
                            if (existing_next->getFVal() > next_node->getFVal())
                                update_open = true;
                            existing_next->copy(*next_node);	// update existing node
                            if (update_open)
                                open_list_.increase(existing_next->open_handle_);  // increase because f-val improved
                        }
                    }
                    if(!is_new_node) {
                        delete next_node;
                    }
                }  // end for loop that generates successors
            }  // end while loop
            releaseNodes();
            std::cout << " search failed " << std::endl;
            return {};
        }

        inline HyperGraphNodeDataPtr<N> popNode() {
            auto node = open_list_.top(); open_list_.pop();
            node->in_openlist_ = false;
            return node;
        }

        inline void pushNode(const HyperGraphNodeDataPtr<N>& node) {
            node->open_handle_ = open_list_.push(node);
            node->in_openlist_ = true;
        }

        typedef boost::heap::pairing_heap<HyperGraphNodeDataPtr <N>, boost::heap::compare<typename HyperGraphNodeData<N>::compare_node> > heap_open_t;
        heap_open_t open_list_;

        // how about considering grid distance of path as focal list ?

        typedef boost::unordered_set<HyperGraphNodeDataPtr<N>, typename HyperGraphNodeData<N>::NodeHasher, typename HyperGraphNodeData<N>::equal_node> hashtable_t;
        hashtable_t allNodes_table_;

        void releaseNodes() {
            for(auto it = allNodes_table_.begin();it!=allNodes_table_.end();it++) {
                delete *it;
            }
            open_list_.clear();
            allNodes_table_.clear();
        }

    };


}

#endif //LAYEREDMAPF_LARGE_AGENT_DEPENDENCY_PATH_SEARCH_H
