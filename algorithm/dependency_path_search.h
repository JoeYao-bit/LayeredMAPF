//
// Created by yaozhuo on 2023/11/29.
//

#ifndef FREENAV_LAYERED_MAPF_DEPENDENCY_PATH_SEARCH_H
#define FREENAV_LAYERED_MAPF_DEPENDENCY_PATH_SEARCH_H

#include "instance_decomposition.h"
#include <boost/heap/pairing_heap.hpp>
#include <unordered_set>

namespace freeNav::LayeredMAPF {

    template<Dimension N>
    struct HyperGraphNode;

    template<Dimension N>
    using HyperGraphNodePtr = HyperGraphNode<N>*;

    template<Dimension N>
    struct HyperGraphNodeData;

    template<Dimension N>
    using HyperGraphNodeDataPtr = HyperGraphNodeData<N>*;

    template<Dimension N>
    using HyperGraphNodeDataPtrs = std::vector<HyperGraphNodeDataPtr<N> >;

    template<Dimension N>
    struct HyperGraphNodeData : public TreeNode<N, HyperGraphNodeDataPtr<N> > {

        explicit HyperGraphNodeData(const HyperGraphNodePtr<N>& current_node_ptr, const HyperGraphNodeDataPtr<N>& parent, bool ignore_cost = false) :
                current_node_ptr_(current_node_ptr), TreeNode<N, HyperGraphNodeDataPtr<N>>(parent) {
            if(parent != nullptr) {
                g_val_ = parent->g_val_;
                // if is a agent node, rather than a free group node
                if(current_node_ptr->agent_grid_ptr_ != nullptr && !ignore_cost) {
                    g_val_ ++;
                }
            } else {
                g_val_ = 1; // the agent itself is not considered as "passed"
            }
        }

        void copy(const HyperGraphNodeData<N>& other_node) {
            g_val_            = other_node.g_val_;
            h_val_            = other_node.h_val_;
            current_node_ptr_ = other_node.current_node_ptr_;
            this->pa_         = other_node.pa_;
            this->ch_         = other_node.ch_;
        }

        HyperGraphNodePtr<N> current_node_ptr_;

        // only considering agent grid it path, ignore free grid group it pass
        // how many agent it need to cross till reach target, if both start and target of an agent occur in the path, dist plus only one
        int g_val_ = MAX<int>; // dist from start to here

        int h_val_ = 0; // estimation dist from here to target

        int getFVal() {
            return g_val_ + h_val_;
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

    //template<Dimension N>
    //using HyperGraphNodePtr = HyperGraphNode<N>*;

    template<Dimension N>
    using HyperGraphNodePtrs = std::vector<HyperGraphNodePtr<N> >;

    // calculate static heuristic table, using BFS, dist to target = 0
    // dist is defined like g_val and h_val, how many agent need to cross to reach target
    template <Dimension N>
    std::vector<int> calculateHyperGraphStaticHeuristic(const HyperGraphNodePtr<N>& target_ptr, const HyperGraphNodePtrs<N> all_hyper_nodes) {

        // the two table update simultaneously
        std::vector<int> heuristic_table(all_hyper_nodes.size(), MAX<int>);
        std::vector<HyperGraphNodeDataPtr<N> > nearest_node_table(all_hyper_nodes.size(), nullptr);

        HyperGraphNodeDataPtrs<N> current_set, all_ptr_set;

        HyperGraphNodeDataPtr<N> init_data_ptr = new HyperGraphNodeData<N>(target_ptr, nullptr);
        all_ptr_set.push_back(init_data_ptr);
        current_set.push_back(init_data_ptr);

        heuristic_table[target_ptr->hyper_node_id_] = 0; // in target, dist to target = 0
        nearest_node_table[target_ptr->hyper_node_id_] = nullptr;
        //std::cout << " target_ptr->hyper_node_id_ " << target_ptr->hyper_node_id_ << std::endl;
        int current_h, next_h;

        int target_id = target_ptr->agent_grid_ptr_->agent_id_;

        assert(target_id%2 == 1);
        int start_id = target_id - 1;
        int count = 0;
        while(!current_set.empty()) {
            //std::cout << "-- " << count << " iteration, current_set size " << current_set.size() << std::endl;
            HyperGraphNodeDataPtrs<N> next_set;
            for(const auto& node_ptr : current_set) {

                current_h = heuristic_table[node_ptr->current_node_ptr_->hyper_node_id_];
                //std::cout << " current_h " << current_h << std::endl;

                for(const auto& neighbor_node_id : node_ptr->current_node_ptr_->connecting_nodes_) {
                    //std::cout << "neighbor_node_id " << neighbor_node_id << std::endl;
                    const auto& neighbor_node_ptr = all_hyper_nodes[neighbor_node_id];
                    //std::cout << "neighbor_node_ptr->hyper_node_id_ " << neighbor_node_ptr->hyper_node_id_ << std::endl;

                    HyperGraphNodeDataPtr<N> next_node_data_ptr = new HyperGraphNodeData<N>(neighbor_node_ptr, node_ptr);
                    all_ptr_set.push_back(next_node_data_ptr);
                    next_h = next_node_data_ptr->g_val_;
                    //std::cout << " next_h " << next_h << std::endl;
                    //std::cout << " heuristic_table[neighbor_node_ptr->hyper_node_id_] " << heuristic_table[neighbor_node_ptr->hyper_node_id_] << std::endl;
                    if(heuristic_table[neighbor_node_ptr->hyper_node_id_] > next_h) {
                        heuristic_table[neighbor_node_ptr->hyper_node_id_] = next_h;
                        nearest_node_table[neighbor_node_ptr->hyper_node_id_] = next_node_data_ptr;
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
    struct HyperGraphNodePathSearch {

        explicit HyperGraphNodePathSearch() {}

        // unordered_set
        std::set<int> getPassingAgents(const HyperGraphNodeDataPtr<N>& node_ptr, bool distinguish_sat = false) {
            std::set<int> retv;
            HyperGraphNodeDataPtr<N> buffer = node_ptr;
            while(buffer != nullptr) {
                //std::cout << " buffer " << buffer << std::endl;
                if(buffer->current_node_ptr_->agent_grid_ptr_ != nullptr) {
                    int raw_agent_id = buffer->current_node_ptr_->agent_grid_ptr_->agent_id_;
                    if(distinguish_sat) {
                        retv.insert(raw_agent_id);
                    } else {
                        retv.insert(raw_agent_id / 2);
                    }
                }
                buffer = buffer->pa_;
            }
            return retv;
        }

        /* determine the path for a agent in the hyper graph, considering avoidance for other agents */
        // search in a Best-First way or Breadth-First-way ? I prefer Best-First way
        // how to set heuristic value ? only considering the last agent node rather than free group node
        // agent in ignore_cost_set is not considered in cost accumulation
        // assume all start node is id of agent, as well as id in avoid_agents
        // return all agent involve in the path, if return empty set, considering as find no result
        // distinguish_sat means whether considering
        std::set<int> search(int hyper_node_id,
                             const HyperGraphNodePtrs<N> all_hyper_nodes,
                             const std::set<int> &avoid_agents,
                             const std::set<int> &passing_agents,
                             const std::vector<int> heuristic_table,
                             bool distinguish_sat = false, const std::set<int>& ignore_cost_set = {}) {
            const auto &start_node_ptr = all_hyper_nodes[hyper_node_id];
            assert(start_node_ptr->agent_grid_ptr_ != nullptr);

            if(start_node_ptr != nullptr) {
                // check whether avoid specific agents
                if(avoid_agents.find(start_node_ptr->agent_grid_ptr_->agent_id_) != avoid_agents.end()) { return {}; }
                // check whether passing specific agents, if passing_agents_ == empty, ignore this constraint
                if(!passing_agents.empty() && passing_agents.find(start_node_ptr->agent_grid_ptr_->agent_id_) == passing_agents.end()) { return {}; }
            }

            const int &current_agent_id = start_node_ptr->agent_grid_ptr_->agent_id_;
            assert(current_agent_id % 2 == 0); // must start from agent's start point
            int other_node_id = current_agent_id + 1;

            HyperGraphNodeDataPtr<N> start_node = new HyperGraphNodeData<N>(start_node_ptr, nullptr);
//            std::cout << "start_node cur and pre " << start_node << " / " << start_node->pa_ << std::endl;
            start_node->h_val_ = heuristic_table[hyper_node_id];
            start_node->open_handle_ = open_list_.push(start_node);
            start_node->in_openlist_ = true;
            allNodes_table_.insert(start_node); // yz: visited vertex

            int count = 0;
            while (!open_list_.empty()) // yz: focal may be empty and this is legal !
            {
//                std::cout << " count = " << count << ", open_list_.size() " << open_list_.size() << std::endl;
                count ++;
                //if(count >= 200) return {};//exit(0);
                auto curr_node = popNode();
                // check if the popped node is a goal
                if(curr_node->current_node_ptr_->agent_grid_ptr_ != nullptr) // if current node is belong to an agent
                {
                    if(curr_node->current_node_ptr_->agent_grid_ptr_->agent_id_ == other_node_id) { // reach target agent
//                        std::cout << " HyperGraphNodePathSearch reach target" << std::endl;
//                        std::cout << "start_node cur and pre " << start_node << " / " << start_node->pa_ << std::endl;

                        auto passed_agents = getPassingAgents(curr_node, distinguish_sat);//curr_node->passed_agents_;
                        releaseNodes();
                        return passed_agents;
                    }
                }

                for(const auto& neighbor_node_id : curr_node->current_node_ptr_->connecting_nodes_)
                {
                    const auto& neighbor_node_ptr = all_hyper_nodes[neighbor_node_id];
                    // if current node is of an agent, check whether it in the avoidance list
                    if(neighbor_node_ptr->agent_grid_ptr_ != nullptr) {
                        // check whether avoid specific agents
                        if(avoid_agents.find(neighbor_node_ptr->agent_grid_ptr_->agent_id_) != avoid_agents.end()) { continue; }
                        // check whether passing specific agents, if passing_agents_ == empty, ignore this constraint
                        if(!passing_agents.empty() && passing_agents.find(neighbor_node_ptr->agent_grid_ptr_->agent_id_) == passing_agents.end()) { continue; }
                    }
                    bool ignore_cost = false;
                    if(!ignore_cost_set.empty() && neighbor_node_ptr->agent_grid_ptr_ != nullptr) {
                        ignore_cost = ignore_cost_set.find(neighbor_node_ptr->agent_grid_ptr_->agent_id_) != ignore_cost_set.end();
                    }
//                    std::cout << "start_node 1 cur and pre " << start_node << " / " << start_node->pa_ << std::endl;
                    auto next_node = new HyperGraphNodeData<N>(neighbor_node_ptr, curr_node, ignore_cost);
                    next_node->h_val_ = heuristic_table[neighbor_node_id];

//                    std::cout << "start_node 2 cur and pre " << start_node << " / " << start_node->pa_ << std::endl;
//                    std::cout << "start_node g_val h_val " << start_node->g_val_ << " / " << start_node->h_val_ << std::endl;
//
//                    std::cout << "next_node g_val h_val " << next_node->g_val_ << " / " << next_node->h_val_ << std::endl;
//                    std::cout << "next_node cur and pre " << next_node << " / " << next_node->pa_ << std::endl;
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
//                            std::cout << " existing_next->getFVal() > curr_node->getFVal() " << existing_next->getFVal() << " > "
//                                      <<  curr_node->getFVal() << std::endl;
//                            std::cout << " reinsert " << existing_next << std::endl;
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
            // std::cout << " HyperGraphNodePathSearch reach failed" << std::endl;
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
                //std::cout<<*it<<" ";
                delete *it;
                //*it = nullptr;
            }
            open_list_.clear();
            allNodes_table_.clear();
        }

    };


}

#endif //FREENAV_DEPENDENCY_PATH_SEARCH_H
