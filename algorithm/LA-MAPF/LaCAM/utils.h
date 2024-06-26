//
// Created by yaozhuo on 2024/6/20.
//

#ifndef LAYEREDMAPF_UTILS_H
#define LAYEREDMAPF_UTILS_H

#include "../common.h"

namespace freeNav::LayeredMAPF::LA_MAPF::LaCAM {

    using Config = std::vector<size_t>;  // locations for all agents

    bool is_same_config(const Config &C1, const Config &C2) {
        const auto N = C1.size();
        for (size_t i = 0; i < N; ++i) {
            if (C1[i] != C2[i]) return false;
        }
        return true;
    }

    using Solution = std::vector<Config>;

    // low-level search node
    // yz: the size of constraint tree grow exponentially as number of agent increases, = pow(num_of_neighbor, num_of_agents)
    //     in the worst case
    struct Constraint {
        std::vector<int> who;
        std::vector<size_t> where;
        const int depth;

        Constraint() : who(std::vector<int>()), where(std::vector<size_t>()), depth(0) {
        }

        // who and where
        Constraint(Constraint *parent, const int& i, const size_t& v)
                : who(parent->who), where(parent->where), depth(parent->depth + 1) {
            who.push_back(i);
            where.push_back(v);
        }

        ~Constraint() {};
    };

    struct ConfigHasher {
        uint operator()(const Config &C) const {
            uint hash = C.size();
            for (auto &v : C) {
                hash ^= v + 0x9e3779b9 + (hash << 6) + (hash >> 2);
            }
            return hash;
        }
    };

    // high-level search node
    struct Node {
        const Config C;
        Node *parent;

        // for low-level search
        std::vector<float> priorities;
        std::vector<int> order;
        std::queue<Constraint *> search_tree; // yz: first in, first out

        int t = 0; // yz: time index of current node

        Node(Config _C, const std::vector<std::vector<int> >& D, Node *_parent = nullptr)
                : C(_C),
                  parent(_parent),
                  priorities(C.size(), 0),
                  order(C.size(), 0),
                  search_tree(std::queue<Constraint *>()) {
            search_tree.push(new Constraint());
            const auto N = C.size();

            // set priorities
            if (parent == nullptr) {
                // initialize
                // yz: set initial priority by each agent's distance to target
                for (size_t i = 0; i < N; ++i) priorities[i] = (float) D[i][C[i]] / N;
                t = 0;
            } else {
                // dynamic priorities, akin to PIBT
                for (size_t i = 0; i < N; ++i) {
                    // yz: inherit priority from previous agent
                    // yz: if not reach target, priority + 1
                    if (D[i][C[i]] != 0) {
                        priorities[i] = parent->priorities[i] + 1;
                    } else {
                        // yz: if reach target, decreases priority to (0, 1)
                        priorities[i] = parent->priorities[i] - (int) parent->priorities[i];
                    }
                }
                // yz: time step + 1
                t = _parent->t + 1;
            }

            // set order
            std::iota(order.begin(), order.end(), 0); // yz: fill with incremental sequence 0, 1, 2,
            // yz: the order of fill is determine by priorities
            std::sort(order.begin(), order.end(),
                      [&](int i, int j) { return priorities[i] > priorities[j]; });
        }

        ~Node() {
            while (!search_tree.empty()) {
                delete search_tree.front();
                search_tree.pop();
            }
        }
    };

    using Nodes = std::vector<Node *>;

    // PIBT agent
    struct Agent {
        const int id;
        size_t v_now;   // current location
        size_t v_next;  // next location
        Agent(int _id) : id(_id), v_now(-1), v_next(-1) {}
    };

    using Agents = std::vector<Agent *>;

// next location candidates, for saving memory allocation
    template <Dimension N>
    using Candidates = std::vector<std::array<size_t, 2*N*2*N + 1> >;

}


#endif //LAYEREDMAPF_UTILS_H
