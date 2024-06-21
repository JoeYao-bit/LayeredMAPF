/*
 * LaCAM algorithm
 */
#ifndef LA_AGENT_LACAM_H
#define LA_AGENT_LACAM_H

#include "../common.h"
#include "utils.h"
#include "../circle_shaped_agent.h"
#include "../block_shaped_agent.h"
#include "../large_agent_mapf.h"

namespace freeNav::LayeredMAPF::LA_MAPF::LaCAM {

    template<Dimension N, typename AgentType>
    struct LargeAgentLaCAM : public LargeAgentMAPF<N, AgentType> {

        LargeAgentLaCAM(const InstanceOrients<N> & instances,
                        const std::vector<AgentType>& agents,
                        DimensionLength* dim,
                        const IS_OCCUPIED_FUNC<N> & isoc,
                        std::mt19937 *_MT)
                        : LargeAgentMAPF<N, AgentType>(instances, agents, dim, isoc),
                          V_size(LargeAgentLaCAM<N, AgentType>::all_poses_.size()),
                          MT(_MT),
                          C_next(Candidates(N, std::array<size_t , 5>())), // yz: 5 means 4 direction + 1 wait
                          tie_breakers(std::vector<float>(V_size, 0)),
                          A(Agents(N, nullptr)),
                          occupied_now(Agents(V_size, nullptr)),
                          occupied_next(Agents(V_size, nullptr)) {
            starts.resize(agents.size());
            targets.resize(agents.size());
            for(size_t agent=0; agent<agents.size(); agent++) {
                starts[agent]  = this->instance_node_ids_[agent].first;
                targets[agent] = this->instance_node_ids_[agent].second;
            }
        }

        std::vector<LAMAPF_Path> transferToSerialPath(const Solution& solution) const {
            // yz: transform to freeNav style path
            std::vector<LAMAPF_Path> retv(solution.front().size());
            for (int t = 0; t < solution.size(); t++) {
                assert(solution[t].size() == this->instance_node_ids_.size());
                for (int agent = 0; agent < solution[t].size(); agent++) {
                    retv[agent].push_back(solution[t][agent]); // this->transferToPose
                    assert(retv[agent].size() - 1 == t);
                }
            }
            return retv;
        }

        bool solve(double time_limit, int cost_lowerbound = 0, int cost_upperbound = MAX_COST) override {

            // setup agents
            for (auto i = 0; i < N; ++i) A[i] = new Agent(i);

            // setup search queues
            std::stack<Node *> OPEN; // yz: std::stack 先进后出（FILO）
            std::unordered_map<Config, Node *, ConfigHasher> CLOSED;
            std::vector<Constraint *> GC;  // garbage collection of constraints

            // insert initial node
            // yz: take all start position as start state
            auto S = new Node(starts, this->agents_heuristic_tables_);
            OPEN.push(S);
            CLOSED[S->C] = S;

            // depth first search
            int loop_cnt = 0;
            std::vector<Config> solution;

            while (!OPEN.empty()) {
            std::cout << "-- high level count " << loop_cnt << " size " << OPEN.size() << std::endl;
                loop_cnt += 1;

                // do not pop here!
                S = OPEN.top();
                // check goal condition
                if (is_same_config(S->C, targets)) {
                    // backtrack
                    // yz: high level search in increasing time index, all agents move simultaneously
                    while (S != nullptr) {
                        solution.push_back(S->C);
                        S = S->parent;
                    }
                    std::reverse(solution.begin(), solution.end());
                    break;
                }

                // low-level search end
                // yz: if have split all current high node's branches (split a constraint to get two branches)
                if (S->search_tree.empty()) {
                    OPEN.pop();
                    continue;
                }

                // create successors at the low-level search
                // yz: search_tree update in a DFS mode,
                // yz: if can move to next depth, keep move, otherwise current state is pop out
                // yz: constraint will traversal all current possible combination of agent move
                // yz: so current config is part of solution, it can always find next solution,
                // yz: so this algorithm is complete
                auto M = S->search_tree.front();
                GC.push_back(M);
                S->search_tree.pop();
                if (M->depth < N) {
                    auto i = S->order[M->depth]; // yz: add constraint in order of priority, which is fixed

//                    auto C = S->C[i]->neighbor;
//                    C.push_back(S->C[i]);
                    auto C = this->agent_sub_graphs_[i].all_edges_[S->C[i]];
                    C.push_back(S->C[i]);
                    if (MT != nullptr) std::shuffle(C.begin(), C.end(), *MT);  // randomize
                    for (auto u : C) {
                        //yz: add current agent's all possible candidates as constraint
                        S->search_tree.push(new Constraint(M, i, u));
                    }
                }
                // create successors at the high-level search
                // yz: try search next config under current config S and constraint M
                // yz: if failed, try add more constraint and find again
                std::cout << "S = " << S << " / M = " << M << std::endl;
                if (!get_new_config(S, M)) continue;

                // create new configuration
                auto C = Config(N, -1);
                for (auto a : A) {
                    C[a->id] = a->v_next;
                }

                // check explored list
                // yz: if the state has been explored, do not add it again, but reinsert it to OPEN
                // yz: LaCAM do not considering the situation that all agent wait, cause cluster 37 failed
                auto iter = CLOSED.find(C);
                if (iter != CLOSED.end()) {
                    OPEN.push(iter->second);
                    continue;
                }

                // insert new search node
                auto S_new = new Node(C, this->agents_heuristic_tables_, S);
                OPEN.push(S_new);
                CLOSED[S_new->C] = S_new;
            }

//            info(1, verbose, "elapsed:", elapsed_ms(deadline), "ms\t",
//                 solution.empty() ? (OPEN.empty() ? "no solution" : "failed")
//                                  : "solution found",
//                 "\tloop_itr:", loop_cnt, "\texplored:", CLOSED.size());
            // memory management
            for (auto a : A) delete a;
            for (auto M : GC) delete M;
            for (auto p : CLOSED) delete p.second;
            if(solution.empty()) { return false; }
            this->solutions_ = transferToSerialPath(solution);
            this->removeStopAfterReachTarget(this->solutions_);

            return true;
        }

        bool get_new_config(Node *S, Constraint *M) {
            std::cout << " flag1 " << std::endl;
            // setup cache
            for (auto a : A) {
                // clear previous cache
                if (a->v_now != -1 && occupied_now[a->v_now] == a) {
                    occupied_now[a->v_now] = nullptr;
                }
                if (a->v_next != -1) {
                    occupied_next[a->v_next] = nullptr;
                    a->v_next = -1;
                }

                // set occupied now
                a->v_now = S->C[a->id]; // yz: current config is saved in S->C
                occupied_now[a->v_now] = a;
            }
            std::cout << " flag2 " << std::endl;

            // add constraints
            // // yz: constraint determine next config without search, which is saved M
            for (auto k = 0; k < M->depth; ++k) {
                const auto i = M->who[k];        // agent
                const auto l = M->where[k];  // loc

                // check vertex collision
                if (occupied_next[l] != nullptr) return false;
                // check swap collision
                auto l_pre = S->C[i];
                if (occupied_next[l_pre] != nullptr && occupied_now[l] != nullptr &&
                    occupied_next[l_pre]->id == occupied_now[l]->id)
                    return false;

                // set occupied_next
                A[i]->v_next = M->where[k];
                occupied_next[l] = A[i];
            }
            std::cout << " flag3 " << std::endl;

            // perform PIBT
            // yz: perform PIBT for each agent in predefined order
            // yz: but as PIBT is recursive, the REAL of search path is not deterministic
            for (auto k : S->order) {
                auto a = A[k];
                if (a->v_next == -1 && !funcPIBT(a, S->t + 1)) {
                    return false;  // planning failure
                }
            }
            std::cout << " flag4 " << std::endl;
            return true;
        }

        bool funcPIBT(Agent *ai, int next_t) {
            const auto i = ai->id;
            const auto& neighbor = this->agent_sub_graphs_[ai->id].all_edges_[ai->v_now];
            const auto K = neighbor.size();

            // get candidates for next locations
            for (size_t k = 0; k < K; ++k) {
                auto u = neighbor[k];
                C_next[i][k] = u;
                if (MT != nullptr)
                    tie_breakers[u] = get_random_float(MT);  // set tie-breaker
            }
            C_next[i][K] = ai->v_now;

            // sort, note: K + 1 is sufficient
            // yz: randomize future locations candidates

            std::sort(C_next[i].begin(), C_next[i].begin() + K + 1,
                      [&](const size_t& v, const size_t& u) {
                          return this->agents_heuristic_tables_[i][v] + tie_breakers[v] <
                                 this->agents_heuristic_tables_[i][u] + tie_breakers[u];
                      });

            // yz: for all current agent's neighbor and wait
            for (size_t k = 0; k < K + 1; ++k) {
                auto u = C_next[i][k];

                // avoid vertex conflicts
                if (occupied_next[u] != nullptr) continue;

                auto &ak = occupied_now[u]; // yz: who occupied this neighbor

                // avoid swap conflicts with constraints
                // yz: avoid swap conflicts with neighbor agents
                if (ak != nullptr && ak->v_next == ai->v_now) continue;

                // reserve next location
                // yz: assume current agent can move to next location
                occupied_next[u] = ai;
                ai->v_next = u;

                // empty or stay
                // yz: if not all neighbor are occupied or current agent just stay
                if (ak == nullptr || u == ai->v_now) return true;

                // priority inheritance
                // yz: if neighboring agent is not move yet
                // yz: then try to move it
                if (ak->v_next == -1 && !funcPIBT(ak, next_t)) continue;

                // success to plan next one step
                return true;
            }

            // failed to secure node
            // yz: when search new config failed,
            occupied_next[ai->v_now] = ai;
            ai->v_next = ai->v_now;
            return false;
        }

//        const Instance *ins;
//        const Deadline *deadline;
        std::mt19937 *MT;

        // solver utils
        //const int N;  // number of agents
        const int V_size;
//        DistTable D;
        Candidates C_next;                // next location candidates
        std::vector<float> tie_breakers;  // random values, used in PIBT
        Agents A;
        Agents occupied_now;   // for quick collision checking
        Agents occupied_next;  // for quick collision checking

        Config starts, targets;

    };

}

#endif