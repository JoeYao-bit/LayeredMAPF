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
#include "large_agent_constraint.h"
#include <random>
namespace freeNav::LayeredMAPF::LA_MAPF::LaCAM {

    template<Dimension N, typename ConstraintTable, typename State>
    struct LargeAgentLaCAM : public LargeAgentMAPF<N, State> {

        LargeAgentLaCAM(const std::vector<std::pair<Pointi<N>, Pointi<N>>> & instances,
                        const std::vector<AgentPtr<N> >& agents,
                        DimensionLength* dim,
                        const IS_OCCUPIED_FUNC<N> & isoc,
                        LargeAgentStaticConstraintTablePtr<N, State> path_constraint,
                        const std::vector<std::pair<size_t, size_t> >& instance_node_ids,
                        const std::vector<std::shared_ptr<State> >& all_poses,
                        const DistanceMapUpdaterPtr<N>& distance_map_updater,
                        const std::vector<SubGraphOfAgent<N, State> >& agent_sub_graphs,
                        const std::vector<std::vector<int> >& agents_heuristic_tables,
                        const std::vector<std::vector<int> >& agents_heuristic_tables_ignore_rotate,

                        double time_limit = 60
                        )
                        : LargeAgentMAPF<N, State>(instances, agents, dim, isoc,
                                                   instance_node_ids,
                                                       all_poses,
                                                       distance_map_updater,
                                                       agent_sub_graphs,
                                                       agents_heuristic_tables,
                                                       agents_heuristic_tables_ignore_rotate,
                                                       time_limit),
                          V_size_(LargeAgentLaCAM<N, ConstraintTable, State>::all_poses_.size()),
                          C_next_(Candidates<N>(agents.size(), std::array<size_t , 2*N*2*N + 1>())), // yz: possible rotation multiple possible transition plus wait
                          tie_breakers(std::vector<float>(V_size_, 0)),
                          A_(AgentLaCAMs(agents.size(), nullptr)),
                          occupied_now_(AgentLaCAMs(V_size_, nullptr)),
                          occupied_next_(AgentLaCAMs(V_size_, nullptr)),
                          path_constraint_(path_constraint) {
            starts_.resize(agents.size());
            targets_.resize(agents.size());
            for(size_t agent=0; agent<agents.size(); agent++) {
                starts_[agent]  = this->instance_node_ids_[agent].first;
                targets_[agent] = this->instance_node_ids_[agent].second;
            }
            std::random_device rd;
            const auto seed = rd();
            MT = new std::mt19937(seed);
        }

        ~LargeAgentLaCAM() {
            delete MT;
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

        bool solve(int cost_lowerbound = 0, int cost_upperbound = MAX_COST) override {
            if(this->remaining_time_ <= 0) { return false; }
            auto start_time = clock();

            // setup agents
            for (auto i = 0; i < this->instance_node_ids_.size(); ++i) A_[i] = new AgentLaCAM(i);

            // setup search queues
            std::stack<Node *> OPEN; // yz: std::stack 先进后出（FILO）
            std::unordered_map<Config, Node *, ConfigHasher> CLOSED;
            std::vector<Constraint *> GC;  // garbage collection of constraints

            // insert initial node
            // yz: take all start position as start state
            auto S = new Node(starts_, this->agents_heuristic_tables_);
            OPEN.push(S);
            CLOSED[S->C] = S;

            // depth first search
            int loop_cnt = 0;
            std::vector<Config> solution;

            while (!OPEN.empty()) {
                //std::cout << "-- high level count " << loop_cnt << " size " << OPEN.size() << std::endl;
                loop_cnt += 1;
                if(loop_cnt >= 100) {
                    auto current_time = clock();
                    if(((double)((current_time - start_time))/CLOCKS_PER_SEC) >= this->remaining_time_) {
                        // run out of time
                        std::cout << "NOTICE: LA-LaCAM run out of time " << std::endl;
                        for (auto a : A_) delete a;
                        for (auto M : GC) delete M;
                        for (auto p : CLOSED) delete p.second;
                        return false;
                    }
                }
                // do not pop here!
                S = OPEN.top();
                // check goal condition
                if (is_same_config(S->C, targets_)) {

                    //std::cout << "-- final S->t = " << S->t << std::endl;
                    // backtrack
                    // yz: high level search in increasing time index, all agents move simultaneously
                    while (S != nullptr) {
//                        std::cout << " heuristic = ";
//                        for(int i=0; i<this->instance_node_ids_.size(); i++) {
//                            std::cout << this->agents_heuristic_tables_[i][S->C[i]] << "|"
//                                      << this->distance_to_target[i][S->C[i]/(2*N)] << ", candidate: ";
//                            for(const auto& c : this->agent_sub_graphs_[i].all_edges_[S->C[i]]) {
//                                std::cout << this->agents_heuristic_tables_[i][c] << "|"
//                                          << this->distance_to_target[i][c/(2*N)] << ", ";
//                            }
//                        }
//                        std::cout << std::endl;
                        solution.push_back(S->C);
                        S = S->parent;
                    }
                    std::reverse(solution.begin(), solution.end());
                    if(!is_feasible_solution(solution)) {
                        std::cerr << "-- solution is not feasible" << std::endl;
                    }
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
                // yz: random distribution of constraint may cause path far away from shortest path
                auto M = S->search_tree.front();
                GC.push_back(M);
                S->search_tree.pop();
                if (M->depth < this->instance_node_ids_.size()) {
                    auto i = S->order[M->depth]; // yz: add constraint in order of priority, which is fixed

//                    auto C = S->C[i]->neighbor;
//                    C.push_back(S->C[i]);
                    auto C = this->agent_sub_graphs_[i].data_ptr_->all_edges_[S->C[i]];
                    C.push_back(S->C[i]);
                    if (MT != nullptr) std::shuffle(C.begin(), C.end(), *MT);  // randomize

                    // yz: me try to limit very agent move in its fastest way to target
                    std::sort(C.begin(), C.end(),
                              [&](const size_t& v, const size_t& u) {
                                  if(this->agents_heuristic_tables_[i][v] == this->agents_heuristic_tables_[i][u]) {
//                                      return distance_to_target[i][v/(2*N)] < distance_to_target[i][u/(2*N)];
                                      return this->agents_heuristic_tables_ignore_rotate_[i][v/(2*N)] <
                                             this->agents_heuristic_tables_ignore_rotate_[i][u/(2*N)];
                                  } else {
                                      return this->agents_heuristic_tables_[i][v] < this->agents_heuristic_tables_[i][u];
                                  }
//                                  if(distance_to_target[i][v/(2*N)] == distance_to_target[i][u/(2*N)]) {
//                                      return this->agents_heuristic_tables_[i][v] < this->agents_heuristic_tables_[i][u];
//                                  } else {
//                                      return distance_to_target[i][v/(2*N)] < distance_to_target[i][u/(2*N)];
//                                  }
                              });

//                    std::cout << "after sort 1: ";
//                    for(const auto& sort_c : C) {
//                        std::cout << this->agents_heuristic_tables_[i][sort_c] << "|"
//                                  << this->distance_to_target[i][sort_c/(2*N)] << "|"
//                                  << *this->all_poses_[sort_c] << ", ";
//                    }
//                    std::cout << std::endl;

                    for (auto u : C) {
                        // avoid conflict with external agents
//                        if(path_constraint != nullptr) {
//                            if(path_constraint->hasCollideWithSAT(this->agent_sub_graphs_[i].agent_.id_, A[i]->v_now, u)) {
//                                std::cout << "LaCAM hasCollideWithSAT_0: "
//                                          << this->agent_sub_graphs_[i].agent_ << ", "
//                                          << *this->agent_sub_graphs_[i].all_nodes_[A[i]->v_now] << "->"
//                                          << *this->agent_sub_graphs_[i].all_nodes_[u]
//                                          << std::endl;
//                                continue;
//                            }
//                        }

                        //yz: add current agent's all possible candidates as constraint
                        S->search_tree.push(new Constraint(M, i, u));
                    }
                }
                // create successors at the high-level search
                // yz: try search next config under current config S and constraint M
                // yz: if failed, try add more constraint and find again
//                std::cout << "S = " << S << " / M = " << M << std::endl;
                if (!get_new_config(S, M)) {
//                    std::cout << " get_new_config failed" << std::endl;
                    continue;
                }

                // avoid external path constraint
                bool pass_check = true;
                for(int agent_id=0; agent_id<this->agents_.size(); agent_id++) {
                    if(path_constraint_ != nullptr) {
                        if(path_constraint_->hasCollideWithSAT(this->agent_sub_graphs_[agent_id].agent_->id_,
                                                              A_[agent_id]->v_now,
                                                              A_[agent_id]->v_next)) {
//                            std::cout << "LaCAM hasCollideWithSAT_0: "
//                                      << this->agent_sub_graphs_[agent_id].agent_ << ", "
//                                      << *this->agent_sub_graphs_[agent_id].all_nodes_[A[agent_id]->v_now] << "->"
//                                      << *this->agent_sub_graphs_[agent_id].all_nodes_[A[agent_id]->v_next]
//                                      << std::endl;
                            pass_check = false;
                        }
                    }
                }

                if(!pass_check) { continue; }

                // create new configuration
                auto C = Config(this->instance_node_ids_.size(), -1);
                for (auto a : A_) {
                    C[a->id] = a->v_next;
                }

                // check explored list
                // yz: if the state has been explored, add it again, reinsert it to OPEN
                // yz: LaCAM do not considering the situation that all agent wait, cause cluster 37 failed
                auto iter = CLOSED.find(C);
                if (iter != CLOSED.end()) {
                    OPEN.push(iter->second);
//                    std::cout << " reinsert " << std::endl;
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

            std::cout << "-- after " << loop_cnt << " LaCAM finish " << std::endl;
            // memory management
            for (auto a : A_) delete a;
            for (auto M : GC) delete M;
            for (auto p : CLOSED) delete p.second;
            if(solution.empty()) { return false; }
            this->solutions_ = transferToSerialPath(solution);
            this->removeStopAfterReachTarget(this->solutions_);

            return true;
        }

        // add large agent constraint
        bool get_new_config(Node *S, Constraint *M) {
            // setup cache
            // replace occupied_now and occupied_next with large agent constraint table
            for (auto a : A_) {
                // clear previous cache
//                if (a->v_now != -1 && occupied_now[a->v_now] == a) {
//                    occupied_now[a->v_now] = nullptr;
//                }
                if (a->v_next != -1) {
//                    occupied_next[a->v_next] = nullptr;
                    a->v_next = -1;
                }

                // set occupied now
                a->v_now = S->C[a->id]; // yz: current config is saved in S->C
//                occupied_now[a->v_now] = a;
            }

            // LargeAgentConstraintTable
            // LargeAgentConstraints
            //LargeAgentConstraintTable<N, AgentType>;
            ConstraintTable constraint_table(this->all_poses_, this->agents_, S->C, this->dim_);

            // add constraints
            // // yz: constraint determine next config without search, which is saved M
            for (auto k = 0; k < M->depth; ++k) {
                const auto i = M->who[k];        // agent
                const auto l = M->where[k];  // loc
                // yz: check whether current constraint collide with other agent, if is, abandon current agent
                if(constraint_table.hasCollide(i, A_[i]->v_now, l)) {
                    return false;
                }

//                if(path_constraint != nullptr) {
//                    std::cout << "path_constraint != nullptr" << std::endl;
//                    if(path_constraint->hasCollideWithSAT(this->agent_sub_graphs_[i].agent_.id_, A[i]->v_now, l)) {
//                        std::cout << "LaCAM hasCollideWithSAT_1: "
//                                  << this->agent_sub_graphs_[i].agent_ << ", "
//                                  << *this->agent_sub_graphs_[i].all_nodes_[A[i]->v_now] << "->"
//                                  << *this->agent_sub_graphs_[i].all_nodes_[l]
//                                  << std::endl;
//                        return false;
//                    }
//                }

//                // check vertex collision
//                if (occupied_next[l] != nullptr) return false;
//                // check swap collision
//                auto l_pre = S->C[i];
//                if (occupied_next[l_pre] != nullptr && occupied_now[l] != nullptr &&
//                    occupied_next[l_pre]->id == occupied_now[l]->id)
//                    return false;

                // set occupied_next
                A_[i]->v_next = l;
                constraint_table.setOccupiedNext(i, A_[i]->v_next);
//                occupied_next[l] = A[i];
            }



            // perform PIBT
            // yz: perform PIBT for each agent in predefined order
            // yz: but as PIBT is recursive, the REAL of search path is not deterministic
            for (auto k : S->order) {
                auto a = A_[k];
                if (a->v_next == -1 && !funcPIBT(a, S->t + 1, constraint_table)) {
                    return false;  // planning failure
                }
            }
            return true;
        }

        bool funcPIBT(AgentLaCAM *ai, int next_t, ConstraintTable& constraint_table) {
            const auto i = ai->id;
            const auto& neighbor = this->agent_sub_graphs_[ai->id].data_ptr_->all_edges_[ai->v_now];
            const auto K = neighbor.size();

            // get candidates for next locations
            for (size_t k = 0; k < K; ++k) {
                auto u = neighbor[k];
                C_next_[i][k] = u;
                if (MT != nullptr)
                    tie_breakers[u] = get_random_float(MT);  // set tie-breaker
            }
            C_next_[i][K] = ai->v_now;

            // sort, note: K + 1 is sufficient
            // yz: randomize future locations candidates

//            std::sort(C_next[i].begin(), C_next[i].begin() + K + 1,
//                      [&](const size_t& v, const size_t& u) {
//                          return this->agents_heuristic_tables_[i][v] + tie_breakers[v] <
//                                 this->agents_heuristic_tables_[i][u] + tie_breakers[u];
//                      });


            std::sort(C_next_[i].begin(), C_next_[i].begin() + K + 1,
                      [&](const size_t& v, const size_t& u) {
                          if(this->agents_heuristic_tables_[i][v] == this->agents_heuristic_tables_[i][u]) {
//                              return distance_to_target[i][v/(2*N)] < distance_to_target[i][u/(2*N)];
                              return this->agents_heuristic_tables_ignore_rotate_[i][v/(2*N)] <
                                     this->agents_heuristic_tables_ignore_rotate_[i][u/(2*N)];
                          } else {
                              return this->agents_heuristic_tables_[i][v] < this->agents_heuristic_tables_[i][u];
                          }
//                          if(distance_to_target[i][v/(2*N)] == distance_to_target[i][u/(2*N)]) {
//                              return this->agents_heuristic_tables_[i][v] < this->agents_heuristic_tables_[i][u];
//                          } else {
//                              return distance_to_target[i][v/(2*N)] < distance_to_target[i][u/(2*N)];
//                          }
                      });

//            std::cout << "after sort 2: ";
//            for(size_t k = 0; k < K + 1; ++k) {
//                std::cout << this->agents_heuristic_tables_[i][C_next[i][k]] << "|"
//                          << this->distance_to_target[i][C_next[i][k]/(2*N)] << "|"
//                          << *this->all_poses_[C_next[i][k]] << ", ";
//            }
//            std::cout << std::endl;

            // yz: for all current agent's neighbor and wait
//            std::vector<std::pair<size_t, int> > candidates;
            for (size_t k = 0; k < K + 1; ++k) {
                auto u = C_next_[i][k];

                // avoid vertex conflicts
//                if (occupied_next[u] != nullptr) continue;
                if(constraint_table.hasCollide(ai->id, ai->v_now, u)) {
//                    std::cout << " flag 3 false" << std::endl;
                    return false;
                }

                // TODO: get neighbor agent in constraint table, which is only one step from collide with current agent
                //auto &ak = occupied_now[u]; // yz: who occupied this neighbor
                std::vector<int> collide_agents = constraint_table.collideWith(ai->id, u);
                // avoid swap conflicts with constraints
                // yz: avoid swap conflicts with neighbor agents
//                if (ak != nullptr && ak->v_next == ai->v_now) continue;

                // reserve next location
                // yz: assume current agent can move to next location
//                occupied_next[u] = ai;
                constraint_table.setOccupiedNext(ai->id, u);
                ai->v_next = u;
//                std::cout << " current " << *this->all_poses_[ai->v_now];
//                std::cout << " select " << this->agents_heuristic_tables_[i][u] << "|"
//                                        << this->distance_to_target[i][u/(2*N)] << ", "
//                                        << *this->all_poses_[u]
//                                        << std::endl;
                // empty or stay
                // yz: if not all neighbor are occupied or current agent just stay
//                if (ak == nullptr || u == ai->v_now) return true;
                if (collide_agents.empty() || u == ai->v_now) { return true; }
//                std::cout << " flag 1" << std::endl;
                // priority inheritance
                // yz: if neighboring agent is not move yet
                // yz: then try to move it
//                if (ak->v_next == -1 && !funcPIBT(ak, next_t)) continue;
                int not_move_yet = -1;
                for(const auto& agent_id : collide_agents) {
                    if (A_[agent_id]->v_next == -1) {
                        not_move_yet = agent_id;
                        break;
                    }
                }
                if(not_move_yet == -1) { return true; }
                if(!funcPIBT(A_[not_move_yet], next_t, constraint_table)) {
                    continue;
                }
                // success to plan next one step
                return true;
            }

            // failed to secure node
            // yz: when search new config failed,
//            occupied_next[ai->v_now] = ai;
//            std::cout << " flag 2 false" << std::endl;
            constraint_table.setOccupiedNext(ai->id, ai->v_now);
            ai->v_next = ai->v_now;
            return false;
        }


        bool is_feasible_solution(const Solution &solution) const {
            if (solution.empty()) return true;

            // check start locations
            if (!is_same_config(solution.front(), starts_)) {
                std::cerr << "invalid starts" << std::endl;
                return false;
            }

            // check goal locations
            if (!is_same_config(solution.back(), targets_)) {
                std::cerr << "invalid targets" << std::endl;
                return false;
            }

            for (size_t t = 1; t < solution.size(); ++t) {
                for (size_t i = 0; i < this->instance_node_ids_.size(); ++i) {
                    auto v_i_from = solution[t - 1][i];
                    auto v_i_to = solution[t][i];
                    const auto& neighbor = this->agent_sub_graphs_[i].data_ptr_->all_edges_[v_i_from];
                    // check connectivity
                    if (v_i_from != v_i_to &&
//                        std::find(v_i_to->neighbor.begin(), v_i_to->neighbor.end(),
//                                  v_i_from) == v_i_to->neighbor.end()
                        std::find(neighbor.begin(), neighbor.end(), v_i_to) == neighbor.end()
                                  ) {
                        std::cerr << "invalid move" << std::endl;
                        return false;
                    }

                    // check conflicts
                    for (size_t j = i + 1; j < this->instance_node_ids_.size(); ++j) {
                        auto v_j_from = solution[t - 1][j];
                        auto v_j_to = solution[t][j];
                        // vertex conflicts
                        if (v_j_to == v_i_to) {
                            std::cerr << "vertex conflict" << std::endl;
                            return false;
                        }
                        // swap conflicts
                        if (v_j_to == v_i_from && v_j_from == v_i_to) {
                            std::cerr << "edge conflict" << std::endl;
                            return false;
                        }
                    }

                    if(path_constraint_ != nullptr) {
//                        std::cout << "path_constraint != nullptr" << std::endl;
                        if(path_constraint_->hasCollideWithSAT(this->agent_sub_graphs_[i].agent_->id_, v_i_from, v_i_to)) {
                            std::cout << "LaCAM hasCollideWithSAT_2: "
                                      << this->agent_sub_graphs_[i].agent_ << ", "
                                      << *this->agent_sub_graphs_[i].data_ptr_->all_nodes_[v_i_from] << "->"
                                      << *this->agent_sub_graphs_[i].data_ptr_->all_nodes_[v_i_to]
                                      << std::endl;
                            return false;
                        }
                    }
                }
            }

            return true;
        }

//        const Instance *ins;
//        const Deadline *deadline;
        std::mt19937 *MT;

        // solver utils
        const int V_size_;
//        DistTable D;
        Candidates<N> C_next_;                // next location candidates
        std::vector<float> tie_breakers;  // random values, used in PIBT
        AgentLaCAMs A_;
        AgentLaCAMs occupied_now_;   // for quick collision checking
        AgentLaCAMs occupied_next_;  // for quick collision checking

        Config starts_, targets_;

        LargeAgentStaticConstraintTablePtr<N, State> path_constraint_ = nullptr;

        // store each grid's distance to target, as a secondary heuristic value
//        std::vector<std::vector<int> > distance_to_target;

    };

}

#endif