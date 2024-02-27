#include "../include/planner.hpp"
#include "../include/post_processing.hpp"
#include "../freeNav-base/dependencies/memory_analysis.h"
#include "../algorithm/basic.h"
//#include "EECBS/inc/driver.h"

namespace LaCAM {

    std::set<int> visited_grid_;

    // (180, 116  and  (179, 117 should consider wait but not wait, because their is no global wait in LaCAM

    Constraint::Constraint() : who(std::vector<int>()), where(Vertices()), depth(0) {
    }

    Constraint::Constraint(Constraint *parent, int i, Vertex *v)
            : who(parent->who), where(parent->where), depth(parent->depth + 1) {
        who.push_back(i);
        where.push_back(v);
    }

    Constraint::~Constraint() {};

    Node::Node(Config _C, DistTable &D, Node *_parent)
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
            for (size_t i = 0; i < N; ++i) priorities[i] = (float) D.get(i, C[i]) / N;
            t = 0;
        } else {
            // dynamic priorities, akin to PIBT
            for (size_t i = 0; i < N; ++i) {
                if (D.get(i, C[i]) != 0) {
                    priorities[i] = parent->priorities[i] + 1;
                } else {
                    priorities[i] = parent->priorities[i] - (int) parent->priorities[i];
                }
            }
            t = _parent->t + 1;
        }

        // set order
        std::iota(order.begin(), order.end(), 0);
        std::sort(order.begin(), order.end(),
                  [&](int i, int j) { return priorities[i] > priorities[j]; });
    }

    Node::~Node() {
        while (!search_tree.empty()) {
            delete search_tree.front();
            search_tree.pop();
        }
    }

    Planner::Planner(const Instance *_ins, const Deadline *_deadline,
                     std::mt19937 *_MT, int _verbose)
            : ins(_ins),
              deadline(_deadline),
              MT(_MT),
              verbose(_verbose),
              N(ins->N),
              V_size(ins->G.size()),
              D(DistTable(ins)),
              C_next(Candidates(N, std::array<Vertex *, 5>())),
              tie_breakers(std::vector<float>(V_size, 0)),
              A(Agents(N, nullptr)),
              occupied_now(Agents(V_size, nullptr)),
              occupied_next(Agents(V_size, nullptr)) {
    }

    Solution Planner::solve() {
        info(1, verbose, "elapsed:", elapsed_ms(deadline), "ms\tstart search");

        // setup agents
        for (auto i = 0; i < N; ++i) A[i] = new Agent(i);

        // setup search queues
        std::stack<Node *> OPEN;
        std::unordered_map<Config, Node *, ConfigHasher> CLOSED;
        std::vector<Constraint *> GC;  // garbage collection of constraints

        // insert initial node
        // yz: take all start position as start state
        auto S = new Node(ins->starts, D);
        OPEN.push(S);
        CLOSED[S->C] = S;

        // depth first search
        int loop_cnt = 0;
        std::vector<Config> solution;
        visited_grid_.clear();

        while (!OPEN.empty() && !is_expired(deadline)) {
            //std::cout << " loop count " << loop_cnt << " size " << OPEN.size() << std::endl;
            loop_cnt += 1;

            // do not pop here!
            S = OPEN.top();
            visited_grid_.insert(S->C[0]->index);
            // check goal condition
            if (is_same_config(S->C, ins->goals)) {
                // backtrack
                // yz: seems high level search in increasing time index
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
            auto M = S->search_tree.front();
            GC.push_back(M);
            S->search_tree.pop();
            if (M->depth < N) {
                auto i = S->order[M->depth];
                auto C = S->C[i]->neighbor;
                C.push_back(S->C[i]);
                if (MT != nullptr) std::shuffle(C.begin(), C.end(), *MT);  // randomize
                for (auto u : C) {
                    //yz: considering avoid static constraint
                    S->search_tree.push(new Constraint(M, i, u));
                }
            }
            // create successors at the high-level search
            if (!get_new_config(S, M)) continue;

            //yz: considering avoid static constraint
            if(ins->ct != nullptr) {
                bool is_legal = true;
                for(auto a : A) {
                    int agent = a->id;
                    int curr_location = S->C[agent]->index, next_location = a->v_next->index;
                    int next_timestep = S->t + 1;// + 1;
                    if(next_location == ins->goals[agent]->index) {
                        if(next_timestep < ins->ct->getHoldingTime(next_location, 0)) {
                            is_legal = false;
                            break;
                        }
                    }
                    if (ins->ct->constrained(next_location, next_timestep)) {
                        is_legal = false;
                        break;
                    }
                    if (ins->ct->constrained(curr_location, next_location, next_timestep)) {
                        is_legal = false;
                        break;
                    }
                }
                if(!is_legal) {
                    continue;
                }
            }

            // create new configuration
            auto C = Config(N, nullptr);
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
            auto S_new = new Node(C, D, S);
            OPEN.push(S_new);
            CLOSED[S_new->C] = S_new;
        }

        info(1, verbose, "elapsed:", elapsed_ms(deadline), "ms\t",
             solution.empty() ? (OPEN.empty() ? "no solution" : "failed")
                              : "solution found",
             "\tloop_itr:", loop_cnt, "\texplored:", CLOSED.size());
        // memory management
        for (auto a : A) delete a;
        for (auto M : GC) delete M;
        for (auto p : CLOSED) delete p.second;

        return solution;
    }

    bool Planner::get_new_config(Node *S, Constraint *M) {
        // setup cache
        for (auto a : A) {
            // clear previous cache
            if (a->v_now != nullptr && occupied_now[a->v_now->id] == a) {
                occupied_now[a->v_now->id] = nullptr;
            }
            if (a->v_next != nullptr) {
                occupied_next[a->v_next->id] = nullptr;
                a->v_next = nullptr;
            }

            // set occupied now
            a->v_now = S->C[a->id];
            occupied_now[a->v_now->id] = a;
        }

        // add constraints
        for (auto k = 0; k < M->depth; ++k) {
            const auto i = M->who[k];        // agent
            const auto l = M->where[k]->id;  // loc

            // check vertex collision
            if (occupied_next[l] != nullptr) return false;
            // check swap collision
            auto l_pre = S->C[i]->id;
            if (occupied_next[l_pre] != nullptr && occupied_now[l] != nullptr &&
                occupied_next[l_pre]->id == occupied_now[l]->id)
                return false;

            // set occupied_next
            A[i]->v_next = M->where[k];
            occupied_next[l] = A[i];
        }


        // perform PIBT
        for (auto k : S->order) {
            //std::cout << " perform PIBT " << ins->N << std::endl;
            auto a = A[k];
            if (a->v_next == nullptr && !funcPIBT(a, S->t + 1)) {
                return false;  // planning failure
            }
        }
        //std::cout << " return true " << std::endl;
        return true;
    }

    bool Planner::funcPIBT(Agent *ai, int next_t) {
        const auto i = ai->id;
        const auto K = ai->v_now->neighbor.size();

        // get candidates for next locations
        for (size_t k = 0; k < K; ++k) {
            auto u = ai->v_now->neighbor[k];
            C_next[i][k] = u;
            if (MT != nullptr)
                tie_breakers[u->id] = get_random_float(MT);  // set tie-breaker
        }
        C_next[i][K] = ai->v_now;

        // sort, note: K + 1 is sufficient
        std::sort(C_next[i].begin(), C_next[i].begin() + K + 1,
                  [&](Vertex *const v, Vertex *const u) {
                      return D.get(i, v) + tie_breakers[v->id] <
                             D.get(i, u) + tie_breakers[u->id];
                  });

        for (size_t k = 0; k < K + 1; ++k) {
            auto u = C_next[i][k];

            // avoid vertex conflicts
            if (occupied_next[u->id] != nullptr) continue;

            auto &ak = occupied_now[u->id];

            // avoid swap conflicts with constraints
            if (ak != nullptr && ak->v_next == ai->v_now) continue;

            // reserve next location
            occupied_next[u->id] = ai;
            ai->v_next = u;

            // empty or stay
            if (ak == nullptr || u == ai->v_now) return true;

            // priority inheritance
            if (ak->v_next == nullptr && !funcPIBT(ak, next_t)) continue;

            // success to plan next one step
            return true;
        }

        // failed to secure node
        occupied_next[ai->v_now->id] = ai;
        ai->v_next = ai->v_now;
        return false;
    }

    Solution solve(const Instance &ins, const int verbose, const Deadline *deadline,
                   std::mt19937 *MT) {
        info(1, verbose, "elapsed:", elapsed_ms(deadline), "ms\tpre-processing");
        auto planner = Planner(&ins, deadline, MT, verbose);
        return planner.solve();
    }

    //CBS_Li::ConstraintTable *ct = nullptr;

    freeNav::Paths<2> lacam_MAPF(freeNav::DimensionLength *dim,
                                 const freeNav::IS_OCCUPIED_FUNC<2> &isoc,
                                 const freeNav::Instances<2> &instance_sat,
                                 CBS_Li::ConstraintTable *ct,
                                 int cutoff_time) {

        // setup instance
        const auto verbose = 1;//std::stoi(program.get<std::string>("verbose"));
        const auto time_limit_sec = cutoff_time;
        //std::stoi(program.get<std::string>("time_limit_sec"));
        const auto seed = 0;
        auto MT = std::mt19937(seed);

        Instance ins(dim, isoc, instance_sat);
        if (!ins.is_valid(1)) {
            std::cout << " result is invalid " << std::endl;
            return {};
        }

         ins.ct = ct;
        // solve
        const Deadline deadline = Deadline(time_limit_sec * 1000);
        //std::cout << " ins->ct " << ins.ct << std::endl;
        const Solution solution = solve(ins, verbose - 1, &deadline, &MT);
        //const double comp_time_ms = deadline.elapsed_ms();

        // failure
        if (solution.empty()) {
            //info(1, verbose, "failed to solve");
            std::cout << "failed to solve" << std::endl;
            return {};
        } else {
            //std::cout << " solution size " << solution[0].size() << std::endl;
        }

        // check feasibility
        if (!is_feasible_solution(ins, solution, verbose)) {
            //info(0, verbose, "invalid solution");
            std::cout << " invalid solution " << std::endl;
            return {};
        }
        // yz: transform to freeNav style path
        freeNav::Paths<2> retv(solution.front().size());
        for(int t=0; t<solution.size(); t++) {
            assert(solution[t].size() == instance_sat.size());
            for(int agent=0; agent<solution[t].size(); agent++) {
                retv[agent].push_back(freeNav::IdToPointi<2>(solution[t][agent]->index, dim));
                assert(retv[agent].size()-1 == t);
            }
        }
        // yz: remove way point when agent is stop
        for(int agent=0; agent<instance_sat.size(); agent++) {
            const freeNav::Pointi<2>& target = retv[agent].back();
            auto& path = retv[agent];
//            std::cout << "before prune" << path << std::endl;
            for(auto iter = path.end(); iter != path.begin(); ) {
                if(*(iter-2) == target) {
                    iter = path.erase(iter-1);
                } else {
                    break;
                }
            }
//            std::cout << "target " << target << " instance_sat[agent].second " << instance_sat[agent].second << std::endl;
//            std::cout << "start " << path.front() << " instance_sat[agent].first " << instance_sat[agent].first << std::endl;
            assert(path.front() == instance_sat[agent].first);
            assert(path.back() == instance_sat[agent].second);
//            std::cout << "after prune" << path << std::endl;
        }
        // debug: check whether resulted path meet ct
//        std::cout << "** lacam internal new path meet ct check, previous path size " << previous_paths.size()
//                  << " current new path size " <<  retv.size() << std::endl;
//        freeNav::Paths<2> new_paths = previous_paths;
//        new_paths.insert(new_paths.end(), retv.begin(), retv.end());
//        std::cout << " is previous path with new paths valid (validateSolution) ? " << freeNav::validateSolution<2>(new_paths) << std::endl;
//        for(int agent=0; agent<instance_sat.size(); agent++) {
//            const auto& current_path = retv[agent];
//            for(int t=1; t<current_path.size(); t++) {
//                int next_location = dim[0] * current_path[t][1] + current_path[t][0],
//                    curr_location = dim[0] * current_path[t-1][1] + current_path[t-1][0];
//                int next_timestep = t;
//                if (ct->constrained(next_location, next_timestep)) {
//                    cout << "CT check Agent " << agent << " have vertex conflict at " << freeNav::IdToPointi<2>(next_location, dim) << " at timestep " << next_timestep << endl;
//                    return {};
//                }
//                if(ct->constrained(curr_location, next_location, next_timestep)) {
//                    cout << "CT check  Agent " << agent << " have edge conflict from " << freeNav::IdToPointi<2>(curr_location, dim)  << " to " << freeNav::IdToPointi<2>(next_location, dim)  << " at timestep " << next_timestep << endl;
//                    return {};
//                }
//            }
//        }

//        if (!solution.empty()) {
//            for (const auto &previous_path : retv) {
//                MAPFPath path_eecbs;
//                for (int i = 0; i < previous_path.size(); i++) {
//                    path_eecbs.push_back(
//                            PathEntry(dim[0] * previous_path[i][1] + previous_path[i][0]));
//                }
//                ct->insert2CT(path_eecbs);
//            }
//        }
        return retv;
    }

}