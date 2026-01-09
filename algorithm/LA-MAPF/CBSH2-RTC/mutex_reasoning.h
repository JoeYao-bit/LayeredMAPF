//
// Created by yaozhuo on 1/6/26.
//

#ifndef LAYEREDMAPF_MUTEX_REASONING_H
#define LAYEREDMAPF_MUTEX_REASONING_H

#include "MDD.h"
#include "ConstraintPropagation.h"

namespace freeNav::LayeredMAPF::LA_MAPF::CBSH2_RTC {

    template<Dimension N, typename State>
    class MutexReasoning {
    public:
        double max_bc_runtime = 0;
        double max_bc_flow_runtime = 0;
        double accumulated_runtime = 0;

        MutexReasoning() {}

        void init(const std::vector<std::shared_ptr<ConstraintTable<N, State> > > &initial_constraints) {
            initial_constraints_ = initial_constraints;
        }

        std::shared_ptr<Conflict> run(const std::vector<Path> &paths, int a1, int a2, CBSNode &node, MDD<N, State> *mdd_1, MDD<N, State> *mdd_2) {
            clock_t t = clock();
            if (a1 > a2) {
                std::swap(a1, a2);
                std::swap(mdd_1, mdd_2);
            }
            auto conflict = findMutexConflict(paths, a1, a2, node, mdd_1, mdd_2);
            accumulated_runtime += (double) (clock() - t) / CLOCKS_PER_SEC;
            return conflict;
        }

        std::vector<std::shared_ptr<SingleAgentSolver<N, State> > > search_engines;  // used to find (single) agents' paths and mdd

    private:

        void cache_constraint(ConstraintsHasher &c1, ConstraintsHasher &c2, std::shared_ptr<Conflict> constraint) {
            lookupTable[c1][c2].push_back(constraint);
        }

        std::shared_ptr<Conflict>
        find_applicable_constraint(ConstraintsHasher &c1, ConstraintsHasher &c2, const std::vector<Path> &paths) {
            if (lookupTable.find(c1) != lookupTable.end()) {
                if (lookupTable[c1].find(c2) != lookupTable[c1].end()) {
                    for (auto &constraint: lookupTable[c1][c2]) {
                        if (constraint == nullptr) {
                            return nullptr;
                        }
                        if (constraint_applicable(paths, constraint)) {
                            return std::make_shared<Conflict>(*constraint);
                        }
                    }
                }
            }
            return nullptr;
        }

        bool has_constraint(ConstraintsHasher &c1, ConstraintsHasher &c2) {
            return lookupTable.find(c1) != lookupTable.end() && lookupTable[c1].find(c2) != lookupTable[c1].end();
        }


        bool constraint_applicable(const std::vector<Path> &paths, std::shared_ptr<Conflict> conflict) {
            if (conflict->priority == conflict_priority::CARDINAL) {
                return true;
            } else {
                return constraint_applicable(paths, conflict->cs1) &&
                       constraint_applicable(paths, conflict->cs2);
            }
        }

        bool constraint_applicable(const std::vector<Path> &paths, std::list<std::shared_ptr<Constraint> > &constraint) {
            for (const auto& c: constraint) {
                if (get<5>(*c) == constraint_type::VERTEX) {
                    int ag   = get<0>(*c);
                    int ag2  = get<1>(*c);
                    int loc  = get<2>(*c);
                    int loc2 = get<3>(*c);
                    int t    = get<4>(*c);
                    int t2   = get<5>(*c);
                    if ((paths[ag])[t].location == loc) {
                        return true;
                    }
                } else if (get<5>(*c) == constraint_type::EDGE) {
                    int ag   = get<0>(*c);
                    int ag2  = get<1>(*c);
                    int loc  = get<2>(*c);
                    int loc_to = get<3>(*c);
                    int t    = get<4>(*c);
                    int t2   = get<5>(*c);
                    if ((paths[ag])[t - 1].location == loc &&
                        (paths[ag])[t].location == loc_to) {
                        return true;
                    }
                }
            }
            return false;
        }

        std::vector<std::shared_ptr<ConstraintTable<N, State> > > initial_constraints_;
        // TODO using MDDs from cache
        // A problem can be whether the modified MDD still being safe for other modules..

        // (cons_hasher_0, cons_hasher_1) -> Constraint
        // Invariant: cons_hasher_0.a < cons_hasher_1.a
        std::unordered_map<ConstraintsHasher,
        std::unordered_map<ConstraintsHasher, std::list<std::shared_ptr<Conflict>>, ConstraintsHasher::Hasher, ConstraintsHasher::EqNode>,
        ConstraintsHasher::Hasher, ConstraintsHasher::EqNode
        > lookupTable;

        std::shared_ptr<Conflict>
        findMutexConflict(const std::vector<Path> &paths, int a1, int a2, CBSNode &node, MDD<N, State> *mdd_1, MDD<N, State> *mdd_2) {
            assert(a1 < a2);
            ConstraintsHasher c_1(a1, &node);
            ConstraintsHasher c_2(a2, &node);
            if (has_constraint(c_1, c_2))
                return find_applicable_constraint(c_1, c_2, paths);

            std::shared_ptr<Conflict> mutex_conflict = nullptr;

            ConstraintPropagation cp(mdd_1, mdd_2);
            cp.init_mutex();
            cp.fwd_mutex_prop();

            if (cp._feasible(mdd_1->levels.size() - 1, mdd_2->levels.size() - 1) >= 0) {
                cache_constraint(c_1, c_2, nullptr);
                return nullptr;
            }

            // generate constraint;
            mutex_conflict = std::make_shared<Conflict>();
            mutex_conflict->mutexConflict(a1, a2);

            MDD mdd_1_cpy(*mdd_1);
            MDD mdd_2_cpy(*mdd_2);

            ConstraintTable ct1(*initial_constraints_[a1]);
            ConstraintTable ct2(*initial_constraints_[a2]);

            ct1.build(node, a1);
            ct2.build(node, a2);
            auto ip = IPMutexPropagation(&mdd_1_cpy, &mdd_2_cpy, search_engines[a1], search_engines[a2], ct1, ct2);
            con_vec a;
            con_vec b;
            std::tie(a, b) = ip.gen_constraints();

            for (auto con:a) {
                get<0>(con) = a1;
                mutex_conflict->cs1.push_back(std::make_shared<Constraint>(con));
            }

            for (auto con:b) {
                get<0>(con) = a2;
                mutex_conflict->cs2.push_back(std::make_shared<Constraint>(con));
            }

            //mutex_conflict->final_len_1 = ip.final_len_0;
            //mutex_conflict->final_len_2 = ip.final_len_1;

            cache_constraint(c_1, c_2, mutex_conflict);

            // prepare for return
            return mutex_conflict;
        }
    };

}

#endif //LAYEREDMAPF_MUTEX_REASONING_H
