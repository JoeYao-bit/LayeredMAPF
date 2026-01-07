//
// Created by yaozhuo on 1/7/26.
//

#ifndef LAYEREDMAPF_INCREMENTALPAIRWISEMUTEXPROPAGATION_H
#define LAYEREDMAPF_INCREMENTALPAIRWISEMUTEXPROPAGATION_H

#include "MDD.h"

namespace freeNav::LayeredMAPF::LA_MAPF::CBSH2_RTC {


    typedef std::vector <Constraint> con_vec;

    template<Dimension N, typename State>
    class ConstraintPropagation;


    template<Dimension N, typename State>
    class IPMutexPropagation{
    private:
        MDD<N, State> *MDD_0;
        MDD<N, State> *MDD_1;
        int init_len_0;
        int init_len_1;
        int incr_limit;

        std::shared_ptr<SingleAgentSolver<N, State> > search_engine_0;
        std::shared_ptr<SingleAgentSolver<N, State> > search_engine_1;

        ConstraintTable<N, State> cons_0;
        ConstraintTable<N, State> cons_1;
    public:
        IPMutexPropagation(MDD<N, State> *MDD_0, MDD<N, State>* MDD_1,
                           std::shared_ptr<SingleAgentSolver<N, State> > se_0,
                           std::shared_ptr<SingleAgentSolver<N, State> > se_1,
                           ConstraintTable<N, State> cons_0,
                           ConstraintTable<N, State> cons_1,
                           int incr_limit = 20
        ) : MDD_0(MDD_0), MDD_1(MDD_1), init_len_0(MDD_0->levels.size()), init_len_1(MDD_1->levels.size()),
            incr_limit(incr_limit), search_engine_0(se_0), search_engine_1(se_1), cons_0(cons_0), cons_1(cons_1) {}

        std::pair<con_vec, con_vec> gen_constraints() {
            /* similar to pairwise ICTS */
            ConstraintPropagation<N, State> cp({MDD_0, MDD_1});

            cp.init_mutex();
            cp.fwd_mutex_prop();

            bool found_solution = false;

            // set len to sum of two agent's current length;
            int inc_len = 0;
            // cout << cp._feasible(init_len_0 - 1, init_len_1 -1) << endl;

            while (!found_solution) {
                MDD_0->increaseBy(cons_0, 1, search_engine_0);
                MDD_1->increaseBy(cons_1, 1, search_engine_1);

                cp.init_mutex();
                cp.fwd_mutex_prop();

                // cout << (!cp.feasible(init_len_0 + inc_len - 1, init_len_1 + inc_len - 1)) << endl;

                if (inc_len > incr_limit) {
                    final_len_0 = std::max(inc_len - 1, 0);
                    final_len_1 = std::max(inc_len - 1, 0);
                    return cp.generate_constraints(init_len_0 + std::max(inc_len - 1, 0), init_len_1 + std::max(inc_len - 1, 0));
                }

                inc_len += 1;

                if (!cp.feasible(init_len_0 + inc_len - 1, init_len_1 + inc_len - 1)) {
                    // TODO more greedy ...
                    if (init_len_0 < init_len_1) {
                        int d_inc_0 = 1, d_inc_1 = 0;
                        int inc_0 = d_inc_0, inc_1 = d_inc_1;

                        while (init_len_0 + inc_0 <= init_len_1 + inc_1 &&
                               cp.feasible(init_len_0 + inc_0 + std::max(inc_len - 1, 0) - 1,
                                           init_len_1 + inc_1 + std::max(inc_len - 1, 0) - 1)) {
                            if (inc_0 > incr_limit) {
                                break;
                            }
                            inc_0 += 1;
                            MDD_0->increaseBy(cons_0, 1, search_engine_0);
                            // cout <<  init_len_0 + inc_0 +  max(inc_len - 1, 0) - 1 << " " << init_len_1 + inc_1 + max(inc_len - 1, 0) - 1 << endl;
                            // cout << cp.feasible( init_len_0 + inc_0 +  max(inc_len - 1, 0) - 1, init_len_1 + inc_1 + max(inc_len - 1, 0) - 1) << endl;
                        }
                        // cout << "increasing a0 " << inc_0 << endl;
                        // cout << init_len_0 + max(inc_0 - 1, 0) + max(inc_len - 1, 0) - 1 << " " << init_len_1 + max(inc_len - 1, 0) - 1 << endl;
                        return cp.generate_constraints(init_len_0 + std::max(inc_0 - 1, 0) + std::max(inc_len - 1, 0) - 1,
                                                       init_len_1 + std::max(inc_len - 1, 0) - 1);

                    } else {
                        int d_inc_0 = 0, d_inc_1 = 1;
                        int inc_0 = d_inc_0, inc_1 = d_inc_1;

                        while (init_len_0 + inc_0 >= init_len_1 + inc_1 &&
                               cp.feasible(init_len_0 + inc_0 + std::max(inc_len - 1, 0) - 1,
                                           init_len_1 + inc_1 + std::max(inc_len - 1, 0) - 1)) {
                            if (inc_1 > incr_limit) {
                                break;
                            }
                            inc_1 += 1;
                            MDD_1->increaseBy(cons_1, 1, search_engine_1);
                        }
                        // cout << "increasing a1 " << inc_1 << endl;
                        return cp.generate_constraints(init_len_0 + std::max(inc_len - 1, 0) - 1,
                                                       init_len_1 + std::max(inc_1 - 1, 0) + std::max(inc_len - 1, 0) - 1);

                    }
                }
            }

            // cout << "no solution found";

            return {{},
                    {}};
        }

        int final_len_0;
        int final_len_1;


    };

}

#endif //LAYEREDMAPF_INCREMENTALPAIRWISEMUTEXPROPAGATION_H
