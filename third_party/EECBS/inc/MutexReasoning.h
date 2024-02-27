#include "IncrementalPairwiseMutexPropagation.hpp"
#include "MDD.h"
namespace CBS_Li {

    class MutexReasoning {
    public:
        double accumulated_runtime = 0;

        MutexReasoning(const Instance &instance,// const vector<ConstraintTable>& initial_constraints,
                       const ConstraintTable &static_constraint_o) :
                instance(instance),
                static_constraint(static_constraint_o)
        //,initial_constraints(initial_constraints)
        {}

        std::shared_ptr<Conflict> run(int a1, int a2, CBSNode &node, MDD *mdd_1, MDD *mdd_2);

        vector<SingleAgentSolver *> search_engines;  // used to find (single) agents' paths and mdd

    private:
        const Instance &instance;
        //const vector<ConstraintTable>& initial_constraints;
        const ConstraintTable &static_constraint;
        // TODO using MDDs from cache
        // A problem can be whether the modified MDD still being safe for other modules..

        // (cons_hasher_0, cons_hasher_1) -> Constraint
        // Invariant: cons_hasher_0.a < cons_hasher_1.a
        boost::unordered_map<ConstraintsHasher,
                boost::unordered_map<ConstraintsHasher, std::shared_ptr<Conflict>, ConstraintsHasher::Hasher, ConstraintsHasher::EqNode>,
                ConstraintsHasher::Hasher, ConstraintsHasher::EqNode
        > lookupTable;

        std::shared_ptr<Conflict> findMutexConflict(int a1, int a2, CBSNode &node, MDD *mdd_1, MDD *mdd_2);
    };

// other TODOs
// TODO duplicated cardinal test in classify conflicts
}