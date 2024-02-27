#pragma once
#include "EECBS/inc/common.h"
#include "Conflict.h"

namespace PBS_Li {

    enum node_selection { NODE_RANDOM, NODE_H, NODE_DEPTH, NODE_CONFLICTS, NODE_CONFLICTPAIRS, NODE_MVC };


    class PBSNode
    {
    public:
        Constraint constraint; // new constraint
        std::list< std::pair< int, CBS_Li::MAPFPath> > paths; // new paths
        int cost = 0; // sum of costs

        size_t depth = 0; // depath of this CT node
        size_t makespan = 0; // makespan over all paths

        uint64_t time_expanded = 0;
        uint64_t time_generated = 0;

        // conflicts in the current paths
        std::list<std::shared_ptr<Conflict> > conflicts;
        // The chosen conflict
        std::shared_ptr<Conflict> conflict;

        PBSNode* parent = nullptr;
        PBSNode* children[2] = {nullptr, nullptr};

        PBSNode() = default;
        PBSNode(PBSNode& parent) : cost(parent.cost), depth(parent.depth+1),
                                   makespan(parent.makespan), conflicts(parent.conflicts), parent(&parent){ }
        void clear();
        void printConstraints(int id) const;
        inline int getNumNewPaths() const { return (int) paths.size(); }
        inline std::string getName() const { return "PBS Node"; }
        std::list<int> getReplannedAgents() const
        {
            std::list<int> rst;
            for (const auto& path : paths)
                rst.push_back(path.first);
            return rst;
        }
    };

    std::ostream& operator<<(std::ostream& os, const PBSNode& node);

}