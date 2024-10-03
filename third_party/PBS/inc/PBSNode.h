#pragma once
#include "EECBS/inc/common.h"
#include "Conflict.h"
#include "../freeNav-base/basic_elements/point.h"

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

        std::string toString(freeNav::DimensionLength* dim) {
            std::stringstream ss;
            ss << "ptr = " << this << ", parent " << parent << " \n";
            ss << "cfs = " << this->conflicts.size() << ": ";
            for(const auto& cf : this->conflicts) {
                ss << cf->a1 << "*" << cf->a2 << ", ";
            }
            ss << "\n";
            ss << "cs: ag " << constraint.high << " > " << constraint.low << "\n";

            for(const auto& path : paths) {
                ss << "ag: " << path.first << ", path = ";
                for(const auto& wp : path.second) {
                    freeNav::Pointi<2> pt = freeNav::IdToPointi<2>(wp.location, dim);
                    ss << pt << "->";
                }
                ss << "\n";
            }
            return ss.str();
        }

    };

    std::ostream& operator<<(std::ostream& os, const PBSNode& node);

}