#include "PBS/inc/PBSNode.h"

namespace PBS_Li {

    void PBSNode::clear() {
        conflicts.clear();
    }

    void PBSNode::printConstraints(int id) const {
        auto curr = this;
        while (curr->parent != nullptr) {
            std::cout << curr->constraint.high << ">" << curr->constraint.low << std::endl;
            curr = curr->parent;
        }
    }

    std::ostream &operator<<(std::ostream &os, const PBSNode &node) {
        os << "Node " << node.time_generated << " ( cost = " << node.cost << ", conflicts = " << node.conflicts.size()
           <<
           " ) with " << node.getNumNewPaths() << " new paths ";
        return os;
    }

}