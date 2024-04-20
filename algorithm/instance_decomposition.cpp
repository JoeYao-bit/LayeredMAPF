#include "instance_decomposition.h"
//#include <Python.h>

namespace freeNav::LayeredMAPF {

    std::ostream &operator<<(std::ostream &os, const std::set<int>& agent_ids) {
        for (const auto &id : agent_ids) {
            os << id << " ";
        }
        return os;
    }

    std::string toString(const std::set<int>& agent_ids) {
        std::stringstream ss;
        for (const auto &id : agent_ids) {
            ss << id << " ";
        }
        return ss.str();
    }

}
