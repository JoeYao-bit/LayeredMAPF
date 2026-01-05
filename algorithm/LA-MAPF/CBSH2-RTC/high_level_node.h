//
// Created by yaozhuo on 2024/5/31.
//

#ifndef LAYEREDMAPF_HIGH_LEVEL_NODE_CBSH2_RTC_H
#define LAYEREDMAPF_HIGH_LEVEL_NODE_CBSH2_RTC_H

#include "../common.h"

namespace freeNav::LayeredMAPF::LA_MAPF::CBSH2_RTC {

class HighLvNode // a virtual base class for high-level node
{
    public:
        Constraints constraints; // new constraints // yz: constraint add in current node ?

        // yz: tree format
        HighLvNode *parent;
        std::list<HighLvNode *> children;

        std::list<std::pair<int, LAMAPF_Path> > paths; // new paths // yz: replanned new path

        Constraint cst; // new constraints // yz: constraint add in current node ?
        // conflicts in the current paths
        std::list<std::shared_ptr<Conflict> > conflicts;
        std::list<std::shared_ptr<Conflict> > unknownConf; // yz: unclassified conflict
        // The chosen conflict
        std::shared_ptr<Conflict> conflict;

        // yz: predicted time step (path length) to target
        int g_val = 0; // sum of costs for CBS, and sum of min f for ECBS
        // yz: have finished path time step (path length)
        int h_val = 0; // admissible h
        int cost_to_go = 0; // informed but inadmissible h
        int distance_to_go = 0; // distance to the goal state
        // yz: time index of waypoint, start from zero
        size_t depth = 0; // depath of this CT node
        size_t makespan = 0; // makespan over all paths
        bool h_computed = false;

        inline int getFVal() const { return g_val + h_val; }

        virtual inline int getFHatVal() const = 0;

        virtual inline int getNumNewPaths() const = 0;

        virtual std::list<int> getReplannedAgents() const = 0;

        virtual inline std::string getName() const = 0;

        void clear() {
            conflicts.clear();
            unknownConf.clear();
            // conflictGraph.clear();
        }
};

class CBSNode : public HighLvNode {
public:
    // the following is used to comapre nodes in the CLEANUP list
    struct compare_node_by_f {
        bool operator()(const CBSNode *n1, const CBSNode *n2) const {
//            if (n1->g_val + n1->h_val == n2->g_val + n2->h_val) {
//                if (n1->distance_to_go == n2->distance_to_go) {
//                    if (n1->g_val + n1->cost_to_go == n2->g_val + n2->cost_to_go) {
//                        return n1->h_val >= n2->h_val;
//                    }
//                    return n1->g_val + n1->cost_to_go >= n2->g_val + n2->cost_to_go;
//                }
//                return n1->distance_to_go >= n2->distance_to_go;
//            }
//            return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
            // below heuristic is faster but longer result
            if(n1->unknownConf.size() + n1->conflicts.size() >= n2->unknownConf.size() + n2->conflicts.size()) {
                if(n1->unknownConf.size() + n1->conflicts.size() == n2->unknownConf.size() + n2->conflicts.size()) {
                    return rand()%2 == 0;
                } else {
                    return true;
                }
            } else {
                return false;
            }
        }
    };  // used by CLEANUP to compare nodes by f_val (top of the heap has min f_val)

    // the following is used to comapre nodes in the FOCAL list
    struct compare_node_by_d {
        bool operator()(const CBSNode *n1, const CBSNode *n2) const {
            if (n1->distance_to_go == n2->distance_to_go) {
                if (n1->g_val + n1->h_val == n2->g_val + n2->h_val) {
                    if (n1->g_val + n1->cost_to_go == n2->g_val + n2->cost_to_go) {
                        return n1->h_val >= n2->h_val;
                    }
                    return n1->g_val + n1->cost_to_go >= n2->g_val + n2->cost_to_go;
                }
                return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
            }
            return n1->distance_to_go >= n2->distance_to_go;
        }
    };  // used by FOCAL to compare nodes by distance_to_go (top of the heap has min distance_to_go)

    // the following is used to compare nodes in the OPEN list
    struct compare_node_by_inadmissible_f {
        bool operator()(const CBSNode *n1, const CBSNode *n2) const {
            if (n1->g_val + n1->cost_to_go == n2->g_val + n2->cost_to_go) {
                if (n1->g_val + n1->h_val == n2->g_val + n2->h_val) {
                    if (n1->distance_to_go == n2->distance_to_go) {
                        return n1->h_val >= n2->h_val;
                    }
                    return n1->distance_to_go >= n2->distance_to_go;
                }
                return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
            }
            return n1->g_val + n1->cost_to_go >= n2->g_val + n2->cost_to_go;
        }
    };  // used by FOCAL to compare nodes by num_of_collisions (top of the heap has min h-val)

    // heap structure
    boost::heap::pairing_heap<CBSNode *, boost::heap::compare<CBSNode::compare_node_by_f> >::handle_type cleanup_handle;
    boost::heap::pairing_heap<CBSNode *, boost::heap::compare<CBSNode::compare_node_by_inadmissible_f> >::handle_type open_handle;
    boost::heap::pairing_heap<CBSNode *, boost::heap::compare<CBSNode::compare_node_by_d> >::handle_type focal_handle;

    CBSNode *parent;
    // yz: first: path id / second: path
    inline int getFHatVal() const override { return g_val + cost_to_go; }

    inline int getNumNewPaths() const override { return (int) paths.size(); }

    inline std::string getName() const override { return "CBS Node"; }

    std::list<int> getReplannedAgents() const override {
        std::list<int> rst;
        for (const auto &path : paths)
            rst.push_back(path.first);
        return rst;
    }

    template<Dimension N>
    std::string toString(const std::vector<PosePtr<int, N> >& all_poses) const {
        std::stringstream ss;
        ss << " ptr = " << this << ", parent " << parent << " \n";
        ss << " g_val = " << g_val << ", parent " << parent << " \n";
        ss << " cf = " << this->conflicts.size() << " \n";
        for(const auto& constraint : constraints) {
            int agent_id = std::get<0>(*constraint);
            ss << " cs: ag" << agent_id;
            Pointi<N> node_from = all_poses[std::get<1>(*constraint)]->pt_;
            ss << ", node_from = " << node_from;
            if(std::get<2>(*constraint) != MAX_NODES) {
                Pointi<N> node_to   = all_poses[std::get<2>(*constraint)]->pt_;
                ss << ", node_to = " << node_to ;
            }
            int time_start      = std::get<3>(*constraint);
            ss << ", ts = " << time_start;

            int time_end        = std::get<4>(*constraint);
            ss << ", te = " << time_end << " \n";
        }
        for(const auto& path : paths) {
            ss << "ag: " << path.first << ", path = ";
            for(const auto& wp : path.second) {
                ss << *all_poses[wp] << "->";
            }
        }
        return ss.str();
    }

    template<Dimension N>
    std::string toString(const std::vector<std::shared_ptr<Pointi<N>>> & all_poses) const {
        std::stringstream ss;
        ss << " ptr = " << this << ", parent " << parent << " \n";
        ss << " g_val = " << g_val << ", parent " << parent << " \n";
        ss << " cf = " << this->conflicts.size() << " \n";
        for(const auto& constraint : constraints) {
            int agent_id = std::get<0>(*constraint);
            ss << " cs: ag" << agent_id;
            Pointi<N> node_from = all_poses[std::get<1>(*constraint)];
            ss << ", node_from = " << node_from;
            if(std::get<2>(*constraint) != MAX_NODES) {
                Pointi<N> node_to   = all_poses[std::get<2>(*constraint)];
                ss << ", node_to = " << node_to ;
            }
            int time_start      = std::get<3>(*constraint);
            ss << ", ts = " << time_start;

            int time_end        = std::get<4>(*constraint);
            ss << ", te = " << time_end << " \n";
        }
        for(const auto& path : paths) {
            ss << "ag: " << path.first << ", path = ";
            for(const auto& wp : path.second) {
                ss << *all_poses[wp] << "->";
            }
        }
        return ss.str();
    }

};


}

#endif //LAYEREDMAPF_HIGH_LEVEL_NODE_H
