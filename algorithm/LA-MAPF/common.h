//
// Created by yaozhuo on 2024/5/19.
//

#ifndef LAYEREDMAPF_COMMON_H
#define LAYEREDMAPF_COMMON_H

#include <limits>
#include <chrono>
#include <stack>

#include <boost/geometry.hpp>
#include <boost/unordered_map.hpp>
#include <boost/heap/pairing_heap.hpp>
#include <boost/graph/subgraph.hpp>
#include <boost/graph/adjacency_list.hpp>

#include <boost/config.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/strong_components.hpp>

#include <random>

#include "../basic.h"
#include "freeNav-base/basic_elements/distance_map_update.h"
#include "freeNav-base/basic_elements/point.h"
#include "freeNav-base/basic_elements/point.h"
#include "freeNav-base/visualization/canvas/canvas.h"
#include "freeNav-base/dependencies/memory_analysis.h"
#include <memory>

namespace freeNav::LayeredMAPF::LA_MAPF {

    template<Dimension N>
    bool isPointSetOverlap(const Pointis<N> &set1, const Pointis<N> &set2, DimensionLength *dim) {
        IdSet ids;
        for (const auto &pt : set1) {
            if (isOutOfBoundary(pt, dim)) { continue; }
            ids.insert(PointiToId(pt, dim));
        }
        Id id;
        for (const auto &pt : set2) {
            if (isOutOfBoundary(pt, dim)) { continue; }
            id = PointiToId(pt, dim);
            if (ids.find(id) != ids.end()) {
                return true;
            }
        }
        return false;
    }

    template<Dimension N>
    using PosePath = std::vector<Pose<int, N> >;

    template<Dimension N>
    using PosePaths = std::vector<PosePath<N> >;

    // <agent id, node from, node to, time range start, time range end>
    typedef std::tuple<int, size_t, size_t, int, int> Constraint;

    typedef std::list<std::shared_ptr<Constraint> > Constraints;

    // conflict resulted by two agents, and constraint store what move and when cause conflict
    //typedef std::pair<Constraint, Constraint> Conflict;

    enum conflict_type {
        STANDARD, TYPE_COUNT
    };

    enum conflict_priority {
        CARDINAL, PSEUDO_CARDINAL, SEMI, NON, UNKNOWN, PRIORITY_COUNT
    };

    struct Conflict {
        Conflict(int a1, int a2, const Constraints &cs1, const Constraints &cs2)
                : a1(a1), a2(a2), cs1(cs1), cs2(cs2) {
            int agent;
            size_t from, to;
            int start_t, end_t;
            // get start time index of conflict 1
            for (const auto &cs : cs1) {
                std::tie(agent, from, to, start_t, end_t) = *cs;
                t1 = start_t < t1 ? start_t : t1;
                t2 = end_t > t2 ? end_t : t2;
            }
            for (const auto &cs : cs2) {
                std::tie(agent, from, to, start_t, end_t) = *cs;
                t1 = start_t < t1 ? start_t : t1;
                t2 = end_t > t2 ? end_t : t2;
            }
        }

        int a1, a2; // agent id
        int t1 = MAX<int>, t2 = 0; // time range of constraints
        Constraints cs1;
        Constraints cs2;

        conflict_type type;
        conflict_priority priority = conflict_priority::UNKNOWN;
        double secondary_priority = 0; // used as the tie-breaking creteria for conflict selection
    };

    typedef std::shared_ptr<Conflict> ConflictPtr;

    typedef std::vector<std::shared_ptr<Conflict> > Conflicts;

    bool operator<(const Conflict &conflict1, const Conflict &conflict2);


    typedef std::vector<size_t> LAMAPF_Path; // node id sequence

    typedef std::vector<LAMAPF_Path> LAMAPF_Paths;

    template<typename T>
    int getSOC(const std::vector<std::vector<T> > &paths) {
        size_t soc = 0;
        for (size_t a1 = 0; a1 < paths.size(); a1++) {
            soc += paths[a1].size() - 1; // yz: soc: sum of cost
        }
        return soc;
    }

    template<typename T>
    int getMakeSpan(const std::vector<std::vector<T> > &paths) {
        size_t mk = 0;
        for (size_t a1 = 0; a1 < paths.size(); a1++) {
            mk = std::max(paths[a1].size(), mk); // yz: soc: sum of cost
        }
        return mk;
    }

    bool isSamePath(const std::vector<size_t> &path1, const std::vector<size_t> &path2);

    template<Dimension N>
    struct Agent {

        explicit Agent(float excircle_radius, float incircle_radius, int id, DimensionLength *dim,
                       const std::string &type_name)
                : excircle_radius_(excircle_radius), incircle_radius_(incircle_radius),
                  id_(id), dim_(dim), type_(type_name) {}

        virtual std::shared_ptr<Agent<N> > copy() const = 0;

        virtual bool isCollide(const Pose<int, N> &pose,
                               DimensionLength *dim,
                               const IS_OCCUPIED_FUNC<N> &isoc,
                               const DistanceMapUpdater<N> &distance_table) const = 0;

        virtual bool isCollide(const Pose<int, N> &edge_from,
                               const Pose<int, N> &edge_to,
                               DimensionLength *dim,
                               const IS_OCCUPIED_FUNC<N> &isoc,
                               const DistanceMapUpdater<N> &distance_table) const = 0;

        virtual bool isCollide(const Pointi<N> &pt,
                               DimensionLength *dim,
                               const IS_OCCUPIED_FUNC<N> &isoc,
                               const DistanceMapUpdater<N> &distance_table) const = 0;

        virtual bool isCollide(const Pointi<N> &edge_from,
                               const Pointi<N> &edge_to,
                               DimensionLength *dim,
                               const IS_OCCUPIED_FUNC<N> &isoc,
                               const DistanceMapUpdater<N> &distance_table) const = 0;

        virtual Pointis<N> getTransferOccupiedGrid(const Pose<int, N> &edge_from,
                                                   const Pose<int, N> &edge_to) const = 0;

        virtual Pointis<N> getTransferOccupiedGrid(const Pointi<N> &edge_from,
                                                   const Pointi<N> &edge_to) const {
            return {edge_from, edge_to};
        };

        virtual std::pair<Pointis<N>, Pointis<N>> getPoseOccupiedGrid(const Pose<int, N> &pose) const = 0;

        virtual std::string serialize() const = 0;

        virtual std::string serialize(const Pose<int, N> &start_pose, const Pose<int, N> &target_pose) const = 0;

        virtual void drawOnCanvas(const Pose<int, 2> &pose,
                                  Canvas &canvas, const cv::Vec3b &color, bool fill = true) const = 0;

        // draw on canvas, in global pose (x,y,yaw)
        virtual void drawOnCanvas(const Pointf<3> &pose,
                                  Canvas &canvas, const cv::Vec3b &color, bool fill = true) const = 0;


//        virtual std::pair<Pointis<N>, Pointis<N>> getCoverageGridWithinPose(const Pose<int, N>& pose) const = 0;

        float excircle_radius_, incircle_radius_;

        int id_;

        DimensionLength *dim_ = nullptr;

        std::string type_ = "DEFAULT"; // what type this agent is, like Circle or Block

    };

    template<Dimension N>
    std::ostream &operator<<(std::ostream &os, const Agent<N> &block) {
        os << block.serialize();
        return os;
    }

    template<typename T>
    std::ostream &operator<<(std::ostream &os, const std::set<T> &set_to_print) {
        for (const auto &data : set_to_print) {
            os << data << " ";
        }
        return os;
    }


    template<Dimension N>
    using AgentPtr = std::shared_ptr<Agent<N> >;

    template<Dimension N>
    using AgentPtrs = std::vector<AgentPtr<N> >;

    template<Dimension N>
    double DistBetweenTwoLines(const Pose<int, N> &s1, const Pose<int, N> &e1,
                               const Pose<int, N> &s2, const Pose<int, N> &e2) {
        assert(N == 2);
        namespace bg = boost::geometry;
        using bg_pt = bg::model::point<int, 2, bg::cs::cartesian>;
        using bg_seg = bg::model::segment<bg_pt>;
        // calculate the shortest distance between two segments
        bg_pt pt1(s1.pt_[0], s1.pt_[1]), pt2(e1.pt_[0], e1.pt_[1]),
                pt3(s2.pt_[0], s2.pt_[1]), pt4(e2.pt_[0], e2.pt_[1]);
        bg_seg seg1(pt1, pt2), seg2(pt3, pt4);
        //std::cout << " bg::distance(seg1, seg2) = " << bg::distance(seg1, seg2) << std::endl;
        return bg::distance(seg1, seg2);

    }

    // check whether one moving circle are collide with one waiting circle
    template<Dimension N>
    double DistBetweenPointAndLine(const Pose<int, N> &s1, const Pose<int, N> &e1, const Pose<int, N> &s2) {
        assert(N == 2);

        namespace bg = boost::geometry;
        using bg_pt = bg::model::point<int, 2, bg::cs::cartesian>;
        using bg_seg = bg::model::segment<bg_pt>;
        // calculate the shortest distance between a point and a segment
        bg_pt pt1(s1.pt_[0], s1.pt_[1]), pt2(e1.pt_[0], e1.pt_[1]),
                pt3(s2.pt_[0], s2.pt_[1]);
        bg_seg seg1(pt1, pt2);
        //std::cout << " bg::distance(seg1, pt3) = " << bg::distance(seg1, pt3) << std::endl;
        return bg::distance(seg1, pt3);

//        Pointis<2> pts1 = a1.getTransferOccupiedGrid(s1, e1);
//        Pointis<2> pts2 = a2.getPoseOccupiedGrid(s2).first;
//        return isPointSetOverlap(pts1, pts2, a1.dim_);
    }

    template<Dimension N>
    using Agents = std::vector<Agent<N> >;


    // check whether two moving circle are collide with each other
    template<Dimension N>
    bool isCollide(const AgentPtr<N> &a1, const Pose<int, N> &s1, const Pose<int, N> &e1,
                   const AgentPtr<N> &a2, const Pose<int, N> &s2, const Pose<int, N> &e2) {

        double dist = DistBetweenTwoLines<N>(s1, e1, s2, e2);
        if (dist >
            a1->excircle_radius_ + a2->excircle_radius_ + std::numeric_limits<double>::epsilon()) { return false; }
        if (dist <
            a1->incircle_radius_ + a2->incircle_radius_ - std::numeric_limits<double>::epsilon()) { return true; }

        Pointis<N> pts1 = a1->getTransferOccupiedGrid(s1, e1);
        Pointis<N> pts2 = a2->getTransferOccupiedGrid(s2, e2);
        return isPointSetOverlap(pts1, pts2, a1->dim_);
    }

    template<Dimension N>
    bool isCollide(const AgentPtr<N> &a1, const Pointi<N> &s1, const Pointi<N> &e1,
                   const AgentPtr<N> &a2, const Pointi<N> &s2, const Pointi<N> &e2) {
        if(s1 == s2 || e1 == e2) { return true; }
        if(s1 == e2 || s2 == e1) { return true; }
        return false;
    }

    // check whether one moving circle are collide with one waiting circle
    template<Dimension N>
    bool isCollide(const AgentPtr<N> &a1, const Pose<int, N> &s1, const Pose<int, N> &e1,
                   const AgentPtr<N> &a2, const Pose<int, N> &s2) {

        double dist = DistBetweenPointAndLine<N>(s1, e1, s2);
        if (dist >
            a1->excircle_radius_ + a2->excircle_radius_ + std::numeric_limits<double>::epsilon()) { return false; }
        if (dist <
            a1->incircle_radius_ + a2->incircle_radius_ - std::numeric_limits<double>::epsilon()) { return true; }

        Pointis<N> pts1 = a1->getTransferOccupiedGrid(s1, e1);
        Pointis<N> pts2 = a2->getPoseOccupiedGrid(s2).first;
        return isPointSetOverlap(pts1, pts2, a1->dim_);
    }

    // check whether one moving circle are collide with one waiting circle
    template<Dimension N>
    bool isCollide(const AgentPtr<N> &a1, const Pose<int, N> &s1,
                   const AgentPtr<N> &a2, const Pose<int, N> &s2, const Pose<int, N> &e2) {
        return isCollide(a2, s2, e2, a1, s1);
    }

    template<Dimension N>
    bool isCollide(const AgentPtr<N> &a1, const Pointi<N> &s1,
                   const AgentPtr<N> &a2, const Pointi<N> &s2, const Pointi<N> &e2) {
        return s1 == s2 || s1 == e2;
    }

    template<Dimension N>
    bool isCollide(const AgentPtr<N> &a1, const Pointi<N> &s1, const Pointi<N> &e1,
                   const AgentPtr<N> &a2, const Pointi<N> &s2) {
        return s1 == s2 || e1 == s2;
    }

    template<Dimension N>
    bool isCollide(const AgentPtr<N> &a1, const Pose<int, N> &s1,
                   const AgentPtr<N> &a2, const Pose<int, N> &s2) {

        double dist = (s1.pt_ - s2.pt_).Norm();

        if (dist >
            a1->excircle_radius_ + a2->excircle_radius_ + std::numeric_limits<double>::epsilon()) { return false; }
        if (dist <
            a1->incircle_radius_ + a2->incircle_radius_ - std::numeric_limits<double>::epsilon()) { return true; }

        Pointis<N> pts1 = a1->getPoseOccupiedGrid(s1).first;
        Pointis<N> pts2 = a2->getPoseOccupiedGrid(s2).first;
        return isPointSetOverlap(pts1, pts2, a1->dim_);
    }

    template<Dimension N>
    bool isCollide(const AgentPtr<N> &a1, const Pointi<N> &s1,
                   const AgentPtr<N> &a2, const Pointi<N> &s2) {

        return s1 == s2;
    }

    class HighLvNode;

    /* [row][col] */
    typedef std::vector<std::vector<int> > RotateMatrix;

    typedef std::vector<RotateMatrix> RotateMatrixs;


    RotateMatrix initializeRotateMatrix(const int &dim, const int &orient);

    // for calculate rotate matrix about 2D and 3D maps
    RotateMatrixs initializeRotateMatrixs(const int &dim);

    std::vector<RotateMatrixs> initializeAllRotateMatrixs();

    template<typename T, Dimension N>
    Point<T, N> rotatePoint(const Point<T, N> &pt, const RotateMatrix &mt) {
        Point<T, N> retv;
        for (int row = 0; row < N; row++) {
            retv[row] = 0;
            for (int col = 0; col < N; col++) {
                retv[row] = retv[row] + pt[col] * mt[col][row];
            }
        }
        return retv;
    }


    template<typename T, Dimension N>
    bool isRectangleOverlap(const Point<T, N> &r1_min, const Point<T, N> &r1_max,
                            const Point<T, N> &r2_min, const Point<T, N> &r2_max) {
        for (int dim = 0; dim < N; dim++) {
            assert(r1_min[dim] < r1_max[dim]);
            assert(r2_min[dim] < r2_max[dim]);
            if (r1_max[dim] < r2_min[dim]) {
                return false;
            }
            if (r1_min[dim] > r2_max[dim]) {
                return false;
            }
        }
        return true;
    }

    template<typename T1, typename T2, Dimension N>
    bool isPointInRectangle(const Point<T1, N> &r1_min, const Point<T1, N> &r1_max,
                            const Point<T2, N> &pt) {
        std::vector<bool> flag(N, false);
        for (int dim = 0; dim < N; dim++) {
            // if in all dimension that two rectangle are overlaped, they are overlaped
            if ((pt[dim] - r1_min[dim]) * (pt[dim] - r1_max[dim]) <= 0) { flag[dim] = true; }
        }
        return flag == std::vector<bool>(N, true);
    }

    // public static element
    extern std::vector<RotateMatrixs> ROTATE_MATRIXS;

    RotateMatrix initRotateMatrix_2D(const double &angle);

    double orientToPi_2D(const int &orient);

    template<typename T>
    Point<T, 2> pointRotate_2D(const Point<T, 2> &pt, const int &orient) {
        return rotatePoint<T, 2>(pt, ROTATE_MATRIXS[2][orient]);
    }

    template<typename T>
    Point<T, 2> pointTransfer_2D(const Point<T, 2> &pt, const Pose<int, 2> &pose) {
        Point<T, 2> rotated_pt = pointRotate_2D(pt, pose.orient_);
        rotated_pt[0] = rotated_pt[0] + pose.pt_[0];
        rotated_pt[1] = rotated_pt[1] + pose.pt_[1];
        return rotated_pt;
    }

    float get_random_float(std::mt19937 *MT, float from = 0, float to = 0);

    // a N-dimensional grid have 2^N corner
    template<Dimension N>
    Pointfs<N> getAllCornerOfGrid() {
        Pointf<N> ptf;
        Pointfs<N> retv;
        Pointis<N> offsets = GetNeightborOffsetGrids<N>(true);
//        std::cout << "offsets " << offsets.size() << std::endl;
        for (const auto &offset : offsets) {
            for (int i = 0; i < N; i++) {
                ptf[i] = offset[i] / 2.;
            }
            retv.push_back(ptf);
        }
        return retv;
    }


    struct ConnectivityGraphData {

        explicit ConnectivityGraphData(int total_nodes) {
            hyper_node_id_map_.resize(total_nodes, MAX<size_t>);
        }

        std::vector<std::set<int> > related_agents_map_; // store each pose collide with what agents' start(2*id) or target(2*id+1)

        // store each node's hyper node id, default to MAX<size_t>
        // may be disposed after we construct ConnectivityGraph
        std::vector<size_t> hyper_node_id_map_;

        // store what agent current hyper node associate with
        // a hyper node may associate with no agent may also associate with multiple agent
        std::vector<std::set<int> > hyper_node_with_agents_;

        std::vector<std::vector<size_t> > all_edges_vec_; // each hyper node's connecting node, store in vector

        std::vector<std::vector<size_t> > all_edges_vec_backward_; // each hyper node's connecting node, store in vector

        std::vector<std::set<size_t> > all_edges_set_; // each hyper node's connecting node, store in set

    };


    typedef std::shared_ptr<ConnectivityGraphData> ConnectivityGraphDataPtr;

    // due to each agent may have different shape, each agent have their own connectivity graph
    struct ConnectivityGraph {

        ConnectivityGraphDataPtr data_ptr_ = nullptr;

        size_t start_hyper_node_ = MAX<size_t>; // where is start in the hyper graph

        size_t target_hyper_node_ = MAX<size_t>; // where is target in the hyper graph

    };


    template<Dimension N>
    struct HyperGraphNodeDataRaw;

    template<Dimension N>
    using HyperGraphNodeDataRawPtr = HyperGraphNodeDataRaw<N> *;

    template<Dimension N>
    struct HyperGraphNodeDataRaw : public TreeNode<N, HyperGraphNodeDataRawPtr<N> > {

        explicit HyperGraphNodeDataRaw(const size_t &current_node,
                                       const HyperGraphNodeDataRawPtr<N> &parent,
                                       const LA_MAPF::ConnectivityGraph &graph,
                                       bool distinguish_sat = false, // whether visited grid distinguish start or target
                                       const std::vector<bool> &ignore_cost_agent_ids = {}) :
                current_node_(current_node), graph_(graph), TreeNode<N, HyperGraphNodeDataRawPtr<N>>(parent) {
            if (parent != nullptr) {
                g_val_ = parent->g_val_;
            }
            // if is a agent node, rather than a free group node
//            for(const int& agent_id : graph_.data_ptr_->hyper_node_with_agents_[current_node_]) {
//                if(!ignore_cost_agent_ids.empty() && ignore_cost_agent_ids[agent_id]) { continue; }
//                visited_agent_.insert(distinguish_sat ? agent_id : agent_id/2);
//            }
            g_val_ = g_val_ + graph_.data_ptr_->hyper_node_with_agents_[current_node_].size();
        }

        void copy(const HyperGraphNodeDataRaw<N> &other_node) {
            g_val_ = other_node.g_val_;
            h_val_ = other_node.h_val_;
            current_node_ = other_node.current_node_;
            this->pa_ = other_node.pa_;
            this->ch_ = other_node.ch_;
        }

        size_t current_node_;

        // only considering agent grid it path, ignore free grid group it pass
        // how many agent it need to cross till reach target, if both start and target of an agent occur in the path, dist plus only one
        // int g_val_ = MAX<int>; // dist from start to here
        // g_val = visited_agent_.size()

        //std::set<int> visited_agent_;

        int g_val_ = 0; // an agent id with not occur twice, so no need for std::set, which is every time consuming

        int h_val_ = 0; // estimation dist from here to target

        const LA_MAPF::ConnectivityGraph &graph_;

        int getFVal() {
            return g_val_ + h_val_;
        }

        struct compare_node {
            // returns true if n1 > n2 (note -- this gives us *min*-heap).
            bool operator()(const HyperGraphNodeDataRawPtr<N> &n1, const HyperGraphNodeDataRawPtr<N> &n2) const {
                return n1->g_val_ + n1->h_val_ >= n2->g_val_ + n2->h_val_;
            }
        };

        struct equal_node {
            // returns true if n1 > n2 (note -- this gives us *min*-heap).
            bool operator()(const HyperGraphNodeDataRawPtr<N> &n1, const HyperGraphNodeDataRawPtr<N> &n2) const {
                return n1->current_node_ == n2->current_node_;
            }
        };

        struct NodeHasher {
            size_t operator()(const HyperGraphNodeDataRawPtr<N> &n) const {
                return std::hash<int>()(n->current_node_); // yz: 按位异或
            }
        };

        bool in_openlist_ = false;

        typedef typename boost::heap::pairing_heap<HyperGraphNodeDataRawPtr<N>, boost::heap::compare<typename HyperGraphNodeDataRaw<N>::compare_node> >::handle_type open_handle_t;

        open_handle_t open_handle_;

    };


    // consider static grid map when construct sub graph
    // check each pair of agent to see if they have conflict
    template<Dimension N, typename State>
    struct LargeAgentPathConstraintTable {
    public:
        explicit LargeAgentPathConstraintTable(
                float max_excircle_radius, // max excircle radius for external and internal agents
                DimensionLength *dim,
                const IS_OCCUPIED_FUNC<N> &isoc,
                const std::vector<AgentPtr<N> > &global_agents, // all agents, global level
                const std::vector<AgentPtr<N> > &local_agents, // all agents, local level
                const std::vector<std::shared_ptr<State>> &all_nodes)
                : global_agents_(global_agents), all_nodes_(all_nodes), dim_(dim) {
            //
        }

        void insertPaths(const std::vector<std::pair<int, LAMAPF_Path> > &agent_paths) {
            for (const auto &agent_path : agent_paths) {
                insertPath(agent_path);
            }
        }

        void insertPath(const std::pair<int, LAMAPF_Path> &agent_path) {
//            std::cout << "insert agent " << agents_[agent_path.first] << "'s path size = " << agent_path.second.size()
//                      << " as constraint " << std::endl;
            if (constraint_table_.find(agent_path.first) == constraint_table_.end()) {
                constraint_table_.insert(agent_path);
            } else {
                constraint_table_[agent_path.first] = agent_path.second;
            }
            max_time_stamp_ = std::max(max_time_stamp_, (int) agent_path.second.size());
        }

        // called after set all paths
        void updateEarliestArriveTimeForAgents(const std::vector<AgentPtr<N> > &agents,
                                               const std::vector<size_t> &target_node_ids) {
            assert(agents.size() == target_node_ids.size());
            earliest_arrive_time_map_.clear();
            for (int k = 0; k < agents.size(); k++) {
                int earliest_time = calculateEarliestArriveTimeForAgent(agents[k], target_node_ids[k]);
                earliest_arrive_time_map_.insert({agents[k].id_, earliest_time});
            }
        }

        // call before use bool hasCollide(...)
        // calculate the earliest possible arrive time
        int calculateEarliestArriveTimeForAgent(const AgentPtr<N> &agent, const size_t &target_node_id) const {
            int temp_earliest_arrive_time_ = 0;
            for (const auto &agent_path : constraint_table_) {
                const auto &path = agent_path.second;
                int local_earliest_arrive_time = 0;
                for (int t = path.size() - 2; t >= std::max(temp_earliest_arrive_time_ - 1, 0); t--) {
                    if (isCollide(global_agents_[agent_path.first], *all_nodes_[path[t]], *all_nodes_[path[t + 1]],
                                  agent, *all_nodes_[target_node_id])) {
                        break;
                    } else {
                        local_earliest_arrive_time = t + 1;
                    }
                }
                temp_earliest_arrive_time_ = std::max(local_earliest_arrive_time, temp_earliest_arrive_time_);
            }
//            std::cout << " set agent " << agent << " earliest_arrive_time_ = " << temp_earliest_arrive_time_ << std::endl;
            return temp_earliest_arrive_time_;
        }

        // whether an agent has conflict at pose
        bool hasCollide(int agent_global_id, int time_index, const size_t &current_node, const size_t &next_node,
                        bool is_goal = false) const {
            const auto &agent = global_agents_[agent_global_id];
            if (is_goal) {
                if (time_index + 1 < earliest_arrive_time_map_.at(agent_global_id)) { return true; }
            }
            for (const auto &agent_path : constraint_table_) {
                const auto &other_agent_id = agent_path.first;
                const auto &other_path = agent_path.second;
                if (agent_global_id == other_agent_id) { continue; }
                if (other_path.size() - 1 <= time_index) {
                    if (isCollide(agent, *all_nodes_[current_node], *all_nodes_[next_node],
                                  global_agents_[other_agent_id], *all_nodes_[other_path.back()])) {
//                        std::cout << "t=" << time_index << " " << agent << " at " << *all_nodes_[current_node] << "->" << *all_nodes_[next_node]
//                                  << " and " << global_agents_[other_agent_id] << " at " << *all_nodes_[other_path.back()]
//                                  << " have conflict" << std::endl;
                        return true;
                    } else {
//                        std::cout << "t=" << time_index << " " << agent << " at " << *all_nodes_[current_node] << "->" << *all_nodes_[next_node]
//                                  << " and " << agents_[other_agent_id] << " at " << *all_nodes_[other_path.back()]
//                                  << " have no conflict" << std::endl;
                    }
                } else {
                    if (isCollide(agent, *all_nodes_[current_node], *all_nodes_[next_node],
                                  global_agents_[other_agent_id], *all_nodes_[other_path[time_index]],
                                  *all_nodes_[other_path[time_index + 1]])) {
//                        std::cout << "t=" << time_index << " " << agent << " at " << *all_nodes_[current_node] << "->" << *all_nodes_[next_node]
//                                  << " and " << global_agents_[other_agent_id] << " at " << *all_nodes_[other_path[time_index]]
//                                  << "->" << *all_nodes_[other_path[time_index+1]]
//                                  << " have conflict" << std::endl;
                        return true;
                    } else {
//                        std::cout << "t=" << time_index << " " << agent << " at " << *all_nodes_[current_node] << "->" << *all_nodes_[next_node]
//                                  << " and " << agents_[other_agent_id] << " at " << *all_nodes_[other_path[time_index]]
//                                  << "->" << *all_nodes_[other_path[time_index+1]]
//                                  << " have no conflict" << std::endl;
                    }
                    // if current agent reach target, check whether it collide with other agent in the future
                    // could be accelerate, by avoid
                    if (is_goal) {
                        for (int t = earliest_arrive_time_map_.at(agent_global_id); t < other_path.size() - 1; t++) {
                            if (isCollide(agent, *all_nodes_[current_node], *all_nodes_[next_node],
                                          global_agents_[other_agent_id], *all_nodes_[other_path[t]],
                                          *all_nodes_[other_path[t + 1]])) {
                                return true;
                            }
                        }
                    }
                }
            }
            return false;
        }

        int getMaxTimeStamp() const {
            return max_time_stamp_;
        }

        int getEarliestTime(const AgentPtr<N> &agent) const {
            return earliest_arrive_time_map_.at(agent.id_);
        }

    private:

        const std::vector<AgentPtr<N> > &global_agents_;

        const std::vector<std::shared_ptr<State>> &all_nodes_;

        std::map<int, LAMAPF_Path> constraint_table_; // global agent id

        DimensionLength *dim_;

        std::map<int, int> earliest_arrive_time_map_; // agent.id_ and the earliest time to visit the target

        int max_time_stamp_ = 0; // every thing is static after max_tme_stamp
    };

    template<Dimension N, typename State>
    using LargeAgentPathConstraintTablePtr = std::shared_ptr<LargeAgentPathConstraintTable<N, State> >;

    template<Dimension N>
    float getMaximumRadius(const std::vector<AgentPtr<N> > &agents) {
        float retv = 0.;
        for (const auto &agent : agents) {
            retv = std::max(retv, agent->excircle_radius_);
        }
        return retv;
    }

    template<Dimension N>
    Pointi<N> toPointi(const Pointi<N>& pt) {
        return pt;
    }

    template<Dimension N>
    Pointi<N> toPointi(const Pose<int, N>& pose) {
        return pose.pt_;
    }

    // use as external static constraint
    // avoid each time traversal all path to check conflict
    template<Dimension N, typename State>
    struct LargeAgentStaticConstraintTable {
        LargeAgentStaticConstraintTable(
                float max_excircle_radius, // max excircle radius for external and internal agents
                DimensionLength *dim,
                const IS_OCCUPIED_FUNC<N> &isoc,
                const std::vector<AgentPtr<N> > &global_agents, // all agents, global level
                const std::vector<AgentPtr<N> > &local_agents, // all agents, local level
                const std::vector<std::shared_ptr<State> > &all_poses)
                : max_excircle_radius_(max_excircle_radius),
                  isoc_(isoc),
                  dim_(dim),
                  global_agents_(global_agents),
                  local_agents_(local_agents),
                  all_poses_(all_poses) {
            total_index_ = getTotalIndexOfSpace<N>(dim);
            //assert(all_poses.size() / (2 * N) == total_index_);
            occupied_table_sat_.resize(total_index_, {});
            static_time_ = 0;
            for (const auto &agent : local_agents_) {
                float radius = max_excircle_radius_ + agent->excircle_radius_;
                // plus one to ensure check including both current node and next node of edge
                points_in_agent_circles_.insert(
                        {agent->id_, GetSphereInflationOffsetGrids<N>((uint) ceil(radius + 1))}); //
            }
        }

        // avoid conflict with  previous agents' target and future agents' start
        // agent global id
        bool hasCollideWithSAT(int agent_global_id, const size_t &current_node, const size_t &next_node) const {
            // 1, get points in range
            auto iter = points_in_agent_circles_.find(agent_global_id);
            assert(iter != points_in_agent_circles_.end());
            const auto &cur_pts = iter->second;
            const auto &center_pt = toPointi<N>(*all_poses_[next_node]);//->pt_;
            Pointi<N> temp_pt;
            Id temp_id;
            for (const auto &pt : cur_pts) {
                temp_pt = pt + center_pt;
                if (!isoc_(temp_pt)) {
                    // 2, only passable points in range
                    temp_id = PointiToId(temp_pt, dim_);
                    for (const auto &agent_pair : occupied_table_sat_[temp_id]) {
                        if (isCollide(global_agents_[agent_global_id], *all_poses_[current_node],
                                      *all_poses_[next_node],
                                      global_agents_[agent_pair.first], *all_poses_[agent_pair.second])) {
                            // debug
//                            if(global_agents_[agent_global_id].id_ == 4 && global_agents_[agent_pair.first].id_ == 3) {
//                                std::cout << "SAT: " << global_agents_[agent_global_id] << " at "
//                                          << *all_poses_[current_node] << "{" << current_node <<  "}"
//                                          << "->" << *all_poses_[next_node] << "{" << next_node <<  "}"
//                                          << " and " << global_agents_[agent_pair.first] << " at "
//                                          << *all_poses_[agent_pair.second]
//                                          << " have conflict" << std::endl;
//
//                                if(isCollide(global_agents_[agent_global_id], *all_poses_[current_node],
//                                             global_agents_[agent_pair.first], *all_poses_[agent_pair.second])) {
//                                    std::cout << "current node collide" << std::endl;
//                                }
//                                if(isCollide(global_agents_[agent_global_id], *all_poses_[next_node],
//                                             global_agents_[agent_pair.first], *all_poses_[agent_pair.second])) {
//                                    std::cout << "next_node node collide" << std::endl;
//                                }
//
//                            }
                            return true;
                        }
                    }
                }
            }
            return false;
        }

        // avoid conflict with  previous agents' target and future agents' start
        // agent global id
        bool hasCollide(int agent_global_id, int time_index, const size_t &current_node, const size_t &next_node,
                        bool is_goal = false) const {
//            std::cout << "agent_global_id = " << agent_global_id << std::endl;
            // 1, get points in range
//            auto iter = points_in_agent_circles_.find(agent_global_id);
//            assert(iter != points_in_agent_circles_.end());
//            const auto& cur_pts = iter->second;
//            const auto& center_pt = all_poses_[next_node]->pt_;
//            Pointi<N> temp_pt;
//            Id temp_id;
//            for(const auto& pt : cur_pts) {
//                temp_pt = pt + center_pt;
//                if(!isoc_(temp_pt)) {
//                    // 2, only passable points in range
//                    temp_id = PointiToId<N>(temp_pt, dim_);
//                    for(const auto& agent_pair : occupied_table_path_[time_index][temp_id]) {
//                        auto cur_path = path_constraint_table_.at(agent_pair.first);
//                        // if have reach target, no need to check
//                        if(time_index >= cur_path.size()-1) { continue; }
//                        if(isCollide(global_agents_[agent_global_id],  *all_poses_[current_node],         *all_poses_[next_node],
//                                     global_agents_[agent_pair.first], *all_poses_[cur_path[time_index]], *all_poses_[cur_path[time_index+1]])) {
//                            return true;
//                        }
//                    }
//                }
//            }
            if (is_goal) {
                if (time_index + 1 < earliest_arrive_time_map_.at(agent_global_id)) {
//                    std::cout << "reach target failed 1" << std::endl;
                    return true;
                }
            }
            const auto &agent = global_agents_[agent_global_id];
            // replace traversal all path with only path in range
            for (const auto &agent_path : path_constraint_table_) {
                const auto &other_agent_id = agent_path.first;
                const auto &other_agent = global_agents_[other_agent_id];
                const auto &other_path = agent_path.second;
                if (agent_global_id == other_agent_id) {
//                    std::cout << " FATAL: agent_global_id == other_agent_id " << std::endl;
                    continue;
                }
                if (other_path.size() - 1 <= time_index) {
                    if (isCollide(agent, *all_poses_[current_node], *all_poses_[next_node],
                                  other_agent, *all_poses_[other_path.back()])) {
//                        std::cout << "t=" << time_index << " " << agent << " at " << *all_poses_[current_node] << "->" << *all_poses_[next_node]
//                                  << " and " << global_agents_[other_agent_id] << " at " << *all_poses_[other_path.back()]
//                                  << " have conflict" << std::endl;
                        return true;
                    } else {
//                        std::cout << "t=" << time_index << " " << agent << " at " << *all_poses_[current_node] << "->" << *all_poses_[next_node]
//                                  << " and " << global_agents_[other_agent_id] << " at " << *all_poses_[other_path.back()]
//                                  << " have no conflict" << std::endl;
                    }
                } else {
                    if (isCollide(agent, *all_poses_[current_node], *all_poses_[next_node],
                                  other_agent, *all_poses_[other_path[time_index]],
                                  *all_poses_[other_path[time_index + 1]])) {
//                        std::cout << "t=" << time_index << " " << agent << " at " << *all_poses_[current_node] << "->" << *all_poses_[next_node]
//                                  << " and " << global_agents_[other_agent_id] << " at " << *all_poses_[other_path[time_index]]
//                                  << "->" << *all_poses_[other_path[time_index+1]]
//                                  << " have conflict" << std::endl;
                        return true;
                    } else {
//                        std::cout << "t=" << time_index << " " << agent << " at " << *all_poses_[current_node] << "->" << *all_poses_[next_node]
//                                  << " and " << global_agents_[other_agent_id] << " at " << *all_poses_[other_path[time_index]]
//                                  << "->" << *all_poses_[other_path[time_index+1]]
//                                  << " have no conflict" << std::endl;
                    }
                }
                // if current agent reach target, check whether it collide with other agent in the future
                // could be accelerate, by avoid
//                std::cout << "ct 4" << std::endl;
                if (is_goal) {
                    for (int t = earliest_arrive_time_map_.at(agent_global_id); t < other_path.size() - 1; t++) {
                        if (isCollide(agent, *all_poses_[current_node], *all_poses_[next_node],
                                      global_agents_[other_agent_id], *all_poses_[other_path[t]],
                                      *all_poses_[other_path[t + 1]])) {
//                            std::cout << "reach target failed 2" << std::endl;
                            return true;
                        }
                    }
                }
            }
//            std::cout << "ct 5" << std::endl;
            if (hasCollideWithSAT(agent_global_id, current_node, next_node)) {
//                std::cout << "reach target failed 3" << std::endl;
                return true;
            }
            return false;
        }

        void insertPoses(const std::vector<AgentPtr<N> > &agents, const std::vector<size_t> &pose_ids) {
            assert(agents.size() == pose_ids.size());
            for (int i = 0; i < pose_ids.size(); i++) {
                insertPose(agents[i]->id_, pose_ids[i]);
            }
        }

        void insertPose(int agent_id_global, const size_t &node_id) {
//            occupied_table_sat_[node_id / (2 * N)].push_back({agent_id_global, node_id});
            Pointi<N> pt = toPointi(*all_poses_[node_id]);
            occupied_table_sat_[PointiToId(pt, dim_)].push_back({agent_id_global, node_id});
        }

        void insertPath(int global_agent_id, const LAMAPF_Path &path) {
            path_constraint_table_.insert({global_agent_id, path});
            static_time_ = std::max((int) path.size() + 1, static_time_);
//            if(occupied_table_path_.size() < path.size()) {
//                occupied_table_path_.resize(path.size());
//            }
//            for(int t=0; t<path.size(); t++) {
//                const auto& node_id = path[t];
//                if(occupied_table_path_[t].size() != total_index_) { occupied_table_path_[t].resize(total_index_); }
//                occupied_table_path_[t][node_id/(2*N)].push_back({global_agent_id, node_id});
//            }
        }

        void updateEarliestArriveTimeForAgents(const std::vector<AgentPtr<N> > &agents,
                                               const std::vector<size_t> &target_node_ids) {
            assert(agents.size() == target_node_ids.size());
            static_time_ = std::max(2, static_time_);
            earliest_arrive_time_map_.clear();
            for (int k = 0; k < agents.size(); k++) {
                int earliest_time = calculateEarliestArriveTimeForAgent(agents[k], target_node_ids[k]);
                earliest_arrive_time_map_.insert({agents[k]->id_, earliest_time});
            }
        }

        // call before use bool hasCollide(...)
        // calculate the earliest possible arrive time
        int calculateEarliestArriveTimeForAgent(const AgentPtr<N> &agent, const size_t &target_node_id) const {
            int temp_earliest_arrive_time_ = 0;
            for (const auto &agent_path : path_constraint_table_) {
                const auto &path = agent_path.second;
                int local_earliest_arrive_time = 0;
                for (int t = path.size() - 2; t >= std::max(temp_earliest_arrive_time_ - 1, 0); t--) {
                    if (isCollide(global_agents_[agent_path.first], *all_poses_[path[t]], *all_poses_[path[t + 1]],
                                  agent, *all_poses_[target_node_id])) {
                        break;
                    } else {
                        local_earliest_arrive_time = t + 1;
                    }
                }
                temp_earliest_arrive_time_ = std::max(local_earliest_arrive_time, temp_earliest_arrive_time_);
            }
//            std::cout << " set agent " << agent << " earliest_arrive_time_ = " << temp_earliest_arrive_time_ << std::endl;
            return temp_earliest_arrive_time_;
        }

        const std::vector<AgentPtr<N> > &global_agents_;
        const std::vector<AgentPtr<N> > &local_agents_;
        const std::vector<std::shared_ptr<State> > &all_poses_;
        const float max_excircle_radius_;
        const IS_OCCUPIED_FUNC<N> &isoc_;
        const DimensionLength *dim_;
        Id total_index_;

        std::vector<size_t> maximum_length_of_paths_; // max length for each map, now only used in independence decomposition

        std::map<int, Pointis<N> > points_in_agent_circles_; // precomputed check range

        // location id and related poses id
        std::vector<std::vector<std::pair<int, size_t> > > occupied_table_sat_;

        // agent.id_ and related path
        // t, location id and related {agent_global_id, poses id}
        std::vector<std::vector<std::vector<std::pair<int, size_t> > > > occupied_table_path_;
        std::map<int, LAMAPF_Path> path_constraint_table_;
        std::map<int, int> earliest_arrive_time_map_; // agent.id_ and the earliest time to visit the target

        int static_time_ = 0; // after which time, every thing is static

    };

    template<Dimension N, typename State>
    using LargeAgentStaticConstraintTablePtr = std::shared_ptr<LargeAgentStaticConstraintTable<N, State> >;

    template<Dimension N, typename State>
    struct SubGraphOfAgentData {

        std::vector<std::shared_ptr<State> > all_nodes_;
        std::vector<std::vector<size_t> > all_edges_;
        std::vector<std::vector<size_t> > all_backward_edges_;
    };

    template<Dimension N, typename State>
    using SubGraphOfAgentDataPtr = std::shared_ptr<SubGraphOfAgentData<N, State> >;

    template<Dimension N, typename State>
    struct SubGraphOfAgent {

        explicit SubGraphOfAgent(AgentPtr<N> agent) : agent_(agent) {}

        SubGraphOfAgentDataPtr<N, State> data_ptr_;

        AgentPtr<N> agent_;
        size_t start_node_id_ = MAX<size_t>;
        size_t target_node_id_ = MAX<size_t>;

    };

    // input: static occupancy map / current solving problem / previous path as constraints
    // output: what path was found, or empty is failed
    template<Dimension N, typename State>
    using LA_MAPF_FUNC = std::function<std::vector<LAMAPF_Path>(const std::vector<std::pair<State, State> > &,
                                                                const std::vector<AgentPtr<N> > &,
                                                                DimensionLength *dim,
                                                                const IS_OCCUPIED_FUNC<N> &,
                                                                const LargeAgentStaticConstraintTablePtr<N, State> &,
                                                                std::vector<std::vector<int> > &,
                                                                double,
                                                                const std::vector<std::pair<size_t, size_t> >&,
                                                                const std::vector<std::shared_ptr<State> > &,
                                                                const DistanceMapUpdaterPtr<N>,
                                                                const std::vector<SubGraphOfAgent<N, State> >,
                                                                const std::vector<std::vector<int> > &,
                                                                const std::vector<std::vector<int> > &,
                                                                ConnectivityGraph *)>;

    template<Dimension N, typename State>
    Conflicts detectAllConflictBetweenPaths(const LAMAPF_Path &p1, const LAMAPF_Path &p2,
                                            const AgentPtr<N> &a1, const AgentPtr<N> &a2,
                                            const std::vector<std::shared_ptr<State>> &all_nodes) {
        int t1 = p1.size() - 1, t2 = p2.size() - 1;
        const auto &longer_agent = p1.size() > p2.size() ? a1 : a2;
        const auto &shorter_agent = longer_agent->id_ == a1->id_ ? a2 : a1;
        const auto &longer_path = longer_agent->id_ == a1->id_ ? p1 : p2;
        const auto &shorter_path = longer_agent->id_ == a1->id_ ? p2 : p1;

        int common_part = std::min(t1, t2);
        std::vector<std::shared_ptr<Conflict> > cfs;
        for (int t = 0; t < common_part - 1; t++) {
            if (isCollide(a1, *all_nodes[p1[t]], *all_nodes[p1[t + 1]],
                          a2, *all_nodes[p2[t]], *all_nodes[p2[t + 1]])) {

//                std::cout << "cs type 1 : " << *all_nodes[p1[t]] << "->" << *all_nodes[p1[t+1]] << ", "
//                                            << *all_nodes[p2[t]] << "->" << *all_nodes[p2[t+1]]
//                                            << "/t:{" << t << "," << t+1 << "}" << std::endl;

                auto c1 = std::make_shared<Constraint>(a1->id_, p1[t], p1[t + 1], t, t + 2);
                auto c2 = std::make_shared<Constraint>(a2->id_, p2[t], p2[t + 1], t, t + 2);
                auto cf = std::make_shared<Conflict>(a1->id_, a2->id_, Constraints{c1}, Constraints{c2});
                cfs.push_back(cf);
            }
        }
        for (int t = common_part - 1; t < std::max(t1, t2) - 1; t++) {
            if (isCollide(longer_agent, *all_nodes[longer_path[t]], *all_nodes[longer_path[t + 1]],
                          shorter_agent, *all_nodes[shorter_path.back()])) {

//                std::cout << "cs type 2 : " << *all_nodes[longer_path[t]] << "->" << *all_nodes[longer_path[t+1]] << ", "
//                                            << *all_nodes[shorter_path.back()]
//                                            << "/t:{" << t << "," << t+1 << "}"
//                                            << std::endl;

                auto c1 = std::make_shared<Constraint>(longer_agent->id_, longer_path[t], longer_path[t + 1], t, t + 2);
                auto c2 = std::make_shared<Constraint>(shorter_agent->id_, shorter_path.back(), MAX_NODES, 0, t + 2);
                auto cf = std::make_shared<Conflict>(longer_agent->id_, shorter_agent->id_, Constraints{c1},
                                                     Constraints{c2});
                cfs.push_back(cf);
            }
        }
        return cfs;
    }

    template<Dimension N, typename State>
    ConflictPtr detectFirstConflictBetweenPaths(const LAMAPF_Path &p1, const LAMAPF_Path &p2,
                                                const AgentPtr<N> &a1, const AgentPtr<N> &a2,
                                                const std::vector<std::shared_ptr<State> > &all_nodes) {
        int t1 = p1.size() - 1, t2 = p2.size() - 1;
        const auto &longer_agent = p1.size() > p2.size() ? a1 : a2;
        const auto &shorter_agent = longer_agent->id_ == a1->id_ ? a2 : a1;
        const auto &longer_path = longer_agent->id_ == a1->id_ ? p1 : p2;
        const auto &shorter_path = longer_agent->id_ == a1->id_ ? p2 : p1;

        int common_part = std::min(t1, t2);
        for (int t = 0; t < common_part - 1; t++) {
            if (isCollide(a1, *all_nodes[p1[t]], *all_nodes[p1[t + 1]],
                          a2, *all_nodes[p2[t]], *all_nodes[p2[t + 1]])) {

                std::cout << "cs type 1 : " << *all_nodes[p1[t]] << "->" << *all_nodes[p1[t + 1]] << ", "
                          << *all_nodes[p2[t]] << "->" << *all_nodes[p2[t + 1]]
                          << "/t:{" << t << "," << t + 1 << "}" << std::endl;

                auto c1 = std::make_shared<Constraint>(a1->id_, p1[t], p1[t + 1], t, t + 2);
                auto c2 = std::make_shared<Constraint>(a2->id_, p2[t], p2[t + 1], t, t + 2);
                auto cf = std::make_shared<Conflict>(a1->id_, a2->id_, Constraints{c1}, Constraints{c2});
                return cf;
            }
        }
        for (int t = common_part - 1; t < std::max(t1, t2) - 1; t++) {
            if (isCollide(longer_agent, *all_nodes[longer_path[t]], *all_nodes[longer_path[t + 1]],
                          shorter_agent, *all_nodes[shorter_path.back()])) {

                std::cout << "cs type 2 : " << *all_nodes[longer_path[t]] << "->" << *all_nodes[longer_path[t + 1]]
                          << ", "
                          << *all_nodes[shorter_path.back()]
                          << "/t:{" << t << "," << t + 1 << "}"
                          << std::endl;

                auto c1 = std::make_shared<Constraint>(longer_agent->id_, longer_path[t], longer_path[t + 1], t, t + 2);
                auto c2 = std::make_shared<Constraint>(shorter_agent->id_, shorter_path.back(), MAX_NODES, t, t + 2);
                auto cf = std::make_shared<Conflict>(longer_agent->id_, shorter_agent->id_, Constraints{c1},
                                                     Constraints{c2});
                return cf;
            }
        }
        return nullptr;
    }

    template<Dimension N, typename State>
    bool isSolutionValid(const LAMAPF_Paths &paths,
                         const std::vector<AgentPtr<N> > &agents,
                         const std::vector<std::shared_ptr<State> > &all_poses) {
        assert(paths.size() == agents.size());
        bool valid = true;
        for (int i = 0; i < paths.size(); i++) {
            for (int j = i + 1; j < paths.size(); j++) {
                if (paths[i].empty() || paths[j].empty()) { continue; }
                auto conflict_ptr = detectFirstConflictBetweenPaths(paths[i], paths[j], agents[i], agents[j],
                                                                    all_poses);
                if (conflict_ptr != nullptr) {
                    std::cout << "FATAL: detect conflicts between agent "
                              << agents[i] << "path_size(" << paths[i].size() << ")" << " and "
                              << agents[j] << "path_size(" << paths[j].size() << ")"
                              << ", at (t1 = " << conflict_ptr->t1 << ", t2 = " << conflict_ptr->t2 << ")" << std::endl;
                    valid = false;
                }
            }
        }
        return valid;
    }

    struct GraphForTest {
        GraphForTest(size_t size = 10) {
            all_edges_.resize(10);
        }

        void addEdge(const size_t &node_from, const size_t &node_to) {
            if (node_from + 1 > all_edges_.size()) {
                all_edges_.resize(node_from + 1);
            }

            if (node_to + 1 > all_edges_.size()) {
                all_edges_.resize(node_to + 1);
            }

            all_edges_[node_from].push_back(node_to);

//            all_backward_edges_[node_to].push_back(node_from);

        }

        std::vector<std::vector<size_t> > all_edges_;
        std::vector<std::vector<size_t> > all_backward_edges_;

    };

    template<typename State>
    std::pair<std::vector<std::set<size_t> >, std::vector<int> > getStrongComponentFromSubGraph(
            const std::vector<std::shared_ptr<State> > &all_poses,
            const std::vector<std::vector<size_t> > &all_edges,
            const std::vector<std::vector<size_t> > &all_backward_edges,
            const std::vector<std::set<int> > &related_agents_map,
            bool directed_graph = true) {

        using namespace boost;
        using Vertex = size_t;

        if (directed_graph) {
            typedef adjacency_list<vecS, vecS, directedS, Vertex> Graph;
            Graph g(all_poses.size());
            for (size_t i = 0; i < all_edges.size(); i++) {
                if (all_poses[i] == nullptr) { continue; }
                if (all_edges[i].empty() || all_backward_edges[i].empty()) {
                    //add_vertex(i, g); // any non-nullptr should have a position
//                        add_edge(Vertex(i), Vertex(i), g);
                    continue;
                }
                for (const size_t &j : all_edges[i]) {
                    assert(i != MAX<size_t> && j != MAX<size_t>);
                    if (related_agents_map[i] != related_agents_map[j]) { continue; }
                    add_edge(Vertex(i), Vertex(j), g);
                }
            }
            std::vector<int> component(num_vertices(g));
            int num = strong_components(g, &component[0]);
            std::vector<std::set<size_t> > retv(num);
//                std::cout << "Total number of strong components: " << num << std::endl;
            for (size_t i = 0; i < component.size(); i++) {
//                    std::cout << "Vertex " << i << " is in component " << component[i] << std::endl;
                retv[component[i]].insert(i);
            }

            return {retv, component};
        } else {
            typedef adjacency_list<vecS, vecS, undirectedS, size_t> Graph;
            Graph g;
            for (size_t i = 0; i < all_edges.size(); i++) {
                if (all_edges[i].empty()) { continue; }
                for (const size_t &j : all_edges[i]) {
                    add_edge(i, j, g);
                }
            }
            std::vector<int> component(num_vertices(g));
            int num = connected_components(g, &component[0]);
            std::vector<std::set<size_t> > retv(num);
//                std::cout << "Total number of strong components: " << num << std::endl;
            for (size_t i = 0; i < component.size(); ++i) {
//                    std::cout << "Vertex " << i << " is in component " << component[i] << std::endl;
                retv[component[i]].insert(i);
            }
            return {retv, component};
        }
    }

    extern MemoryRecorder memory_recorder;

    std::pair<int, std::set<int> > getMaxLevel(const std::vector<std::set<int> > &all_levels);

    size_t getMaxLevelSize(const std::vector<std::set<int> > &all_levels);


    // get the i th largest level, i_th start with 0
    std::pair<int, std::set<int> > getMaxLevel(const std::vector<std::set<int> > &all_levels, const int &i_th);

    template<Dimension N>
    bool isMAPFInstanceSolvable(const Pointi<N> &start_pt, const Pointi<N> &target_pt, const IS_OCCUPIED_FUNC<N> &isoc,
                                DimensionLength *dim) {
        Pointis<N> neighbors = GetNearestOffsetGrids<N>();

        Id total_index = getTotalIndexOfSpace<N>(dim);

        std::vector<bool> visited(total_index, false);
        std::vector<Pointi<N> > buffer = {start_pt}, next_buffer;
        visited[PointiToId<N>(start_pt, dim)] = true;

        while (!buffer.empty()) {
            next_buffer.clear();
            for (const Pointi<N> &pt : buffer) {
                for (const Pointi<N> &offset : neighbors) {
                    Pointi<N> new_pt = pt + offset;
                    if (isoc(new_pt)) { continue; }
                    if (new_pt == target_pt) {
                        return true;
                    }
                    Id new_id = PointiToId<N>(new_pt, dim);
                    if (visited[new_id]) { continue; }
                    visited[new_id] = true;
                    next_buffer.push_back(new_pt);
                }
            }
            std::swap(buffer, next_buffer);
        }
        return false;
    }

    template<Dimension N>
    bool MAPF_DecompositionValidCheckGridMap(const Instances<N> &instances,
                                             const std::vector<std::set<int> > &all_levels,
                                             DimensionLength *dim,
                                             const IS_OCCUPIED_FUNC<N> &isoc) {

        for (int i = 0; i < all_levels.size(); i++) {
            Id total_index = getTotalIndexOfSpace<N>(dim);
            std::vector<bool> avoid_locs(total_index, false);

            for (int j = 0; j < all_levels.size(); j++) {
                if (j == i) continue;
                const auto &current_level = all_levels[j];
                for (const int &agent_id : current_level) {
                    Id id;
                    if (j < i) {
                        id = PointiToId(instances[agent_id].second, dim);
                    } else {
                        id = PointiToId(instances[agent_id].first, dim);
                    }
                    avoid_locs[id] = true;
                }
            }

            auto new_isoc = [&](const Pointi<N> &pt) -> bool {
                if (isOutOfBoundary(pt, dim)) { return true; }
                return isoc(pt) || avoid_locs[PointiToId(pt, dim)];
            };

            for (const int &agent_id : all_levels[i]) {
                if (!isMAPFInstanceSolvable<N>(instances[agent_id].first, instances[agent_id].scond, new_isoc, dim)) {
                    return false;
                }
            }
        }
        return true;
    }


    template<Dimension N, typename State>
    std::vector<int> constructHeuristicTable(const LA_MAPF::SubGraphOfAgent<N, State>& sub_graph,
                                             const size_t& goal_id) {
        struct Node {
            int node_id;
            int value;

            Node() = default;

            Node(int node_id, int value) : node_id(node_id), value(value) {}

            // the following is used to compare nodes in the OPEN list
            struct compare_node {
                // returns true if n1 > n2 (note -- this gives us *min*-heap).
                bool operator()(const Node &n1, const Node &n2) const {
                    return n1.value >= n2.value;
                }
            };  // used by OPEN (heap) to compare nodes (top of the heap has min f-val, and then highest g-val)
        };
        std::vector<int> agent_heuristic(sub_graph.data_ptr_->all_nodes_.size(), MAX<int>);

        // generate a heap that can save nodes (and an open_handle)
        boost::heap::pairing_heap<Node, boost::heap::compare<typename Node::compare_node> > heap;

        Node root(goal_id, 0);
        agent_heuristic[goal_id] = 0;
        heap.push(root);  // add root to heap
        // yz: compute heuristic from goal to visible grid via Breadth First Search
        //     search the static shortest path
        while (!heap.empty()) {
            Node curr = heap.top();
            heap.pop();
            for (const size_t& next_location : sub_graph.data_ptr_->all_backward_edges_[curr.node_id]) {
                int new_heuristic_value = curr.value + 1;
                if (agent_heuristic[next_location] > new_heuristic_value) {
                    agent_heuristic[next_location] = new_heuristic_value;
                    Node next(next_location, new_heuristic_value);
                    heap.push(next);
                }
            }
        }

        // debug
//            for(int i=0; i<sub_graph.all_nodes_.size(); i++) {
//                if(sub_graph.all_nodes_[i] != nullptr) {
//                    assert(agent_heuristic[i] != MAX<int>);
//                }
//            }
        // shrink table
//            for(int k=0; k<sub_graph.all_nodes_.size()/(2*N); k++) {
//                //
//            }
        return agent_heuristic;
    }

    // only for poseptr / lamapf
    template<Dimension N>
    std::vector<int> constructHeuristicTableIgnoreRotate(const LA_MAPF::SubGraphOfAgent<N, Pose<int, N>>& sub_graph,
                                                         const size_t& goal_id, DimensionLength* dim, const IS_OCCUPIED_FUNC<N>& isoc) {
        struct Node {
            Pointi<N> node_pt;
            int value;

            Node() = default;

            Node(const Pointi<N>& node_pt, int value) : node_pt(node_pt), value(value) {}

            // the following is used to compare nodes in the OPEN list
            struct compare_node {
                // returns true if n1 > n2 (note -- this gives us *min*-heap).
                bool operator()(const Node &n1, const Node &n2) const {
                    return n1.value >= n2.value;
                }
            };  // used by OPEN (heap) to compare nodes (top of the heap has min f-val, and then highest g-val)
        };

        std::vector<int> retv(sub_graph.data_ptr_->all_nodes_.size()/(2*N), MAX<int>);
        // 1, get grid connectivity graph
        std::vector<bool> c_graph(sub_graph.data_ptr_->all_nodes_.size()/(2*N), false);
        for(int i=0; i<sub_graph.data_ptr_->all_nodes_.size(); i++) {
            if(sub_graph.data_ptr_->all_nodes_[i] != nullptr) {
//                    std::cout << "flag 4" << std::endl;
                c_graph[i/(2*N)] = true;
            }
        }
        // 2, wave front to get heuristic table
        // generate a heap that can save nodes (and an open_handle)
        boost::heap::pairing_heap<Node, boost::heap::compare<typename Node::compare_node> > heap;
        Pointi<N> target_pt = IdToPointi<N>(goal_id/(2*N), dim);
        Node root(target_pt, 0);
        retv[goal_id/(2*N)] = 0;
        heap.push(root);  // add root to heap
        // yz: compute heuristic from goal to visible grid via Breadth First Search
        //     search the static shortest path
        Pointis<N> offsets = GetNearestOffsetGrids<N>();
        while (!heap.empty()) {
            Node curr = heap.top();
            heap.pop();
            for (const Pointi<N>& offset : offsets) {
                Pointi<N> next_location = curr.node_pt + offset;
                if(isoc(next_location)) { continue; }
                Id id = PointiToId<N>(next_location, dim);
                if(!c_graph[id]) { continue; }
                int new_heuristic_value = curr.value + 1;
                if (retv[id] > new_heuristic_value) {
                    retv[id] = new_heuristic_value;
                    Node next(next_location, new_heuristic_value);
                    heap.push(next);
                }

            }
        }
        return retv;
    }


    template<Dimension N>
    size_t toPointId(const std::shared_ptr<Pose<int, N>>& state_ptr, size_t id) {
        return id/(2*N);
    }

    template<Dimension N>
    size_t toPointId(const std::shared_ptr<Pointi<N>>& state_ptr, size_t id) {
        return id;
    }

}

#endif //LAYEREDMAPF_COMMON_H
