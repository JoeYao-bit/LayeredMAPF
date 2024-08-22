//
// Created by yaozhuo on 2024/5/19.
//

#ifndef LAYEREDMAPF_COMMON_H
#define LAYEREDMAPF_COMMON_H

#include <limits>

#include <boost/geometry.hpp>
#include <boost/unordered_map.hpp>
#include <boost/heap/pairing_heap.hpp>
#include <random>

#include "../basic.h"
#include "../freeNav-base/basic_elements/distance_map_update.h"
#include "../../freeNav-base/basic_elements/point.h"
#include "../freeNav-base/basic_elements/point.h"

namespace freeNav::LayeredMAPF::LA_MAPF {

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
        Conflict(int a1, int a2, const Constraints& cs1, const Constraints& cs2)
        : a1(a1), a2(a2), cs1(cs1), cs2(cs2) {
            int agent; size_t from, to;
            int start_t, end_t;
            // get start time index of conflict 1
            for(const auto& cs : cs1) {
                std::tie(agent, from, to, start_t, end_t) = *cs;
                t1 = start_t < t1 ? start_t : t1;
                t2 = end_t > t2   ? end_t   : t2;
            }
            for(const auto& cs : cs2) {
                std::tie(agent, from, to, start_t, end_t) = *cs;
                t1 = start_t < t1 ? start_t : t1;
                t2 = end_t > t2   ? end_t   : t2;
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
    int getSOC(const std::vector<std::vector<T> >& paths) {
        size_t soc = 0;
        for (size_t a1 = 0; a1 < paths.size(); a1++) {
            soc += paths[a1].size() - 1; // yz: soc: sum of cost
        }
        return soc;
    }

    template<typename T>
    int getMakeSpan(const std::vector<std::vector<T> >& paths) {
        size_t mk = 0;
        for (size_t a1 = 0; a1 < paths.size(); a1++) {
            mk = std::max(paths[a1].size(), mk); // yz: soc: sum of cost
        }
        return mk;
    }

    bool isSamePath(const std::vector<size_t>& path1, const std::vector<size_t>& path2);

    template <Dimension N>
    struct Agent {

        explicit Agent(float excircle_radius, float incircle_radius, int id, DimensionLength* dim)
        : excircle_radius_(excircle_radius), incircle_radius_(incircle_radius),id_(id),dim_(dim) {}

        virtual bool isCollide(const Pose<int, N>& pose,
                               DimensionLength* dim,
                               const IS_OCCUPIED_FUNC<N>& isoc,
                               const DistanceMapUpdater<N>& distance_table) const = 0;

        virtual bool isCollide(const Pose<int, N>& edge_from,
                               const Pose<int, N>& edge_to,
                               DimensionLength* dim,
                               const IS_OCCUPIED_FUNC<N>& isoc,
                               const DistanceMapUpdater<N>& distance_table) const = 0;

        virtual Pointis<N> getTransferOccupiedGrid(const Pose<int, N>& edge_from,
                                                   const Pose<int, N>& edge_to) const = 0;

        virtual std::pair<Pointis<N>, Pointis<N>> getPoseOccupiedGrid(const Pose<int, N>& pose) const = 0;

//        virtual std::pair<Pointis<N>, Pointis<N>> getCoverageGridWithinPose(const Pose<int, N>& pose) const = 0;

        float excircle_radius_, incircle_radius_;

        int id_;

        DimensionLength* dim_ = nullptr;

    };


    template <Dimension N>
    using Agents = std::vector<Agent<N> >;

    class HighLvNode;

    /* [row][col] */
    typedef std::vector<std::vector<int> > RotateMatrix;

    typedef std::vector<RotateMatrix> RotateMatrixs;


    RotateMatrix initializeRotateMatrix(const int& dim, const int& orient);

    // for calculate rotate matrix about 2D and 3D maps
    RotateMatrixs initializeRotateMatrixs(const int& dim);

    std::vector<RotateMatrixs> initializeAllRotateMatrixs();

    template<typename T, Dimension N>
    Point<T, N> rotatePoint(const Point<T, N>& pt, const RotateMatrix& mt) {
        Point<T, N> retv;
        for(int row=0; row<N; row++) {
            retv[row] = 0;
            for(int col=0; col<N; col++) {
                retv[row] = retv[row] + pt[col]*mt[col][row];
            }
        }
        return retv;
    }

    template <Dimension N>
    bool isPointSetOverlap(const Pointis<N>& set1, const Pointis<N>& set2, DimensionLength* dim) {
        IdSet ids;
        for(const auto& pt : set1) {
            if(isOutOfBoundary(pt, dim)) { continue; }
            ids.insert(PointiToId(pt, dim));
        }
        Id id;
        for(const auto& pt : set2) {
            if(isOutOfBoundary(pt, dim)) { continue; }
            id = PointiToId(pt, dim);
            if(ids.find(id) != ids.end()) {
                return true;
            }
        }
        return false;
    }

    template<typename T, Dimension N>
    bool isRectangleOverlap(const Point<T, N>& r1_min, const Point<T, N>& r1_max,
                            const Point<T, N>& r2_min, const Point<T, N>& r2_max) {
        for(int dim=0; dim<N; dim++) {
            assert(r1_min[dim] < r1_max[dim]);
            assert(r2_min[dim] < r2_max[dim]);
            if(r1_max[dim] < r2_min[dim]) {
                return false;
            }
            if(r1_min[dim] > r2_max[dim]) {
                return false;
            }
        }
        return true;
    }

    template<typename T1, typename T2, Dimension N>
    bool isPointInRectangle(const Point<T1, N>& r1_min, const Point<T1, N>& r1_max,
                            const Point<T2, N>& pt) {
        std::vector<bool> flag(N, false);
        for(int dim=0; dim<N; dim++) {
            // if in all dimension that two rectangle are overlaped, they are overlaped
            if( (pt[dim] - r1_min[dim]) * (pt[dim] - r1_max[dim]) <= 0 ) { flag[dim] = true; }
        }
        return flag == std::vector<bool>(N, true);
    }

    // public static element
    extern std::vector<RotateMatrixs> ROTATE_MATRIXS;

    RotateMatrix initRotateMatrix_2D(const double& angle);

    double orientToPi_2D(const int& orient);

    template<typename T>
    Point<T, 2> pointRotate_2D(const Point<T, 2>& pt, const int& orient) {
        return rotatePoint<T, 2>(pt, ROTATE_MATRIXS[2][orient]);
    }

    template<typename T>
    Point<T, 2> pointTransfer_2D(const Point<T, 2>& pt, const Pose<int, 2>& pose) {
        Point<T, 2> rotated_pt = pointRotate_2D(pt, pose.orient_);
        rotated_pt[0] = rotated_pt[0] + pose.pt_[0];
        rotated_pt[1] = rotated_pt[1] + pose.pt_[1];
        return  rotated_pt;
    }

    float get_random_float(std::mt19937 *MT, float from=0, float to=0);

    // a N-dimensional grid have 2^N corner
    template<Dimension N>
    Pointfs<N> getAllCornerOfGrid() {
        Pointf<N> ptf;
        Pointfs<N> retv;
        Pointis<N> offsets = GetNeightborOffsetGrids<N>(true);
//        std::cout << "offsets " << offsets.size() << std::endl;
        for(const auto& offset : offsets) {
            for(int i=0; i<N; i++) {
                ptf[i] = offset[i]/2.;
            }
            retv.push_back(ptf);
        }
        return retv;
    }

    // due to each agent may have different shape, each agent have their own connectivity graph
    struct ConnectivityGraph {

        explicit ConnectivityGraph(int total_nodes) {
            hyper_node_id_map_.resize(total_nodes, MAX<size_t>);
            related_agents_map_.resize(total_nodes, {});
        }

        std::vector<std::vector<int> > related_agents_map_; // store each pose collide with what agents' start(2*id) or target(2*id+1)

        // store each node's hyper node id, default to MAX<size_t>
        // may be disposed after we construct ConnectivityGraph
        std::vector<size_t> hyper_node_id_map_;

        // store what agent current hyper node associate with
        // a hyper node may associate with no agent may also associate with multiple agent
        std::vector<std::vector<int> > hyper_node_with_agents_;

        std::vector<std::vector<size_t> > all_edges_vec_; // each hyper node's connecting node, store in vector

        std::vector<std::set<size_t> > all_edges_set_; // each hyper node's connecting node, store in set

        size_t start_hyper_node_; // where is start in the hyper graph

        size_t target_hyper_node_; // where is target in the hyper graph

    };

    // consider static grid map when construct sub graph
    // check each pair of agent to see if they have conflict
    template<Dimension N, typename AgentType>
    struct LargeAgentPathConstraintTable {
    public:
        explicit LargeAgentPathConstraintTable(float max_excircle_radius, // max excircle radius for external and internal agents
                                               DimensionLength* dim,
                                               const IS_OCCUPIED_FUNC<N>& isoc,
                                               const std::vector<AgentType>& global_agents, // all agents, global level
                                               const std::vector<AgentType>& local_agents, // all agents, local level
                                               const std::vector<PosePtr<int, N> >& all_nodes)
                : global_agents_(global_agents), all_nodes_(all_nodes), dim_(dim) {
            //
        }

        void insertPaths(const std::vector<std::pair<int, LAMAPF_Path> >& agent_paths) {
            for(const auto& agent_path : agent_paths) {
                insertPath(agent_path);
            }
        }

        void insertPath(const std::pair<int, LAMAPF_Path>& agent_path) {
//            std::cout << "insert agent " << agents_[agent_path.first] << "'s path size = " << agent_path.second.size()
//                      << " as constraint " << std::endl;
            if(constraint_table_.find(agent_path.first) == constraint_table_.end()) {
                constraint_table_.insert(agent_path);
            } else {
                constraint_table_[agent_path.first] = agent_path.second;
            }
            max_time_stamp_ = std::max(max_time_stamp_, (int)agent_path.second.size());
        }

        void updateEarliestArriveTimeForAgents(const std::vector<AgentType>& agents, const std::vector<size_t>& target_node_ids) {
            assert(agents.size() == target_node_ids.size());
            earliest_arrive_time_map_.clear();
            for(int k=0; k<agents.size(); k++) {
                int earliest_time = calculateEarliestArriveTimeForAgent(agents[k], target_node_ids[k]);
                earliest_arrive_time_map_.insert({agents[k].id_, earliest_time});
            }
        }

        // call before use bool hasCollide(...)
        // calculate the earliest possible arrive time
        int calculateEarliestArriveTimeForAgent(const AgentType& agent, const size_t& target_node_id) const {
            int temp_earliest_arrive_time_ = 0;
            for(const auto& agent_path : constraint_table_) {
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
            std::cout << " set agent " << agent << " earliest_arrive_time_ = " << temp_earliest_arrive_time_ << std::endl;
            return temp_earliest_arrive_time_;
        }

        // whether an agent has conflict at pose
        bool hasCollide(int agent_global_id, int time_index, const size_t & current_node, const size_t & next_node, bool is_goal = false) const {
            const auto& agent = global_agents_[agent_global_id];
            if(is_goal) {
                if(time_index + 1 < earliest_arrive_time_map_.at(agent_global_id)) { return true; }
            }
            for(const auto& agent_path : constraint_table_) {
                const auto& other_agent_id = agent_path.first;
                const auto& other_path = agent_path.second;
                if(agent_global_id == other_agent_id) { continue; }
                if(other_path.size() - 1 <= time_index) {
                    if(isCollide(agent, *all_nodes_[current_node], *all_nodes_[next_node],
                                 global_agents_[other_agent_id], *all_nodes_[other_path.back()])) {
//                        std::cout << "t=" << time_index << " " << agent << " at " << *all_nodes_[current_node] << "->" << *all_nodes_[next_node]
//                                  << " and " << agents_[other_agent_id] << " at " << *all_nodes_[other_path.back()]
//                                  << " have conflict" << std::endl;
                        return true;
                    } else {
//                        std::cout << "t=" << time_index << " " << agent << " at " << *all_nodes_[current_node] << "->" << *all_nodes_[next_node]
//                                  << " and " << agents_[other_agent_id] << " at " << *all_nodes_[other_path.back()]
//                                  << " have no conflict" << std::endl;
                    }
                } else {
                    if(isCollide(agent, *all_nodes_[current_node], *all_nodes_[next_node],
                                 global_agents_[other_agent_id], *all_nodes_[other_path[time_index]], *all_nodes_[other_path[time_index+1]])) {
//                        std::cout << "t=" << time_index << " " << agent << " at " << *all_nodes_[current_node] << "->" << *all_nodes_[next_node]
//                                  << " and " << agents_[other_agent_id] << " at " << *all_nodes_[other_path[time_index]]
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
                    if(is_goal) {
                        for(int t=earliest_arrive_time_map_.at(agent_global_id); t<other_path.size()-1; t++) {
                            if(isCollide(agent, *all_nodes_[current_node], *all_nodes_[next_node],
                                         global_agents_[other_agent_id], *all_nodes_[other_path[t]], *all_nodes_[other_path[t+1]])) {
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

        int getEarliestTime(const AgentType& agent) const {
            return earliest_arrive_time_map_.at(agent.id_);
        }

    private:

        const std::vector<AgentType>& global_agents_;

        const std::vector<PosePtr<int, N> >& all_nodes_;

        std::map<int, LAMAPF_Path> constraint_table_; // global agent id

        DimensionLength* dim_;

        std::map<int, int> earliest_arrive_time_map_; // agent.id_ and the earliest time to visit the target

        int max_time_stamp_ = 0; // every thing is static after max_tme_stamp
    };

    template<Dimension N, typename AgentType>
    using LargeAgentPathConstraintTablePtr = std::shared_ptr<LargeAgentPathConstraintTable<N, AgentType> >;

    template<typename AgentType>
    float getMaximumRadius(const std::vector<AgentType>& agents) {
        float retv = 0.;
        for(const auto& agent : agents) {
            retv = std::max(retv, agent.excircle_radius_);
        }
        return retv;
    }

    // use as external static constraint
    // avoid each time traversal all path to check conflict
    template<Dimension N, typename AgentType>
    struct LargeAgentStaticConstraintTable {
        LargeAgentStaticConstraintTable(float max_excircle_radius, // max excircle radius for external and internal agents
                                        DimensionLength* dim,
                                        const IS_OCCUPIED_FUNC<N>& isoc,
                                        const std::vector<AgentType>& global_agents, // all agents, global level
                                        const std::vector<AgentType>& local_agents, // all agents, local level
                                        const std::vector<PosePtr<int, N> >& all_poses)
                                        : max_excircle_radius_(max_excircle_radius),
                                          isoc_(isoc),
                                          dim_(dim),
                                          global_agents_(global_agents),
                                          local_agents_(local_agents),
                                          all_poses_(all_poses) {
            total_index_ = getTotalIndexOfSpace<N>(dim);
            assert(all_poses.size()/(2*N) == total_index_);
            occupied_table_sat_.resize(total_index_, {});
            for(const auto& agent : local_agents_) {
                float radius = max_excircle_radius_ + agent.excircle_radius_;
                points_in_agent_circles_.insert({agent.id_, GetSphereInflationOffsetGrids<N>((uint)ceil(radius))});
            }
        }

        // avoid conflict with  previous agents' target and future agents' start
        // agent global id
        bool hasCollideWithSAT(int agent_global_id, const size_t & current_node, const size_t & next_node) const {
            // 1, get points in range
            auto iter = points_in_agent_circles_.find(agent_global_id);
            assert(iter != points_in_agent_circles_.end());
            const auto& cur_pts = iter->second;
            const auto& center_pt = all_poses_[next_node]->pt_;
            Pointi<N> temp_pt;
            Id temp_id;
            for(const auto& pt : cur_pts) {
                temp_pt = pt + center_pt;
                if(!isoc_(temp_pt)) {
                    // 2, only passable points in range
                    temp_id = PointiToId(temp_pt, dim_);
                    for(const auto& agent_pair : occupied_table_sat_[temp_id]) {
                        if(isCollide(global_agents_[agent_global_id], *all_poses_[next_node],
                                     global_agents_[agent_pair.first], *all_poses_[agent_pair.second])) {
                            return true;
                        }
                    }
                }
            }
            return false;
        }

        // avoid conflict with  previous agents' target and future agents' start
        // agent global id
        bool hasCollide(int agent_global_id, int time_index, const size_t & current_node, const size_t & next_node, bool is_goal = false) const {
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
            if(is_goal) {
                if(time_index + 1 < earliest_arrive_time_map_.at(agent_global_id)) { return true; }
            }
            const auto& agent = global_agents_[agent_global_id];
            for(const auto& agent_path : path_constraint_table_) {
                const auto& other_agent_id = agent_path.first;
                const auto& other_agent = global_agents_[other_agent_id];
                const auto& other_path = agent_path.second;
                if(agent_global_id == other_agent_id) {
                    std::cout << " FATAL: agent_global_id == other_agent_id " << std::endl;
                    continue;
                }
                if(other_path.size() - 1 <= time_index) {
                    if(isCollide(agent, *all_poses_[current_node], *all_poses_[next_node],
                                 other_agent, *all_poses_[other_path.back()])) {
//                        std::cout << "t=" << time_index << " " << agent << " at " << *all_nodes_[current_node] << "->" << *all_nodes_[next_node]
//                                  << " and " << agents_[other_agent_id] << " at " << *all_nodes_[other_path.back()]
//                                  << " have conflict" << std::endl;
                        return true;
                    } else {
//                        std::cout << "t=" << time_index << " " << agent << " at " << *all_nodes_[current_node] << "->" << *all_nodes_[next_node]
//                                  << " and " << agents_[other_agent_id] << " at " << *all_nodes_[other_path.back()]
//                                  << " have no conflict" << std::endl;
                    }
                } else {
                    if(isCollide(agent, *all_poses_[current_node], *all_poses_[next_node],
                                 other_agent, *all_poses_[other_path[time_index]], *all_poses_[other_path[time_index+1]])) {
//                        std::cout << "t=" << time_index << " " << agent << " at " << *all_nodes_[current_node] << "->" << *all_nodes_[next_node]
//                                  << " and " << agents_[other_agent_id] << " at " << *all_nodes_[other_path[time_index]]
//                                  << "->" << *all_nodes_[other_path[time_index+1]]
//                                  << " have conflict" << std::endl;
                        return true;
                    } else {
//                        std::cout << "t=" << time_index << " " << agent << " at " << *all_nodes_[current_node] << "->" << *all_nodes_[next_node]
//                                  << " and " << agents_[other_agent_id] << " at " << *all_nodes_[other_path[time_index]]
//                                  << "->" << *all_nodes_[other_path[time_index+1]]
//                                  << " have no conflict" << std::endl;
                    }
                }
                // if current agent reach target, check whether it collide with other agent in the future
                // could be accelerate, by avoid
                if(is_goal) {
                    for(int t=earliest_arrive_time_map_.at(agent_global_id); t<other_path.size()-1; t++) {
                        if(isCollide(agent, *all_poses_[current_node], *all_poses_[next_node],
                                     global_agents_[other_agent_id], *all_poses_[other_path[t]], *all_poses_[other_path[t+1]])) {
                            return true;
                        }
                    }
                }
            }
            if(hasCollideWithSAT(agent_global_id, current_node, next_node)) {
                return true;
            }
            return false;
        }

        void insertPoses(const std::vector<AgentType>& agents, const std::vector<size_t>& pose_ids) {
            assert(agents.size() == pose_ids.size());
            for(int i=0; i<pose_ids.size(); i++) {
                insertPose(agents[i].id_, pose_ids[i]);
            }
        }

        void insertPose(int agent_id_global, const size_t& node_id) {
            occupied_table_sat_[node_id/(2*N)].push_back({agent_id_global, node_id});
        }

        void insertPath(int global_agent_id, const LAMAPF_Path& path) {
            path_constraint_table_.insert({global_agent_id, path});
//            if(occupied_table_path_.size() < path.size()) {
//                occupied_table_path_.resize(path.size());
//            }
//            for(int t=0; t<path.size(); t++) {
//                const auto& node_id = path[t];
//                if(occupied_table_path_[t].size() != total_index_) { occupied_table_path_[t].resize(total_index_); }
//                occupied_table_path_[t][node_id/(2*N)].push_back({global_agent_id, node_id});
//            }
        }

        void updateEarliestArriveTimeForAgents(const std::vector<AgentType>& agents, const std::vector<size_t>& target_node_ids) {
            assert(agents.size() == target_node_ids.size());
            earliest_arrive_time_map_.clear();
            for(int k=0; k<agents.size(); k++) {
                int earliest_time = calculateEarliestArriveTimeForAgent(agents[k], target_node_ids[k]);
                earliest_arrive_time_map_.insert({agents[k].id_, earliest_time});
            }
        }

        // call before use bool hasCollide(...)
        // calculate the earliest possible arrive time
        int calculateEarliestArriveTimeForAgent(const AgentType& agent, const size_t& target_node_id) const {
            int temp_earliest_arrive_time_ = 0;
            for(const auto& agent_path : path_constraint_table_) {
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
            std::cout << " set agent " << agent << " earliest_arrive_time_ = " << temp_earliest_arrive_time_ << std::endl;
            return temp_earliest_arrive_time_;
        }

        const std::vector<AgentType>& global_agents_;
        const std::vector<AgentType>& local_agents_;
        const std::vector<PosePtr<int, N> >& all_poses_;
        const float max_excircle_radius_;
        const IS_OCCUPIED_FUNC<N>& isoc_;
        const DimensionLength* dim_;
        Id total_index_;

        std::map<int, Pointis<N> > points_in_agent_circles_; // precomputed check range

        // location id and related poses id
        std::vector<std::vector<std::pair<int, size_t> > > occupied_table_sat_;

        // agent.id_ and related path
        // t, location id and related {agent_global_id, poses id}
        std::vector<std::vector<std::vector<std::pair<int, size_t> > > > occupied_table_path_;
        std::map<int, LAMAPF_Path> path_constraint_table_;
        std::map<int, int> earliest_arrive_time_map_; // agent.id_ and the earliest time to visit the target

    };

    template<Dimension N, typename AgentType>
    using LargeAgentStaticConstraintTablePtr = std::shared_ptr<LargeAgentStaticConstraintTable<N, AgentType> >;

    template<Dimension N>
    struct SubGraphOfAgent {
        int agent_id_;
        std::vector<PosePtr<int, N> > all_nodes_;
        std::vector<std::vector<size_t> > all_edges_;
        std::vector<std::vector<size_t> > all_backward_edges_;
    };

    // input: static occupancy map / current solving problem / previous path as constraints
    // output: what path was found, or empty is failed
    template<Dimension N, typename AgentType>
    using LA_MAPF_FUNC = std::function<std::vector<LAMAPF_Path>(const InstanceOrients<N> &,
                                                                const std::vector<AgentType>&,
                                                                DimensionLength* dim,
                                                                const IS_OCCUPIED_FUNC<N> &,
                                                                const LargeAgentStaticConstraintTablePtr<N, AgentType>&,
                                                                std::vector<std::vector<int> >&,
                                                                int,
                                                                const std::vector<PosePtr<int, N> >,
                                                                const DistanceMapUpdaterPtr<N>,
                                                                const std::vector<SubGraphOfAgent<N> >,
                                                                const std::vector<std::vector<int> >&,
                                                                const std::vector<std::vector<int> >&)>;

    template<Dimension N, typename AgentType>
    Conflicts detectAllConflictBetweenPaths(const LAMAPF_Path& p1, const LAMAPF_Path& p2,
                                            const AgentType& a1, const AgentType& a2,
                                            const std::vector<PosePtr<int, N> >& all_nodes) {
        int t1 = p1.size()-1, t2 = p2.size()-1;
        const auto& longer_agent  = p1.size() > p2.size() ? a1 : a2;
        const auto& shorter_agent = longer_agent.id_ == a1.id_ ? a2 : a1;
        const auto& longer_path   = longer_agent.id_ == a1.id_ ? p1 : p2;
        const auto& shorter_path  = longer_agent.id_ == a1.id_ ? p2 : p1;

        int common_part = std::min(t1, t2);
        std::vector<std::shared_ptr<Conflict> > cfs;
        for(int t=0; t<common_part-1; t++) {
            if(isCollide(a1, *all_nodes[p1[t]], *all_nodes[p1[t+1]],
                         a2, *all_nodes[p2[t]], *all_nodes[p2[t+1]])) {

//                std::cout << "cs type 1 : " << *all_nodes[p1[t]] << "->" << *all_nodes[p1[t+1]] << ", "
//                                            << *all_nodes[p2[t]] << "->" << *all_nodes[p2[t+1]]
//                                            << "/t:{" << t << "," << t+1 << "}" << std::endl;

                auto c1 = std::make_shared<Constraint>(a1.id_, p1[t], p1[t+1], t, t+2);
                auto c2 = std::make_shared<Constraint>(a2.id_, p2[t], p2[t+1], t, t+2);
                auto cf = std::make_shared<Conflict>(a1.id_, a2.id_, Constraints{c1}, Constraints{c2});
                cfs.push_back(cf);
            }
        }
        for(int t=common_part-1; t<std::max(t1, t2) - 1; t++) {
            if(isCollide(longer_agent, *all_nodes[longer_path[t]], *all_nodes[longer_path[t+1]],
                         shorter_agent, *all_nodes[shorter_path.back()])) {

//                std::cout << "cs type 2 : " << *all_nodes[longer_path[t]] << "->" << *all_nodes[longer_path[t+1]] << ", "
//                                            << *all_nodes[shorter_path.back()]
//                                            << "/t:{" << t << "," << t+1 << "}"
//                                            << std::endl;

                auto c1 = std::make_shared<Constraint>(longer_agent.id_,  longer_path[t],      longer_path[t+1], t, t+2);
                auto c2 = std::make_shared<Constraint>(shorter_agent.id_, shorter_path.back(), MAX_NODES,        0, t+2);
                auto cf = std::make_shared<Conflict>(longer_agent.id_, shorter_agent.id_, Constraints{c1}, Constraints{c2});
                cfs.push_back(cf);
            }
        }
        return cfs;
    }

    template<Dimension N, typename AgentType>
    ConflictPtr detectFirstConflictBetweenPaths(const LAMAPF_Path& p1, const LAMAPF_Path& p2,
                                                const AgentType& a1, const AgentType& a2,
                                                const std::vector<PosePtr<int, N> >& all_nodes) {
        int t1 = p1.size()-1, t2 = p2.size()-1;
        const auto& longer_agent  = p1.size() > p2.size() ? a1 : a2;
        const auto& shorter_agent = longer_agent.id_ == a1.id_ ? a2 : a1;
        const auto& longer_path   = longer_agent.id_ == a1.id_ ? p1 : p2;
        const auto& shorter_path  = longer_agent.id_ == a1.id_ ? p2 : p1;

        int common_part = std::min(t1, t2);
        for(int t=0; t<common_part-1; t++) {
            if(isCollide(a1, *all_nodes[p1[t]], *all_nodes[p1[t+1]],
                         a2, *all_nodes[p2[t]], *all_nodes[p2[t+1]])) {

                std::cout << "cs type 1 : " << *all_nodes[p1[t]] << "->" << *all_nodes[p1[t+1]] << ", "
                                            << *all_nodes[p2[t]] << "->" << *all_nodes[p2[t+1]]
                                            << "/t:{" << t << "," << t+1 << "}" << std::endl;

                auto c1 = std::make_shared<Constraint>(a1.id_, p1[t], p1[t+1], t, t+2);
                auto c2 = std::make_shared<Constraint>(a2.id_, p2[t], p2[t+1], t, t+2);
                auto cf = std::make_shared<Conflict>(a1.id_, a2.id_, Constraints{c1}, Constraints{c2});
                return cf;
            }
        }
        for(int t=common_part-1; t<std::max(t1, t2) - 1; t++) {
            if(isCollide(longer_agent, *all_nodes[longer_path[t]], *all_nodes[longer_path[t+1]],
                         shorter_agent, *all_nodes[shorter_path.back()])) {

                std::cout << "cs type 2 : " << *all_nodes[longer_path[t]] << "->" << *all_nodes[longer_path[t+1]] << ", "
                                            << *all_nodes[shorter_path.back()]
                                            << "/t:{" << t << "," << t+1 << "}"
                                            << std::endl;

                auto c1 = std::make_shared<Constraint>(longer_agent.id_,  longer_path[t],      longer_path[t+1], t, t+2);
                auto c2 = std::make_shared<Constraint>(shorter_agent.id_, shorter_path.back(), MAX_NODES,        t, t+2);
                auto cf = std::make_shared<Conflict>(longer_agent.id_, shorter_agent.id_, Constraints{c1}, Constraints{c2});
                return cf;
            }
        }
        return nullptr;
    }

    template<Dimension N, typename AgentType>
    bool isSolutionValid(const LAMAPF_Paths& paths,
                         const std::vector<AgentType>& agents,
                         const std::vector<PosePtr<int, N> >& all_poses) {
        std::cout << __FUNCTION__ << std::endl;
        assert(paths.size() == agents.size());
        bool valid = true;
        for(int i=0; i<paths.size(); i++) {
            for(int j=i+1; j<paths.size(); j++) {
                if(paths[i].empty() || paths[j].empty()) { continue; }
                auto conflict_ptr = detectFirstConflictBetweenPaths(paths[i], paths[j], agents[i], agents[j], all_poses);
                if(conflict_ptr != nullptr) {
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

}

#endif //LAYEREDMAPF_COMMON_H
