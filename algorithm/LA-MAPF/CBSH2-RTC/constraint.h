//
// Created by yaozhuo on 2024/5/7.
//

#ifndef LAYEREDMAPF_CONSTRAINT_CBSH2_RTC_H
#define LAYEREDMAPF_CONSTRAINT_CBSH2_RTC_H

#include "../common.h"
#include "constraint_avoidance_table.h"
#include "high_level_node.h"

namespace freeNav::LayeredMAPF::LA_MAPF::CBSH2_RTC {

    template<Dimension N, typename State>
    struct ConstraintTable {

        // agent id / edge id (edge id / node size = from node id, edge id % node size = to node id)
        typedef std::tuple<int, size_t> EdgeInfo;

    public:

        ConstraintTable(int agent_id,
                        const std::vector<AgentPtr<N> >& agents,
                        const std::vector<std::shared_ptr<State>>& all_poses,
                        DimensionLength* dim,
                        const IS_OCCUPIED_FUNC<N> & isoc)
                        : agent_id_(agent_id), node_size_(all_poses.size()), agents_(agents),
                          all_poses_(all_poses), dim_(dim), isoc_(isoc)  {
            // get max excircle radius /  min incircle radius
            for(const auto& agent : agents) {
                if(agent->excircle_radius_ > max_excircle_radius_) {
                    max_excircle_radius_ = agent->excircle_radius_;
                }
                if(agent->incircle_radius_ < min_incircle_radius_) {
                    min_incircle_radius_ = agent->incircle_radius_;
                }
            }
        }

        inline size_t getEdgeIndex(const size_t& from, const size_t& to) const { return from * node_size_ + to; }


        void insert2CT(const size_t& from, const size_t& to, int t_min, int t_max) {
            insert2CT(getEdgeIndex(from, to), t_min, t_max);
        }

        void insert2CT(const size_t& loc, int t_min, int t_max) {
            assert(loc >= 0);
            ct_[loc].emplace_back(t_min, t_max);
            if (t_max < MAX_TIMESTEP && t_max > ct_max_timestep_) {
                ct_max_timestep_ = t_max;
            } else if (t_max == MAX_TIMESTEP && t_min > ct_max_timestep_) {
                ct_max_timestep_ = t_min;
            }
        }

        void insertCT(const Constraints& csts, int agent) {
            // <agent id, node from, node to, time range start, time range end>
            int agent_id, start_t, end_t;
            size_t node_from, node_to;
            constraint_type cst;
            for(const auto& cs : csts) {
                std::tie(agent_id, node_from, node_to, start_t, end_t, cst) = *cs;
                if(agent_id != agent) { continue; }
                length_min_ = std::max(length_min_, end_t + 1);
                if(node_to != MAX_NODES) {
                    insert2CT(node_from, node_to, start_t, end_t);
                } else {
                    insert2CT(node_from, start_t, end_t);
                }
            }
        }

        bool constrained(const size_t& id, int t) const {
            assert(id >= 0);
            const auto &it = ct_.find(id);
            if (it == ct_.end()) {
                return false;
            }
            //std::cout << "constrained id,t = " << id << ", " << t << std::endl;
            // yz: whether current vertex is occupied in the time range
            for (const auto &constraint: it->second) {
                //std::cout << "constraint.first/second = " << constraint.first << "/" << constraint.second << std::endl;
                if (constraint.first <= t && t < constraint.second) {
                    return true;
                }
            }
            return false;
        }

        // insert a landmark, i.e., the agent has to be at the given location at the given timestep
        void insertLandmark(size_t loc,
                            int t) {
            auto it = landmarks_.find(t);
            if (it == landmarks_.end()) {
                landmarks_[t] = loc;
                if (t > latest_timestep_)
                    latest_timestep_ = t;
            } else
                assert(it->second == loc);
        }


// return the location-time pairs on the barrier in an increasing order of their timesteps
        std::list<std::pair<int, int>> decodeBarrier(int x, int y, int t) {
            std::list<std::pair<int, int>> rst;
//            int x1 = x / num_col, y1 = x % num_col;
//            int x2 = y / num_col, y2 = y % num_col;
//            if (x1 == x2) {
//                if (y1 < y2)
//                    for (int i = min(y2 - y1, t); i >= 0; i--) {
//                        rst.emplace_back(x1 * num_col + y2 - i, t - i);
//                    }
//                else
//                    for (int i = min(y1 - y2, t); i >= 0; i--) {
//                        rst.emplace_back(x1 * num_col + y2 + i, t - i);
//                    }
//            } else // y1== y2
//            {
//                if (x1 < x2)
//                    for (int i = min(x2 - x1, t); i >= 0; i--) {
//                        rst.emplace_back((x2 - i) * num_col + y1, t - i);
//                    }
//                else
//                    for (int i = min(x1 - x2, t); i >= 0; i--) {
//                        rst.emplace_back((x2 + i) * num_col + y1, t - i);
//                    }
//            }
            return rst;
        }

        // build the constraint table for the given agent at the given node
        void build(const CBSNode &node, int agent) {
            auto curr = &node;
            while (curr->parent != nullptr) {
                int a, t1, t2;
                size_t x, y;
                constraint_type type;
                std::tie(a, x, y, t1, t2, type) = *curr->constraints.front();
                switch (type) {
                    case constraint_type::LEQLENGTH:
                        assert(curr->constraints.size() <= 2);
                        if (agent == a) // this agent has to reach its goal at or before timestep t.
                            length_max_ = std::min(length_max_, t1);
                        else // other agents cannot stay at x at or after timestep t
                            insert2CT(x, t1, MAX_TIMESTEP);
                        if ((int) curr->constraints.size() == 2) // generated by a corridor-target conflict
                        {
                            std::tie(a, x, y, t1, t2, type) = *curr->constraints.back();
                            if (type == constraint_type::GLENGTH && a == agent)
                                length_min_ = std::max(length_min_,
                                                 t1 + 1); // path of this agent should be of length at least t + 1
                            else if (type == constraint_type::RANGE && a == agent) {
                                insert2CT(x, y, t1 + 1); // the agent cannot stay at x from timestep y to timestep t.
                            } else {
                                assert(1); // this should never happen
                            }
                        }
                        break;
                    case constraint_type::GLENGTH:
                        assert(curr->constraints.size() == 1);
                        if (a == agent) // path of agent_id should be of length at least t + 1
                            length_min_ = std::max(length_min_, t1 + 1);
                        break;
                    case constraint_type::POSITIVE_VERTEX:
                        assert(curr->constraints.size() == 1);
                        if (agent == a) // this agent has to be at x at timestep t
                        {
                            insertLandmark(x, t1);
                        } else // other agents cannot stay at x at timestep t
                        {
                            insert2CT(x, t1, t1 + 1);
                        }
                        break;
                    case constraint_type::POSITIVE_EDGE:
                        assert(curr->constraints.size() == 1);
                        if (agent == a) // this agent has to be at x at timestep t - 1 and be at y at timestep t
                        {
                            insertLandmark(x, t1 - 1);
                            insertLandmark(y, t1);
                        } else // other agents cannot stay at x at timestep t - 1, be at y at timestep t, or traverse edge (y, x) from timesteps t - 1 to t
                        {
                            insert2CT(x, t1 - 1, t1);
                            insert2CT(y, t1, t1 + 1);
                            insert2CT(y, x, t1, t1 + 1);
                        }
                        break;
                    case constraint_type::VERTEX:
                        if (a == agent) {
                            for (const auto &constraint : curr->constraints) // we might have multiple vertex constraints generated by mutex propagation
                            {
                                std::tie(a, x, y, t1, t2,type) = *constraint;
                                insert2CT(x, t1, t1 + 1);
                            }
                        }
                        break;
                    case constraint_type::EDGE:
                        assert(curr->constraints.size() == 1);
                        if (a == agent)
                            insert2CT(x, y, t1, t1 + 1);
                        break;
                    case constraint_type::BARRIER:
                        if (a == agent) {
                            for (auto constraint : curr->constraints) {
                                std::tie(a, x, y, t1, t2, type) = *constraint;
                                assert(a == agent);
                                auto states = decodeBarrier(x, y, t1); // state = (location, timestep)
                                for (const auto &state : states) {
                                    insert2CT(state.first, state.second, state.second + 1);
                                }
                            }
                        }
                        break;
                    case constraint_type::RANGE:
                        assert(curr->constraints.size() == 1);
                        if (a == agent) {
                            insert2CT(x, y, t1 + 1); // the agent cannot stay at x from timestep y to timestep t.
                        }
                        break;
                }
                curr = curr->parent;
            }
            if (latest_timestep_ < length_min_)
                latest_timestep_ = length_min_;
            if (length_max_ < MAX_TIMESTEP && latest_timestep_ < length_max_)
                latest_timestep_ = length_max_;
        }

        bool constrained(const size_t& curr_loc, const size_t& next_loc, int next_t) const {
            return constrained(getEdgeIndex(curr_loc, next_loc), next_t);
        }

        // build the conflict avoidance table
        void insert2CAT(int agent, const std::vector<LAMAPF_Path> &paths) {
            for (size_t ag = 0; ag < paths.size(); ag++) {
                if (ag == agent || paths[ag].empty())
                    continue;
                insert2CAT(ag, paths[ag]);
            }
        }

       // yz: set a path as soft constraint, avoid other have conflict with this path
        void insert2CAT(int agent_id, const LAMAPF_Path &path) {
            cat_max_timestep_ = std::max(cat_max_timestep_, (int) path.size() - 1);
//            cat_.insertAgentPathOccGrids(agents_[agent_id], path);
        }

        // get how many conflict the agent by transfer from node curr_id to next_id have
        // using collision check between agents, so need agent information
//        int getNumOfConflictsForStep(const PosePtr<int, N>& curr_ps, const PosePtr<int, N>& next_ps, int next_timestep) const {
//            Pointis<N> occ_grids = agents_[agent_id_].getTransferOccupiedGrid(*curr_ps, *next_ps);
//            return cat_.getNumOfConflictsForStep(occ_grids, agent_id_, next_timestep);
//        }

        // the earliest timestep that the agent can hold the location after earliest_timestep
        int getHoldingTime(const size_t& node_id, int earliest_timestep) const
        {
            int rst = earliest_timestep;
            // CT
            auto it = ct_.find(node_id);
            if (it != ct_.end()) {
                for (auto time_range : it->second)
                    rst = std::max(rst, time_range.second);
            }
            return rst;
        }

        int getMaxTimestep() const // everything is static after the max timestep
        {
            int rst = std::max(std::max(ct_max_timestep_, cat_max_timestep_), length_min_);
            if (length_max_ < MAX_TIMESTEP)
                rst = std::max(rst, length_max_);
            return rst;
        }

        //private:

        DimensionLength* dim_;

        const IS_OCCUPIED_FUNC<N>& isoc_;

        const int node_size_;

        int agent_id_;

        const std::vector<AgentPtr<N> >& agents_;

        const std::vector<std::shared_ptr<State>>& all_poses_;

        float max_excircle_radius_ = 0;

        float min_incircle_radius_ = MAX<float>;

        int ct_max_timestep_ = 0;

        int cat_max_timestep_ = 0;

        int length_min_ = 0;

        int length_max_ = MAX_TIMESTEP;

        typedef boost::unordered_map<size_t, std::list<std::pair<int, int> > > CT; // constraint table

        CT ct_; // node -> time range, or edge -> time range

        std::unordered_map<size_t, size_t> landmarks_; // <timestep, location>: the agent must be at the given location at the given timestep

        int latest_timestep_ = 0; // No negative constraints after this timestep.

    };

    template<Dimension N, typename State>
    using ConstraintTablePtr = std::shared_ptr<ConstraintTable<N, State> >;

}

#endif //LAYEREDMAPF_CONSTRAINT_H
