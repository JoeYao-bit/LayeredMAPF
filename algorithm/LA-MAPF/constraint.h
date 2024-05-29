//
// Created by yaozhuo on 2024/5/7.
//

#ifndef LAYEREDMAPF_CONSTRAINT_H
#define LAYEREDMAPF_CONSTRAINT_H

#include "common.h"

namespace freeNav::LayeredMAPF::LA_MAPF {



    template<Dimension N, typename AgentType>
    struct ConstraintTable {

        // agent id / edge id (edge id / node size = from node id, edge id % node size = to node id)
        typedef std::tuple<int, size_t> EdgeInfo;

    public:

        ConstraintTable(int agent_id,
                        const std::vector<AgentType>& agents,
                        const std::vector<PosePtr<N> >& all_poses,
                        DimensionLength* dim,
                        const IS_OCCUPIED_FUNC<N> & isoc) : agent_id_(agent_id), node_size_(all_poses.size()), agents_(agents), all_poses_(all_poses), dim_(dim), isoc_(isoc) {
            // get max excircle radius /  min incircle radius
            for(const auto& agent : agents) {
                if(agent.excircle_radius_ > max_excircle_radius_) {
                    max_excircle_radius_ = agent.excircle_radius_;
                }
                if(agent.incircle_radius_ < min_incircle_radius_) {
                    min_incircle_radius_ = agent.incircle_radius_;
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

        bool constrained(const size_t& id, int t) const {
            assert(id >= 0);
            const auto &it = ct_.find(id);
            if (it == ct_.end()) {
                return false;
            }
            // yz: whether current vertex is occupied in the time range
            for (const auto &constraint: it->second) {
                if (constraint.first <= t && t < constraint.second)
                    return true;
            }
            return false;
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
            if (cat_.empty()) {
                cat_.resize(node_size_);
                cat_goals_.resize(node_size_, MAX_TIMESTEP);
            }
            assert(cat_goals_[path.back()] == MAX_TIMESTEP);
            cat_goals_[path.back()] = path.size() - 1;
            // every agent is at its start when timestep = 0
            for (auto timestep = (int) path.size() - 1; timestep >= 1; timestep--) {
                int loc = path[timestep], pre_loc = path[timestep-1];
                if (cat_[loc].size() <= timestep) {
                    cat_[loc].resize(timestep + 1, {});
                }
                cat_[loc][timestep].push_back(std::make_pair(agent_id, getEdgeIndex(pre_loc, loc)));
            }
            cat_max_timestep_ = std::max(cat_max_timestep_, (int) path.size() - 1);
        }

        // get how many conflict the agent by transfer from node curr_id to next_id have
        // using collision check between agents, so need agent information
        int getNumOfConflictsForStep(const PosePtr<N>& curr_ps, const PosePtr<N>& next_ps, int next_timestep) const {
            int rst = 0;
            const float& agent_excircle_radius = agents_[agent_id_].excircle_radius_;
            if (!cat_.empty()) {
                // check whether curr and next node have other agent in time
                float sensitive_radius = agent_excircle_radius + agent_excircle_radius;
                DimensionLength temp_dim[N];
                Pointi<N> basic_offset;
                for(int dim=0; dim<N; dim++) {
                    temp_dim[dim] = (DimensionLength)ceil(sensitive_radius);
                    basic_offset[dim] = temp_dim[dim] / 2;
                }
                Id local_total_index = getTotalIndexOfSpace<N>(temp_dim);
                Pointi<N> near_offset, near_curr_pt, near_next_pt;
                Id near_next_pt_id;
                // should use a radius limitation to make it a circle, now it is a square
                for(size_t offset_id=0; offset_id<local_total_index; offset_id++) {
                    near_offset = IdToPointi<N>(offset_id, temp_dim);
                    near_next_pt = next_ps->pt_ - basic_offset + near_offset;
                    // check other path that close to next position, as they may cause conflict
                    if(!isOutOfBoundary(near_next_pt, dim_)) {
                        near_next_pt_id = PointiToId(near_next_pt, dim_);
                        for(size_t orient=0; orient<2*N; orient ++) {
                            size_t near_next_node_id = near_next_pt_id*2*N + orient;
                            if(!cat_[next_timestep-1][near_next_node_id].empty()) {
                                // current agent go to next node while another agent just leave next node
                                std::vector<EdgeInfo> other_edges = cat_[next_timestep-1][near_next_node_id];
                                for(const auto& edge_info : other_edges) {
                                    size_t other_node_id = std::get<1>(edge_info) % node_size_;
                                    if(isCollide(agents_[agent_id_], *curr_ps, *next_ps,
                                                 agents_[std::get<0>(edge_info)], *all_poses_[near_next_node_id], *all_poses_[other_node_id])) {
                                        rst++;
                                    }
                                }
                            }
                            if(!cat_[next_timestep][near_next_node_id].empty()) {
                                // both current agent and another agent go to next node
                                std::vector<EdgeInfo> other_edges = cat_[next_timestep][near_next_node_id];
                                for(const auto& edge_info : other_edges) {
                                    size_t other_node_id = std::get<1>(edge_info) / node_size_;
                                    if(isCollide(agents_[agent_id_], *curr_ps, *next_ps,
                                                 agents_[std::get<0>(edge_info)], *all_poses_[other_node_id], *all_poses_[near_next_node_id])) {
                                        rst++;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            return rst;
        }

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

        const std::vector<AgentType>& agents_;

        const std::vector<PosePtr<N> >& all_poses_;

        float max_excircle_radius_ = 0;

        float min_incircle_radius_ = MAX<float>;

        int ct_max_timestep_ = 0;

        int cat_max_timestep_ = 0;

        int length_min_ = 0;

        int length_max_ = MAX_TIMESTEP;

        typedef boost::unordered_map<size_t, std::list<std::pair<int, int> > > CT; // constraint table

        CT ct_; // node -> time range, or edge -> time range

        typedef std::vector<std::vector<std::vector<EdgeInfo> > > CAT; // [time_index][node_id][agent_id, edge_id]

        CAT cat_; // store collision state of grid map, depth is time range

        std::vector<int> cat_goals_; // the visit time of goal

    };




}

#endif //LAYEREDMAPF_CONSTRAINT_H
